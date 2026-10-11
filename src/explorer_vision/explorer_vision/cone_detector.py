"""Find orange traffic cones in a camera image with classic image processing.

This module knows nothing about ROS. It takes a BGR image (the channel order
OpenCV uses everywhere) and returns bounding boxes, so it can be read, run and
tested on its own, with a picture from disk.

The method has four steps:

1. Colour. Convert to HSV and keep the pixels whose hue is orange and whose
   saturation is high. Hue is the right channel to threshold because light and
   shadow change the brightness (V) of a surface a lot and its hue very little.
2. Clean-up. A morphological opening removes single stray pixels, a closing
   fills small holes inside a blob.
3. Blobs. Each connected region of the mask becomes a contour.
4. Shape. A blob is only a cone if it is big enough, taller than it is wide,
   and narrower at the top than at the bottom. This is what rejects an orange
   crate or an orange ball.
"""

from dataclasses import dataclass
from typing import List

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectorThresholds:
    """Every number the detector depends on, in one place.

    OpenCV stores hue as degrees divided by two, so that it fits in one byte:
    the range is 0..179, not 0..359. The cone colour RGB (255, 106, 0) has a
    hue of 25 degrees, which is 12 here. Rendered with a sun and ambient
    light, the cone's pixels measure hue 6..12, saturation above 125 and
    brightness above 100 (see test/data/rendered_cones.png), so the limits
    below leave a margin on every side.
    """

    hue_min: int = 3
    hue_max: int = 24
    saturation_min: int = 110
    value_min: int = 40
    # Blobs with fewer pixels than this are noise, or cones too far away to
    # say anything about their shape.
    min_area: float = 80.0
    # Height divided by width of the bounding box. The cone in the simulator is
    # 0.70 m tall and 0.40 m wide, which gives 1.75.
    min_aspect: float = 1.2
    # Mean width of the top third divided by mean width of the bottom third.
    # A perfect triangle gives 0.2, a rectangle or a ball gives 1.0.
    max_taper: float = 0.6


@dataclass(frozen=True)
class Detection:
    """One cone in the image: an axis-aligned box in pixels.

    The origin is the top-left corner of the image, x grows to the right and
    y grows downwards, as in every OpenCV function.
    """

    x: int
    y: int
    width: int
    height: int
    # 0..1, how well the blob fills the triangle inscribed in its box.
    score: float
    # True when the box touches the image border. Part of the cone is then
    # outside the picture, so neither the height nor the centre of the box
    # describes the whole cone.
    clipped: bool

    @property
    def left(self) -> int:
        """Pixel column of the left edge."""
        return self.x

    @property
    def right(self) -> int:
        """Pixel column just past the right edge."""
        return self.x + self.width

    @property
    def centre_x(self) -> float:
        return self.x + self.width / 2.0

    @property
    def centre_y(self) -> float:
        return self.y + self.height / 2.0


def orange_mask(image_bgr: np.ndarray, thresholds: DetectorThresholds) -> np.ndarray:
    """Return a binary image: 255 where the pixel is saturated orange, else 0."""
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    lower = np.array(
        [thresholds.hue_min, thresholds.saturation_min, thresholds.value_min],
        dtype=np.uint8)
    upper = np.array([thresholds.hue_max, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)

    # Opening = erode then dilate: specks smaller than the kernel disappear,
    # larger blobs keep their size. Closing = dilate then erode: small holes
    # (a highlight, a JPEG artefact) are filled.
    small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, small)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, large)
    return mask


def taper_ratio(blob: np.ndarray) -> float:
    """Measure how much narrower a blob is at the top than at the bottom.

    `blob` is the filled blob cut out along its bounding box (255 inside).
    The result is the mean width of the top third divided by the mean width
    of the bottom third: small for a cone, about 1 for a box or a ball, above
    1 for anything that is wider at the top.
    """
    row_widths = np.count_nonzero(blob, axis=1).astype(np.float64)
    third = max(1, len(row_widths) // 3)
    top = row_widths[:third].mean()
    bottom = row_widths[-third:].mean()
    return float(top / bottom) if bottom > 0.0 else float('inf')


def detect_cones(
        image_bgr: np.ndarray,
        thresholds: DetectorThresholds = DetectorThresholds()) -> List[Detection]:
    """Find the cones in a BGR image, largest (nearest) first."""
    image_height, image_width = image_bgr.shape[:2]
    mask = orange_mask(image_bgr, thresholds)

    # RETR_EXTERNAL: only the outer outline of each blob, holes are ignored.
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detections: List[Detection] = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < thresholds.min_area:
            continue

        x, y, width, height = cv2.boundingRect(contour)
        if height < thresholds.min_aspect * width:
            continue

        # Draw this one blob, filled, into an image the size of its box. The
        # contour is shifted so that the box's corner becomes (0, 0).
        blob = np.zeros((height, width), dtype=np.uint8)
        cv2.drawContours(blob, [contour], -1, 255, cv2.FILLED, offset=(-x, -y))
        if taper_ratio(blob) > thresholds.max_taper:
            continue

        # A triangle fills half of its bounding box. The score says how close
        # the blob's area is to that: 1.0 for a perfect triangle.
        triangle_area = 0.5 * width * height
        score = min(area, triangle_area) / max(area, triangle_area)

        clipped = (x <= 0 or y <= 0
                   or x + width >= image_width or y + height >= image_height)
        detections.append(Detection(x, y, width, height, float(score), clipped))

    detections.sort(key=lambda d: d.width * d.height, reverse=True)
    return detections


def draw_detections(
        image_bgr: np.ndarray,
        detections: List[Detection],
        labels: List[str]) -> np.ndarray:
    """Return a copy of the image with one labelled box per detection.

    A complete cone gets a green box, a clipped one a yellow box.
    """
    annotated = image_bgr.copy()
    for detection, label in zip(detections, labels):
        colour = (0, 220, 255) if detection.clipped else (0, 255, 0)
        top_left = (detection.x, detection.y)
        bottom_right = (detection.right - 1, detection.y + detection.height - 1)
        cv2.rectangle(annotated, top_left, bottom_right, colour, 2)
        text_origin = (detection.x, max(12, detection.y - 5))
        cv2.putText(annotated, label, text_origin, cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, colour, 1, cv2.LINE_AA)
    return annotated
