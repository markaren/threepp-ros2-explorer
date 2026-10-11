"""A tiny stand-in for the simulator, so the tests need neither ROS nor a GPU.

It answers the two questions the tests ask:

* What does the camera see? `draw_scene` paints a cluttered grey and blue
  background, `draw_cone` paints a cone where the pinhole model says it is.
* What does the lidar measure? `cast_scan` intersects 360 rays with the cones
  and a wall.

Both use the same geometry as the real robot (see docs/interfaces.md): camera
0.18 m ahead of `base_link` and 0.25 m up, lidar on the origin and 0.30 m up.
"""

import math
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from explorer_vision.cone_locator import CameraIntrinsics, ConeModel, Scan

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 360
VERTICAL_FOV_DEG = 50.0
CAMERA_X = 0.18
CAMERA_Z = 0.25
LIDAR_Z = 0.30
CONE = ConeModel(height=0.70, base_radius=0.20)

# OpenCV colours are B, G, R. The cone is RGB (255, 106, 0).
CONE_BGR = (0, 106, 255)
FLOOR_BGR = (120, 120, 120)
WALL_BGR = (136, 112, 96)
TEAL_BGR = (128, 128, 0)
WHITE_BGR = (235, 235, 235)


def intrinsics() -> CameraIntrinsics:
    """Pinhole intrinsics of a 640 x 360 camera with a 50 degree vertical view."""
    focal = (IMAGE_HEIGHT / 2.0) / math.tan(math.radians(VERTICAL_FOV_DEG) / 2.0)
    return CameraIntrinsics(fx=focal, fy=focal, cx=IMAGE_WIDTH / 2.0, cy=IMAGE_HEIGHT / 2.0)


def project(x: float, y: float, z: float) -> Tuple[float, float]:
    """Project a point given in `base_link` to a pixel (u, v)."""
    k = intrinsics()
    depth = x - CAMERA_X
    u = k.cx - k.fx * y / depth
    v = k.cy - k.fy * (z - CAMERA_Z) / depth
    return u, v


def cone_outline(x: float, y: float) -> np.ndarray:
    """Return the cone's silhouette as a polygon in pixels: base circle plus apex."""
    points = [project(x, y, CONE.height)]
    for step in range(48):
        angle = 2.0 * math.pi * step / 48
        points.append(project(x + CONE.base_radius * math.cos(angle),
                              y + CONE.base_radius * math.sin(angle), 0.0))
    return cv2.convexHull(np.array(points, dtype=np.float32)).reshape(-1, 2)


def cone_box(x: float, y: float) -> Tuple[float, float, float, float]:
    """Return the ideal bounding box (left, top, right, bottom) of a cone."""
    outline = cone_outline(x, y)
    return (float(outline[:, 0].min()), float(outline[:, 1].min()),
            float(outline[:, 0].max()), float(outline[:, 1].max()))


def draw_scene(seed: int = 0) -> np.ndarray:
    """Paint a warehouse-like background with no orange in it."""
    image = np.empty((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8)
    image[:205] = WALL_BGR
    image[205:] = FLOOR_BGR
    cv2.rectangle(image, (60, 120), (170, 230), TEAL_BGR, cv2.FILLED)
    cv2.rectangle(image, (480, 40), (510, 215), WHITE_BGR, cv2.FILLED)
    cv2.rectangle(image, (250, 150), (330, 205), (90, 84, 80), cv2.FILLED)
    cv2.line(image, (0, 300), (IMAGE_WIDTH, 260), (150, 150, 150), 3)

    # Sensor noise, so that no surface is one perfectly flat colour.
    noise = np.random.default_rng(seed).normal(0.0, 4.0, image.shape)
    return np.clip(image.astype(np.float64) + noise, 0, 255).astype(np.uint8)


def draw_cone(image: np.ndarray, x: float, y: float, brightness: float = 1.0) -> None:
    """Paint a cone standing at (x, y) in `base_link`, lit from the left.

    The right half is drawn darker, like a surface turned away from the sun:
    the brightness changes across the cone, the hue does not.
    """
    lit = tuple(int(channel * brightness) for channel in CONE_BGR)
    shaded = tuple(int(channel * brightness * 0.45) for channel in CONE_BGR)

    outline = cone_outline(x, y)
    cv2.fillConvexPoly(image, np.round(outline).astype(np.int32), shaded, cv2.LINE_AA)

    apex_u = project(x, y, CONE.height)[0]
    left_half = outline[outline[:, 0] <= apex_u]
    if len(left_half) >= 3:
        cv2.fillConvexPoly(image, np.round(left_half).astype(np.int32), lit, cv2.LINE_AA)


def through_jpeg(image: np.ndarray, quality: int = 80) -> np.ndarray:
    """Compress and decompress, as the image is on its way through ROS."""
    _, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR)


def cast_scan(cones: Sequence[Tuple[float, float]], wall_x: Optional[float]) -> Scan:
    """Simulate the lidar of the contract: 360 beams, beam 180 looks ahead.

    Each cone is a circle (its cross-section at the height of the scan plane),
    the wall is the line x = wall_x. Beams that hit nothing report +inf.
    """
    beam_count = 360
    angle_min = -math.pi
    angle_increment = 2.0 * math.pi / beam_count
    radius = CONE.radius_at(LIDAR_Z)

    ranges: List[float] = []
    for beam in range(beam_count):
        angle = angle_min + beam * angle_increment
        direction = (math.cos(angle), math.sin(angle))
        nearest = math.inf

        if wall_x is not None and direction[0] > 1e-9:
            nearest = wall_x / direction[0]

        for cone_x, cone_y in cones:
            # Ray from the origin against a circle: solve |t * d - c| = radius.
            along = direction[0] * cone_x + direction[1] * cone_y
            discriminant = radius * radius - (cone_x * cone_x + cone_y * cone_y) + along * along
            if discriminant >= 0.0:
                hit = along - math.sqrt(discriminant)
                if 0.0 < hit < nearest:
                    nearest = hit

        ranges.append(nearest if nearest <= 8.0 else math.inf)

    return Scan(angle_min=angle_min, angle_increment=angle_increment,
                ranges=np.array(ranges), range_min=0.12, range_max=8.0)
