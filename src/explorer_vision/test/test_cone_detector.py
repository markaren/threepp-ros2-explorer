"""The detector on painted images: what it must find, and what it must not."""

import cv2
import numpy as np
import pytest

import synthetic_world as world
from explorer_vision.cone_detector import Detection, DetectorThresholds
from explorer_vision.cone_detector import cone_mask, detect_cones


def overlap(detection: Detection, box) -> float:
    """Intersection over union of a detection and a (left, top, right, bottom) box."""
    left, top, right, bottom = box
    inter_width = min(detection.right, right) - max(detection.left, left)
    inter_height = min(detection.y + detection.height, bottom) - max(detection.y, top)
    if inter_width <= 0 or inter_height <= 0:
        return 0.0
    intersection = inter_width * inter_height
    union = (detection.width * detection.height
             + (right - left) * (bottom - top) - intersection)
    return intersection / union


def test_empty_warehouse_has_no_cones():
    assert detect_cones(world.draw_scene()) == []


def test_finds_two_cones_in_clutter():
    image = world.draw_scene()
    world.draw_cone(image, 3.0, 0.4)
    world.draw_cone(image, 2.0, -0.8)

    detections = detect_cones(world.through_jpeg(image))

    assert len(detections) == 2
    # Largest first: the cone at 2 m looks bigger than the one at 3 m.
    assert overlap(detections[0], world.cone_box(2.0, -0.8)) > 0.8
    assert overlap(detections[1], world.cone_box(3.0, 0.4)) > 0.8
    assert all(not detection.clipped for detection in detections)
    assert all(detection.score > 0.7 for detection in detections)


def test_finds_a_cone_in_shadow_and_a_distant_one():
    image = world.draw_scene()
    world.draw_cone(image, 2.5, 0.6, brightness=0.5)
    world.draw_cone(image, 5.0, -0.5)

    detections = detect_cones(world.through_jpeg(image))

    assert len(detections) == 2
    assert overlap(detections[0], world.cone_box(2.5, 0.6)) > 0.75
    assert overlap(detections[1], world.cone_box(5.0, -0.5)) > 0.6


def test_rejects_orange_things_of_the_wrong_shape():
    image = world.draw_scene()
    # A crate: wider than tall.
    cv2.rectangle(image, (40, 240), (140, 300), world.CONE_BGR, cv2.FILLED)
    # A pole: taller than wide, but as wide at the top as at the bottom.
    cv2.rectangle(image, (200, 180), (225, 300), world.CONE_BGR, cv2.FILLED)
    # A ball.
    cv2.circle(image, (330, 280), 35, world.CONE_BGR, cv2.FILLED)
    # A funnel: a cone standing on its tip.
    funnel = np.array([[420, 220], [480, 220], [450, 320]], dtype=np.int32)
    cv2.fillConvexPoly(image, funnel, world.CONE_BGR)

    assert detect_cones(world.through_jpeg(image)) == []


def test_distractors_do_not_hide_a_real_cone():
    image = world.draw_scene()
    cv2.rectangle(image, (40, 240), (140, 300), world.CONE_BGR, cv2.FILLED)
    world.draw_cone(image, 3.0, -0.3)

    detections = detect_cones(world.through_jpeg(image))

    assert len(detections) == 1
    assert overlap(detections[0], world.cone_box(3.0, -0.3)) > 0.8


def test_cone_cut_by_the_image_border_is_flagged():
    image = world.draw_scene()
    # At 2 m and 1.5 m to the right, the cone's axis projects to about
    # column 638: half of the cone lies outside the 640 pixel wide image.
    world.draw_cone(image, 2.0, -1.5)

    detections = detect_cones(world.through_jpeg(image))

    assert len(detections) == 1
    assert detections[0].clipped
    assert detections[0].right == world.IMAGE_WIDTH


def test_small_blobs_are_ignored():
    image = world.draw_scene()
    world.draw_cone(image, 3.0, 0.0)

    # The cone at 3 m covers about 2700 pixels.
    strict = DetectorThresholds(min_area=5000.0)

    assert len(detect_cones(image)) == 1
    assert detect_cones(image, strict) == []


def test_hue_limits_are_what_selects_orange():
    image = world.draw_scene()
    world.draw_cone(image, 3.0, 0.0)

    # Only accept hues around green: the orange cone must disappear.
    green_only = DetectorThresholds(hue_min=50, hue_max=70)

    assert detect_cones(image, green_only) == []


# From here on: traffic cones with a white reflective band, and the light grey
# plaster wall that looks just like that band to a colour threshold.

@pytest.mark.parametrize('distance, min_overlap', [(2.0, 0.8), (3.0, 0.8), (4.0, 0.75), (5.0, 0.7)])
def test_banded_cone_in_front_of_a_light_wall_is_one_box(distance, min_overlap):
    image = world.draw_scene(wall_bgr=world.PLASTER_BGR)
    world.draw_cone(image, distance, 0.3, band=True)

    detections = detect_cones(world.through_jpeg(image))

    # One box around tip, band and base. Not two boxes, and not a box that
    # has grown into the wall.
    assert len(detections) == 1
    assert overlap(detections[0], world.cone_box(distance, 0.3)) > min_overlap


def test_banded_cone_in_shadow_and_in_front_of_a_dark_wall():
    image = world.draw_scene()
    world.draw_cone(image, 2.5, 0.6, brightness=0.6, band=True)
    world.draw_cone(image, 3.5, -0.6, band=True)

    detections = detect_cones(world.through_jpeg(image))

    assert len(detections) == 2
    assert overlap(detections[0], world.cone_box(2.5, 0.6)) > 0.8
    assert overlap(detections[1], world.cone_box(3.5, -0.6)) > 0.75


def test_plain_cone_in_front_of_a_light_wall_keeps_its_box():
    image = world.draw_scene(wall_bgr=world.PLASTER_BGR)
    world.draw_cone(image, 3.0, 0.3)

    detections = detect_cones(world.through_jpeg(image))

    assert len(detections) == 1
    assert overlap(detections[0], world.cone_box(3.0, 0.3)) > 0.8


def test_white_object_behind_a_cone_is_not_merged_into_it():
    image = world.draw_scene(wall_bgr=world.PLASTER_BGR)
    # The white pillar of the scene covers columns 480..510 and reaches far
    # above the cone that is painted in front of it here.
    world.draw_cone(image, 3.0, -1.28)
    world.draw_cone(image, 4.0, 1.0, band=True)

    detections = detect_cones(world.through_jpeg(image))

    assert len(detections) == 2
    assert overlap(detections[0], world.cone_box(3.0, -1.28)) > 0.8
    assert overlap(detections[1], world.cone_box(4.0, 1.0)) > 0.75


def test_distractors_are_still_rejected_in_front_of_a_light_wall():
    image = world.draw_scene(wall_bgr=world.PLASTER_BGR)
    cv2.rectangle(image, (40, 240), (140, 300), world.CONE_BGR, cv2.FILLED)
    cv2.rectangle(image, (200, 150), (225, 300), world.CONE_BGR, cv2.FILLED)
    cv2.circle(image, (330, 280), 35, world.CONE_BGR, cv2.FILLED)
    funnel = np.array([[420, 220], [480, 220], [450, 320]], dtype=np.int32)
    cv2.fillConvexPoly(image, funnel, world.CONE_BGR)
    # An orange crate with a white label across it: orange above and below
    # the white, so the label joins the crate. It is still not cone-shaped.
    cv2.rectangle(image, (540, 230), (620, 300), world.CONE_BGR, cv2.FILLED)
    cv2.rectangle(image, (540, 255), (620, 270), world.BAND_BGR, cv2.FILLED)

    assert detect_cones(world.through_jpeg(image)) == []


def column_image(rows) -> np.ndarray:
    """Build a 9 pixel wide BGR image from a list of (row count, colour)."""
    stripes = [np.full((count, 9, 3), colour, dtype=np.uint8) for count, colour in rows]
    return np.vstack(stripes)


def test_white_needs_orange_above_and_below_within_reach():
    grey = (60, 60, 60)
    image = column_image([
        (10, world.BAND_BGR),   # rows 0..9: white with orange only below
        (10, world.CONE_BGR),   # rows 10..19
        (20, world.BAND_BGR),   # rows 20..39: white between orange and orange
        (10, world.CONE_BGR),   # rows 40..49
        (10, world.BAND_BGR),   # rows 50..59: white with orange only above
        (10, grey),             # rows 60..69: too dark to be white
    ])

    joined = cone_mask(image, DetectorThresholds(band_reach=30))[:, 4] > 0
    assert not joined[:8].any()
    assert joined[10:50].all()
    assert not joined[52:].any()

    # The same picture with a reach shorter than the white gap: its middle
    # has no orange near enough on either side, and stays empty.
    too_short = cone_mask(image, DetectorThresholds(band_reach=4))[:, 4] > 0
    assert not too_short[27:33].any()
