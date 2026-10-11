"""The detector on an image from a real renderer (see render_test_image.py).

The scene: grey floor, blue-grey wall at 4.5 m, two cones at (3.0, 0.4) and
(2.0, -0.8) in `base_link`, lit by a sun and ambient light, seen by a camera
0.25 m above the floor with a vertical field of view of 50 degrees.
"""

import math
from pathlib import Path

import cv2

import synthetic_world as world
from explorer_vision.cone_detector import detect_cones
from explorer_vision.cone_locator import locate_cone
from test_cone_detector import overlap

IMAGE_PATH = Path(__file__).resolve().parent / 'data' / 'rendered_cones.png'
CONES = [(2.0, -0.8), (3.0, 0.4)]    # nearest first, as the detector reports them
WALL_X = 4.5


def test_detector_finds_both_rendered_cones():
    image = cv2.imread(str(IMAGE_PATH))
    assert image is not None, 'run render_test_image.py to create the image'

    detections = detect_cones(world.through_jpeg(image))

    assert len(detections) == 2
    for detection, (cone_x, cone_y) in zip(detections, CONES):
        # The boxes must agree with the pinhole model the locator relies on.
        assert overlap(detection, world.cone_box(cone_x, cone_y)) > 0.8
        assert not detection.clipped


def test_rendered_cones_are_placed_within_10_cm():
    image = cv2.imread(str(IMAGE_PATH))
    scan = world.cast_scan(CONES, WALL_X)

    detections = detect_cones(world.through_jpeg(image))

    assert len(detections) == 2
    for detection, (cone_x, cone_y) in zip(detections, CONES):
        position = locate_cone(detection.left, detection.right, detection.height,
                               world.intrinsics(), scan)
        assert position is not None
        assert math.hypot(position.x - cone_x, position.y - cone_y) < 0.1
