"""The camera + lidar fusion on a synthetic scan: a cone in front of a wall."""

import math

import numpy as np
import pytest

import synthetic_world as world
from explorer_vision.cone_detector import detect_cones
from explorer_vision.cone_locator import (
    Scan, distance_from_pixel_height, locate_cone, pixel_column_to_bearing, scan_to_points)

WALL_X = 4.5


def locate(cone_x: float, cone_y: float, scan: Scan, **overrides):
    """Run the locator on the ideal bounding box of a cone at (cone_x, cone_y)."""
    left, top, right, bottom = world.cone_box(cone_x, cone_y)
    return locate_cone(left, right, bottom - top, world.intrinsics(), scan, **overrides)


def test_bearing_is_positive_to_the_left():
    k = world.intrinsics()
    assert pixel_column_to_bearing(k.cx, k) == 0.0
    assert pixel_column_to_bearing(k.cx - 100, k) > 0.0
    assert pixel_column_to_bearing(k.cx + 100, k) < 0.0
    # The image edge of a 640 pixel wide image with this focal length.
    assert math.degrees(pixel_column_to_bearing(0, k)) == pytest.approx(39.7, abs=0.1)


def test_pixel_height_gives_a_rough_distance():
    k = world.intrinsics()
    _, top, _, bottom = world.cone_box(3.0, 0.0)
    estimate = distance_from_pixel_height(bottom - top, k, world.CONE.height)
    # Distance from the camera, which sits 0.18 m ahead of base_link.
    assert estimate == pytest.approx(3.0 - world.CAMERA_X, rel=0.1)


def test_scan_to_points_follows_the_contract():
    ranges = np.full(360, np.inf)
    ranges[180] = 2.0      # straight ahead
    ranges[270] = 1.0      # 90 degrees to the left
    ranges[0] = 0.05       # closer than range_min: not a real return
    scan = Scan(angle_min=-math.pi, angle_increment=2.0 * math.pi / 360,
                ranges=ranges, range_min=0.12, range_max=8.0)

    points = scan_to_points(scan)

    assert points.shape == (2, 2)
    assert points[0] == pytest.approx([2.0, 0.0], abs=1e-9)
    assert points[1] == pytest.approx([0.0, 1.0], abs=1e-9)


@pytest.mark.parametrize('cone_x, cone_y', [
    (3.0, 0.4),     # the standard case
    (3.0, 0.0),     # straight ahead
    (2.0, -1.0),    # to the right
    (1.2, 0.5),     # close and off-axis: the parallax between camera and lidar is 3.5 degrees
    (4.2, 1.5),     # far, near the wall
])
def test_cone_in_front_of_a_wall_is_found_within_10_cm(cone_x, cone_y):
    scan = world.cast_scan([(cone_x, cone_y)], WALL_X)

    position = locate(cone_x, cone_y, scan)

    assert position is not None
    assert math.hypot(position.x - cone_x, position.y - cone_y) < 0.1


def test_gate_rejects_the_wall_when_the_lidar_missed_the_cone():
    # The camera sees a cone at 3 m, but the scan contains only the wall
    # behind it at 4.5 m. Without the gate the cone would be put on the wall.
    wall_only = world.cast_scan([], WALL_X)

    assert locate(3.0, 0.4, wall_only) is None
    # Proof that it is the gate that says no: without it, the wall is returned.
    on_the_wall = locate(3.0, 0.4, wall_only, distance_tolerance=10.0)
    assert on_the_wall is not None
    assert on_the_wall.x > WALL_X


def test_picks_the_right_cone_when_two_are_in_view():
    scan = world.cast_scan([(3.0, 0.4), (2.0, -0.8)], WALL_X)

    first = locate(3.0, 0.4, scan)
    second = locate(2.0, -0.8, scan)

    assert math.hypot(first.x - 3.0, first.y - 0.4) < 0.1
    assert math.hypot(second.x - 2.0, second.y + 0.8) < 0.1


def test_no_returns_inside_the_box_gives_none():
    empty = world.cast_scan([], None)
    assert locate(3.0, 0.4, empty) is None


def test_cones_beyond_max_range_are_not_localised():
    scan = world.cast_scan([(6.0, 0.0)], 7.5)

    assert locate(6.0, 0.0, scan) is None
    assert locate(6.0, 0.0, scan, max_range=8.0) is not None


def test_detector_and_locator_together():
    """The whole chain without ROS: painted image -> box -> fused position."""
    cones = [(3.0, 0.4), (2.0, -0.8)]
    image = world.draw_scene()
    for cone_x, cone_y in cones:
        world.draw_cone(image, cone_x, cone_y)
    scan = world.cast_scan(cones, WALL_X)

    detections = detect_cones(world.through_jpeg(image))
    positions = [locate_cone(d.left, d.right, d.height, world.intrinsics(), scan)
                 for d in detections]

    # Detections come largest first: the cone at 2 m, then the one at 3 m.
    assert len(positions) == 2
    assert math.hypot(positions[0].x - 2.0, positions[0].y + 0.8) < 0.1
    assert math.hypot(positions[1].x - 3.0, positions[1].y - 0.4) < 0.1
