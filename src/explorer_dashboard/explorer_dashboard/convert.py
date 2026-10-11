"""Conversions at the edges of the dashboard. Pure numpy: no ROS, no threepp.

Two worlds meet in this program, and they disagree about which way is up:

    ROS (REP 103):       x forward, y left, z up
    threepp / three.js:  x right,   y up,   z towards the viewer

Both are right-handed, so one is a rotation of the other. The contract of this demo
(docs/interfaces.md) fixes the rule:

    three = (ros.x, ros.z, -ros.y)

and a yaw of theta about ROS z is a rotation of theta about threepp y. Everything
inside the dashboard stays in ROS coordinates until the very last moment, and these
functions are the only place where the axes are swapped. Converting "at the edge"
keeps the mistake of mixing the two conventions out of the rest of the code.
"""

import math
from typing import Tuple

import numpy as np

from explorer_dashboard.world_model import Grid, Pose2D

# A cell counts as a wall when the mapper is at least this sure (percent).
OCCUPIED_THRESHOLD = 65

# Colours of the floor texture (red, green, blue, 0..255).
UNKNOWN_RGB = (13, 18, 28)
FREE_RGB = (44, 62, 84)
OCCUPIED_RGB = (150, 226, 255)


def ros_xy_to_three(points_xy: np.ndarray, height: float) -> np.ndarray:
    """Turn (N, 2) floor points in ROS x, y into (N, 3) threepp points.

    `height` is the ROS z of all the points (metres above the floor). It becomes the
    threepp y.
    """
    points_xy = np.asarray(points_xy, dtype=np.float32).reshape(-1, 2)
    three = np.empty((len(points_xy), 3), dtype=np.float32)
    three[:, 0] = points_xy[:, 0]
    three[:, 1] = height
    three[:, 2] = -points_xy[:, 1]
    return three


def three_to_ros_xy(x: float, z: float) -> Tuple[float, float]:
    """The inverse on the floor: a threepp (x, z) is ROS (x, y) = (x, -z)."""
    return x, -z


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """The rotation about z that a quaternion contains.

    ROS sends orientations as quaternions. A robot on a flat floor only turns about
    z, so one angle is all we need: this is the standard yaw term of the conversion
    from a quaternion to roll, pitch and yaw.
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def scan_to_map_xy(ranges: np.ndarray, angle_min: float, angle_increment: float,
                   range_min: float, range_max: float, laser: Pose2D) -> np.ndarray:
    """Turn the ranges of a LaserScan into (N, 2) points in the map frame.

    Beam i points at angle_min + i * angle_increment in the laser's own frame. A beam
    that hit nothing reports +inf (or a value outside [range_min, range_max]): those
    are dropped, there is no point to draw. `laser` is the pose of the lidar in the
    map frame AT THE TIME OF THE SCAN (see DashboardNode._release_scan for why the
    time matters).
    """
    ranges = np.asarray(ranges, dtype=np.float32)
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    angles = laser.yaw + angle_min + angle_increment * np.nonzero(valid)[0]
    hits = ranges[valid]
    return np.column_stack((laser.x + hits * np.cos(angles),
                            laser.y + hits * np.sin(angles))).astype(np.float32)


def grid_to_rgb(cells: np.ndarray) -> np.ndarray:
    """Colour an occupancy grid: (rows, cols) int8 -> (rows, cols, 3) uint8.

    Unknown is dark, free is lighter, occupied is bright, and a cell the mapper is
    unsure about lands in between. A lookup table with one entry per possible cell
    value (-1, 0, 1, ..., 100) does this for the whole grid in a single numpy
    indexing operation, with no Python loop over 60 000 cells.
    """
    table = np.empty((102, 3), dtype=np.float32)
    table[0] = UNKNOWN_RGB
    probability = np.linspace(0.0, 1.0, 101, dtype=np.float32)[:, None]
    table[1:] = (1.0 - probability) * np.float32(FREE_RGB) + probability * np.float32(
        OCCUPIED_RGB)
    index = np.clip(cells.astype(np.int16) + 1, 0, 101)
    return table.astype(np.uint8)[index]


def cell_centres(grid: Grid, rows: np.ndarray, cols: np.ndarray) -> np.ndarray:
    """The (N, 2) ROS x, y centres of the grid cells (rows[i], cols[i])."""
    return np.column_stack((grid.origin_x + (cols + 0.5) * grid.resolution,
                            grid.origin_y + (rows + 0.5) * grid.resolution)).astype(
                                np.float32)
