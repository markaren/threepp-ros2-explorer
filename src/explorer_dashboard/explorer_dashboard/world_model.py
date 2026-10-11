"""What the robot believes, as plain data.

These small types are the only thing the ROS half and the drawing half of the
dashboard share. The ROS half turns messages into them, the drawing half turns them
into pictures. Neither rclpy nor threepp is imported here, and everything is in ROS
coordinates (frame "map": x forward, y left, z up, metres, radians).

The classes are frozen (immutable): once a value sits in a Mailbox, two threads can
see it, and data that cannot change needs no lock.
"""

import enum
from dataclasses import dataclass
from typing import Optional

import numpy as np

from explorer_dashboard.mailbox import Mailbox


@dataclass(frozen=True)
class Pose2D:
    """Where the robot is on the floor, and which way it faces."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Grid:
    """An occupancy grid: the floor cut into square cells.

    cells[row, col] is -1 for "never seen", otherwise the probability in percent
    (0..100) that the cell is occupied. The centre of a cell is at
    x = origin_x + (col + 0.5) * resolution, y = origin_y + (row + 0.5) * resolution.
    """

    resolution: float
    origin_x: float
    origin_y: float
    cells: np.ndarray

    @property
    def width_m(self) -> float:
        return self.cells.shape[1] * self.resolution

    @property
    def height_m(self) -> float:
        return self.cells.shape[0] * self.resolution


@dataclass(frozen=True)
class Scan:
    """One sweep of the lidar, already transformed into the map frame."""

    points_xy: np.ndarray  # (N, 2): where each beam hit something
    height: float          # how high above the floor the lidar sits


@dataclass(frozen=True)
class Frontiers:
    """The borders between known and unknown floor that the explorer may visit."""

    points_xy: np.ndarray                 # (N, 2) centroids of the frontier clusters
    target_xy: Optional[np.ndarray]       # (2,) the one it chose, or None


class RobotState(enum.Enum):
    """What the explorer is doing. Mirrors the constants of ExplorationStatus.msg.

    The drawing half must not import ROS message types, so the ROS half translates
    the message constants into this enum (see STATE_FROM_MSG in dashboard_node.py).
    """

    UNKNOWN = 'NO CONTACT'
    IDLE = 'IDLE'
    EXPLORING = 'EXPLORING'
    RETURNING = 'RETURNING'
    NAVIGATING = 'NAVIGATING'
    DONE = 'DONE'


@dataclass(frozen=True)
class Status:
    """The numbers of the head-up display."""

    state: RobotState
    frontier_count: int
    explored_area: float  # m^2
    distance: float       # m
    elapsed: float        # s


class Inbox:
    """One Mailbox per thing the dashboard shows.

    The ROS thread posts, the render thread takes. Nothing else crosses between the
    two threads in this direction.
    """

    def __init__(self) -> None:
        self.grid: Mailbox[Grid] = Mailbox()
        self.pose: Mailbox[Pose2D] = Mailbox()
        self.scan: Mailbox[Scan] = Mailbox()
        self.plan: Mailbox[np.ndarray] = Mailbox()       # (N, 2) waypoints
        self.frontiers: Mailbox[Frontiers] = Mailbox()
        self.cones: Mailbox[np.ndarray] = Mailbox()      # (N, 2) cone positions
        self.status: Mailbox[Status] = Mailbox()
        self.safety_stop: Mailbox[bool] = Mailbox()
        # The robot's camera picture, ready to become a texture: (H, W, 3) RGB bytes,
        # bottom row first.
        self.camera: Mailbox[np.ndarray] = Mailbox()
