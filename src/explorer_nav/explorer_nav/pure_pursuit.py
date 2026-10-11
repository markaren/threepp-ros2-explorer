"""Pure pursuit: follow a path by chasing a point on it. Plain numpy, no ROS.

Picture a carrot on a stick held a fixed distance (the lookahead) ahead of the robot ON THE
PATH. The robot always steers along the circle that passes through its own position and the
carrot, tangent to its current heading. As the robot moves, the carrot moves along the path,
and the robot is pulled along smoothly, rounding off the corners.

For a differential drive robot the command is (v, w): forward speed and turn rate. If the
carrot is at distance d and at angle alpha off the robot's nose, the circle through it has
curvature 2 * sin(alpha) / d, and driving that circle at speed v means w = curvature * v.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class PurePursuitConfig:
    lookahead: float = 0.5              # m: how far ahead on the path the carrot is
    max_linear_speed: float = 0.6       # m/s
    max_angular_speed: float = 1.5      # rad/s
    # Pure pursuit breaks down when the carrot is behind or far to the side: the arc it
    # asks for is wide and sweeps the robot off the path, into whatever stands next to it.
    # Above this heading error the robot stops and rotates on the spot instead...
    turn_in_place_angle: float = 0.9    # rad
    # ...and drives off again once it points at the carrot to within this angle. Two
    # different thresholds (hysteresis) prevent it from flickering between the two modes.
    resume_angle: float = 0.25          # rad
    heading_gain: float = 2.5           # 1/s: turn rate per radian of heading error
    goal_tolerance: float = 0.12        # m: closer than this to the end of the path = arrived
    slowdown_distance: float = 0.8      # m: start braking this far before the end
    min_linear_speed: float = 0.08      # m/s: braking never goes below this, or we never arrive
    yaw_tolerance: float = 0.08         # rad: accuracy of the final heading, when one is asked for


@dataclass
class Command:
    linear: float       # m/s
    angular: float      # rad/s
    reached: bool       # the end of the path has been reached, the command is zero


def wrap_angle(angle: float) -> float:
    """Bring an angle into [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class PurePursuit:
    """Computes velocity commands that take the robot along a polyline."""

    def __init__(self, config: PurePursuitConfig | None = None) -> None:
        self.config = config if config is not None else PurePursuitConfig()
        self._points = np.zeros((0, 2))
        self._segment_lengths = np.zeros(0)
        self._final_yaw: float | None = None
        self._segment = 0           # index of the path segment the robot is on
        self._turning = False       # currently rotating on the spot
        self._position_reached = False

    def set_path(self, points: np.ndarray, final_yaw: float | None = None) -> None:
        """Follow a new path: an (N, 2) array of x, y points, the first one near the robot.

        final_yaw: if given, the robot turns to this heading after reaching the last point.
        """
        points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        self._points = points
        self._segment_lengths = np.hypot(np.diff(points[:, 0]), np.diff(points[:, 1]))
        self._final_yaw = final_yaw
        self._segment = 0
        self._position_reached = False
        # _turning is kept on purpose. Paths are replaced about once a second while
        # driving, and forgetting the mode each time would defeat the hysteresis.

    def clear(self) -> None:
        """Forget the path. compute() then returns a zero command."""
        self.set_path(np.zeros((0, 2)))
        self._turning = False

    def has_path(self) -> bool:
        return len(self._points) >= 1

    def remaining_distance(self, x: float, y: float) -> float:
        """Path length from the robot's position on the path to the end."""
        if len(self._points) < 2:
            if len(self._points) == 1:
                return float(np.hypot(self._points[0, 0] - x, self._points[0, 1] - y))
            return 0.0
        fraction = self._fraction_along_segment(self._segment, x, y)
        on_this_segment = (1.0 - fraction) * self._segment_lengths[self._segment]
        return float(on_this_segment + self._segment_lengths[self._segment + 1:].sum())

    def compute(self, x: float, y: float, yaw: float) -> Command:
        """The command for a robot at pose (x, y, yaw). Call this at a steady rate."""
        config = self.config
        if not self.has_path():
            return Command(0.0, 0.0, False)

        self._advance_segment(x, y)
        goal = self._points[-1]
        distance_to_goal = float(np.hypot(goal[0] - x, goal[1] - y))
        on_last_segment = self._segment >= len(self._points) - 2
        # "Close to the last point" is only an arrival on the last segment: a path that
        # winds around a shelf passes close to its own end long before it gets there.
        if on_last_segment and distance_to_goal < config.goal_tolerance:
            self._position_reached = True

        if self._position_reached:
            return self._turn_to_final_yaw(yaw)

        carrot = self._carrot(x, y)
        carrot_distance = float(np.hypot(carrot[0] - x, carrot[1] - y))
        heading_error = wrap_angle(math.atan2(carrot[1] - y, carrot[0] - x) - yaw)

        if self._turning:
            if abs(heading_error) < config.resume_angle:
                self._turning = False
        elif abs(heading_error) > config.turn_in_place_angle:
            self._turning = True

        if self._turning:
            angular = float(np.clip(config.heading_gain * heading_error,
                                    -config.max_angular_speed, config.max_angular_speed))
            return Command(0.0, angular, False)

        # Brake linearly over the last stretch, but keep creeping until we are there.
        remaining = self.remaining_distance(x, y)
        linear = config.max_linear_speed * min(1.0, remaining / config.slowdown_distance)
        linear = max(linear, config.min_linear_speed)

        curvature = 2.0 * math.sin(heading_error) / max(carrot_distance, 1e-3)
        angular = curvature * linear
        if abs(angular) > config.max_angular_speed:
            # The arc is too tight for this speed. Keep the arc, lower the speed.
            angular = math.copysign(config.max_angular_speed, angular)
            linear = angular / curvature
        return Command(linear, angular, False)

    def _turn_to_final_yaw(self, yaw: float) -> Command:
        config = self.config
        if self._final_yaw is None:
            return Command(0.0, 0.0, True)
        yaw_error = wrap_angle(self._final_yaw - yaw)
        if abs(yaw_error) < config.yaw_tolerance:
            return Command(0.0, 0.0, True)
        angular = float(np.clip(config.heading_gain * yaw_error,
                                -config.max_angular_speed, config.max_angular_speed))
        return Command(0.0, angular, False)

    def _fraction_along_segment(self, segment: int, x: float, y: float) -> float:
        """Where the robot is along a segment: 0 at its start, 1 at its end.

        This is the robot's position projected onto the segment, clamped to its ends.
        """
        start = self._points[segment]
        direction = self._points[segment + 1] - start
        squared_length = float(direction @ direction)
        if squared_length < 1e-12:
            return 1.0
        fraction = ((x - start[0]) * direction[0] + (y - start[1]) * direction[1]) / squared_length
        return min(1.0, max(0.0, float(fraction)))

    def _distance_to_segment(self, segment: int, x: float, y: float) -> float:
        start = self._points[segment]
        end = self._points[segment + 1]
        closest = start + self._fraction_along_segment(segment, x, y) * (end - start)
        return float(np.hypot(closest[0] - x, closest[1] - y))

    def _advance_segment(self, x: float, y: float) -> None:
        """Decide which segment of the path the robot is on.

        It moves on to a later segment as soon as it is closer to that one than to the
        current one. Only segments that begin within one lookahead distance ahead are
        considered, and progress never goes backwards. Searching the whole path for the
        closest point would be simpler, but it jumps when the path comes back past the
        robot (down one aisle, up the next).
        """
        last_segment = len(self._points) - 2
        if last_segment < 1:
            return
        best_segment = self._segment
        best_distance = self._distance_to_segment(self._segment, x, y)
        fraction = self._fraction_along_segment(self._segment, x, y)
        path_ahead = (1.0 - fraction) * self._segment_lengths[self._segment]
        segment = self._segment + 1
        while segment <= last_segment and path_ahead <= self.config.lookahead:
            distance = self._distance_to_segment(segment, x, y)
            if distance < best_distance:
                best_distance = distance
                best_segment = segment
            path_ahead += self._segment_lengths[segment]
            segment += 1
        self._segment = best_segment

    def _carrot(self, x: float, y: float) -> np.ndarray:
        """The point one lookahead distance ahead of the robot, measured along the path."""
        if len(self._points) == 1:
            return self._points[0]
        fraction = self._fraction_along_segment(self._segment, x, y)
        to_go = self.config.lookahead
        segment = self._segment
        while True:
            start = self._points[segment]
            end = self._points[segment + 1]
            left_on_segment = (1.0 - fraction) * self._segment_lengths[segment]
            if to_go <= left_on_segment:
                travelled = fraction + to_go / max(self._segment_lengths[segment], 1e-9)
                return start + travelled * (end - start)
            if segment == len(self._points) - 2:
                return end          # the path ends before the lookahead distance does
            to_go -= left_on_segment
            segment += 1
            fraction = 0.0
