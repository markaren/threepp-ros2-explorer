"""Let Nav2 do the driving: the explorer's path follower, with an action client inside.

With `use_nav2` the explorer still decides WHERE the robot goes (which frontier, home, a
clicked goal), but not HOW. It hands the target to Nav2, and Nav2 plans the path, follows
it, avoids what stands in the way, and tries to free the robot when it is stuck:

    explorer  --  navigate_to_pose (action)  -->  Nav2  --  cmd_vel_nav  -->  safety_monitor

To the explorer this class looks like pure_pursuit.PurePursuit: set_path(), clear(),
has_path(), compute(). That is the whole trick of the swap. The explorer was written
against four methods, so whatever offers those four methods can drive the robot.

Everything here returns at once. Sending a goal, cancelling one and waiting for the
outcome are all asynchronous: the answer arrives later, as a callback on one of the
executor's threads. That matters, because the explorer calls these methods while it
holds its lock.
"""

from __future__ import annotations

import functools
import math
import threading

import numpy as np
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from explorer_nav.pure_pursuit import Command, PurePursuitConfig

# A target this close to the one Nav2 is already driving to is not sent again. A frontier
# recedes while the robot looks into it, and Nav2 plans afresh once a second by itself.
SAME_GOAL_DISTANCE = 0.15       # m


class Nav2Driver:
    """Sends the end of each path to Nav2 as a goal, and reports when Nav2 got there."""

    def __init__(self, node: Node, config: PurePursuitConfig, map_frame: str) -> None:
        self.config = config            # the explorer reads max_linear_speed for its deadlines
        self._node = node
        self._map_frame = map_frame
        self._client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

        # Guards everything below. The explorer's threads call the methods, the action
        # client's callbacks run on other threads of the same executor.
        self._lock = threading.Lock()
        self._goal: tuple[float, float] | None = None   # where Nav2 was last asked to go
        # Counts the goals. A callback that belongs to an older goal recognises itself
        # by its number and keeps out of the way.
        self._serial = 0
        self._handle = None             # the running goal, once Nav2 has accepted it
        self._reached = False
        self._failed = False

    # ---- the four methods of PurePursuit ----------------------------------------------

    def set_path(self, points: np.ndarray, final_yaw: float | None = None) -> None:
        """Drive to the END of this path. How to get there is Nav2's business.

        final_yaw: the heading to end with. Nav2 always wants one, so without it the
        robot ends the way the path ends: for a frontier that is looking into the unknown.
        """
        points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        if len(points) == 0:
            self.clear()
            return
        x, y = float(points[-1, 0]), float(points[-1, 1])
        if final_yaw is None:
            final_yaw = 0.0
            if len(points) >= 2:
                final_yaw = math.atan2(points[-1, 1] - points[-2, 1], points[-1, 0] - points[-2, 0])

        with self._lock:
            if self._goal is not None and not self._failed and math.hypot(
                    x - self._goal[0], y - self._goal[1]) < SAME_GOAL_DISTANCE:
                return      # Nav2 is on its way there already, or has arrived
            if not self._client.server_is_ready():
                # Nav2 is still starting up. The explorer plans again in a second.
                self._node.get_logger().info('Waiting for Nav2 (navigate_to_pose)',
                                             throttle_duration_sec=5.0)
                return
            self._serial += 1
            serial = self._serial
            self._goal = (x, y)
            self._handle = None
            self._reached = False
            self._failed = False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(final_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(final_yaw / 2.0)
        # A new goal while one is running does not queue up behind it: Nav2 drops the old
        # one and heads for the new one without stopping (preemption).
        request = self._client.send_goal_async(goal)
        request.add_done_callback(functools.partial(self._on_goal_response, serial))

    def clear(self) -> None:
        """Stop driving: cancel whatever Nav2 is doing for us."""
        with self._lock:
            self._serial += 1
            handle = self._handle
            self._handle = None
            self._goal = None
            self._reached = False
            self._failed = False
        if handle is not None:
            handle.cancel_goal_async()

    def has_path(self) -> bool:
        with self._lock:
            return self._goal is not None

    def compute(self, x: float, y: float, yaw: float) -> Command:
        """No command, Nav2's controller sends those. Only: has the goal been reached?"""
        with self._lock:
            return Command(0.0, 0.0, self._reached)

    # ---- what PurePursuit does not have -----------------------------------------------

    def take_failure(self) -> bool:
        """True, once, after Nav2 gave up on the goal (it has tried its recoveries first)."""
        with self._lock:
            failed = self._failed
            if failed:
                self._failed = False
                self._goal = None
            return failed

    # ---- callbacks of the action client ------------------------------------------------

    def _on_goal_response(self, serial: int, future) -> None:
        """Nav2 answered the goal request: accepted or rejected."""
        handle = future.result()
        with self._lock:
            current = serial == self._serial
            cleared = self._goal is None
            if current and not handle.accepted:
                # Not a failure. Nav2's action server answers as soon as bt_navigator is
                # configured, but it turns every goal down until the lifecycle manager
                # has activated it, a second or two later. Forget the goal: the explorer
                # plans again in a second and asks again.
                self._goal = None
            elif current:
                self._handle = handle
        if not handle.accepted:
            self._node.get_logger().info('Nav2 does not accept goals yet (still starting)',
                                         throttle_duration_sec=5.0)
            return
        if not current:
            # The explorer changed its mind before Nav2 answered. If it asked for another
            # goal, that one replaces this one inside Nav2. If it asked to stop, this goal
            # must not be left running.
            if cleared:
                handle.cancel_goal_async()
            return
        handle.get_result_async().add_done_callback(functools.partial(self._on_result, serial))

    def _on_result(self, serial: int, future) -> None:
        """A goal ended: reached, aborted by Nav2, or cancelled."""
        status = future.result().status
        with self._lock:
            if serial != self._serial:
                return      # an older goal, replaced or cancelled by us. Not news
            self._handle = None
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._reached = True
            else:
                self._failed = True
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info('Nav2 did not reach the goal')
