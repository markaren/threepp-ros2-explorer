"""The render loop: where the ROS half and the drawing half of the dashboard meet.

This file imports neither rclpy nor threepp. It only moves things between the two
halves, once per frame, on the render thread:

    1. empty the mailboxes the ROS thread has filled, and hand the news to the scene
    2. turn key presses into requests for the ROS thread
    3. draw
    4. run the jobs other threads have queued for the render thread (screenshots)

A frame never waits for ROS, and ROS never waits for a frame (except the screenshot
service, which asks for exactly that). If the mapper dies, the window keeps turning
and zooming, and simply shows the last map it got.
"""

import queue
import time
from collections import deque
from typing import Deque, List, Tuple

from explorer_dashboard.dashboard_node import DashboardNode
from explorer_dashboard.hud import HudData
from explorer_dashboard.window import DashboardWindow
from explorer_dashboard.world_model import RobotState, Status

NOTICE_SECONDS = 8.0   # how long a message for the operator stays on the HUD


class DashboardApp:
    """Runs the render loop until the window is closed or ROS shuts down."""

    def __init__(self, node: DashboardNode, window: DashboardWindow) -> None:
        self._node = node
        self._window = window
        # What the HUD shows until the first messages arrive.
        self._status = Status(RobotState.UNKNOWN, 0, 0.0, 0.0, 0.0)
        self._safety_stop = False
        self._cone_count = 0
        self._have_map = False
        self._framed: tuple[float, float, float, float] | None = None   # x0, y0, x1, y1 [m]
        self._have_pose = False
        self._notices: Deque[Tuple[float, str]] = deque(maxlen=3)   # (expiry, text)

    def run(self) -> None:
        # The node's context stops being ok on Ctrl+C or "rclpy.shutdown()", and
        # next_frame() returns False when the operator closes the window.
        while self._node.context.ok() and self._window.next_frame(self._frame):
            pass

    def _frame(self) -> None:
        self._pick_up_mail()
        self._handle_keys()
        self._update_hud()
        self._window.draw()
        # After draw(): a screenshot job must find the finished frame.
        self._node.render_jobs.run_pending()

    def _pick_up_mail(self) -> None:
        """Take what is new out of each mailbox. take() returns None for "no news"."""
        inbox = self._node.inbox
        world = self._window.world

        grid = inbox.grid.take()
        if grid is not None:
            world.set_grid(grid)
            self._frame_map(grid)
            self._have_map = True

        pose = inbox.pose.take()
        if pose is not None:
            world.set_robot(pose)
            self._have_pose = True

        scan = inbox.scan.take()
        if scan is not None:
            world.set_scan(scan)

        plan = inbox.plan.take()
        if plan is not None:
            world.set_plan(plan)

        frontiers = inbox.frontiers.take()
        if frontiers is not None:
            world.set_frontiers(frontiers)

        cones = inbox.cones.take()
        if cones is not None:
            world.set_cones(cones)
            self._cone_count = len(cones)

        image = inbox.camera.take()
        if image is not None:
            self._window.hud.set_camera_image(image)

        status = inbox.status.take()
        if status is not None:
            self._status = status

        safety_stop = inbox.safety_stop.take()
        if safety_stop is not None:
            self._safety_stop = safety_stop

        # Notices are events, so they come through a queue: every one is shown.
        while True:
            try:
                self._show_notice(self._node.notices.get_nowait())
            except queue.Empty:
                break

    def _show_notice(self, text: str) -> None:
        self._notices.append((time.monotonic() + NOTICE_SECONDS, text))

    def _frame_map(self, grid) -> None:
        """Place the camera so that the whole map is in view.

        Our own mapper publishes a grid of a fixed size, so its first map already says
        how big the world is. A SLAM system (mapper:=slam_toolbox) publishes a grid that
        grows with what the robot has seen. So the map is framed again whenever it
        reaches outside what was framed last, but only while the camera is still where
        the last framing left it: once the operator has moved it, it is theirs.
        """
        x0, y0 = grid.origin_x, grid.origin_y
        x1, y1 = x0 + grid.width_m, y0 + grid.height_m
        if self._framed is not None:
            fx0, fy0, fx1, fy1 = self._framed
            inside = x0 >= fx0 - 0.5 and y0 >= fy0 - 0.5 and x1 <= fx1 + 0.5 and y1 <= fy1 + 0.5
            if inside or not self._window.camera_untouched():
                return
            x0, y0, x1, y1 = min(x0, fx0), min(y0, fy0), max(x1, fx1), max(y1, fy1)
        self._framed = (x0, y0, x1, y1)
        # The factor leaves room for the near edge, which perspective widens.
        # The HUD's column covers the left part of the window, so the map is
        # framed in what is left of it: seen from far enough away to fit there,
        # and looked at off-centre so that it slides to the right.
        column = self._window.hud.column_fraction()
        view_width = 1.08 * (x1 - x0) / (1.0 - column)
        self._window.look_at_area(
            (x0 + x1) / 2.0 - 0.5 * column * view_width, (y0 + y1) / 2.0, view_width)

    def _handle_keys(self) -> None:
        """Each command key asks the ROS thread to act. Nothing here blocks."""
        node = self._node
        for key in self._window.pressed_keys():
            if key == 'E':
                node.run_on_ros_thread(node.start_exploring)
            elif key == 'C':
                node.run_on_ros_thread(node.cancel_exploring)
            elif key == 'R':
                node.run_on_ros_thread(node.reset)
                self._window.world.clear_trail()
            elif key == 'G':
                point = self._window.floor_point_under_cursor()
                if point is None:
                    self._show_notice('goal: the cursor is not over the floor')
                else:
                    node.run_on_ros_thread(node.send_goal_pose, point[0], point[1])
            elif key == 'F':
                # The camera belongs to the render thread: no detour through ROS.
                self._window.follow = not self._window.follow

    def _update_hud(self) -> None:
        now = time.monotonic()
        while self._notices and self._notices[0][0] < now:
            self._notices.popleft()
        waiting_for: List[str] = []
        if not self._have_map:
            waiting_for.append('map')
        if not self._have_pose:
            waiting_for.append('robot pose (tf)')
        if self._status.state is RobotState.UNKNOWN:
            waiting_for.append('explorer status')
        self._window.hud.update(HudData(
            status=self._status,
            cone_count=self._cone_count,
            safety_stop=self._safety_stop,
            follow=self._window.follow,
            waiting_for=waiting_for,
            notices=[text for _, text in self._notices]))
