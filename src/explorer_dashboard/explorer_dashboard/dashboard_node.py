"""The ROS half of the dashboard: a node that listens, and the operator's buttons.

No threepp in this file. Everything the node learns goes into a Mailbox (see
mailbox.py), and everything the operator asks for arrives here as a method call that
runs on the ROS thread.

Threads:
    main thread   the render loop (app.py). Owns the window.
    ROS thread    executor.spin(), started in main() below. Runs EVERY callback and
                  EVERY method of DashboardNode, one at a time. That is why the node
                  itself needs no lock for its own fields: only one thread ever
                  touches them. The lock lives in the Mailbox, the single place where
                  the two threads meet.
"""

import concurrent.futures
import functools
import os
import queue
import threading
import time
from typing import Callable, Optional

from action_msgs.msg import GoalStatus
from explorer_interfaces.action import Explore
from explorer_interfaces.msg import ExplorationStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from explorer_dashboard import convert
from explorer_dashboard.mailbox import RenderJobs
from explorer_dashboard.world_model import Frontiers, Grid, Inbox, Pose2D, RobotState
from explorer_dashboard.world_model import Scan, Status

# The state is a number on the wire. The .msg file names each value, and the generated
# Python class carries those names as constants, so no bare 0, 1, 2 appears here.
STATE_FROM_MSG = {
    ExplorationStatus.IDLE: RobotState.IDLE,
    ExplorationStatus.EXPLORING: RobotState.EXPLORING,
    ExplorationStatus.RETURNING: RobotState.RETURNING,
    ExplorationStatus.NAVIGATING: RobotState.NAVIGATING,
    ExplorationStatus.DONE: RobotState.DONE,
}

POSE_RATE = 30.0           # Hz. How often the robot's pose is looked up
SCREENSHOT_TIMEOUT = 3.0   # s. How long the service waits for the render thread


def pose_from_transform(transform: TransformStamped) -> Pose2D:
    """Reduce a 3D transform to the pose on the floor that the dashboard draws."""
    position = transform.transform.translation
    turn = transform.transform.rotation
    return Pose2D(position.x, position.y,
                  convert.yaw_from_quaternion(turn.x, turn.y, turn.z, turn.w))


def marker_positions(message: MarkerArray, namespace: str) -> np.ndarray:
    """The (N, 2) positions of the markers of one namespace.

    A MarkerArray is a list of commands, not a list of objects: ADD (create or move),
    DELETE, DELETEALL. We replay them in order. The contract says every message starts
    with DELETEALL, so one message always describes the complete set.
    """
    positions = {}
    for marker in message.markers:
        if marker.action == Marker.DELETEALL:
            positions.clear()
        elif marker.ns != namespace:
            continue
        elif marker.action == Marker.ADD:
            positions[marker.id] = (marker.pose.position.x, marker.pose.position.y)
        elif marker.action == Marker.DELETE:
            positions.pop(marker.id, None)
    return np.float32(list(positions.values())).reshape(-1, 2)


class DashboardNode(Node):
    """Subscribes to what the robot believes and offers the operator's commands."""

    def __init__(self) -> None:
        super().__init__('dashboard')

        # ---- Parameters -----------------------------------------------------------
        # Set from the command line (--ros-args -p headless:=true) or a launch file.
        self.headless: bool = self._parameter(
            'headless', False,
            'Create the canvas without a visible window (for tests and screenshots).')
        self.width: int = self._parameter('width', 1280, 'Window width in pixels.')
        self.height: int = self._parameter('height', 720, 'Window height in pixels.')
        self._auto_explore: bool = self._parameter(
            'auto_explore', False,
            'Send the Explore goal without a key press, as soon as the action server '
            'is up. Lets a launch file start a hands-free demo.')
        self._return_home: bool = self._parameter(
            'return_home', True,
            'Ask the robot to drive back to its start pose when exploration is done.')
        self._screenshot_dir: str = self._parameter(
            'screenshot_dir', '.', 'Directory the screenshot service writes PNGs to.')

        # ---- The three ways out of the ROS thread (see mailbox.py) -----------------
        self.inbox = Inbox()                    # state: newest wins
        self.notices: 'queue.SimpleQueue[str]' = queue.SimpleQueue()   # events
        self.render_jobs = RenderJobs()         # work for the render thread
        self._save_frame: Optional[Callable[[str], None]] = None
        self._screenshot_count = 0

        # ---- Quality of service ---------------------------------------------------
        # A subscription only receives data if its QoS is COMPATIBLE with the
        # publisher's. The rule of thumb: a subscriber may ask for less than the
        # publisher offers, never for more. So every profile below is copied from the
        # QoS column of docs/interfaces.md.
        #
        # "Transient local" is ROS 2's latching: the publisher keeps its last message
        # and hands it to every subscriber that joins later. A dashboard started in
        # the middle of a run gets the current map at once, without waiting for the
        # next one. Asking for transient local from a publisher that does not offer it
        # (a "volatile" one) would be incompatible, and we would hear nothing.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Reliable, but only the newest message is kept: an old plan is worthless.
        newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(OccupancyGrid, 'map', self._on_map, latched)
        self.create_subscription(MarkerArray, 'cones', self._on_cones, latched)
        self.create_subscription(Bool, 'safety_stop', self._on_safety_stop, latched)
        self.create_subscription(Path, 'plan', self._on_plan, newest)
        self.create_subscription(MarkerArray, 'frontiers', self._on_frontiers, newest)
        self.create_subscription(ExplorationStatus, 'exploration_status',
                                 self._on_status, 10)
        # The sensor data profile is "best effort": a lost scan is not sent again,
        # because the next one is only 100 ms away and newer. A reliable subscriber
        # cannot hear a best effort publisher, so this one must be best effort too.
        self.create_subscription(LaserScan, 'scan', self._on_scan,
                                 qos_profile_sensor_data)
        self._goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', newest)

        # ---- tf2 ------------------------------------------------------------------
        # The robot's pose is not a topic of its own. Nodes publish single links of a
        # tree of frames (map -> odom -> base_link -> laser) on /tf. The listener
        # collects them in the buffer, and the buffer can then answer "where is frame
        # A in frame B, at time T?" for any pair, chaining the links for us.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._waiting_scan: Optional[LaserScan] = None
        self.create_timer(1.0 / POSE_RATE, self._on_pose_timer)

        # ---- Services and the action ------------------------------------------------
        # Creating a client never blocks and never fails, even when no server exists.
        # Whether a server is there is checked later, each time a button is pressed.
        self._reset_world = self.create_client(Trigger, 'reset_world')
        self._clear_map = self.create_client(Trigger, 'clear_map')
        self._explore_client = ActionClient(self, Explore, 'explore')
        self._explore_requested = False
        self._goal_handle = None
        # "~" is replaced by the node's own name: the service is dashboard/screenshot.
        self.create_service(Trigger, '~/screenshot', self._on_screenshot)
        if self._auto_explore:
            self._auto_timer = self.create_timer(1.0, self._on_auto_explore_timer)

    def _parameter(self, name: str, default, description: str):
        """Declare a parameter with a description and return its value.

        The description shows up in "ros2 param describe /dashboard <name>".
        """
        descriptor = ParameterDescriptor(description=description)
        return self.declare_parameter(name, default, descriptor).value

    # ---- Between the threads --------------------------------------------------------

    def run_on_ros_thread(self, work: Callable[..., None], *arguments) -> None:
        """Called by the RENDER thread: run a method of this node on the ROS thread.

        The mirror image of RenderJobs. A key press is noticed by the render loop, but
        sending a goal belongs to the ROS thread, so the render loop does not call
        start_exploring() itself: it asks the executor to. create_task() is safe to
        call from any thread, and it wakes the executor up.
        """
        self.executor.create_task(work, *arguments)

    def set_frame_saver(self, save_frame: Callable[[str], None]) -> None:
        """Give the node the function that writes the current frame to a PNG.

        The node never calls it. It only wraps it in a job for the render thread.
        """
        self._save_frame = save_frame

    def notify(self, text: str) -> None:
        """Tell the operator something: on the HUD and in the log."""
        self.get_logger().info(text)
        self.notices.put(text)

    # ---- Subscription callbacks: convert, post, return --------------------------------
    # A callback must be quick, because the executor runs one callback at a time and a
    # slow one delays all the others. And it must not draw (see mailbox.py). So each
    # one turns its message into plain data and leaves it in a Mailbox.

    def _on_map(self, message: OccupancyGrid) -> None:
        info = message.info
        # data is one long row-major array: cell (row, col) is data[row * width + col].
        cells = np.array(message.data, dtype=np.int8).reshape(info.height, info.width)
        self.inbox.grid.post(Grid(info.resolution, info.origin.position.x,
                                  info.origin.position.y, cells))

    def _on_plan(self, message: Path) -> None:
        waypoints = [(stamped.pose.position.x, stamped.pose.position.y)
                     for stamped in message.poses]
        self.inbox.plan.post(np.float32(waypoints).reshape(-1, 2))

    def _on_frontiers(self, message: MarkerArray) -> None:
        target = marker_positions(message, 'target')
        self.inbox.frontiers.post(Frontiers(marker_positions(message, 'frontiers'),
                                            target[0] if len(target) else None))

    def _on_cones(self, message: MarkerArray) -> None:
        self.inbox.cones.post(marker_positions(message, 'cones'))

    def _on_safety_stop(self, message: Bool) -> None:
        self.inbox.safety_stop.post(message.data)

    def _on_status(self, message: ExplorationStatus) -> None:
        self.inbox.status.post(Status(
            STATE_FROM_MSG.get(message.state, RobotState.UNKNOWN),
            message.frontier_count, message.explored_area, message.distance,
            message.elapsed))

    def _on_scan(self, message: LaserScan) -> None:
        # Not posted yet: the scan first needs the pose it was taken from.
        self._waiting_scan = message

    # ---- tf2 lookups ---------------------------------------------------------------------

    def _on_pose_timer(self) -> None:
        try:
            # Time() is time zero, which tf2 reads as "the newest you have".
            transform = self._tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return   # Nobody publishes the frames yet. Try again on the next tick.
        self.inbox.pose.post(pose_from_transform(transform))
        self._release_scan()

    def _release_scan(self) -> None:
        """Place the waiting scan in the map, using the pose AT THE SCAN'S STAMP.

        A scan describes the world as the lidar saw it at header.stamp. While the
        robot turns, the pose of "now" is already a few degrees further, and at 8 m
        range a few degrees smear a wall by half a metre. tf2 keeps a history, so we
        ask for the transform at the scan's own time.

        The transform for that instant may not have arrived yet (scan and tf travel on
        different topics). Then the lookup raises, the scan keeps waiting, and the
        next timer tick tries again.
        """
        scan = self._waiting_scan
        if scan is None:
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', scan.header.frame_id, Time.from_msg(scan.header.stamp))
        except TransformException:
            return
        self._waiting_scan = None
        points = convert.scan_to_map_xy(scan.ranges, scan.angle_min,
                                        scan.angle_increment, scan.range_min,
                                        scan.range_max, pose_from_transform(transform))
        self.inbox.scan.post(Scan(points, transform.transform.translation.z))

    # ---- The Explore action (key E, key C) ------------------------------------------------
    # An action is a conversation in three steps, and each answer arrives later, as a
    # callback on a future:
    #   1. send the goal         -> the server accepts or rejects it
    #   2. while it runs         -> feedback messages
    #   3. ask for the result    -> arrives when the job is finished or cancelled
    # Nothing here waits. A blocking wait inside a callback would stall the executor,
    # and the answer we wait for could then never be delivered: a deadlock.

    def start_exploring(self) -> None:
        if not self._explore_client.server_is_ready():
            self.notify('explore: no action server (is the explorer running?)')
            return
        if self._explore_requested:
            self.notify('explore: already running (C cancels)')
            return
        goal = Explore.Goal()
        goal.time_limit = 0.0   # no time limit
        goal.return_home = self._return_home
        self._explore_requested = True
        response = self._explore_client.send_goal_async(
            goal, feedback_callback=self._on_explore_feedback)
        response.add_done_callback(self._on_explore_response)
        self.notify('explore: goal sent')

    def _on_explore_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._explore_requested = False
            self.notify('explore: the explorer rejected the goal')
            return
        # The goal handle is our ticket for this one job: cancelling and asking for
        # the result both go through it.
        self._goal_handle = goal_handle
        self.notify('explore: goal accepted')
        goal_handle.get_result_async().add_done_callback(self._on_explore_result)

    def _on_explore_feedback(self, feedback_message) -> None:
        # Feedback only reaches the client that sent the goal. The same status is
        # also published on exploration_status for everybody else, so the HUD works
        # no matter who started the exploration.
        self._on_status(feedback_message.feedback.status)

    def _on_explore_result(self, future) -> None:
        outcome = future.result()
        result = outcome.result
        self._goal_handle = None
        self._explore_requested = False
        if outcome.status == GoalStatus.STATUS_CANCELED:
            headline = 'explore: cancelled'
        elif result.success:
            headline = 'explore: finished'
        else:
            headline = 'explore: stopped early'
        self.notify(f'{headline}. {result.message} ({result.explored_area:.0f} m2, '
                    f'{result.distance:.0f} m, {result.duration:.0f} s)')

    def cancel_exploring(self) -> None:
        if self._goal_handle is None:
            self.notify('cancel: this dashboard has no exploration running')
            return
        self._goal_handle.cancel_goal_async().add_done_callback(self._on_cancel_response)

    def _on_cancel_response(self, future) -> None:
        if future.result().goals_canceling:
            self.notify('cancel: accepted, the robot stops')
        else:
            self.notify('cancel: the explorer refused')

    def _on_auto_explore_timer(self) -> None:
        """auto_explore: wait quietly for the action server, then send the goal once."""
        if self._explore_client.server_is_ready():
            self._auto_timer.cancel()
            self.start_exploring()

    # ---- Reset (key R) and goal_pose (key G) ----------------------------------------------

    def reset(self) -> None:
        """Put the robot back at the start (sim) and forget the map (mapper)."""
        for client, server in ((self._reset_world, 'sim'), (self._clear_map, 'mapper')):
            name = client.srv_name
            # service_is_ready() answers at once. wait_for_service() without a timeout
            # would block this thread until the server appears, possibly for ever.
            if not client.service_is_ready():
                self.notify(f'{name}: no service (is the {server} running?)')
                continue
            # call_async() returns a future at once. The answer comes as a callback.
            reply = client.call_async(Trigger.Request())
            reply.add_done_callback(functools.partial(self._on_trigger_reply, name))

    def _on_trigger_reply(self, name: str, future) -> None:
        reply = future.result()
        verdict = 'done' if reply.success else 'failed'
        self.notify(f'{name}: {reply.message or verdict}')

    def send_goal_pose(self, x: float, y: float) -> None:
        """Publish a goal for the explorer, like the "2D Goal Pose" tool of RViz."""
        goal = PoseStamped()
        # A pose means nothing without its frame and time: the header carries both.
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0   # the identity rotation: no preferred heading
        self._goal_publisher.publish(goal)
        if self._goal_publisher.get_subscription_count() == 0:
            self.notify('goal_pose: published, but nobody listens (explorer running?)')
        else:
            self.notify(f'goal_pose: x {x:.1f} m, y {y:.1f} m')

    # ---- The screenshot service --------------------------------------------------------

    def _on_screenshot(self, request: Trigger.Request,
                       response: Trigger.Response) -> Trigger.Response:
        """Runs on the ROS thread, but only the render thread may read the frame.

        So the work is handed over as a job, and this callback waits for the Future.
        Waiting blocks the executor (it is single-threaded), which is acceptable for
        the one frame it normally takes. The timeout makes sure a stuck render loop
        cannot hang the service for ever.
        """
        self._screenshot_count += 1
        name = time.strftime('dashboard_%Y%m%d_%H%M%S') + f'_{self._screenshot_count}.png'
        path = os.path.abspath(os.path.join(self._screenshot_dir, name))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        job = self.render_jobs.submit(functools.partial(self._save_frame, path))
        try:
            job.result(timeout=SCREENSHOT_TIMEOUT)
            response.success = True
            response.message = path
        except concurrent.futures.TimeoutError:
            job.cancel()   # the render loop will skip it if it ever gets there
            response.success = False
            response.message = 'the render loop did not answer in time'
        except Exception as error:
            # Whatever went wrong on the render thread was carried over by the Future.
            response.success = False
            response.message = f'could not write {path}: {error}'
        return response


def spin_until_shutdown(executor: SingleThreadedExecutor) -> None:
    """The body of the ROS thread: handle callbacks until told to stop."""
    try:
        executor.spin()
    except ExternalShutdownException:
        pass   # Ctrl+C shut rclpy down while the executor was waiting: a normal exit.


def main(args=None) -> None:
    # Imported here, not at the top: these two modules pull in threepp, and this file
    # is the ROS half. A reader of the node above never needs to know they exist.
    from explorer_dashboard.app import DashboardApp
    from explorer_dashboard.window import DashboardWindow

    rclpy.init(args=args)
    node = DashboardNode()

    # The window is created on the main thread, so the main thread is the render
    # thread from here on.
    window = DashboardWindow(node.width, node.height, node.headless)
    node.set_frame_saver(window.save_frame)

    # The executor is what calls our callbacks. It gets a thread of its own, so that
    # the main thread is free to draw.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=spin_until_shutdown, args=(executor,),
                                  name='ros')
    ros_thread.start()

    try:
        DashboardApp(node, window).run()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop in the reverse order of starting, and JOIN the thread before destroying
        # what it uses. Tearing a node down under a running executor is a classic
        # crash at exit.
        executor.shutdown()
        ros_thread.join()
        node.destroy_node()
        rclpy.try_shutdown()
        window.close()


if __name__ == '__main__':
    main()
