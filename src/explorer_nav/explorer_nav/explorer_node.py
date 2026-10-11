"""The `explorer` node: decides where the robot goes, and drives it there.

The algorithms are in frontiers.py, planner.py and pure_pursuit.py. This file is the ROS
side, and the place where the threads meet:

    map        --+                  -->  cmd_vel_nav          (20 Hz while driving)
    tf         --+-->  [ explorer ] -->  plan, frontiers      (on every replan)
    goal_pose  --+                  -->  exploration_status   (2 Hz, always)
                                    <->  explore              (action)

Three things happen at three different speeds, each in a callback of its own:

    control   20 Hz   pose -> pure pursuit -> velocity command.   Must never be late.
    planning  ~1 Hz   map -> frontiers -> distance field -> path.  Takes tens of ms.
    explore   minutes the action: start a job, report progress, wait for the end.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from nav_msgs.msg import Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from explorer_interfaces.action import Explore
from explorer_interfaces.msg import ExplorationStatus
from explorer_nav.frontiers import Frontier, find_frontiers
from explorer_nav.occupancy_grid import GridGeometry
from explorer_nav.planner import (
    UNREACHABLE,
    Costmap,
    DistanceField,
    build_costmap,
    compute_distance_field,
    extract_path,
    nearest_traversable,
    simplify_path,
)
from explorer_nav.pure_pursuit import PurePursuit, PurePursuitConfig

# The states in which the robot is supposed to be moving.
DRIVING_STATES = (ExplorationStatus.EXPLORING, ExplorationStatus.RETURNING,
                  ExplorationStatus.NAVIGATING)

# Every deadline for reaching a target includes this much time for turning and
# accelerating, on top of patience_factor times the pure driving time [s].
PATIENCE_BASE = 15.0


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """The rotation about the z axis contained in a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass
class Candidate:
    """A frontier the robot could drive to."""

    frontier: Frontier
    row: int            # the cell to drive to: the reachable frontier cell nearest the centroid
    col: int
    x: float            # the same cell in metres, map frame
    y: float
    cost: float         # path cost from the robot [m]


@dataclass
class Job:
    """Everything about one job: one Explore action, or one drive to a clicked goal.

    The fields in the first block are shared between threads and may only be read or
    written while holding Explorer._lock. The fields in the second block belong to the
    planning callback alone.
    """

    state: int
    started: float                                  # node clock [s]
    time_limit: float = 0.0                         # s, 0 = none
    return_home: bool = False
    distance: float = 0.0                           # m driven so far
    elapsed: float = 0.0                            # s, frozen when the job ends
    target: tuple[float, float] | None = None       # where the robot is heading right now
    path_done: bool = False                         # the follower reached the end of its path
    frontier_count: int = 0
    exploration_complete: bool = False
    message: str = ''
    finished: threading.Event = field(default_factory=threading.Event)

    home: tuple[float, float, float] | None = None              # pose when the job started
    goal: tuple[float, float, float | None] | None = None       # fixed goal (x, y, yaw)
    blacklist: list[tuple[float, float]] = field(default_factory=list)
    cycles_without_frontier: int = 0
    failed_plans: int = 0
    last_plan_time: float = -math.inf
    progress_anchor: tuple[float, float] | None = None
    progress_time: float = 0.0
    chase_point: tuple[float, float] | None = None              # the spot the deadline is for
    chase_deadline: float = math.inf
    replan_seconds: list[float] = field(default_factory=list)


class Explorer(Node):

    def __init__(self) -> None:
        super().__init__('explorer')

        # ---- Parameters ----
        self._robot_radius = self._parameter(
            'robot_radius', 0.22, 'Radius of the robot [m]')
        self._inflation_margin = self._parameter(
            'inflation_margin', 0.14,
            'Extra distance the robot centre keeps from obstacles, on top of its radius [m]')
        self._comfort_cells = self._parameter(
            'comfort_cells', 3,
            'Width of the band next to the inflated obstacles where driving is allowed '
            'but costs extra [cells]. 0 gives the shortest paths, hugging the obstacles')
        self._occupied_threshold = self._parameter(
            'occupied_threshold', 50,
            'Map cells with at least this occupancy [percent] count as obstacles')
        self._min_frontier_size = self._parameter(
            'min_frontier_size', 8, 'Frontier clusters with fewer cells are ignored')
        self._replan_period = self._parameter(
            'replan_period', 1.0, 'Time between two replans while driving [s]')
        self._switch_factor = self._parameter(
            'switch_factor', 0.6,
            'Abandon the current frontier only for one whose path cost is below this '
            'fraction of the current cost. 1.0 = always take the cheapest (flip-flops)')
        self._same_frontier_distance = self._parameter(
            'same_frontier_distance', 1.5,
            'A frontier this close to the current target counts as the same one [m]')
        self._stuck_timeout = self._parameter(
            'stuck_timeout', 6.0,
            'Give up on a target when the robot has not moved for this long [s]')
        self._patience_factor = self._parameter(
            'patience_factor', 3.0,
            'Give up on a target when getting there takes more than this many times '
            'the driving time of the planned path (plus 15 s)')
        self._blacklist_radius = self._parameter(
            'blacklist_radius', 0.8,
            'Frontiers this close to a target the robot gave up on are skipped [m]')
        self._finish_cycles = self._parameter(
            'finish_cycles', 3,
            'Exploration is complete after this many replans in a row without a frontier')
        control_rate = self._parameter(
            'control_rate', 20.0, 'Rate of the velocity commands [Hz]')
        follower_defaults = PurePursuitConfig()
        follower_config = PurePursuitConfig(
            lookahead=self._parameter(
                'lookahead', follower_defaults.lookahead,
                'Pure pursuit lookahead distance [m]. Larger = smoother, cuts corners more'),
            max_linear_speed=self._parameter(
                'max_linear_speed', follower_defaults.max_linear_speed,
                'Top forward speed [m/s]'),
            max_angular_speed=self._parameter(
                'max_angular_speed', follower_defaults.max_angular_speed,
                'Top turn rate [rad/s]'),
            goal_tolerance=self._parameter(
                'goal_tolerance', follower_defaults.goal_tolerance,
                'The robot has arrived when it is this close to the end of the path [m]'),
        )
        self._map_frame = self._parameter('map_frame', 'map', 'Frame of the map')
        self._base_frame = self._parameter('base_frame', 'base_link', 'Frame of the robot')

        # ---- Shared state, and the lock that guards it ----
        # This node runs on several threads at once (see main, and the callback groups
        # below). Two callbacks may therefore touch the same variable at the same moment,
        # exactly like two std::thread in C++. Everything listed here is only read or
        # written inside `with self._lock:`, the Python spelling of a std::lock_guard.
        # The rule that keeps this painless: hold the lock to copy data in or out, never
        # while computing. Planning takes tens of milliseconds, and the 20 Hz control
        # callback must not wait for it.
        self._lock = threading.Lock()
        self._job: Job | None = None
        self._follower = PurePursuit(follower_config)
        self._map_data: np.ndarray | None = None
        self._map_geometry: GridGeometry | None = None
        self._explored_area = 0.0
        self._action_running = False
        self._last_pose: tuple[float, float, float] | None = None

        # Only used by the control callback.
        self._was_driving = False

        # ---- tf2: where is the robot? ----
        # The simulator does send the pose on the odom topic too. We ask tf2 instead,
        # because the question is "where is base_link in the MAP frame", and only tf2
        # knows how map, odom and base_link hang together. Time() means "the newest
        # transform you have", which is what a controller wants.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- Callback groups: who may run at the same time as whom ----
        # A MultiThreadedExecutor runs callbacks on a pool of threads, but it asks each
        # callback's group first:
        #   MutuallyExclusive = at most one callback of this group at any time,
        #   Reentrant         = no restriction, even the same callback twice.
        # Callbacks without a group land in the node's default group, which is mutually
        # exclusive. If everything stayed there, a 60 ms replan would delay the velocity
        # command behind it, and the action (which blocks for minutes) would freeze the
        # whole node. So each of the three speeds gets a group of its own.
        self._control_group = MutuallyExclusiveCallbackGroup()
        self._planning_group = MutuallyExclusiveCallbackGroup()
        self._action_group = ReentrantCallbackGroup()

        # ---- Publishers ----
        # Commands: reliable, with a little queue. Every command should arrive, and the
        # subscriber (the safety node) is on the same machine.
        self._cmd_publisher = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        # Plan and frontiers are for display: reliable, and only the newest is of interest.
        latest_only = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._plan_publisher = self.create_publisher(Path, 'plan', latest_only)
        self._frontier_publisher = self.create_publisher(MarkerArray, 'frontiers', latest_only)
        self._status_publisher = self.create_publisher(
            ExplorationStatus, 'exploration_status', 10)

        # ---- Subscribers ----
        # The QoS of a subscriber must be compatible with the publisher's. The mapper
        # offers the map as reliable + transient local, and by asking for transient local
        # we receive the last map immediately, even if we start after the mapper.
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map_subscription = self.create_subscription(
            OccupancyGridMsg, 'map', self._on_map, map_qos)
        self._goal_subscription = self.create_subscription(
            PoseStamped, 'goal_pose', self._on_goal_pose, latest_only)

        # ---- The action server ----
        # Why an action and not a service? Exploring takes minutes. A service call would
        # leave the caller waiting without news and without a way to stop it. An action
        # is a goal, a stream of feedback, a result at the end, and a cancel button.
        self._action_server = ActionServer(
            self, Explore, 'explore',
            execute_callback=self._execute_explore,
            goal_callback=self._on_explore_goal,
            cancel_callback=self._on_explore_cancel,
            callback_group=self._action_group)

        # ---- Timers ----
        self._control_timer = self.create_timer(
            1.0 / control_rate, self._control_tick, callback_group=self._control_group)
        self._planning_timer = self.create_timer(
            0.1, self._planning_tick, callback_group=self._planning_group)
        self._status_timer = self.create_timer(0.5, self._publish_status)

        self.get_logger().info('Explorer ready: send an Explore goal, or click a goal pose')

    def _parameter(self, name: str, default, description: str):
        """Declare a parameter with a description and return its value."""
        descriptor = ParameterDescriptor(description=description)
        return self.declare_parameter(name, default, descriptor).value

    def _now(self) -> float:
        """The node's clock in seconds. It follows simulated time if use_sim_time is set."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _robot_pose(self) -> tuple[float, float, float] | None:
        """The newest known pose (x, y, yaw) of the robot in the map frame, or None."""
        try:
            transform = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
        except TransformException:
            return None     # normal during the first moments, before any transform arrived
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        return translation.x, translation.y, yaw

    # ------------------------------------------------------------------ subscriptions

    def _on_map(self, message: OccupancyGridMsg) -> None:
        geometry = GridGeometry(
            resolution=message.info.resolution,
            origin_x=message.info.origin.position.x,
            origin_y=message.info.origin.position.y,
            width=message.info.width,
            height=message.info.height)
        data = np.array(message.data, dtype=np.int8).reshape(geometry.height, geometry.width)
        free_cells = np.count_nonzero((data >= 0) & (data < self._occupied_threshold))
        with self._lock:
            # The planner gets a reference to this array, not a copy. That is safe
            # because the array is never modified again: the next map is a new array.
            self._map_data = data
            self._map_geometry = geometry
            self._explored_area = free_cells * geometry.resolution ** 2

    def _on_goal_pose(self, message: PoseStamped) -> None:
        """The operator clicked a goal: drive there, unless an exploration is running."""
        frame = message.header.frame_id
        # map and odom coincide in this demo (the mapper publishes the identity between
        # them), so a goal in either frame can be used as it is.
        if frame not in ('', self._map_frame, 'odom'):
            self.get_logger().warning(f'Ignoring goal_pose in frame "{frame}", expected "map"')
            return
        orientation = message.pose.orientation
        yaw = yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        goal = (message.pose.position.x, message.pose.position.y, yaw)
        with self._lock:
            if self._action_running:
                self.get_logger().warning('Ignoring goal_pose: an exploration is running')
                return
            self._start_job_locked(Job(
                state=ExplorationStatus.NAVIGATING, started=self._now(), goal=goal,
                target=(goal[0], goal[1])))
        self.get_logger().info(f'Navigating to ({goal[0]:.2f}, {goal[1]:.2f})')

    # ------------------------------------------------------------------ jobs

    def _start_job_locked(self, job: Job) -> None:
        """Make job the current job. The caller holds the lock."""
        previous = self._job
        if previous is not None:
            previous.finished.set()
        self._job = job
        self._follower.clear()

    def _finish_job(self, job: Job, message: str,
                    final_state: int = ExplorationStatus.DONE) -> None:
        """End a job: stop the robot, record the outcome, wake up whoever waits for it."""
        with self._lock:
            if self._job is not job or job.finished.is_set():
                return
            job.state = final_state
            job.elapsed = self._now() - job.started
            job.message = message
            job.target = None
            job.frontier_count = 0
            self._follower.clear()
        self._publish_plan([])
        self._publish_markers(None, [], [], None)
        if job.replan_seconds:
            self.get_logger().info(
                f'{message}. Drove {job.distance:.1f} m in {job.elapsed:.0f} s. '
                f'{len(job.replan_seconds)} replans, mean '
                f'{np.mean(job.replan_seconds) * 1e3:.0f} ms, '
                f'slowest {np.max(job.replan_seconds) * 1e3:.0f} ms')
        else:
            self.get_logger().info(message)
        job.finished.set()

    # ------------------------------------------------------------------ the action

    def _on_explore_goal(self, goal_request: Explore.Goal) -> GoalResponse:
        """A client asks for an exploration. Accept or reject, quickly.

        A second goal while one is running is REJECTED. The alternatives would be to
        queue it (explore again when the building is already explored?) or to let it
        replace the running one (then a second operator silently takes the robot away
        from the first). There is one robot, so there is one job. Whoever wants a
        different one cancels the running goal first, and knows that they did.
        """
        with self._lock:
            if self._action_running:
                self.get_logger().warning('Explore goal rejected: already exploring')
                return GoalResponse.REJECT
            # Claimed here, under the lock, and not later in the execute callback: two
            # goals arriving together must not both see "not running".
            self._action_running = True
        return GoalResponse.ACCEPT

    def _on_explore_cancel(self, goal_handle) -> CancelResponse:
        """A client asks to cancel. Always fine, the execute callback does the stopping."""
        return CancelResponse.ACCEPT

    def _execute_explore(self, goal_handle) -> Explore.Result:
        """Runs for as long as the exploration runs, on a thread of the executor.

        This callback BLOCKS: it loops until the job is over, which can be minutes. With
        the default single-threaded executor nothing else would run during that time: no
        map would arrive, no command would go out, and not even the cancel request could
        be received. That is why this node is spun by a MultiThreadedExecutor and why the
        action has a callback group of its own.

        The actual exploring is done by the planning and control timers. This callback
        only starts the job, streams feedback, and watches for the end or a cancel.
        """
        request = goal_handle.request
        job = Job(state=ExplorationStatus.EXPLORING, started=self._now(),
                  time_limit=float(request.time_limit), return_home=bool(request.return_home))
        with self._lock:
            self._start_job_locked(job)
        self.get_logger().info(
            f'Exploring (time limit {job.time_limit:.0f} s, return home: {job.return_home})')

        feedback = Explore.Feedback()
        last_feedback = -math.inf
        try:
            # Event.wait is the condition-variable way of sleeping: it returns True the
            # moment another thread calls finished.set(), or False after the timeout.
            while not job.finished.wait(timeout=0.1):
                if not rclpy.ok():
                    return Explore.Result()
                if goal_handle.is_cancel_requested:
                    self._finish_job(job, 'Exploration cancelled',
                                     final_state=ExplorationStatus.IDLE)
                    goal_handle.canceled()
                    return self._result_of(job)
                if time.monotonic() - last_feedback >= 0.5:
                    last_feedback = time.monotonic()
                    feedback.status = self._status_message()
                    goal_handle.publish_feedback(feedback)
            # The action SUCCEEDED in the sense that it ran to its end. Whether the
            # building was explored completely is a separate statement: result.success.
            goal_handle.succeed()
            return self._result_of(job)
        finally:
            with self._lock:
                self._action_running = False

    def _result_of(self, job: Job) -> Explore.Result:
        result = Explore.Result()
        with self._lock:
            result.success = job.exploration_complete
            result.message = job.message
            result.explored_area = float(self._explored_area)
            result.distance = float(job.distance)
            result.duration = float(job.elapsed)
        return result

    # ------------------------------------------------------------------ status

    def _status_message(self) -> ExplorationStatus:
        status = ExplorationStatus()
        with self._lock:
            job = self._job
            status.explored_area = float(self._explored_area)
            if job is None:
                status.state = ExplorationStatus.IDLE
                return status
            status.state = job.state
            status.frontier_count = job.frontier_count
            status.distance = float(job.distance)
            if job.state in DRIVING_STATES:
                status.elapsed = float(self._now() - job.started)
            else:
                status.elapsed = float(job.elapsed)
            if job.target is not None:
                status.target.x = float(job.target[0])
                status.target.y = float(job.target[1])
        return status

    def _publish_status(self) -> None:
        self._status_publisher.publish(self._status_message())

    # ------------------------------------------------------------------ control, 20 Hz

    def _control_tick(self) -> None:
        """One step of the path follower: pose in, velocity command out.

        The simulator stops the robot when no command arrives for 0.5 s, like the motor
        controller of a real robot. So while a job is running, a command goes out on
        EVERY tick, also when it is zero (no pose yet, no path yet, turning done).
        """
        pose = self._robot_pose()
        command = Twist()
        with self._lock:
            job = self._job
            driving = job is not None and job.state in DRIVING_STATES
            if driving and pose is not None:
                if self._last_pose is not None:
                    job.distance += math.hypot(pose[0] - self._last_pose[0],
                                               pose[1] - self._last_pose[1])
                self._last_pose = pose
                result = self._follower.compute(*pose)
                command.linear.x = result.linear
                command.angular.z = result.angular
                if result.reached:
                    job.path_done = True
            if not driving:
                self._last_pose = None
        if driving:
            self._cmd_publisher.publish(command)
            self._was_driving = True
        elif self._was_driving:
            # The job just ended: say "stop" once, explicitly. After that we stay silent
            # and leave the topic to whoever else may want to drive (a teleop node).
            self._cmd_publisher.publish(Twist())
            self._was_driving = False

    # ------------------------------------------------------------------ planning, ~1 Hz

    def _planning_tick(self) -> None:
        """Runs ten times a second and decides whether it is time to plan again."""
        with self._lock:
            job = self._job
            if job is None or job.state not in DRIVING_STATES:
                return
            state = job.state
            target = job.target
            data = self._map_data
            geometry = self._map_geometry
            path_done = job.path_done
            has_path = self._follower.has_path()
        now = self._now()

        if job.time_limit > 0.0 and now - job.started > job.time_limit:
            self._finish_job(job, f'Time limit of {job.time_limit:.0f} s reached')
            return

        pose = self._robot_pose()
        if pose is None or data is None or geometry is None:
            self.get_logger().info('Waiting for the map and the robot pose',
                                   throttle_duration_sec=5.0)
            return
        if job.home is None:
            job.home = pose

        stuck = self._is_stuck(job, pose, now, has_path)
        if state == ExplorationStatus.EXPLORING:
            self._explore_step(job, target, pose, data, geometry, now, path_done, stuck)
        else:
            returning = state == ExplorationStatus.RETURNING
            self._goal_step(job, returning, pose, data, geometry, now, path_done, stuck)

    def _is_stuck(self, job: Job, pose: tuple[float, float, float], now: float,
                  has_path: bool) -> bool:
        """True when the robot has a path but has not moved for stuck_timeout seconds.

        Typical cause: the safety node holds the robot back from something the planner
        considers passable. Standing still without a path is waiting, not being stuck.
        """
        anchor = job.progress_anchor
        moved = anchor is None or math.hypot(pose[0] - anchor[0], pose[1] - anchor[1]) > 0.25
        if moved or not has_path:
            job.progress_anchor = (pose[0], pose[1])
            job.progress_time = now
            return False
        return now - job.progress_time > self._stuck_timeout

    def _watch_the_clock(self, job: Job, point: tuple[float, float], cost: float,
                         now: float) -> None:
        """Give the robot a deadline for reaching a point: a generous multiple of the
        time the path should take.

        A robot can fail to arrive without ever standing still. A gap that is just wide
        enough opens and closes on the map with the sensor noise, the planned route flips
        between "through the gap" and "all the way around", and the robot shuttles to and
        fro. The stuck timer never fires, because the robot is moving. The deadline does.

        A new deadline is only set for a new point. A frontier that recedes while the
        robot follows it keeps moving, so it keeps getting fresh deadlines, as it should.
        """
        chase = job.chase_point
        if chase is not None and math.hypot(point[0] - chase[0], point[1] - chase[1]) < 0.75:
            return
        driving_time = cost / self._follower.config.max_linear_speed
        job.chase_point = point
        job.chase_deadline = now + PATIENCE_BASE + self._patience_factor * driving_time

    def _explore_step(self, job: Job, target: tuple[float, float] | None,
                      pose: tuple[float, float, float], data: np.ndarray,
                      geometry: GridGeometry, now: float, path_done: bool, stuck: bool) -> None:
        """Pick a frontier and plan the path to it, when it is time to do so.

        Time to plan is: once per replan_period, and at once when the robot arrived, when
        the target is no longer a frontier (the robot has seen what was there, usually
        long before arriving), or when the robot is stuck.
        """
        target_open = target is not None and self._has_unknown_nearby(data, geometry, target)
        target_vanished = target is not None and not target_open
        too_slow = target is not None and now > job.chase_deadline
        due = now - job.last_plan_time >= self._replan_period
        if not (due or path_done or target_vanished or stuck or too_slow):
            return

        if target is not None and target_open and (stuck or path_done or too_slow):
            # The robot cannot get there, or it stands there and still cannot see what
            # lies beyond, or it has been on its way for far too long. Trying again
            # would loop for ever: give this frontier up.
            if stuck:
                reason = 'the robot is stuck'
            elif path_done:
                reason = 'nothing more to see from there'
            else:
                reason = 'no progress, it takes far longer than the path promised'
            self.get_logger().info(
                f'Giving up on the frontier at ({target[0]:.1f}, {target[1]:.1f}): {reason}')
            job.blacklist.append(target)
            job.chase_point = None
            target = None
        if stuck:
            job.progress_anchor = None      # start the stuck timer afresh for the new plan

        started = time.perf_counter()
        free = (data >= 0) & (data < self._occupied_threshold)
        unknown = data < 0
        if np.count_nonzero(free) * geometry.resolution ** 2 < 1.0:
            return      # the mapper has not seen anything yet: too early to draw conclusions
        costmap, field = self._distance_field(free, data, geometry, pose)
        frontiers = find_frontiers(free, unknown, self._min_frontier_size)
        candidates = self._reachable_candidates(job, frontiers, field, geometry)
        chosen = self._choose(candidates, target)

        new_target = None
        path_points: list[tuple[float, float]] = []
        if chosen is not None:
            # The same field that priced the frontiers now yields the path to the winner.
            cells = simplify_path(extract_path(field, chosen.row, chosen.col), costmap)
            path_points = [geometry.cell_to_world(row, col) for row, col in cells]
            path_points[0] = (pose[0], pose[1])
            new_target = (chosen.x, chosen.y)
            self._watch_the_clock(job, new_target, chosen.cost, now)
            # Hidden by default. Show with: --ros-args --log-level explorer:=debug
            self.get_logger().debug(
                f'{len(candidates)} of {len(frontiers)} frontiers reachable, heading for '
                f'({chosen.x:.1f}, {chosen.y:.1f}), path cost {chosen.cost:.1f} m')
        job.replan_seconds.append(time.perf_counter() - started)
        job.last_plan_time = now

        with self._lock:
            if self._job is not job or job.state != ExplorationStatus.EXPLORING:
                return      # the job ended or changed while we were planning
            job.frontier_count = len(candidates)
            job.path_done = False
            job.target = new_target
            if chosen is not None:
                self._follower.set_path(np.array(path_points))
            else:
                self._follower.clear()
        self._publish_plan(path_points)
        self._publish_markers(geometry, frontiers, candidates, new_target)

        if chosen is not None:
            job.cycles_without_frontier = 0
            return
        job.cycles_without_frontier += 1
        if job.cycles_without_frontier < self._finish_cycles:
            return      # one empty look is not proof, the map may still be filling in

        home = job.home
        drive_home = job.return_home and home is not None
        with self._lock:
            if self._job is not job or job.state != ExplorationStatus.EXPLORING:
                return      # cancelled a moment ago: do not send the robot home now
            job.exploration_complete = True
            if drive_home:
                job.state = ExplorationStatus.RETURNING
                job.goal = home
                job.target = (home[0], home[1])
        if drive_home:
            self.get_logger().info('No frontier left: exploration complete, driving home')
            job.last_plan_time = -math.inf
            job.progress_anchor = None
            job.chase_point = None
            job.chase_deadline = math.inf
        else:
            self._finish_job(job, 'Exploration complete')

    def _goal_step(self, job: Job, returning: bool, pose: tuple[float, float, float],
                   data: np.ndarray, geometry: GridGeometry, now: float, path_done: bool,
                   stuck: bool) -> None:
        """Drive to a fixed goal: the start pose (RETURNING) or a clicked pose (NAVIGATING)."""
        if path_done:
            self._finish_job(
                job, 'Exploration complete, back at the start' if returning else 'Goal reached')
            return
        if stuck or now > job.chase_deadline:
            self._finish_job(
                job, 'Exploration complete, but the robot got stuck on the way home'
                if returning else 'Stuck on the way to the goal')
            return
        if now - job.last_plan_time < self._replan_period:
            return

        # The path is planned again every second although the goal stays where it is:
        # the map keeps changing, and a crate may have been dropped into the old path.
        started = time.perf_counter()
        goal = job.goal
        free = (data >= 0) & (data < self._occupied_threshold)
        costmap, field = self._distance_field(free, data, geometry, pose)
        path_points: list[tuple[float, float]] = []
        if field is not None:
            goal_cell = self._reachable_cell_near(field, geometry, goal[0], goal[1])
            if goal_cell is not None:
                cost = field.cost_to(*goal_cell) * geometry.resolution
                self._watch_the_clock(job, (goal[0], goal[1]), cost, now)
                cells = simplify_path(extract_path(field, *goal_cell), costmap)
                path_points = [geometry.cell_to_world(row, col) for row, col in cells]
                path_points[0] = (pose[0], pose[1])
                if goal_cell == geometry.world_to_cell(goal[0], goal[1]):
                    path_points[-1] = (goal[0], goal[1])    # the exact spot, not the cell centre
        job.replan_seconds.append(time.perf_counter() - started)
        job.last_plan_time = now

        if not path_points:
            job.failed_plans += 1
            with self._lock:
                if self._job is job:
                    self._follower.clear()
            if job.failed_plans >= self._finish_cycles:
                self._finish_job(
                    job, 'Exploration complete, but there is no way home'
                    if returning else 'There is no way to the goal')
            return
        job.failed_plans = 0
        with self._lock:
            if self._job is not job or job.state not in DRIVING_STATES:
                return
            job.path_done = False
            self._follower.set_path(np.array(path_points), final_yaw=goal[2])
        self._publish_plan(path_points)
        self._publish_markers(geometry, [], [], (goal[0], goal[1]))

    # ------------------------------------------------------------------ planning helpers

    def _distance_field(self, free: np.ndarray, data: np.ndarray, geometry: GridGeometry,
                        pose: tuple[float, float, float]
                        ) -> tuple[Costmap, DistanceField | None]:
        """Build the costmap from the map and spread the wave from the robot's cell.

        The field is None when the robot is outside the map or completely boxed in.
        """
        occupied = data >= self._occupied_threshold
        # The robot centre must stay robot_radius + inflation_margin away from every
        # obstacle. Half a cell is added because an obstacle can be anywhere inside its
        # cell, not only at the centre.
        inflation_cells = (self._robot_radius + self._inflation_margin) / geometry.resolution + 0.5
        costmap = build_costmap(free, occupied, inflation_cells, self._comfort_cells)
        row, col = geometry.world_to_cell(pose[0], pose[1])
        if not geometry.contains(row, col):
            self.get_logger().warning('The robot is outside the map', throttle_duration_sec=5.0)
            return costmap, None
        field = compute_distance_field(costmap, row, col)
        if field is None:
            self.get_logger().warning('The robot is boxed in: no free cell within reach',
                                      throttle_duration_sec=5.0)
        return costmap, field

    def _reachable_candidates(self, job: Job, frontiers: list[Frontier],
                              field: DistanceField | None,
                              geometry: GridGeometry) -> list[Candidate]:
        """The frontiers the robot can drive to, with the cost of getting there.

        The cost is read straight from the distance field: no path is planned here.
        """
        candidates = []
        if field is None:
            return candidates
        for frontier in frontiers:
            distances = field.distance[frontier.rows, frontier.cols]
            reachable = distances != UNREACHABLE
            if not reachable.any():
                continue        # for example a gap that is too narrow for the robot
            # Aim for the middle of the frontier, that is where the view is best. The
            # centroid itself may lie in a wall, so take the reachable cell closest to it.
            offset = np.hypot(frontier.rows - frontier.centroid_row,
                              frontier.cols - frontier.centroid_col)
            offset[~reachable] = np.inf
            best = int(np.argmin(offset))
            row = int(frontier.rows[best])
            col = int(frontier.cols[best])
            x, y = geometry.cell_to_world(row, col)
            if any(math.hypot(x - bx, y - by) < self._blacklist_radius
                   for bx, by in job.blacklist):
                continue
            cost = float(distances[best]) * geometry.resolution
            candidates.append(Candidate(frontier, row, col, x, y, cost))
        return candidates

    def _choose(self, candidates: list[Candidate],
                current_target: tuple[float, float] | None) -> Candidate | None:
        """The cheapest frontier, unless we are already on our way to a good enough one.

        Two frontiers at about the same cost would otherwise take turns being the
        cheapest, as the costs wobble with every map update, and the robot would turn
        back and forth between them without reaching either. So the robot keeps its
        target until that is gone or another one is CLEARLY cheaper (hysteresis again).
        """
        if not candidates:
            return None
        cheapest = min(candidates, key=lambda candidate: candidate.cost)
        if current_target is None:
            return cheapest
        # The frontier we were driving to has moved a bit since the last plan (it recedes
        # as the robot looks into it). The candidate nearest to the old target is "it".
        current = min(candidates, key=lambda candidate: math.hypot(
            candidate.x - current_target[0], candidate.y - current_target[1]))
        moved = math.hypot(current.x - current_target[0], current.y - current_target[1])
        if moved > self._same_frontier_distance:
            return cheapest
        if cheapest.cost < self._switch_factor * current.cost:
            return cheapest
        return current

    def _has_unknown_nearby(self, data: np.ndarray, geometry: GridGeometry,
                            point: tuple[float, float], radius_cells: int = 5) -> bool:
        """Is there still unexplored space around this point? If not, the frontier is gone."""
        row, col = geometry.world_to_cell(point[0], point[1])
        window = data[max(row - radius_cells, 0):row + radius_cells + 1,
                      max(col - radius_cells, 0):col + radius_cells + 1]
        return bool((window < 0).any())

    def _reachable_cell_near(self, field: DistanceField, geometry: GridGeometry,
                             x: float, y: float, tolerance_cells: float = 6.0
                             ) -> tuple[int, int] | None:
        """The goal's cell, or the closest reachable cell if the goal itself is blocked.

        A goal clicked a little too close to a shelf should still work.
        """
        row, col = geometry.world_to_cell(x, y)
        if geometry.contains(row, col) and field.is_reachable(row, col):
            return row, col
        return nearest_traversable(field.distance != UNREACHABLE, row, col, tolerance_cells)

    # ------------------------------------------------------------------ display

    def _publish_plan(self, points: list[tuple[float, float]]) -> None:
        """Publish the path for display. An empty list erases the old one."""
        path = Path()
        path.header.frame_id = self._map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._plan_publisher.publish(path)

    def _publish_markers(self, geometry: GridGeometry | None, frontiers: list[Frontier],
                         candidates: list[Candidate],
                         target: tuple[float, float] | None) -> None:
        """One sphere per frontier cluster, plus one for the chosen target.

        geometry converts the frontier centroids from cells to metres. It may be None
        when there are no frontiers to draw.
        """
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        # Markers stay on screen until told otherwise, so every message starts by
        # wiping the previous set. Otherwise explored frontiers would linger for ever.
        wipe = Marker()
        wipe.header.frame_id = self._map_frame
        wipe.header.stamp = stamp
        wipe.action = Marker.DELETEALL
        markers.markers.append(wipe)

        open_frontiers = {id(candidate.frontier) for candidate in candidates}
        for index, frontier in enumerate(frontiers):
            x, y = geometry.cell_to_world(frontier.centroid_row, frontier.centroid_col)
            if id(frontier) in open_frontiers:
                color = (1.0, 0.6, 0.0)     # orange: the robot can and will go there
            else:
                color = (0.5, 0.5, 0.5)     # grey: unreachable, or given up
            markers.markers.append(
                self._sphere('frontiers', index, x, y, 0.25, color, stamp))
        if target is not None:
            markers.markers.append(
                self._sphere('target', 0, target[0], target[1], 0.4, (0.1, 0.9, 0.2), stamp))
        self._frontier_publisher.publish(markers)

    def _sphere(self, namespace: str, marker_id: int, x: float, y: float, diameter: float,
                color: tuple[float, float, float], stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = diameter
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 0.9
        return marker


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Explorer()
    # Four threads: control, planning, the blocking action, and one for everything
    # else (map, tf, status, goal_pose).
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # No "stop" command is sent from here: after Ctrl+C the connection to ROS is
        # already going down. The simulator's 0.5 s command watchdog stops the robot,
        # which is exactly what that watchdog is for.
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
