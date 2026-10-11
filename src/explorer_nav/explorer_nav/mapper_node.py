"""The `mapper` node: turns laser scans into the map everybody else uses.

All the mathematics is in occupancy_grid.py. This file is the ROS side of it:

    scan  --+
            +-->  [ mapper ]  -->  map        (nav_msgs/OccupancyGrid, 2 Hz)
    tf    --+                 -->  tf_static  (map -> odom, identity)
                              <->  clear_map  (service)
"""

from __future__ import annotations

import array
import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from explorer_nav.occupancy_grid import GridGeometry, LogOddsConfig, OccupancyGrid

# A scan whose pose is still unknown after this long is thrown away.
MAX_SCAN_WAIT = 1.0     # s
# How many scans may wait for their transform. At 10 Hz this is 2 s worth.
MAX_PENDING_SCANS = 20


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """The rotation about the z axis contained in a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Mapper(Node):

    def __init__(self) -> None:
        super().__init__('mapper')

        # ---- Parameters ----
        # A parameter is a named value that can be set from outside without touching the
        # code: in a launch file, or with `--ros-args -p resolution:=0.05`. The description
        # is what `ros2 param describe /mapper resolution` prints.
        resolution = self._parameter('resolution', 0.1, 'Side length of one map cell [m]')
        origin_x = self._parameter('origin_x', -4.0, 'x of the lower-left map corner [m]')
        origin_y = self._parameter('origin_y', -10.0, 'y of the lower-left map corner [m]')
        width_m = self._parameter('width_m', 30.0, 'Extent of the map along x [m]')
        height_m = self._parameter('height_m', 20.0, 'Extent of the map along y [m]')
        publish_rate = self._parameter('publish_rate', 2.0, 'How often the map is published [Hz]')
        self._map_frame = self._parameter('map_frame', 'map', 'Frame the map is drawn in')
        self._odom_frame = self._parameter('odom_frame', 'odom', 'Frame of the wheel odometry')
        defaults = LogOddsConfig()
        config = LogOddsConfig(
            hit=self._parameter(
                'log_odds_hit', defaults.hit,
                'Evidence added to the cell where a beam ended (log-odds, positive)'),
            miss=self._parameter(
                'log_odds_miss', defaults.miss,
                'Evidence added to every cell a beam passed through (log-odds, negative)'),
            minimum=self._parameter(
                'log_odds_min', defaults.minimum,
                'Lower clamp: how sure the map may become that a cell is free'),
            maximum=self._parameter(
                'log_odds_max', defaults.maximum,
                'Upper clamp: how sure the map may become that a cell is occupied'),
        )

        geometry = GridGeometry(
            resolution=resolution, origin_x=origin_x, origin_y=origin_y,
            width=int(round(width_m / resolution)), height=int(round(height_m / resolution)))
        self._grid = OccupancyGrid(geometry, config)

        # ---- tf2: where was the lidar when the scan was taken? ----
        # A scan is a list of distances measured FROM THE LIDAR. To draw them on the map we
        # need the pose of the lidar in the map frame at the moment of the scan. Nobody
        # sends us that pose directly. The simulator publishes odom -> base_link and
        # base_link -> laser, we publish map -> odom, and tf2 chains them. The Buffer
        # stores the last ten seconds of every transform, so we can ask for the past:
        # "map -> laser at 12:00:03.250".
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # map -> odom never changes in this demo, so it is published once, as a STATIC
        # transform. Static transforms go out with "transient local" durability: the
        # middleware keeps the message and hands it to every node that starts later.
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_map_to_odom()

        # ---- Publisher: the map ----
        # QoS (quality of service) is the delivery contract of a topic. For the map:
        #   reliable        = resend until it arrives. A lost map would be missing for 0.5 s.
        #   transient local = keep the last map for late joiners. A dashboard started in
        #                     the middle of a run gets the map at once.
        #   depth 1         = only the newest map is of any interest.
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map_publisher = self.create_publisher(OccupancyGridMsg, 'map', map_qos)

        # ---- Subscriber: the laser ----
        # Sensor data wants the opposite contract: best effort, small queue. If a scan is
        # lost, the next one is 0.1 s away and more useful than a resent old one.
        # A reliable subscriber would also not match the simulator's best-effort
        # publisher at all: no error, just silence. (`ros2 topic info -v scan` shows both.)
        self._pending_scans: deque[tuple[LaserScan, float]] = deque(maxlen=MAX_PENDING_SCANS)
        self._scan_subscription = self.create_subscription(
            LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)

        # ---- Service ----
        # A service is a remote function call: one request, one answer, done quickly.
        self._clear_service = self.create_service(Trigger, 'clear_map', self._on_clear_map)

        # ---- Timers ----
        self._process_timer = self.create_timer(0.02, self._process_pending_scans)
        self._publish_timer = self.create_timer(1.0 / publish_rate, self._publish_map)

        # Threads: this node is spun by the default single-threaded executor (see main).
        # It runs ONE callback at a time, so the scan callback, the two timers and the
        # service can share self._grid and self._pending_scans without a lock. All of them
        # are short. The explorer node cannot get away with this, see explorer_node.py.

        self._scans_integrated = 0
        self._integration_seconds = 0.0
        self._slowest_integration = 0.0

        self.get_logger().info(
            f'Mapping {width_m:.0f} x {height_m:.0f} m at {resolution} m per cell '
            f'({geometry.width} x {geometry.height} cells)')

    def _parameter(self, name: str, default, description: str):
        """Declare a parameter with a description and return its value."""
        descriptor = ParameterDescriptor(description=description)
        return self.declare_parameter(name, default, descriptor).value

    def _publish_map_to_odom(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._odom_frame
        transform.transform.rotation.w = 1.0    # the identity: no translation, no rotation
        self._static_broadcaster.sendTransform(transform)

    def _on_scan(self, scan: LaserScan) -> None:
        """A scan arrived. Do not process it yet, just queue it.

        The transform for the scan's time stamp travels on another topic (tf) and often
        arrives a few milliseconds AFTER the scan. Looking it up right here would fail
        about half the time. Blocking in this callback until it arrives would stall the
        executor, and with it the very callback that receives the transform. So the scan
        waits in a queue, and a timer picks it up once its transform is there.
        """
        self._pending_scans.append((scan, time.monotonic()))

    def _process_pending_scans(self) -> None:
        """Integrate, oldest first, every queued scan whose pose is known by now."""
        while self._pending_scans:
            scan, received = self._pending_scans[0]
            stamp = Time.from_msg(scan.header.stamp)
            if self._tf_buffer.can_transform(self._map_frame, scan.header.frame_id, stamp):
                self._pending_scans.popleft()
                self._integrate(scan, stamp)
            elif time.monotonic() - received > MAX_SCAN_WAIT:
                self._pending_scans.popleft()
                self.get_logger().warning(
                    f'Dropped a scan: no transform {self._map_frame} -> '
                    f'{scan.header.frame_id} for its time stamp after {MAX_SCAN_WAIT} s',
                    throttle_duration_sec=5.0)
            else:
                # Not there yet. Younger scans need even younger transforms: try again
                # at the next timer tick.
                break

    def _integrate(self, scan: LaserScan, stamp: Time) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, scan.header.frame_id, stamp)
        except TransformException as error:
            self.get_logger().warning(f'Transform lookup failed: {error}')
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)

        started = time.perf_counter()
        self._grid.integrate_scan(
            translation.x, translation.y, yaw, np.asarray(scan.ranges, dtype=np.float64),
            scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)
        elapsed = time.perf_counter() - started

        self._scans_integrated += 1
        self._integration_seconds += elapsed
        self._slowest_integration = max(self._slowest_integration, elapsed)
        if self._scans_integrated % 100 == 0:
            # Debug messages are hidden by default. Show them with
            #   --ros-args --log-level mapper:=debug
            mean = self._integration_seconds / self._scans_integrated
            self.get_logger().debug(
                f'{self._scans_integrated} scans integrated, mean {mean * 1e3:.1f} ms, '
                f'slowest {self._slowest_integration * 1e3:.1f} ms')

    def _publish_map(self) -> None:
        geometry = self._grid.geometry
        message = OccupancyGridMsg()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._map_frame
        message.info.map_load_time = message.header.stamp
        message.info.resolution = geometry.resolution
        message.info.width = geometry.width
        message.info.height = geometry.height
        message.info.origin.position.x = geometry.origin_x
        message.info.origin.position.y = geometry.origin_y
        message.info.origin.orientation.w = 1.0
        # The message wants a flat sequence, row after row: data[row * width + col].
        # That is exactly numpy's own memory order, so the raw bytes can be handed over
        # as they are. rclpy stores an int8[] field as array.array('b'), and building one
        # from bytes is much faster than from a Python list of 60 000 numbers.
        message.data = array.array('b', self._grid.to_occupancy_data().tobytes())
        self._map_publisher.publish(message)

    def _on_clear_map(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._grid.clear()
        self._pending_scans.clear()
        self._publish_map()
        response.success = True
        response.message = 'Map cleared'
        self.get_logger().info('Map cleared on request')
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Mapper()
    try:
        # spin() hands the thread to an executor: it waits for messages, timers and
        # service calls, and runs the matching callback, one after the other, until
        # the node is shut down.
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
