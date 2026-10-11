"""A stand-in for the simulator: a 2D warehouse, a robot and a lidar, in numpy.

Test tooling, not part of the installed package. It publishes what the contract
(docs/interfaces.md) says the simulator publishes (scan, odom, tf, tf_static) and listens
to cmd_vel_nav directly, without the safety node in between. That is enough to run the
mapper and the explorer end to end, with no graphics and no C++:

    python test/fake_world.py
    python -m explorer_nav.mapper_node
    python -m explorer_nav.explorer_node
    ros2 action send_goal explore explorer_interfaces/action/Explore "{return_home: true}"

The world is made of line segments (walls, shelves, crates) and circles (traffic cones).
The lidar is a bundle of rays intersected with all of them at once.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

BEAMS = 360
RANGE_MIN = 0.12
RANGE_MAX = 8.0
ROBOT_RADIUS = 0.22
TICK_RATE = 30.0                # Hz: odometry and tf
TICKS_PER_SCAN = 3              # the lidar runs at 10 Hz
COMMAND_TIMEOUT = 0.5           # s without a command before the robot stops
MAX_LINEAR_SPEED = 0.8          # m/s
MAX_ANGULAR_SPEED = 2.0         # rad/s
MAX_LINEAR_ACCELERATION = 1.5   # m/s^2
MAX_ANGULAR_ACCELERATION = 6.0  # rad/s^2
SAFETY_DISTANCE = 0.30          # m: the real safety node zeroes forward speed below this
SAFETY_HALF_ANGLE = 0.6         # rad: what counts as "in front" here


def box(x_min: float, y_min: float, x_max: float, y_max: float) -> list[tuple[float, ...]]:
    """The four sides of an axis-aligned rectangle, as segments (x1, y1, x2, y2)."""
    return [(x_min, y_min, x_max, y_min), (x_max, y_min, x_max, y_max),
            (x_max, y_max, x_min, y_max), (x_min, y_max, x_min, y_min)]


def build_warehouse() -> tuple[np.ndarray, np.ndarray]:
    """The test world: 26 x 16 m, x in [-2, 24], y in [-8, 8], the robot starts at (0, 0).

    Returns (segments, circles): an (N, 4) array of x1, y1, x2, y2 and an (M, 3) array of
    centre x, centre y, radius.
    """
    segments = box(-2.0, -8.0, 24.0, 8.0)                   # the outer walls
    for y_min in (-5.5, -2.3, 1.5, 4.7):                    # four rows of shelves, 0.8 m deep
        segments += box(3.0, y_min, 11.0, y_min + 0.8)
        segments += box(13.0, y_min, 21.0, y_min + 0.8)
    for x, y in ((6.0, 0.9), (16.0, -3.6), (22.6, 6.4), (0.5, -6.5)):   # crates, 0.6 m
        segments += box(x - 0.3, y - 0.3, x + 0.3, y + 0.3)
    circles = [(9.0, -0.7, 0.2), (12.0, 3.4, 0.2), (18.0, 0.5, 0.2), (1.5, 6.5, 0.2)]  # cones
    return np.array(segments), np.array(circles)


def cast_rays(x: float, y: float, angles: np.ndarray, segments: np.ndarray,
              circles: np.ndarray) -> np.ndarray:
    """Distance from (x, y) to the first thing each ray hits, +inf if it hits nothing.

    A ray is o + t * d. A segment is p + u * s with u in [0, 1]. Setting them equal and
    solving with the 2D cross product gives t and u for every ray-segment pair at once:
    the arrays below have one row per ray and one column per segment.
    """
    direction_x = np.cos(angles)[:, np.newaxis]
    direction_y = np.sin(angles)[:, np.newaxis]

    start_x = segments[:, 0] - x
    start_y = segments[:, 1] - y
    side_x = segments[:, 2] - segments[:, 0]
    side_y = segments[:, 3] - segments[:, 1]
    denominator = direction_x * side_y - direction_y * side_x
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (start_x * side_y - start_y * side_x) / denominator
        u = (start_x * direction_y - start_y * direction_x) / denominator
    t[~((t > 0.0) & (u >= 0.0) & (u <= 1.0))] = np.inf
    nearest = t.min(axis=1)

    # Circles: |o + t * d - c|^2 = r^2 is a quadratic equation in t.
    centre_x = circles[:, 0] - x
    centre_y = circles[:, 1] - y
    along = direction_x * centre_x + direction_y * centre_y
    discriminant = along ** 2 - (centre_x ** 2 + centre_y ** 2 - circles[:, 2] ** 2)
    with np.errstate(invalid='ignore'):
        t = along - np.sqrt(discriminant)
    t[~((discriminant >= 0.0) & (t > 0.0))] = np.inf
    return np.minimum(nearest, t.min(axis=1))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def approach(current: float, target: float, max_change: float) -> float:
    """Move current towards target, by at most max_change."""
    return current + min(max(target - current, -max_change), max_change)


class FakeWorld(Node):

    def __init__(self) -> None:
        super().__init__('sim')
        self._range_noise = self.declare_parameter('range_noise', 0.01).value
        self._emulate_safety = self.declare_parameter('emulate_safety', True).value
        self._segments, self._circles = build_warehouse()
        self._random = np.random.default_rng(1)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._linear = 0.0
        self._angular = 0.0
        self._commanded_linear = 0.0
        self._commanded_angular = 0.0
        self._last_command_time = -math.inf
        self._tick_count = 0

        # Numbers for judging a run.
        self._closest_approach = math.inf
        self._collisions = 0
        self._safety_ticks = 0
        self._distance = 0.0

        self._scan_publisher = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self._odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._command_subscription = self.create_subscription(
            Twist, 'cmd_vel_nav', self._on_command, 10)

        self._publish_static_transforms()
        self._tick_timer = self.create_timer(1.0 / TICK_RATE, self._tick)
        self._report_timer = self.create_timer(5.0, self._report)
        self.get_logger().info('Fake world running: robot at (0, 0), facing +x')

    def _seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_static_transforms(self) -> None:
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for child, x, z in (('laser', 0.0, 0.30), ('camera_link', 0.18, 0.25)):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = 'base_link'
            transform.child_frame_id = child
            transform.transform.translation.x = x
            transform.transform.translation.z = z
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self._static_broadcaster.sendTransform(transforms)

    def _on_command(self, command: Twist) -> None:
        self._commanded_linear = min(max(command.linear.x, -MAX_LINEAR_SPEED), MAX_LINEAR_SPEED)
        self._commanded_angular = min(max(command.angular.z, -MAX_ANGULAR_SPEED),
                                      MAX_ANGULAR_SPEED)
        self._last_command_time = self._seconds()

    def _clearance(self, x: float, y: float) -> float:
        """Distance from the point (x, y) to the nearest obstacle surface."""
        angles = np.linspace(-math.pi, math.pi, 180, endpoint=False)
        return float(cast_rays(x, y, angles, self._segments, self._circles).min())

    def _tick(self) -> None:
        time_step = 1.0 / TICK_RATE
        target_linear = self._commanded_linear
        target_angular = self._commanded_angular
        if self._seconds() - self._last_command_time > COMMAND_TIMEOUT:
            target_linear = 0.0     # the watchdog: no fresh command, no motion
            target_angular = 0.0

        if self._emulate_safety and target_linear > 0.0:
            front = self._yaw + np.linspace(-SAFETY_HALF_ANGLE, SAFETY_HALF_ANGLE, 31)
            ahead = cast_rays(self._x, self._y, front, self._segments, self._circles).min()
            if ahead < SAFETY_DISTANCE:
                target_linear = 0.0
                self._safety_ticks += 1

        self._linear = approach(self._linear, target_linear, MAX_LINEAR_ACCELERATION * time_step)
        self._angular = approach(self._angular, target_angular,
                                 MAX_ANGULAR_ACCELERATION * time_step)

        new_x = self._x + self._linear * math.cos(self._yaw) * time_step
        new_y = self._y + self._linear * math.sin(self._yaw) * time_step
        clearance = self._clearance(new_x, new_y)
        if clearance < ROBOT_RADIUS:
            self._collisions += 1       # the robot body would be inside an obstacle
            self._linear = 0.0
        else:
            self._distance += math.hypot(new_x - self._x, new_y - self._y)
            self._x = new_x
            self._y = new_y
            self._closest_approach = min(self._closest_approach, clearance)
        self._yaw += self._angular * time_step

        # One stamp for everything published in this tick. The contract promises that
        # every scan carries the stamp of an odom -> base_link transform.
        stamp = self.get_clock().now().to_msg()
        self._publish_odometry(stamp)
        if self._tick_count % TICKS_PER_SCAN == 0:
            self._publish_scan(stamp)
        self._tick_count += 1

    def _publish_odometry(self, stamp) -> None:
        quaternion = quaternion_from_yaw(self._yaw)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self._tf_broadcaster.sendTransform(transform)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_link'
        odometry.pose.pose.position.x = self._x
        odometry.pose.pose.position.y = self._y
        odometry.pose.pose.orientation.z = quaternion[2]
        odometry.pose.pose.orientation.w = quaternion[3]
        odometry.twist.twist.linear.x = self._linear
        odometry.twist.twist.angular.z = self._angular
        self._odom_publisher.publish(odometry)

    def _publish_scan(self, stamp) -> None:
        increment = 2.0 * math.pi / BEAMS
        angles = self._yaw - math.pi + increment * np.arange(BEAMS)
        ranges = cast_rays(self._x, self._y, angles, self._segments, self._circles)
        ranges += self._random.normal(0.0, self._range_noise, BEAMS)
        ranges[ranges >= RANGE_MAX] = np.inf
        ranges[ranges < RANGE_MIN] = RANGE_MIN

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - increment
        scan.angle_increment = increment
        scan.scan_time = TICKS_PER_SCAN / TICK_RATE
        scan.range_min = RANGE_MIN
        scan.range_max = RANGE_MAX
        scan.ranges = ranges.astype(np.float32).tolist()
        self._scan_publisher.publish(scan)

    def _report(self) -> None:
        self.get_logger().info(
            f'robot at ({self._x:.2f}, {self._y:.2f}), yaw {self._yaw:.2f}, '
            f'driven {self._distance:.1f} m, closest approach of the centre '
            f'{self._closest_approach:.2f} m, collisions {self._collisions}, '
            f'safety stop ticks {self._safety_ticks}')


def main() -> None:
    rclpy.init()
    node = FakeWorld()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
