"""The ROS 2 node `cone_spotter`: finds traffic cones and puts them on the map.

This file is only glue. The three algorithms live in their own modules and do
not know that ROS exists:

    cone_detector.py   image            -> bounding boxes
    cone_locator.py    box + lidar scan -> position in front of the robot
    cone_map.py        many sightings   -> a list of distinct cones

What the node adds is everything a real robot needs around them: receiving the
messages, pairing an image with the scan taken at the same instant, asking tf2
where the robot was at that instant, and publishing the results.
"""

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

from explorer_vision.cone_detector import Detection, DetectorThresholds
from explorer_vision.cone_detector import detect_cones, draw_detections
from explorer_vision.cone_locator import CameraIntrinsics, ConeModel, ConePosition, Scan
from explorer_vision.cone_locator import locate_cone
from explorer_vision.cone_map import ConeMap

# An image and a scan belong together when their stamps differ by less than
# this. The simulator gives both the identical stamp, the margin only absorbs
# rounding.
PAIRING_TOLERANCE_NS = 20_000_000

# How many scans to remember. At 10 Hz this is one second, far more than the
# time an image needs to arrive after its scan.
SCAN_HISTORY = 10

# How long to wait for the transform that belongs to an image.
TRANSFORM_TIMEOUT_S = 0.1


def stamp_to_ns(stamp: TimeMsg) -> int:
    """Turn a message stamp (seconds + nanoseconds) into one integer."""
    return Time.from_msg(stamp).nanoseconds


def base_link_to_map(transform: TransformStamped, position: ConePosition) -> Tuple[float, float]:
    """Move a point from the robot's frame to the map frame.

    The robot drives on a flat floor, so of the full 3D transform only the
    translation in x and y and the rotation about z (the yaw) matter:

        x_map = t.x + cos(yaw) * x - sin(yaw) * y
        y_map = t.y + sin(yaw) * x + cos(yaw) * y

    tf2 stores rotations as quaternions. For a rotation about z alone the
    quaternion is (0, 0, sin(yaw / 2), cos(yaw / 2)), the general formula
    below also holds when x and y are not exactly zero.
    """
    q = transform.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    t = transform.transform.translation
    x_map = t.x + math.cos(yaw) * position.x - math.sin(yaw) * position.y
    y_map = t.y + math.sin(yaw) * position.x + math.cos(yaw) * position.y
    return x_map, y_map


@dataclass(frozen=True)
class Frame:
    """One camera image after the detector has looked at it."""

    header: Header
    image: np.ndarray
    detections: List[Detection]


class ConeSpotter(Node):
    """Camera + lidar in, cones on the map out."""

    def __init__(self) -> None:
        super().__init__('cone_spotter')

        # Parameters are a node's settings, visible and changeable from the
        # outside: `ros2 param list`, `ros2 param set`, or `-p name:=value` on
        # the command line. The description shows up in `ros2 param describe`.
        defaults = DetectorThresholds()
        self._declare('hue_min', defaults.hue_min,
                      'Lower hue limit of "orange", OpenCV scale 0..179.')
        self._declare('hue_max', defaults.hue_max,
                      'Upper hue limit of "orange", OpenCV scale 0..179.')
        self._declare('saturation_min', defaults.saturation_min,
                      'Minimum saturation (0..255). Lower it to accept paler orange.')
        self._declare('value_min', defaults.value_min,
                      'Minimum brightness (0..255). Lower it to accept cones in shadow.')
        self._declare('min_blob_area', defaults.min_area,
                      'Orange blobs with fewer pixels than this are ignored.')
        self._declare('cone_height', 0.70, 'Real height of a cone, metres.')
        self._declare('cone_base_radius', 0.20, 'Real radius of a cone at the floor, metres.')
        self._declare('merge_radius', 0.6,
                      'Sightings closer than this to a known cone are the same cone, metres.')
        self._declare('confirmations', 3,
                      'Sightings needed before a cone is published on "cones".')
        self._declare('max_range', 5.0,
                      'Cones that look farther away than this are not localised, metres.')
        self._declare('distance_tolerance', 0.30,
                      'Allowed disagreement between lidar distance and the distance '
                      'estimated from the cone\'s pixel height, as a fraction.')
        self._declare('camera_offset_x', 0.18,
                      'How far the camera sits ahead of base_link, metres.')
        self._declare('lidar_height', 0.30,
                      'Height of the lidar scan plane above the floor, metres.')

        self._cone = ConeModel(height=self.get_parameter('cone_height').value,
                               base_radius=self.get_parameter('cone_base_radius').value)
        self._confirmations: int = self.get_parameter('confirmations').value
        self._cone_map = ConeMap(merge_radius=self.get_parameter('merge_radius').value,
                                 confirmations=self._confirmations)

        self._intrinsics: Optional[CameraIntrinsics] = None
        # Recent scans as (stamp in nanoseconds, scan). A deque with maxlen
        # drops its oldest entry by itself when a new one is appended.
        self._scans: Deque[Tuple[int, Scan]] = deque(maxlen=SCAN_HISTORY)
        # The image that arrived before its scan, if any. See _on_image.
        self._waiting: Optional[Frame] = None

        # tf2 keeps a short history of every coordinate frame transform that is
        # published on /tf. The buffer stores them, the listener is the pair of
        # subscriptions that fills the buffer.
        #
        # We ask the buffer from inside the image callback, and may have to
        # WAIT there for a transform that is still on its way. While a callback
        # waits, something else must keep receiving tf messages, or the wait
        # can never end. Two things make that work:
        #
        # * The listener puts its subscriptions in their own callback group.
        #   All callbacks of THIS class share the node's default group, which
        #   is mutually exclusive: they never run at the same time, so the
        #   members above need no lock.
        # * main() spins the node with a MultiThreadedExecutor. One of its
        #   threads can sit in our callback while another one delivers tf.
        #
        # TransformListener(..., spin_thread=True) looks like the shorter
        # answer, but in rclpy it hands the WHOLE node to a private executor,
        # and a node can only belong to one executor: spinning it in main()
        # takes it away again. Either way all callbacks end up on a single
        # thread, and a callback that waits for tf blocks the tf callbacks.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # QoS (quality of service) says how hard the middleware tries to
        # deliver. Sensor data: "best effort", a short queue. A lost camera
        # frame is not worth resending, the next one is already on its way.
        # A subscriber must not ask for more than the publisher offers, so we
        # use the same profile as the simulator.
        self.create_subscription(
            CompressedImage, 'camera/image/compressed', self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, 'camera/camera_info', self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)

        # Results are worth more than raw data: "reliable" resends what got
        # lost. The number is the queue depth.
        self._detections_pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self._annotated_pub = self.create_publisher(
            CompressedImage, 'camera/annotated/compressed', qos_profile_sensor_data)

        # "Transient local" makes the publisher keep its last message and hand
        # it to every subscriber that connects later. A dashboard started after
        # the robot has finished still gets the full list of cones.
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._cones_pub = self.create_publisher(MarkerArray, 'cones', latched)
        self.create_timer(0.5, self._publish_cones)

        self.get_logger().info('cone_spotter is waiting for camera images')

    def _declare(self, name: str, default, description: str) -> None:
        self.declare_parameter(name, default, ParameterDescriptor(description=description))

    def _thresholds(self) -> DetectorThresholds:
        """Read the detector's settings from the parameters.

        This is done for every image, so `ros2 param set /cone_spotter
        saturation_min 60` shows its effect in the next annotated frame.
        """
        return DetectorThresholds(
            hue_min=self.get_parameter('hue_min').value,
            hue_max=self.get_parameter('hue_max').value,
            saturation_min=self.get_parameter('saturation_min').value,
            value_min=self.get_parameter('value_min').value,
            min_area=self.get_parameter('min_blob_area').value)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._intrinsics = CameraIntrinsics.from_k(msg.k)

    def _on_scan(self, msg: LaserScan) -> None:
        scan = Scan(angle_min=msg.angle_min,
                    angle_increment=msg.angle_increment,
                    ranges=np.asarray(msg.ranges, dtype=np.float64),
                    range_min=msg.range_min,
                    range_max=msg.range_max)
        scan_ns = stamp_to_ns(msg.header.stamp)
        self._scans.append((scan_ns, scan))

        # Was an image waiting for exactly this scan? Then it can be finished.
        frame = self._waiting
        if frame is not None and self._same_instant(scan_ns, stamp_to_ns(frame.header.stamp)):
            self._waiting = None
            self._finish(frame, scan)

    def _on_image(self, msg: CompressedImage) -> None:
        # The message carries a JPEG file in memory. imdecode does what imread
        # does for a file on disk, and returns a BGR image.
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warn('could not decode the camera image',
                                   throttle_duration_sec=5.0)
            return

        detections = detect_cones(image, self._thresholds())
        self._publish_detections(msg.header, detections)
        frame = Frame(msg.header, image, detections)

        # The previous image is still waiting: its scan was lost on the way
        # (best effort!). Finish it without a scan, so nothing piles up.
        if self._waiting is not None:
            self._finish(self._waiting, None)
            self._waiting = None

        # The image and its scan are sent in the same instant, on two topics,
        # and ROS does not promise which one is delivered first. So both
        # orders are handled: the scan is already here (the usual case), or
        # the image waits until _on_scan sees it. An image without cones has
        # nothing to localise and never waits.
        scan = self._scan_at(msg.header.stamp)
        if scan is None and detections:
            self._waiting = frame
        else:
            self._finish(frame, scan)

    def _finish(self, frame: Frame, scan: Optional[Scan]) -> None:
        """Localise the cones of one image, show them, and add them to the map."""
        positions = self._locate(frame.detections, scan)
        self._publish_annotated(frame.header, frame.image, frame.detections, positions)
        self._remember(frame.header.stamp, positions)

    def _locate(
            self, detections: List[Detection],
            scan: Optional[Scan]) -> List[Optional[ConePosition]]:
        """Fuse every detection with the lidar scan taken at the same instant.

        Returns one entry per detection: a position in `base_link`, or None.
        """
        nothing: List[Optional[ConePosition]] = [None] * len(detections)
        if not detections:
            return nothing
        if self._intrinsics is None:
            self.get_logger().warn('no camera_info yet, cannot localise',
                                   throttle_duration_sec=5.0)
            return nothing
        if scan is None:
            self.get_logger().warn('no scan with the stamp of this image, cannot localise',
                                   throttle_duration_sec=5.0)
            return nothing

        positions: List[Optional[ConePosition]] = []
        for detection in detections:
            if detection.clipped:
                # Part of the cone is outside the image: the box is too short
                # for the distance check and off-centre for the bearing. The
                # robot moves, a later frame will show the whole cone.
                positions.append(None)
                continue
            positions.append(locate_cone(
                detection.left, detection.right, detection.height,
                self._intrinsics, scan,
                cone=self._cone,
                camera_offset_x=self.get_parameter('camera_offset_x').value,
                lidar_height=self.get_parameter('lidar_height').value,
                max_range=self.get_parameter('max_range').value,
                distance_tolerance=self.get_parameter('distance_tolerance').value))
        return positions

    def _scan_at(self, stamp: TimeMsg) -> Optional[Scan]:
        """Return the remembered scan whose stamp matches, or None.

        Every message header carries the time its data was MEASURED. Messages
        of different topics arrive in no guaranteed order and with different
        delays, so "the latest scan" is not good enough: the robot may have
        turned in between. Matching stamps is what makes fusion correct.
        """
        wanted_ns = stamp_to_ns(stamp)
        for scan_ns, scan in reversed(self._scans):
            if self._same_instant(scan_ns, wanted_ns):
                return scan
        return None

    @staticmethod
    def _same_instant(first_ns: int, second_ns: int) -> bool:
        return abs(first_ns - second_ns) <= PAIRING_TOLERANCE_NS

    def _remember(self, stamp: TimeMsg, positions: List[Optional[ConePosition]]) -> None:
        """Move the localised cones into the map frame and store them."""
        located = [position for position in positions if position is not None]
        if not located:
            return

        # "Where was base_link, expressed in map, when the image was taken?"
        # Asking for the image's stamp and not for "now" matters: the robot
        # has moved on while the image was compressed, sent and processed.
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_link', Time.from_msg(stamp),
                timeout=Duration(seconds=TRANSFORM_TIMEOUT_S))
        except TransformException as error:
            self.get_logger().warn(f'no transform map <- base_link: {error}',
                                   throttle_duration_sec=5.0)
            return

        for position in located:
            x_map, y_map = base_link_to_map(transform, position)
            cone = self._cone_map.add_sighting(x_map, y_map)
            if cone.sightings == self._confirmations:
                self.get_logger().info(
                    f'cone {cone.id} confirmed at ({cone.x:.2f}, {cone.y:.2f}) in map')

    def _publish_detections(self, header: Header, detections: List[Detection]) -> None:
        """Publish the boxes in the standard message type for 2D detections.

        The header is copied from the image: a detection describes the world
        as it was when the frame was taken, in the frame of that camera.
        """
        array = Detection2DArray()
        array.header = header
        for detection in detections:
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'cone'
            hypothesis.hypothesis.score = detection.score

            detection_msg = Detection2D()
            detection_msg.header = header
            detection_msg.results.append(hypothesis)
            detection_msg.bbox.center.position.x = detection.centre_x
            detection_msg.bbox.center.position.y = detection.centre_y
            detection_msg.bbox.size_x = float(detection.width)
            detection_msg.bbox.size_y = float(detection.height)
            array.detections.append(detection_msg)
        self._detections_pub.publish(array)

    def _publish_annotated(
            self, header: Header, image: np.ndarray,
            detections: List[Detection],
            positions: List[Optional[ConePosition]]) -> None:
        """Publish the camera image with the boxes drawn, for rqt_image_view."""
        labels = []
        for detection, position in zip(detections, positions):
            if position is not None:
                labels.append(f'cone {math.hypot(position.x, position.y):.1f} m')
            elif detection.clipped:
                labels.append('cone (clipped)')
            else:
                labels.append('cone (no range)')
        annotated = draw_detections(image, detections, labels)

        encoded_ok, jpeg = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not encoded_ok:
            return
        msg = CompressedImage()
        msg.header = header
        msg.format = 'jpeg'
        msg.data = jpeg.tobytes()
        self._annotated_pub.publish(msg)

    def _publish_cones(self) -> None:
        """Publish every confirmed cone as a marker, twice a second."""
        now = self.get_clock().now().to_msg()

        # A MarkerArray only ever adds or changes markers. Starting with
        # DELETEALL makes every message the complete truth: a receiver that
        # shows exactly this message can never be left with stale markers.
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = now
        clear.ns = 'cones'
        clear.action = Marker.DELETEALL

        array = MarkerArray()
        array.markers.append(clear)
        for cone in self._cone_map.confirmed_cones():
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = now
            marker.ns = 'cones'
            marker.id = cone.id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            # A marker's position is its centre, so a cylinder that stands on
            # the floor sits at half its height.
            marker.pose.position.x = cone.x
            marker.pose.position.y = cone.y
            marker.pose.position.z = self._cone.height / 2.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * self._cone.base_radius
            marker.scale.y = 2.0 * self._cone.base_radius
            marker.scale.z = self._cone.height
            marker.color.r = 1.0
            marker.color.g = 0.42
            marker.color.b = 0.0
            marker.color.a = 1.0
            array.markers.append(marker)
        self._cones_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConeSpotter()
    # Two threads: see the comment at the tf2 buffer in ConeSpotter.__init__.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
