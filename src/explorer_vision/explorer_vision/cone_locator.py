"""Place a detected cone on the floor plan by fusing the camera and the lidar.

Neither sensor can do this alone:

* The camera knows WHICH direction the cone is in (the pixel column gives a
  bearing) and that it IS a cone, but one image gives no reliable distance.
* The lidar measures distances to within centimetres, but a scan is only a
  ring of ranges: it cannot tell a cone from a table leg.

So the camera chooses the bearing and the lidar supplies the range. All
coordinates here are in the robot's frame `base_link`: x forward, y left,
metres. The lidar sits on the origin, the camera 0.18 m ahead of it.

No ROS in this module, only numpy.
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """The pinhole camera model: focal lengths and principal point, in pixels."""

    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_k(cls, k: Sequence[float]) -> 'CameraIntrinsics':
        """Build from the 9 numbers of CameraInfo.k, K = [fx 0 cx; 0 fy cy; 0 0 1]."""
        return cls(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]))


@dataclass(frozen=True)
class Scan:
    """The part of a sensor_msgs/LaserScan that the locator needs.

    Beam i points at `angle_min + i * angle_increment`, measured from the
    robot's x axis, counter-clockwise. A beam that hit nothing holds +inf.
    """

    angle_min: float
    angle_increment: float
    ranges: np.ndarray
    range_min: float
    range_max: float


@dataclass(frozen=True)
class ConeModel:
    """The size of the cones we are looking for, in metres."""

    height: float = 0.70
    base_radius: float = 0.20

    def radius_at(self, z: float) -> float:
        """Return the cone's radius at height z above the floor.

        The lidar does not scan along the floor but at its mounting height,
        where the cone is already thinner than at its base.
        """
        fraction_left = max(0.0, 1.0 - z / self.height)
        return self.base_radius * fraction_left


@dataclass(frozen=True)
class ConePosition:
    """Where the cone's axis meets the floor, in `base_link`."""

    x: float
    y: float


def pixel_column_to_bearing(u: float, intrinsics: CameraIntrinsics) -> float:
    """Return the horizontal direction a pixel column looks in, in radians.

    The camera looks along the robot's +x. The centre column `cx` therefore
    sees the bearing 0. A column to the LEFT of the centre (u < cx) sees
    something to the robot's left, and left is positive y in ROS, so the
    bearing is positive:

        bearing = atan2(cx - u, fx)

    The image's u axis grows to the right while the robot's y axis grows to
    the left, hence `cx - u` and not `u - cx`.
    """
    return math.atan2(intrinsics.cx - u, intrinsics.fx)


def distance_from_pixel_height(
        height_px: float, intrinsics: CameraIntrinsics, cone_height: float) -> float:
    """Estimate the distance to a cone from how tall it looks.

    Similar triangles in the pinhole model: an object of height H at distance
    d covers h = fy * H / d pixels, so d = fy * H / h.

    This is a rough estimate (one pixel of error in h matters a lot for a
    distant cone), which is why it is only used to CHECK the lidar.
    """
    return intrinsics.fy * cone_height / height_px


def scan_to_points(scan: Scan) -> np.ndarray:
    """Turn the valid lidar returns into points, one row (x, y) per return.

    Polar to Cartesian: x = r cos(angle), y = r sin(angle). Beams that hit
    nothing (+inf) or report a range outside the sensor's limits are dropped.
    """
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    return np.column_stack((ranges[valid] * np.cos(angles[valid]),
                            ranges[valid] * np.sin(angles[valid])))


def locate_cone(
        box_left: float,
        box_right: float,
        box_height: float,
        intrinsics: CameraIntrinsics,
        scan: Scan,
        cone: ConeModel = ConeModel(),
        camera_offset_x: float = 0.18,
        lidar_height: float = 0.30,
        max_range: float = 5.0,
        distance_tolerance: float = 0.30,
        cluster_gap: float = 0.15,
        min_cluster_points: int = 2) -> Optional[ConePosition]:
    """Return the cone's position in `base_link`, or None if it cannot be trusted.

    box_left, box_right: the pixel columns of the bounding box's two sides.
    box_height: the height of the bounding box in pixels.
    camera_offset_x: how far the camera sits ahead of the lidar.
    lidar_height: height of the scan plane above the floor.
    max_range: cones that look farther away than this are not localised.
    distance_tolerance: how much the lidar and the camera may disagree about
        the distance, as a fraction of the camera's estimate.
    cluster_gap: returns whose distances differ by less than this belong to
        the same object.
    min_cluster_points: a single return is too easily a stray, ask for more.
    """
    # Step 1, camera only: how far away does the cone LOOK?
    camera_distance = distance_from_pixel_height(box_height, intrinsics, cone.height)
    if camera_distance > max_range:
        return None

    # Step 2: the lidar returns as points, and their position relative to the
    # camera. The camera is not where the lidar is, so the two sensors see the
    # same point under different bearings (parallax). Subtracting the camera's
    # position first makes the comparison with pixel bearings exact.
    points = scan_to_points(scan)
    forward_of_camera = points[:, 0] - camera_offset_x
    left_of_camera = points[:, 1]
    bearings = np.arctan2(left_of_camera, forward_of_camera)

    # Step 3: keep the returns that lie inside the bounding box, seen from the
    # camera. The left edge of the box has the larger bearing (see
    # pixel_column_to_bearing).
    bearing_left = pixel_column_to_bearing(box_left, intrinsics)
    bearing_right = pixel_column_to_bearing(box_right, intrinsics)
    inside_box = ((forward_of_camera > 0.0)
                  & (bearings <= bearing_left) & (bearings >= bearing_right))
    candidates = points[inside_box]
    if len(candidates) == 0:
        return None

    # Step 4: the nearest cluster. The box is as wide as the cone's base, so
    # some beams inside it pass beside the cone and hit whatever stands
    # behind. Sorted by distance, the cone's returns come first, then a jump.
    distances = np.hypot(candidates[:, 0], candidates[:, 1])
    order = np.argsort(distances)
    candidates = candidates[order]
    distances = distances[order]
    jumps = np.flatnonzero(np.diff(distances) > cluster_gap)
    cluster_size = int(jumps[0]) + 1 if len(jumps) > 0 else len(distances)
    if cluster_size < min_cluster_points:
        return None
    cluster = candidates[:cluster_size]

    # Step 5: from the cluster to the cone's axis. The lidar sees the FRONT of
    # the cone. The axis lies one cone radius farther along the same ray,
    # where the radius is taken at the height of the scan plane.
    centroid = cluster.mean(axis=0)
    direction = math.atan2(centroid[1], centroid[0])
    axis_distance = float(distances[0]) + cone.radius_at(lidar_height)
    position = ConePosition(x=axis_distance * math.cos(direction),
                            y=axis_distance * math.sin(direction))

    # Step 6, the sanity gate: do the two sensors agree? If the cone was
    # hidden from the lidar, or the detector fired on something that is not a
    # cone, the nearest cluster is some other object (often the wall behind)
    # and its distance does not fit the cone's apparent size.
    #
    # The pinhole formula measures distance along the optical axis, which is
    # the robot's x axis, so that is what the lidar result is compared on.
    lidar_distance = position.x - camera_offset_x
    if abs(lidar_distance - camera_distance) > distance_tolerance * camera_distance:
        return None
    return position
