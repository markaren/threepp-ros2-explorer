# The contract between the nodes

Six programs make up the demo. They never call each other's code. Everything they know
about each other is on this page: the names and types of the topics, services and actions,
and the coordinate frames. That is the point of ROS: if two nodes agree on this page, one
can be written in C++ and the other in Python, by different people, and they still fit.

All names are *relative* (no leading slash), so the whole robot can be pushed into a
namespace later (`/robot1/scan`, `/robot2/scan`) without touching the code.

## Nodes

| Node | Package | Language | Role |
|---|---|---|---|
| `sim` | `explorer_sim` | C++ / threepp | The world and the robot hardware: moves the robot, casts the lidar, draws the truth |
| `safety_monitor` | `explorer_safety` | C++ | Reflex layer: stops forward motion when something is too close |
| `mapper` | `explorer_nav` | Python / numpy | Turns laser scans into an occupancy grid |
| `explorer` | `explorer_nav` | Python / numpy | Picks frontiers, plans paths, follows them |
| `cone_spotter` | `explorer_vision` | Python / OpenCV | Finds traffic cones in the camera image and places them on the map |
| `dashboard` | `explorer_dashboard` | Python / threepp | The operator's 3D view of what the robot *believes*, and the buttons |

## Coordinate frames (tf2)

ROS convention (REP 103): **x forward, y left, z up**, metres, radians, right-handed.
Yaw is the rotation about z, counter-clockwise seen from above.

```
map ──► odom ──► base_link ──► laser
                          └──► camera_link
```

| Transform | Published by | Kind | Meaning |
|---|---|---|---|
| `map -> odom` | `mapper` | static, identity | There is no localisation in this demo, so the map frame and the odometry frame coincide. A SLAM system would move this transform to correct odometry drift. |
| `odom -> base_link` | `sim` | dynamic, 30 Hz | Where the robot thinks it is, from wheel odometry |
| `base_link -> laser` | `sim` | static | Lidar mount: 0.30 m above the base, no rotation |
| `base_link -> camera_link` | `sim` | static | Camera mount: 0.18 m forward, 0.25 m up |

The robot starts at the origin of `odom`, facing +x.

threepp, like three.js, has **y up**. Both threepp programs convert at their edges with the
same rule: `three = (ros.x, ros.z, -ros.y)`. A yaw of θ about ROS z is a rotation of θ about
threepp y.

## Topics

| Topic | Type | Publisher | Subscribers | Rate | QoS |
|---|---|---|---|---|---|
| `scan` | `sensor_msgs/LaserScan` | `sim` | `mapper`, `safety_monitor`, `dashboard` | 10 Hz | sensor data (best effort, depth 5) |
| `odom` | `nav_msgs/Odometry` | `sim` | anyone | 30 Hz | reliable, depth 10 |
| `tf`, `tf_static` | `tf2_msgs/TFMessage` | `sim`, `mapper` | all | | tf2 defaults |
| `camera/image/compressed` | `sensor_msgs/CompressedImage` (`jpeg`) | `sim` | `cone_spotter`, `rqt_image_view` | 10 Hz | sensor data |
| `camera/camera_info` | `sensor_msgs/CameraInfo` | `sim` | `cone_spotter` | 10 Hz | sensor data |
| `detections` | `vision_msgs/Detection2DArray` | `cone_spotter` | anyone | per image | reliable, depth 10 |
| `cones` | `visualization_msgs/MarkerArray` | `cone_spotter` | `dashboard`, RViz | 2 Hz | reliable, transient local, depth 1 |
| `ground_truth` | `geometry_msgs/PoseStamped` | `sim` | for comparing against `odom` | 30 Hz | reliable, depth 10 |
| `cmd_vel_nav` | `geometry_msgs/Twist` | `explorer` | `safety_monitor` | 20 Hz | reliable, depth 10 |
| `cmd_vel` | `geometry_msgs/Twist` | `safety_monitor` | `sim` | 20 Hz | reliable, depth 10 |
| `safety_stop` | `std_msgs/Bool` | `safety_monitor` | `dashboard` | on change | reliable, transient local, depth 1 |
| `map` | `nav_msgs/OccupancyGrid` | `mapper` | `explorer`, `dashboard`, RViz | 2 Hz | reliable, transient local, depth 1 |
| `plan` | `nav_msgs/Path` | `explorer` | `dashboard`, RViz | on replan | reliable, depth 1 |
| `frontiers` | `visualization_msgs/MarkerArray` | `explorer` | `dashboard`, RViz | on replan | reliable, depth 1 |
| `exploration_status` | `explorer_interfaces/ExplorationStatus` | `explorer` | `dashboard` | 2 Hz | reliable, depth 10 |
| `goal_pose` | `geometry_msgs/PoseStamped` | `dashboard`, RViz "2D Goal Pose" | `explorer` | on click | reliable, depth 1 |

Details that matter:

* **`scan`**: 360 beams, `angle_min = -pi`, `angle_increment = 2*pi/360`, so beam `i` points at
  `-pi + i * increment` in the `laser` frame (beam 180 looks straight ahead). `range_min = 0.12`,
  `range_max = 8.0`. A beam that hits nothing reports `+inf`. `header.frame_id = "laser"`.
  The stamp of every scan is *identical* to the stamp of an `odom -> base_link` transform, so a
  tf2 lookup at the scan's stamp needs no interpolation.
* **`camera/image/compressed`**: 640 x 360 JPEG, `header.frame_id = "camera_link"`. The stamp is
  the moment the frame was *rendered*, not the moment it was published. `camera/camera_info`
  carries the same stamp and the pinhole intrinsics `K = [fx 0 cx; 0 fy cy; 0 0 1]`, so nobody
  has to hard-code a field of view. The camera looks along the robot's +x. A pixel column `u`
  therefore sees the bearing `atan2(cx - u, fx)` (positive = to the left) in `base_link`.
* **`detections`**: one `Detection2D` per cone seen, `class_id = "cone"`, bounding box in pixels,
  stamped with the stamp of the image it came from (a detection describes the world as it was
  when the frame was taken).
* **`cones`**: every distinct cone found so far, in frame `map`: one `CYLINDER` marker each,
  namespace `"cones"`, ids stable (cone 0 stays id 0). A `DELETEALL` marker comes first.
* **`cmd_vel`**: only `linear.x` (m/s) and `angular.z` (rad/s) are used. The sim limits them to
  0.8 m/s and 2.0 rad/s, limits acceleration, and **stops the robot if no command arrives for
  0.5 s**. A controller must therefore keep publishing, exactly as on a real robot.
* **`map`**: `header.frame_id = "map"`, values `-1` unknown, `0..100` occupancy probability in
  percent. `data[row * width + col]`, where the cell centre is at
  `x = origin.x + (col + 0.5) * resolution`, `y = origin.y + (row + 0.5) * resolution`.
* **`frontiers`**: one `SPHERE` marker per frontier cluster at its centroid, namespace
  `"frontiers"`, in frame `map`. The chosen target is a marker in namespace `"target"`.
  A `DELETEALL` marker comes first in every message.

## Services

| Service | Type | Server | Effect |
|---|---|---|---|
| `reset_world` | `std_srvs/Trigger` | `sim` | Robot back to the start pose, dropped obstacles removed |
| `spawn_obstacle` | `explorer_interfaces/SpawnObstacle` | `sim` | Drops a crate at (x, y) in the `odom` frame |
| `clear_map` | `std_srvs/Trigger` | `mapper` | Forgets the map |
| `sim/screenshot`, `dashboard/screenshot` | `std_srvs/Trigger` | `sim`, `dashboard` | Writes the current frame to a PNG, answers with its path |

## Action

`explore` (`explorer_interfaces/action/Explore`), served by `explorer`.

An action is the right tool here, and a service is not: exploring takes minutes, the caller
wants progress reports while it runs, and must be able to cancel it.

* **Goal**: `time_limit` (seconds, 0 = none), `return_home` (drive back to the start when done).
* **Feedback**: an `ExplorationStatus`, about twice a second.
* **Result**: `success`, `message`, `explored_area` (m²), `distance` (m), `duration` (s).
* **Cancel**: accepted at any time. The robot stops.

## Custom interface definitions

They live in `src/explorer_interfaces`. See the `.msg`, `.srv` and `.action` files there.
