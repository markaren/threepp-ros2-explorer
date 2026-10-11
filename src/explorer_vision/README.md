# explorer_vision

The node `cone_spotter` finds the orange traffic cones in the robot's camera image and
puts them on the map. The camera alone cannot do that (one image gives a direction, not
a distance) and the lidar alone cannot either (a scan cannot tell a cone from a table
leg). Together they can. This is the smallest useful example of **sensor fusion**.

```
camera/image/compressed --> [ detect ] --> boxes --+
                                                   +--> [ locate ] --> position in base_link
scan ----------------------------------------------+                        |
                                                                          tf2 (at the image's stamp)
                                                                            |
                                              cones  <-- [ cone map ] <-- position in map
```

## The code is split in two

| File | Knows ROS? | Job |
|---|---|---|
| `explorer_vision/cone_detector.py` | no (OpenCV + numpy) | image in, bounding boxes out |
| `explorer_vision/cone_locator.py` | no (numpy) | box + lidar scan in, position in `base_link` out |
| `explorer_vision/cone_map.py` | no (plain Python) | many sightings in, a list of distinct cones out |
| `explorer_vision/cone_spotter_node.py` | yes | subscriptions, time stamps, tf2, publishers, parameters |

The three algorithm modules can be imported, run and tested without a ROS installation.
Keep that habit in your own nodes: the node file is glue, the thinking lives in plain
functions that a unit test can call.

## Step 1: detect (camera)

1. **Colour.** Convert the image to HSV and keep the pixels with an orange hue and a high
   saturation. Hue is the channel to trust: light and shadow change the brightness of the
   cone a lot, its hue only a little. Note that OpenCV stores hue as degrees / 2
   (0..179). The cone colour RGB (255, 106, 0) has hue 25 degrees, so 12 in OpenCV.
   The band is 3 to 14, and its upper end is what keeps cardboard and wood out: they are
   yellow-brown, hue 17 to 21, and in a shadow as saturated as a cone.
2. **The white band.** A traffic cone has a white reflective band, which cuts the orange
   mask into two blobs: a wide base and a small tip. Neither is cone-shaped, and a box
   around the base alone is too short for the distance check of step 2. So the band is
   added back to the mask. "White" (low saturation, not dark) is not enough for that,
   because the plaster walls are just as white. A white pixel counts only if there is
   orange **above it and below it in the same pixel column**, within `band_reach`
   pixels. That holds for the band (tip above, base below) and for nothing else: the
   wall beside the band has orange below but none above, because a cone only gets
   narrower upwards. The two searches are dilations of the orange mask with a kernel
   that is one pixel wide and `band_reach` tall, anchored at its lower or upper end.
3. **Clean-up.** A morphological opening removes specks from the orange mask, a closing
   fills small holes and the seams between orange and white.
4. **Blobs.** `cv2.findContours` gives one outline per connected region.
5. **Shape.** A blob is a cone only if
   * its area is at least `min_blob_area` pixels,
   * its box is taller than wide: `height >= 1.2 * width`,
   * it is narrower at the top than at the bottom: the mean width of the top third
     divided by the mean width of the bottom third is below 0.6 (a triangle gives 0.2, a
     box or a ball gives 1.0).

The score of a detection is how well the blob fills the triangle inscribed in its box
(1.0 = perfect triangle). A box that touches the image border is marked `clipped`: part
of the cone is outside the picture, so the box is not trusted for localisation.

## Step 2: locate (camera + lidar)

The camera intrinsics come from `camera/camera_info`: `K = [fx 0 cx; 0 fy cy; 0 0 1]`.

**Bearing of a pixel column.** The camera looks along the robot's +x. A column `u` sees
the direction

```
bearing(u) = atan2(cx - u, fx)        positive = to the left
```

It is `cx - u` and not `u - cx` because image columns grow to the right, while the
robot's y axis points to the left (REP 103: x forward, y left, z up).

**Bearing of a lidar return, seen from the camera.** Every lidar return becomes a point
`(x, y) = (r cos a, r sin a)` in `base_link`. The camera sits 0.18 m ahead of the lidar,
so it sees the same point in a slightly different direction (parallax, 3.5 degrees for a
cone at 1.2 m and 0.5 m to the side):

```
bearing_from_camera(x, y) = atan2(y, x - 0.18)
```

**Fusion.** Keep the returns with `bearing(u_right) <= bearing_from_camera <= bearing(u_left)`,
sort them by distance and take the nearest cluster (everything up to the first jump of
more than 0.15 m). Beams that pass beside the cone and hit the wall behind it fall away
here. The lidar sees the front of the cone, so the cone's axis is one radius farther
along the ray, where the radius is taken at the height of the scan plane:
`r(z) = base_radius * (1 - z / cone_height)`, 0.114 m at z = 0.30 m.

**Sanity gate.** The camera has its own, rough opinion about the distance. A cone of
real height `H` that is `h` pixels tall is at

```
d_camera = fy * H / h
```

If the lidar result differs from this by more than `distance_tolerance` (30 %), the
fusion is rejected: the lidar hit something else (the wall behind a hidden cone), or
the orange thing is not a cone of the expected size. The same estimate is used to skip
cones that look farther away than `max_range`.

## Step 3: remember (the cone map)

The cone's position is moved from `base_link` to `map` with tf2, **at the stamp of the
image**, not at "now": the robot keeps driving while the image is compressed, sent and
processed. Then:

* a sighting within `merge_radius` (0.6 m) of a known cone is the same cone, and refines
  its position with a running mean: `mean += (sample - mean) / n`,
* otherwise it starts a candidate,
* a candidate becomes a cone after `confirmations` (3) sightings and gets the next free
  id. Ids never change afterwards.

## What the node publishes

| Topic | Type | When | Content |
|---|---|---|---|
| `detections` | `vision_msgs/Detection2DArray` | every image | the boxes, `class_id = "cone"`, with the header (stamp and frame) of the image |
| `camera/annotated/compressed` | `sensor_msgs/CompressedImage` | every image | the camera image with the boxes drawn |
| `cones` | `visualization_msgs/MarkerArray` | 2 Hz | every confirmed cone in frame `map` (a `DELETEALL` marker, then one `CYLINDER` each) |

Three ROS ideas are worth finding in `cone_spotter_node.py`:

* **Pairing by stamp.** The simulator takes the image and the scan in the same tick and
  gives both the identical stamp. The node keeps the last scans in a `deque` and fuses
  an image only with the scan that has the same stamp. "The latest scan" would be wrong
  whenever the robot turns. ROS does not promise which of the two arrives first, so an
  image that is early waits for its scan.
* **tf2 at a point in time.** `lookup_transform('map', 'base_link', stamp, timeout)`.
* **Waiting inside a callback.** While the image callback waits for a transform, another
  thread must keep receiving `/tf`, or the wait never ends. That is why `main()` uses a
  `MultiThreadedExecutor`. The node's own callbacks share one mutually exclusive
  callback group, so they still never run at the same time and need no mutex.

## Watch it work

With the demo running (`pixi run -e jazzy demo`), in a second `pixi shell -e jazzy`:

```
ros2 run rqt_image_view rqt_image_view
```

and choose `/camera/annotated/compressed` in the list of topics (`/camera/image/compressed`
is the untouched camera image, for comparison).

Green box: a complete cone, labelled with its fused distance when the lidar agreed
(`cone 3.0 m`) and with `cone (no range)` when it did not. Yellow box: a cone cut by the
image border.

```
ros2 topic echo detections --once
ros2 topic echo cones --once
ros2 param list /cone_spotter
```

To run only this node (it then waits for images):

```
ros2 run explorer_vision cone_spotter
```

## Parameters to play with

The detector's limits, `max_range` and `distance_tolerance` are read for every image, so
a change shows in the next frame. The rows marked "set at start" are read once, when
the node starts:

```
ros2 param set /cone_spotter saturation_min 200
ros2 param set /cone_spotter hue_max 8
ros2 param set /cone_spotter min_blob_area 2000.0
```

A parameter keeps the type it was declared with. The defaults in the table show which
it is: `110` is an integer, `80.0` is a double. A double needs the decimal point
(`2000.0`, not `2000`) and an integer must not have one, or `ros2 param set` refuses
the value.

| Parameter | Default | Try this |
|---|---|---|
| `hue_min`, `hue_max` | 3, 14 | Narrow the range until the shaded side of the cone drops out of the box. |
| `saturation_min` | 110 | Raise it: the cone's anti-aliased edge goes first. |
| `value_min` | 40 | Raise it to 150: only the sunlit side is left. Is that still cone-shaped? |
| `min_blob_area` | 80.0 | Raise it: distant cones are no longer detected. |
| `band_reach` | 60 | Set it to 0: the white band no longer joins tip and base. The box shrinks to the tip, and the label turns to `cone (no range)`. |
| `white_saturation_max`, `white_value_min` | 80, 80 | What counts as white (the band). Raise `white_value_min` to 230 and no band is bright enough any more. |
| `max_range` | 5.0 | Lower it and the robot must drive closer before a cone appears on the map. |
| `distance_tolerance` | 0.30 | Set it to 10.0 and the gate is off. Watch for cones that land on walls. |
| `merge_radius` | 0.6 | Set at start, for example `--ros-args -p merge_radius:=0.05`: one cone becomes many. |
| `confirmations` | 3 | Set at start. With 1, every false sighting becomes a cone at once. |
| `cone_height`, `cone_base_radius` | 0.70, 0.20 | Set at start. The real size of the cones. A wrong height makes the gate reject everything. |
| `camera_offset_x`, `lidar_height` | 0.18, 0.30 | Where the sensors are mounted, as in `docs/interfaces.md`. |

## Run the tests

The tests need Python, numpy, OpenCV and pytest, but no running ROS system:

```
pixi run -e jazzy python -m pytest src/explorer_vision/test -q
```

* `test_cone_detector.py`: painted scenes. Cones in clutter, in shadow, far away, cut by
  the image border, and orange things that are not cones (a crate, a pole, a ball, a
  funnel). Cones with a white band from 2 to 5 m in front of a light plaster wall, a
  white pillar behind a cone, and the rule "orange above and below" on a single column.
* `test_cone_locator.py`: a simulated scan of a cone in front of a wall. The position
  must be right to within 0.1 m, and when the cone's returns are removed, the gate must
  refuse to put the cone on the wall.
* `test_cone_map.py`: merging, confirmation, stable ids.
* `test_rendered_image.py`: the detector on `test/data/rendered_cones.png`, an image
  rendered with threepp: four lit 3D cones with the simulator's texture and rubber foot
  at 2, 3, 4 and 5 m, not painted triangles. `test/render_test_image.py` recreates it.

`test/synthetic_world.py` is a miniature simulator for the tests: a pinhole camera that
paints cones, and a ray caster that plays lidar.

## How this relates to the vision chapter of the course

In the vision chapter the detector is a neural network (an ONNX model), and its node
publishes `vision_msgs/Detection2DArray`. This node
publishes exactly the same message type on `detections`. That is on purpose: everything
downstream (the locator, the map, the dashboard) only sees boxes with a class and a
score, and does not care how they were found.

Here the detector is a colour threshold, so the demo has no model file to ship and every
step can be inspected. To use a trained model instead, replace one module,
`cone_detector.py`: keep the function `detect_cones(image) -> list of Detection` and
put the network behind it. The colour threshold is also honest about its limits: it
only works because nothing else in the warehouse is orange, and because the one thing
that comes close (cardboard) differs enough in hue to be told apart. A network is what
you reach for when that is no longer true.

## Limits

* Only cones that are completely inside the image are localised.
* A cone must be hit by at least two lidar beams. With one beam per degree and a cone
  that is 0.23 m wide at the height of the scan plane, that holds up to about 6 m.
* The cone map never forgets. A cone that is moved after it was confirmed stays where
  it was first seen, and appears a second time at its new place.
