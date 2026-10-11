# explorer_dashboard: what the robot believes

One node, `dashboard`, written in Python. It draws its window with the `threepp` Python
module, which is the same C++ engine as the simulator's, reached through pybind11
bindings.

Nothing in this window comes from the simulator directly. The map is the `map` topic,
the path is `plan`, the cones are `cones`, the picture is the camera topic. If a node is
not running, its part of the window is missing, and the dashboard says which.

```powershell
ros2 run explorer_dashboard dashboard
ros2 run explorer_dashboard dashboard --ros-args -p auto_explore:=true
```

| Key | Does | Through |
|---|---|---|
| `E` | start exploring | action client, `explore` |
| `C` | cancel | the same action |
| `G` | send the robot to the point under the mouse | publisher, `goal_pose` |
| `R` | reset the simulator, clear the map | service clients, `reset_world` and `clear_map` |
| `F` | camera follows the robot | nothing: the camera is the dashboard's own |

## Reading order

| File | Knows ROS | Knows threepp | What it is |
|---|---|---|---|
| [`mailbox.py`](explorer_dashboard/mailbox.py) | no | no | Start here: how two threads share data, and why Python needs the lock too |
| [`world_model.py`](explorer_dashboard/world_model.py) | no | no | Plain data types, and the inbox: one mailbox per topic |
| [`convert.py`](explorer_dashboard/convert.py) | no | no | ROS axes to threepp axes, grid to picture (numpy) |
| [`dashboard_node.py`](explorer_dashboard/dashboard_node.py) | yes | no | Subscriptions and their QoS, tf2, the action client, the service clients |
| [`scene.py`](explorer_dashboard/scene.py) | no | yes | The 3D picture: floor texture, walls, robot, trail, markers |
| [`hud.py`](explorer_dashboard/hud.py) | no | yes | The 2D overlay: status, camera picture, legend |
| [`window.py`](explorer_dashboard/window.py) | no | yes | Canvas, renderer, camera, keys |
| [`app.py`](explorer_dashboard/app.py) | no | no | The render loop that joins the two halves |

The split is the same as in the simulator, for the same reason: OpenGL belongs to one
thread. The render loop keeps the main thread and is the only code that calls threepp.
The ROS executor spins on a second thread, and its callbacks do one thing: put the
newest message in a mailbox.

## Two things to know

* The Python bindings of threepp do not report key presses or the mouse position yet,
  only whether a key is down right now. `window.py` turns that into "was just pressed"
  (edge detection), and `cursor.py` asks Windows where the mouse is. On other systems
  `G` aims at the centre of the view.
* `threepp` is a pip package, not a ROS package, so `rosdep` does not know it. It is
  installed by `pixi.toml` (`[feature.jazzy.pypi-dependencies]`).
