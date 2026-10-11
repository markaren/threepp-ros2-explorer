# explorer_sim: the world and the robot

One node, `sim`, written in C++ with [threepp](https://github.com/markaren/threepp). It
plays the part of the robot's hardware: it takes velocity commands and gives back what a
lidar, a camera and wheel encoders would. It also opens the window that shows the truth.

```powershell
ros2 run explorer_sim sim                              # on its own
ros2 run explorer_sim sim --ros-args -p view:=chase    # start riding behind the robot
ros2 param list /sim                                   # everything that can be set
```

## Reading order

| File | Lines of interest |
|---|---|
| [`src/main.cpp`](src/main.cpp) | Start here. 110 lines: the three threads, and the mailboxes and queues through which they meet |
| [`include/explorer_sim/types.hpp`](include/explorer_sim/types.hpp) | The data that crosses between the two halves. No threepp, no ROS |
| [`include/explorer_sim/mailbox.hpp`](include/explorer_sim/mailbox.hpp), [`thread_safe_queue.hpp`](include/explorer_sim/thread_safe_queue.hpp) | The book's two classes |
| [`src/sim_node.cpp`](src/sim_node.cpp) | The ROS half: publishers and their QoS, tf, services that wait on a future, parameters |
| [`include/explorer_sim/simulator.hpp`](include/explorer_sim/simulator.hpp) | The simulator's whole public face, and which of it is thread-safe |
| [`src/world.cpp`](src/world.cpp) | The building as a list of boxes, and collision. No graphics |
| [`src/window.cpp`](src/window.cpp) | `Simulator::run()`: the window and the frame loop. The four numbered steps at its end are the heartbeat of the simulator |
| [`src/simulator.cpp`](src/simulator.cpp) | What happens: `step()` (physics), `castLidar()`, `captureCamera()`, and the thread-safe requests |
| [`src/scene.cpp`](src/scene.cpp) | What is drawn: the building with its baked light, what moves in it, the polished floor. Skip it unless graphics is what you came for |
| [`src/overlay.cpp`](src/overlay.cpp) | What the window says in words: the status card, legend, key hints and camera card, and where each goes |
| [`src/hud.cpp`](src/hud.cpp) | The tools for that: rounded cards and text as geometry, in 180 lines |
| [`src/simulator_impl.hpp`](src/simulator_impl.hpp) | The private class those files share |

## The rule of this package

`simulator.cpp`, `scene.cpp` and `window.cpp` include threepp and never ROS. `sim_node.cpp` includes
ROS and never threepp. They meet only in `types.hpp`. The CMake file enforces it: the
simulator is a library that does not link `rclcpp`.

Why bother? Because it keeps each half small enough to understand, because the
simulator can be tested and reused without ROS, and because it makes the thread border
visible: everything that crosses it is declared in one 90-line header.

## What the simulator simulates

* **The robot**: a differential drive with speed and acceleration limits, as a disc of
  radius 0.22 m that cannot enter boxes. Fixed timestep, 240 Hz.
* **Lidar**: 360 beams, 10 Hz, 8 m, Gaussian range noise (`lidar_noise`). One
  `threepp::Raycaster` ray per beam against the collision boxes and the cones.
* **Camera**: 640 x 360, 10 Hz, 50 degrees vertical field of view, rendered with
  `threepp::CameraSensor` on the same tick as the lidar and with the same stamp.
* **Odometry**: 30 Hz. Exact, unless `odom_drift` is raised.
* **A watchdog**: no command for 0.5 s means stop.

What it does not simulate: wheel slip, inertia beyond the acceleration limits, motion
blur, lens distortion, lidar beams bending around the robot's motion during a scan.

## Seeing versus colliding

Every object in [`world.cpp`](src/world.cpp) is a box. The box is what the robot bumps
into and what the lidar hits. What you see is something else: the building and its
props as they were made and lit in Blender ([`assets/warehouse`](assets/README.md)),
with the boxes hidden. So the lidar and the eye do not see quite the same thing: a rack
is open between its shelves, but its box is solid. A wall's box is 1.4 m high, the wall
you see 4.2 m. Real simulators make the same trade for the same reason, and it is worth
knowing when a simulated sensor disagrees with the picture.

## Adding something to the warehouse

In `World::warehouse()`:

```cpp
// centre x, centre y, size x, size y, height, kind, model, quarter turns
w.boxes.push_back({10.0, 0.0, 1.2, 0.8, 0.9, Kind::Prop, "pallet_a", 1});
```

The model names are the files in [`assets/models`](assets/models). Without a model
name, the box itself is drawn. Rebuild, and the lidar, the collision and the picture all
know about it.

The new pallet will look a little out of place: it is lit by ordinary lamps, while
everything around it has its light baked in. To give it baked light too, add the same
line to `PROPS` in [`tools/blender/make_warehouse.py`](../../tools/blender/make_warehouse.py)
and run that script (five and a half minutes, needs Blender). A new *wall* is different:
the walls you see are part of the building's file, so a wall added only to `world.cpp`
stops the robot and the lidar and is not drawn until the script knows it as well.
