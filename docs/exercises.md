# Exercises

The ten quick experiments in the [README](../README.md#things-to-try) need no code. These
do. They are ordered roughly by effort, and each names the files you will touch.

Rebuild one package after a change with

```powershell
pixi run -e jazzy build --packages-select <package>
```

The algorithm modules of `explorer_nav` and `explorer_vision` have unit tests that run
without ROS. Run them before and after you change anything there:

```powershell
pixi run -e jazzy test
```

## Warm-up

**1. A battery.** Make the simulator publish `battery` (`sensor_msgs/BatteryState`) once
a second. Charge drops with distance driven and a little with time. Show the percentage
in the simulator's status line.
*Touches:* `types.hpp` (a field in `Telemetry`), `simulator.cpp` (where the charge
is computed, in `step()`), `window.cpp` (where a `Telemetry` sample is filled),
`sim_node.cpp` (where it is published), `overlay.cpp` (the status card).
*Think about:* which thread computes it, which thread publishes it, and what carries it
between them.

**2. A horn.** Add a service `honk` (`std_srvs/Trigger`) to the simulator that makes the
robot's light ring flash for a second. Call it from the command line.
*Touches:* `simulator.hpp/.cpp` (a `requestHonk()` returning a future), `sim_node.cpp`.
*Think about:* why can the service callback not just set the ring's colour?

**3. Your own launch file.** Write `teleop.launch.py` that starts the simulator, the
safety monitor, the mapper and the dashboard, but not the explorer, so you can map the
building by driving it yourself with `teleop_twist_keyboard`.
*Touches:* `explorer_bringup/launch`.

**4. Slower near people.** The cones mark places where people work. Make the explorer
halve its speed within 1.5 m of any confirmed cone.
*Touches:* `explorer_node.py` (subscribe to `cones`).
*Think about:* the QoS you need to receive a message that was published before you
subscribed.

## Core

**5. Low battery, go home.** With exercise 1 done: when the charge falls below 20 %, the
explorer abandons exploration and returns to the start. The action result says why.
*Touches:* `explorer_node.py`, `Explore.action` (a reason in the result).
*Think about:* you changed an interface. Which packages have to be rebuilt, and what
happens if you forget one?

**6. A reflex for the rear.** The safety monitor only guards forward motion. Make it
guard reversing too, and publish *which* side is blocked.
*Touches:* `safety_monitor.cpp`, and a new message in `explorer_interfaces` (or two
`Bool` topics: argue for one or the other).

**7. The safety monitor as a component.** Turn `SafetyMonitor` into an `rclcpp`
component and load it into a container from the launch file. Then load it into the
*same process* as a second component of your own.
*Think about:* what does sharing a process buy for a 640 x 360 image stream? Look up
"intra-process communication".

**8. Odometry you cannot trust.** Set `odom_drift` to 0.5 and watch the map fall apart.
Then write a node that publishes `map -> odom` so as to cancel the drift, using
`ground_truth` (cheating, but it shows where a localiser plugs in). Remove the mapper's
own static `map -> odom` first.
*Touches:* `mapper_node.py`, a new node.
*Think about:* the result still drifts a little between corrections. Why is `odom`
kept as a frame at all, when `map` is the correct one?

**9. Look at everything.** The explorer stops when the *lidar* has seen the whole
building, and sometimes drives home with a cone unseen, because the camera looks only
ahead. Add a second grid to the mapper that records which cells the *camera* has covered
(80 degrees wide, 5 m far, blocked by occupied cells), publish it, and make the explorer
keep going until that grid is complete too.
*Touches:* `occupancy_grid.py`, `mapper_node.py`, `explorer_node.py`.
*Think about:* "frontier" now has two meanings. Can one piece of code serve both?

**10. The ONNX detector.** Replace the colour threshold in `cone_detector.py` with the
ONNX model from Part 6 of the book. Nothing else in the package should change.
*Think about:* the detector now takes tens of milliseconds. What happens to the image
queue, and which QoS setting decides that? Keep stamping detections with the *image's*
time.

**11. Record and replay.** Record a run with `ros2 bag record -a`. Then stop everything
and replay it with only the mapper and the dashboard running.
*Think about:* the map appears again without a simulator or a robot. What does that tell
you about how to debug a perception problem that happens once a day on a real robot?

## Stretch

**12. A map that outlives the run.** Add a `save_map` service to the mapper that writes
the grid to disk (a PGM image and a YAML file, the format `nav2_map_server` uses), and a
parameter that loads one at start.

**13. A second robot.** Two robots, each with its own namespace, exploring one building
and sharing one map.
*You will run into:* tf2's absolute `/tf` topic and global frame names; two lidars that
see each other's robot; two explorers that pick the same frontier.
*Start with:* two simulators' worth of robot in one `sim` process, frames prefixed
`robot1/` and `robot2/`, one mapper that takes both scans.

**14. Real SLAM.** Add `ros-jazzy-slam-toolbox` to `pixi.toml`, turn on `odom_drift`,
and replace the mapper with `slam_toolbox`. The explorer, the dashboard and the cone
spotter should not need to change, because they only depend on the `map` topic and the
`map -> odom` transform.
*Think about:* that sentence is the whole argument for standard interfaces.

**15. Your own project behind these topics.** The simulator publishes what a small
robot's drivers would. If your semester project has a car with a lidar, make it publish
`scan` and `tf` and subscribe `cmd_vel` with the names and types in
[interfaces.md](interfaces.md), and run *this* repository's mapper, explorer and
dashboard against *your* simulator.
