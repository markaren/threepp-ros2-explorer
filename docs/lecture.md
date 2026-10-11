# Showing it in a lecture

A script for about fifteen minutes in front of a projector. Each step says what to do,
what the room sees, and the point it makes.

## Before the students arrive

```powershell
cd threepp-ros2-explorer
pixi run -e jazzy build                                                       # nothing to do, if it was built before
pixi run -e jazzy ros2 launch explorer_bringup demo.launch.py auto_explore:=false
```

* Simulator on the left half of the screen, dashboard on the right.
* A third terminal, already inside `pixi shell -e jazzy`, with a large font.
* If the room's Wi-Fi is shared with other ROS users: `$env:ROS_DOMAIN_ID = "17"` in
  every terminal first.

Nothing moves yet. The dashboard is empty apart from the patch of floor the lidar can
see from the start position.

## 1. Two pictures of one world (2 min)

Point at the simulator: *this is the world.* Point at the dashboard: *this is what the
robot knows about it.* Orbit both with the mouse. The dashboard shows a few metres of
floor and nothing else.

> The robot does not get to look at the left window. Everything it knows came through
> a topic.

Press `F` twice in the simulator for the chase view, then back.

## 2. Let it go (3 min)

Press `E` in the dashboard. While it explores, narrate what is on screen:

* blue dots: *frontiers*, the edges of the known world. It drives to the nearest.
* the yellow line: the planned path, replanned every second.
* the pink dots in the simulator: where the lidar's 360 beams end, ten times a second.
* the inset: the robot's camera. When a cone has been seen three times it appears in
  the dashboard.

Leave it running for the rest of the lecture.

## 3. What is actually running (3 min)

In the terminal:

```powershell
ros2 node list
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /exploration_status --once
rqt_graph
```

> Six programs. Two in C++, four in Python. Nobody started a broker.

Open `docs/interfaces.md` next to the graph: the page *is* the graph.

Then the smallest node, `src/explorer_safety/src/safety_monitor.cpp`: two subscriptions,
two publishers, a mutex, and a comment about why the mutex is there.

## 4. Throw something at it (2 min)

Simulator in chase view (press `F` until the key strip says "camera: chase"). Click on
the floor a metre in front of the robot. A crate
falls. The robot brakes, the crate appears in the dashboard's map a moment later, the
yellow path bends around it.

> Three nodes reacted, in this order: the reflex in C++ (milliseconds), the mapper,
> the planner. The reflex knows nothing about maps. That is why it is fast, and why
> it still works when the planner is wrong.

## 5. Break a sensor, live (2 min)

```powershell
ros2 param set /sim lidar_noise 0.08
```

The walls in the dashboard turn fuzzy. Nobody restarted anything.

```powershell
ros2 param set /sim lidar_noise 0.01
```

If there is time, the bigger one: `ros2 param set /sim odom_drift 1.0`. Within a minute
the map in the dashboard bends: everything the robot maps from now on is drawn at an
angle to what it mapped before (about 40 degrees and 6 m off after 40 seconds), while
in the simulator nothing has changed.

> The left window is right. The right window is what the robot believes. This is the
> problem SLAM exists to solve.

And it can be shown solved: stop the demo, start it with `mapper:=slam_toolbox`, set the
same drift. The map stays straight, because slam_toolbox, not our mapper, now decides
where `odom` sits in the `map`. Nothing else was changed.

## 6. Where your project fits (2 min)

Show `src/explorer_sim/src/main.cpp`: three threads that meet only in mailboxes and
queues, the book's `Mailbox` and `ThreadSafeQueue`.

> This is the simulator you are building, with a ROS node standing next to its render
> loop. If yours publishes `scan` and `tf` and listens to `cmd_vel`, the Python side
> of this demo will drive *your* car.

Press `P` in the simulator: post-processing off, then on.

> Same scene, same OpenGL renderer. Three image passes.

## If something goes wrong on stage

| It does this | Do this |
|---|---|
| Nothing moves after `E` | Is the dashboard the focused window? Otherwise send the goal from the terminal: `ros2 action send_goal /explore explorer_interfaces/action/Explore "{return_home: true}"` |
| The robot is stuck | `R` in the dashboard (resets the simulator and the map), then `E` |
| A window is frozen | Ctrl-C in the launch terminal, start again. It is ready in a few seconds |
| The laptop struggles | Start with `effects:=false` |
