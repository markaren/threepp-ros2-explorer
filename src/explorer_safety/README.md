# explorer_safety: the reflex

One node, `safety_monitor`, in one C++ file of 160 lines:
[`src/safety_monitor.cpp`](src/safety_monitor.cpp). If you read only one file in this
repository, read that one. It has everything a node has (two subscriptions, two
publishers, parameters, state shared between callbacks) and nothing else.

```
explorer --cmd_vel_nav--> safety_monitor --cmd_vel--> sim
                                ^
                               scan
```

The planner's commands pass through it on the way to the motors. It looks at the strip
of floor straight ahead, as wide as the robot plus a margin, and scales the forward
speed: full speed with 0.5 m of clearance or more, zero at 0.1 m. Turning is never
limited, so the planner can always steer away. With no scan for half a second it lets
nothing through.

It publishes `safety_stop` (`std_msgs/Bool`, latched) while it is holding the robot
still.

Try it without the planner:

```powershell
ros2 launch explorer_bringup sim.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_nav
```

Drive at a wall. Then remap to `cmd_vel` instead, which bypasses the monitor, and drive
at the wall again.
