# explorer_interfaces: the custom types

Three small files, no code:

| File | Kind | Used for |
|---|---|---|
| [`msg/ExplorationStatus.msg`](msg/ExplorationStatus.msg) | message | what the explorer is doing, on a topic and as action feedback |
| [`srv/SpawnObstacle.srv`](srv/SpawnObstacle.srv) | service | drop a crate into the simulated world |
| [`action/Explore.action`](action/Explore.action) | action | explore the building: a goal, progress reports, a result, and it can be cancelled |

At build time `rosidl` turns each into a C++ header and a Python class. That is how a
C++ node and a Python node agree on the bytes without ever seeing each other's source:
the `.proto` and `protoc` workflow of Part 4 of the book, under another name.

```powershell
ros2 interface show explorer_interfaces/action/Explore
ros2 interface list | findstr explorer
```

Everything else in the demo uses standard types (`sensor_msgs/LaserScan`,
`nav_msgs/OccupancyGrid`, `geometry_msgs/Twist`, ...). Reach for a standard type first:
RViz, `rqt` and everybody else's nodes already understand those. A custom type is for
what the standard ones cannot say.

A change here means rebuilding this package **and every package that uses it**:

```powershell
pixi run -e jazzy build --packages-up-to explorer_bringup
```
