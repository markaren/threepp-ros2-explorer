# explorer_bringup: starting it all

No code. This package installs three kinds of file, which is all a "bringup" package
ever does:

| File | What it is |
|---|---|
| [`launch/demo.launch.py`](launch/demo.launch.py) | Starts every node. Arguments switch parts on and off |
| [`launch/sim.launch.py`](launch/sim.launch.py) | Only the simulator and the safety monitor. `demo.launch.py` includes it |
| [`config/explorer.yaml`](config/explorer.yaml) | The parameters worth changing, for all nodes, in one place |
| [`config/slam_toolbox.yaml`](config/slam_toolbox.yaml) | Parameters for slam_toolbox, when it takes the mapper's place (`mapper:=slam_toolbox`) |
| [`config/nav2.yaml`](config/nav2.yaml) | Parameters for Nav2, when it plans and drives (`navigation:=nav2`) |
| [`rviz/explorer.rviz`](rviz/explorer.rviz) | An RViz layout for the demo's topics (`rviz:=true`) |
| [`rviz/explorer_nav2.rviz`](rviz/explorer_nav2.rviz) | The same with Nav2's costmaps, footprint and carrot. Opened when `navigation:=nav2` |

```powershell
ros2 launch explorer_bringup demo.launch.py --show-args
ros2 launch explorer_bringup demo.launch.py auto_explore:=false rviz:=true
```

The launch files read the *installed* copy of `explorer.yaml`. After editing it:

```powershell
pixi run -e jazzy build --packages-select explorer_bringup
```
