# explorer_nav: the robot's brain

Two nodes, written in Python with numpy:

| Node | Start with | What it does |
|---|---|---|
| `mapper` | `ros2 run explorer_nav mapper` | Turns laser scans into an occupancy grid (`map`) |
| `explorer` | `ros2 run explorer_nav explorer` | Finds frontiers, plans paths, drives the robot (`explore` action) |

The names and types of everything they publish and subscribe are in
[`docs/interfaces.md`](../../docs/interfaces.md). This page is about what happens inside.

## How the code is organised

The algorithms and the ROS code are kept apart on purpose.

| File | Knows about ROS? | Contents |
|---|---|---|
| `explorer_nav/occupancy_grid.py` | no | The log-odds grid, scan integration |
| `explorer_nav/frontiers.py` | no | Frontier cells and their clusters |
| `explorer_nav/planner.py` | no | Costmap, distance field (Dijkstra), path extraction |
| `explorer_nav/pure_pursuit.py` | no | Path follower |
| `explorer_nav/nav2_driver.py` | yes | Stands in for the path follower with `use_nav2`: sends the target to Nav2 |
| `explorer_nav/mapper_node.py` | yes | Node `mapper`: topics, tf2, a service, parameters |
| `explorer_nav/explorer_node.py` | yes | Node `explorer`: the action, threads, the state machine |
| `test/test_*.py` | no | Unit tests of the four algorithm modules |
| `test/fake_world.py` | yes | A tiny 2D simulator, to run the two nodes without the real one |

The four algorithm modules are plain functions and classes on numpy arrays. You can import
them in a notebook, feed them a hand-made grid and look at the result, and the tests do
exactly that. The two node files contain no mathematics: they move data between ROS and the
algorithms, and they deal with time and threads. When something goes wrong, this split
tells you where to look: wrong path on a correct map is an algorithm bug, a correct
path that the robot does not follow is a node (or timing) bug.

## The algorithms

### Occupancy grid (`occupancy_grid.py`)

The floor is divided into square cells (0.1 m by default). Each cell stores how strongly
we believe it is occupied, as *log-odds*:

    L = log( p / (1 - p) )        p = 1 / (1 + exp(-L))

`L = 0` means p = 0.5: no idea. Each laser beam is a piece of evidence. Every cell the
beam passed through is probably free, and the cell where it ended is probably occupied.
In log-odds form, Bayes' rule for combining independent evidence is an addition:

    L_new = clamp( L_old + L_measurement,  L_min,  L_max )

    L_measurement = log_odds_hit  (+0.85, p = 0.70)   for the cell where the beam ended
    L_measurement = log_odds_miss (-0.40, p = 0.40)   for the cells the beam passed through

A beam that reports `inf` hit nothing, so it clears all cells up to `range_max` and marks
none as occupied. The clamp keeps the map able to change its mind: without it, a cell seen
free for ten minutes would need ten minutes of hits before a newly dropped crate showed up.
With the defaults a new obstacle is on the map after three scans (0.3 s).

A beam is traced as three rays, spread over its angular slot, because two neighbouring
beams are 14 cm apart at 8 m and would leave stripes of unseen cells between them. The two
side rays stop at the shorter of their own beam and the neighbouring beam. With the full
length they would poke through walls seen at a shallow angle and erase them again
(`test_walls_seen_at_a_shallow_angle_are_not_erased` shows that failure).

The whole scan is integrated without a Python loop: all beams are sampled into one
`(rays x samples)` array of points, converted to cell indices, and collected in two boolean
masks (seen free, seen occupied). One scan takes about 5 ms.

For the `map` topic the log-odds become percent (0..100), and cells never observed
become -1.

### Frontiers (`frontiers.py`)

A *frontier cell* is a free cell with at least one unknown neighbour: the edge of the known
world. Frontier cells that touch each other (diagonals included) form a *cluster*, for
example the open end of an aisle. Clusters smaller than `min_frontier_size` are noise and
are dropped. Exploration is then one loop: drive to a frontier, the map grows, the frontier
moves or disappears, repeat until none is left.

### Planner (`planner.py`)

1. **Costmap.** The planner treats the robot as a point, so every obstacle is grown
   ("inflated") by `robot_radius + inflation_margin`. Unknown cells are forbidden as well,
   but not inflated. Around the forbidden zone lies a band of `comfort_cells` cells where
   driving is allowed but expensive, so paths keep to the middle of an aisle when they can.
2. **Distance field.** A wave spreads from the robot over all allowed cells. The time at
   which it reaches a cell is the cost of driving there. This is Dijkstra's algorithm.
   Because every step has a small integer cost, the priority queue is replaced by a ring of
   buckets (Dial's algorithm), and each wave is processed as one numpy array. A full field
   on a 300 x 200 grid takes 20 to 40 ms.
3. **Path.** From the target cell, step to the neighbour with the lowest distance until
   you are back at the robot. Then the staircase of cells is simplified into a few straight
   segments by line-of-sight checks.

One distance field answers both questions of the explorer: *which frontier is cheapest to
reach* (read the field at every frontier) and *what is the path there* (walk downhill from
the winner). That is why it is a field from the robot, and not A* to one goal.

If the robot's own cell is forbidden (a crate was dropped right next to it, so the robot is
inside the inflated zone), the wave starts from the nearest allowed cell and the path gets
a short "escape" segment in front. Refusing to plan would leave the robot stuck for ever.

### Path follower (`pure_pursuit.py`)

Pure pursuit chases a "carrot": the point one `lookahead` distance ahead on the path. With
the carrot at distance `d` and at angle `alpha` off the robot's nose, the circle through
robot and carrot has the curvature `2 * sin(alpha) / d`, and the command is

    w = curvature * v

Three additions make it usable: when the carrot is far off to the side or behind
(`alpha` above 0.9 rad) the robot turns on the spot instead of driving a wide arc into a
shelf, it brakes over the last 0.8 m, and it lowers `v` when the arc would need more than
`max_angular_speed`.

### The explorer's decisions (`explorer_node.py`)

While exploring, the node replans once per `replan_period`, and immediately when the robot
arrives, when the target is no longer a frontier, or when the robot is stuck.

* It takes the frontier with the lowest path cost, but keeps its current target unless
  another one costs less than `switch_factor` times as much. Without this, two frontiers of
  about equal cost take turns being the cheapest and the robot dithers between them.
* A frontier is blacklisted when the robot does not move for `stuck_timeout` seconds on its
  way there, when getting there takes more than `patience_factor` times the driving time
  of the planned path, or when it is still a frontier after the robot arrived.
* After `finish_cycles` replans in a row without a reachable frontier the building counts
  as explored. With `return_home` the robot then drives back to where it started
  (state `RETURNING`) before the action reports its result.

When no action is running, a pose on `goal_pose` (the "2D Goal Pose" tool of RViz) makes the
robot drive there (state `NAVIGATING`).

## Threads in the explorer node

The mapper is spun by the default single-threaded executor: one callback at a time, no
locks needed. The explorer cannot work that way. Its action callback *blocks* for the whole
exploration, and planning takes tens of milliseconds while a velocity command is due every
50 ms. So it uses a `MultiThreadedExecutor` and three callback groups (control, planning,
action). The state these threads share is guarded by one lock, which is only held while
copying data in or out, never while computing. Read `Explorer.__init__` and
`_execute_explore` for the reasoning, it is the part of this package that corresponds most
directly to the concurrency chapters of the course.

## Parameters to play with

Set them on the command line, for example
`ros2 run explorer_nav explorer --ros-args -p lookahead:=1.0`, or in the launch file.
`ros2 param describe /explorer lookahead` prints the description of each.

| Node | Parameter | Default | Try this | What you should see |
|---|---|---|---|---|
| mapper | `resolution` | 0.1 | 0.05 | A sharper map, four times the cells, everything slower. Thin stripes of unknown cells appear between distant beams |
| mapper | `log_odds_hit` | 0.85 | 0.2 | Walls need many scans to appear, the robot plans through them at first |
| mapper | `log_odds_miss` | -0.4 | -2.0 | Walls seen at a shallow angle get erased again: holes in the map |
| mapper | `log_odds_max` | 3.5 | 50.0 | A removed obstacle (`reset_world`) stays on the map for a very long time |
| explorer | `inflation_margin` | 0.14 | 0.0 | Paths scrape past corners and the safety node keeps stopping the robot |
| explorer | `inflation_margin` | 0.14 | 0.5 | Aisles close up: frontiers turn grey (unreachable), exploration ends early |
| explorer | `comfort_cells` | 3 | 0 | Shortest paths, tight around every corner |
| explorer | `lookahead` | 0.5 | 1.5 | Smoother driving, but corners are cut, sometimes into the inflated zone |
| explorer | `lookahead` | 0.5 | 0.15 | The robot wiggles along the path |
| explorer | `switch_factor` | 0.6 | 1.0 | Flip-flopping between frontiers of similar cost |
| explorer | `min_frontier_size` | 8 | 1 | The robot visits every last unknown cell behind a cone |
| explorer | `replan_period` | 1.0 | 5.0 | The robot keeps driving to places it has long since seen |
| explorer | `max_linear_speed` | 0.6 | 0.3 | Slower, and noticeably more accurate around corners |

## Running the tests

The unit tests need numpy and pytest, but no ROS and no build:

    pixi run -e jazzy python -m pytest src/explorer_nav/test -q

Add `-s` to see the measured times of the two speed tests (scan integration: budget 10 ms,
full replan: budget 100 ms, both asserted with a factor of three in reserve so a busy
machine does not fail them). `pixi run -e jazzy test` runs these tests and the ones of
`explorer_vision` in one go.

## Running the nav stack without the simulator

`test/fake_world.py` is a stand-in for the simulator: a 2D warehouse, a robot and a lidar
in about 250 lines of numpy. It publishes `scan`, `odom` and `tf` as the contract says and
listens to `cmd_vel_nav` directly. In four terminals (after `pixi shell -e jazzy`):

    python src/explorer_nav/test/fake_world.py
    ros2 run explorer_nav mapper
    ros2 run explorer_nav explorer
    ros2 action send_goal --feedback explore explorer_interfaces/action/Explore "{return_home: true}"

Every five seconds the fake world prints where the robot is, how close its centre has come
to an obstacle, and whether it has collided. Useful first questions to ask the running
system:

    ros2 topic hz scan                      # 10 Hz?
    ros2 topic info -v map                  # who publishes, who subscribes, with which QoS?
    ros2 run tf2_ros tf2_echo map base_link
    ros2 topic echo exploration_status
