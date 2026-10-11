"""Tests of the planner. No ROS needed."""

import time

import numpy as np

from explorer_nav.frontiers import find_frontiers
from explorer_nav.planner import (
    UNREACHABLE,
    build_costmap,
    compute_distance_field,
    extract_path,
    inflate,
    path_length_cells,
    simplify_path,
)

INFLATION_CELLS = 4.1       # (0.22 m radius + 0.14 m margin) / 0.1 m + half a cell


def room_with_wall(height: int = 60, width: int = 80) -> tuple[np.ndarray, np.ndarray]:
    """A walled room, cut in two by a wall that leaves a gap at the top.

        +----------------------+
        |          gap         |
        |           #          |
        |   start   #   goal   |
        |           #          |
        +----------------------+
    """
    occupied = np.zeros((height, width), dtype=bool)
    occupied[0, :] = occupied[-1, :] = True
    occupied[:, 0] = occupied[:, -1] = True
    occupied[0:40, 40] = True       # the dividing wall: rows 0..39, the gap is rows 40..58
    free = ~occupied
    return free, occupied


def test_inflate_grows_a_disc():
    occupied = np.zeros((21, 21), dtype=bool)
    occupied[10, 10] = True
    inflated = inflate(occupied, 3.0)
    assert inflated[10, 13] and inflated[13, 10] and inflated[10, 7]
    assert not inflated[10, 14]
    assert not inflated[13, 13], 'the corner of the square is outside the disc'


def test_unknown_cells_are_not_traversable_and_not_inflated():
    free = np.zeros((30, 30), dtype=bool)
    occupied = np.zeros((30, 30), dtype=bool)
    free[:, :15] = True             # the left half is known free, the right half unknown
    costmap = build_costmap(free, occupied, INFLATION_CELLS)
    assert costmap.traversable[10, 14], 'the robot may drive right up to the unknown'
    assert not costmap.traversable[10, 15]


def test_path_goes_around_the_wall():
    free, occupied = room_with_wall()
    costmap = build_costmap(free, occupied, INFLATION_CELLS)
    start = (15, 20)
    goal = (15, 60)

    field = compute_distance_field(costmap, *start)
    assert field is not None
    path = extract_path(field, *goal)
    assert path is not None
    assert path[0] == start and path[-1] == goal

    # Every step is to a 4-neighbour, and every cell is one the robot may stand on.
    for (row_a, col_a), (row_b, col_b) in zip(path[:-1], path[1:]):
        assert abs(row_a - row_b) + abs(col_a - col_b) == 1
    assert all(costmap.traversable[cell] for cell in path)

    # The straight line is 40 cells. The detour through the gap is much longer, and it
    # passes above the end of the wall (row 39) with the inflation distance to spare.
    assert len(path) > 80
    assert max(row for row, _ in path) >= 39 + 4

    simplified = simplify_path(path, costmap)
    assert simplified[0] == start and simplified[-1] == goal
    assert len(simplified) < len(path) / 4, 'a handful of straight segments is enough'
    assert path_length_cells(simplified) <= path_length_cells(path) + 1e-9
    # The shortcuts stay clear of the obstacles too: sample them and check.
    for cell_a, cell_b in zip(simplified[:-1], simplified[1:]):
        for t in np.linspace(0.0, 1.0, 50):
            row = int(round(cell_a[0] + t * (cell_b[0] - cell_a[0])))
            col = int(round(cell_a[1] + t * (cell_b[1] - cell_a[1])))
            assert costmap.traversable[row, col]


def test_paths_prefer_the_middle_of_a_corridor():
    """The comfort cost keeps the robot away from walls when there is room."""
    occupied = np.zeros((31, 100), dtype=bool)
    occupied[0, :] = occupied[-1, :] = True
    occupied[:, 0] = occupied[:, -1] = True
    costmap = build_costmap(~occupied, occupied, INFLATION_CELLS)
    # Start and goal hug the lower wall. The cheapest path swings out towards the middle.
    field = compute_distance_field(costmap, 6, 6)
    path = extract_path(field, 6, 93)
    assert max(row for row, _ in path) >= 8


def test_distance_field_serves_frontier_choice_and_path():
    """One field answers both: which frontier is cheapest, and how to get there."""
    free, occupied = room_with_wall()
    unknown = np.zeros_like(free)
    # Two unexplored patches: one near the start, one on the far side of the wall.
    for rows, cols in [(slice(20, 30), slice(5, 8)), (slice(10, 20), slice(70, 75))]:
        unknown[rows, cols] = True
        free[rows, cols] = False
    costmap = build_costmap(free, occupied, INFLATION_CELLS)
    field = compute_distance_field(costmap, 15, 20)

    frontiers = find_frontiers(free, unknown, min_size=5)
    assert len(frontiers) == 2
    costs = []
    for frontier in frontiers:
        reachable = field.distance[frontier.rows, frontier.cols]
        costs.append((int(reachable.min()), frontier))
    costs.sort(key=lambda item: item[0])
    nearest_cost, nearest = costs[0]
    assert nearest_cost < UNREACHABLE
    assert nearest.centroid_col < 40, 'the patch on the same side of the wall is cheaper'
    assert costs[1][0] > 2 * nearest_cost


def test_enclosed_goal_is_unreachable():
    free, occupied = room_with_wall()
    occupied[0:60, 40] = True       # close the gap
    free = ~occupied
    costmap = build_costmap(free, occupied, INFLATION_CELLS)
    field = compute_distance_field(costmap, 15, 20)
    assert not field.is_reachable(15, 60)
    assert extract_path(field, 15, 60) is None


def test_robot_escapes_from_an_inflated_start_cell():
    """A crate dropped next to the robot must not paralyse the planner."""
    free, occupied = room_with_wall()
    start = (15, 20)
    occupied[14:17, 22:25] = True   # a crate 2 cells (0.2 m) in front of the robot centre
    free = ~occupied
    costmap = build_costmap(free, occupied, INFLATION_CELLS)
    assert not costmap.traversable[start], 'the test needs a forbidden start cell'

    field = compute_distance_field(costmap, *start)
    assert field is not None
    assert field.source != start
    assert costmap.traversable[field.source]
    assert field.source[1] < start[1], 'the way out leads away from the crate'

    path = extract_path(field, 15, 60)
    assert path is not None
    assert path[0] == start and path[1] == field.source and path[-1] == (15, 60)

    simplified = simplify_path(path, costmap)
    assert simplified[0] == start and simplified[1] == field.source


def test_no_escape_when_everything_is_blocked():
    occupied = np.ones((30, 30), dtype=bool)
    costmap = build_costmap(~occupied, occupied, INFLATION_CELLS)
    assert compute_distance_field(costmap, 15, 15) is None


def warehouse(height: int = 200, width: int = 300) -> tuple[np.ndarray, np.ndarray]:
    """A 300 x 200 grid with rows of shelves: long winding routes, the planner's worst case."""
    occupied = np.zeros((height, width), dtype=bool)
    occupied[0, :] = occupied[-1, :] = True
    occupied[:, 0] = occupied[:, -1] = True
    for index, col in enumerate(range(30, 280, 25)):
        if index % 2 == 0:
            occupied[0:170, col:col + 8] = True     # shelf from the bottom, gap at the top
        else:
            occupied[30:200, col:col + 8] = True    # shelf from the top, gap at the bottom
    return ~occupied, occupied


def test_full_replan_is_fast_enough():
    """Budget: 100 ms for costmap + distance field + path. Asserted with slack x3.

    The control loop shares the Python interpreter with the planner, and the simulator
    stops the robot when commands pause for 0.5 s.
    """
    free, occupied = warehouse()
    durations = []
    for _ in range(5):
        started = time.perf_counter()
        costmap = build_costmap(free, occupied, INFLATION_CELLS)
        field = compute_distance_field(costmap, 15, 15)
        path = extract_path(field, 15, 290)
        simplified = simplify_path(path, costmap)
        durations.append(time.perf_counter() - started)
    assert path is not None and len(simplified) >= 2
    best = min(durations)
    print(f'full replan: best {best * 1e3:.1f} ms, median {np.median(durations) * 1e3:.1f} ms, '
          f'path of {len(path)} cells simplified to {len(simplified)} points')
    assert best < 0.300
