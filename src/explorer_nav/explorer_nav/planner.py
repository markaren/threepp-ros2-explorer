"""Grid path planning: costmap, distance field, path extraction. Plain numpy, no ROS.

Three steps turn a map into a path:

1. build_costmap: decide where the CENTRE of the robot may be. The robot is a disc, the
   planner treats it as a point, so every obstacle is grown by the robot's radius.
2. compute_distance_field: let a wave spread from the robot over all cells it may stand
   on. The moment the wave arrives in a cell is the cost of driving there.
3. extract_path: from any target cell, walk downhill in the distance field. That leads
   back to the robot along the cheapest route.

Step 2 is computed once per replan and answers two questions at the same time: "which
frontier is cheapest to reach?" (read the field at each frontier) and "what is the path
there?" (walk downhill from the winner). That is why this planner is a distance field and
not A*, which finds one path to one goal.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Distance value of a cell the wave never reached.
UNREACHABLE = np.iinfo(np.int32).max

NEIGHBOURS_4 = ((-1, 0), (1, 0), (0, -1), (0, 1))


@dataclass
class Costmap:
    """Where the robot centre may be, and how much the planner dislikes each cell."""

    traversable: np.ndarray     # bool: the robot centre may be in this cell
    cost: np.ndarray            # int32: cost of stepping INTO this cell, at least 1


@dataclass
class DistanceField:
    """Result of compute_distance_field."""

    distance: np.ndarray            # int32 (height, width): cost from the robot, or UNREACHABLE
    start: tuple[int, int]          # the cell the robot is in
    source: tuple[int, int]         # the cell the wave started from, see compute_distance_field

    def cost_to(self, row: int, col: int) -> int:
        return int(self.distance[row, col])

    def is_reachable(self, row: int, col: int) -> bool:
        return self.distance[row, col] != UNREACHABLE


def disc_offsets(radius_cells: float) -> list[tuple[int, int]]:
    """All (d_row, d_col) offsets that lie within radius_cells of (0, 0)."""
    reach = int(np.ceil(radius_cells))
    offsets = []
    for d_row in range(-reach, reach + 1):
        for d_col in range(-reach, reach + 1):
            if d_row * d_row + d_col * d_col <= radius_cells * radius_cells:
                offsets.append((d_row, d_col))
    return offsets


def inflate(occupied: np.ndarray, radius_cells: float) -> np.ndarray:
    """Grow every occupied cell into a disc of the given radius.

    The loop runs over the offsets of the disc (about 50), not over the cells: each pass
    stamps a shifted copy of ALL occupied cells into the result.
    """
    height, width = occupied.shape
    rows, cols = np.nonzero(occupied)
    inflated = np.zeros_like(occupied)
    for d_row, d_col in disc_offsets(radius_cells):
        shifted_rows = rows + d_row
        shifted_cols = cols + d_col
        inside = ((shifted_rows >= 0) & (shifted_rows < height)
                  & (shifted_cols >= 0) & (shifted_cols < width))
        inflated[shifted_rows[inside], shifted_cols[inside]] = True
    return inflated


def grow_by_one_cell(mask: np.ndarray) -> np.ndarray:
    """The mask plus every cell that touches it (4-neighbours)."""
    grown = mask.copy()
    grown[1:, :] |= mask[:-1, :]
    grown[:-1, :] |= mask[1:, :]
    grown[:, 1:] |= mask[:, :-1]
    grown[:, :-1] |= mask[:, 1:]
    return grown


def build_costmap(free: np.ndarray, occupied: np.ndarray, inflation_cells: float,
                  comfort_cells: int = 3) -> Costmap:
    """Turn the map into a costmap.

    free, occupied: boolean grids. A cell that is neither is unknown.

    * Cells within inflation_cells of an occupied cell are forbidden: with its centre
      there, the robot would touch the obstacle (or come closer than the safety margin).
    * Unknown cells are forbidden too: nobody has seen what is there. They are NOT
      inflated, or the robot could never drive up to the edge of the known world.
    * The next comfort_cells rings around the forbidden zone are allowed but expensive
      (the closer, the more), so the wave prefers the middle of an aisle and paths only
      squeeze past an obstacle when there is no other way.
    """
    inflated = inflate(occupied, inflation_cells)
    traversable = free & ~inflated
    cost = np.ones(free.shape, dtype=np.int32)
    near_obstacle = inflated
    for _ in range(comfort_cells):
        near_obstacle = grow_by_one_cell(near_obstacle)
        cost += near_obstacle
    return Costmap(traversable, cost)


def nearest_traversable(traversable: np.ndarray, row: int, col: int,
                        max_cells: float) -> tuple[int, int] | None:
    """The traversable cell closest to (row, col), or None if none is within max_cells."""
    rows, cols = np.nonzero(traversable)
    if rows.size == 0:
        return None
    squared = (rows - row) ** 2 + (cols - col) ** 2
    nearest = int(np.argmin(squared))
    if squared[nearest] > max_cells * max_cells:
        return None
    return int(rows[nearest]), int(cols[nearest])


def compute_distance_field(costmap: Costmap, start_row: int, start_col: int,
                           max_escape_cells: float = 12.0) -> DistanceField | None:
    """Cost of the cheapest 4-connected route from the robot to every reachable cell.

    This is Dijkstra's algorithm, in the form of a wavefront. Because every step costs a
    small whole number (costmap.cost), the priority queue can be replaced by a ring of
    buckets: bucket t holds the cells the wave reaches at time t. The wave of time t is
    processed as ONE numpy array, its neighbours are computed for all cells at once, and
    each neighbour is dropped into the bucket of its arrival time t + cost. (This variant
    is known as Dial's algorithm.) A heap-based Dijkstra in pure Python needs about half a
    second for a 300 x 200 grid, this one a few tens of milliseconds.

    The start cell may be forbidden: the map is noisy, and somebody may drop a crate right
    next to the robot, which puts the robot inside the inflated zone. Refusing to plan
    would leave it stuck there for ever. Instead the wave starts from the nearest
    traversable cell (the "source"), and extract_path adds the short hop from the robot to
    it. Returns None if there is no traversable cell within max_escape_cells.
    """
    height, width = costmap.traversable.shape

    # Close the outermost ring of cells. A wave cell is then never on the border, and its
    # four neighbours always exist: no bounds checks, no wrap-around in the flat indices.
    passable = costmap.traversable.copy()
    passable[0, :] = False
    passable[-1, :] = False
    passable[:, 0] = False
    passable[:, -1] = False

    source = nearest_traversable(passable, start_row, start_col, max_escape_cells)
    if source is None:
        return None

    # Work on flat (1D) views: cell (row, col) has the index row * width + col, and its
    # four neighbours are at index -1, +1, -width, +width.
    flat_passable = passable.ravel()
    flat_cost = costmap.cost.ravel()
    neighbour_steps = np.array([-1, 1, -width, width], dtype=np.intp)
    distance = np.full(height * width, UNREACHABLE, dtype=np.int32)

    bucket_count = int(flat_cost.max()) + 1
    buckets: list[list[np.ndarray]] = [[] for _ in range(bucket_count)]
    source_index = source[0] * width + source[1]
    distance[source_index] = 0
    buckets[0].append(np.array([source_index], dtype=np.intp))
    waiting = 1         # number of index arrays in all buckets together

    time = 0
    while waiting > 0:
        bucket = buckets[time % bucket_count]
        if bucket:
            waiting -= len(bucket)
            wave = np.concatenate(bucket)
            bucket.clear()
            # A cell may have been put here by several neighbours (keep one copy), and it
            # may have been reached earlier by a cheaper route in the meantime (drop it).
            wave = np.unique(wave[distance[wave] == time])

            neighbours = (wave[:, np.newaxis] + neighbour_steps).ravel()
            arrival = time + flat_cost[neighbours]
            improved = flat_passable[neighbours] & (arrival < distance[neighbours])
            neighbours = neighbours[improved]
            arrival = arrival[improved]
            distance[neighbours] = arrival
            for step_cost in range(1, bucket_count):
                arriving = neighbours[arrival == time + step_cost]
                if arriving.size > 0:
                    buckets[(time + step_cost) % bucket_count].append(arriving)
                    waiting += 1
        time += 1

    return DistanceField(distance.reshape(height, width), (start_row, start_col), source)


def extract_path(field: DistanceField, goal_row: int, goal_col: int) -> list[tuple[int, int]] | None:
    """The cheapest cell-by-cell route from the robot to the goal, or None if unreachable.

    Start at the goal and always step to the neighbour with the lowest distance. Every
    reachable cell has a neighbour that is closer to the source, so this ends at the
    source, where the distance is 0. Reversed, it is the path.
    """
    distance = field.distance
    if not field.is_reachable(goal_row, goal_col):
        return None
    path = [(goal_row, goal_col)]
    row, col = goal_row, goal_col
    while (row, col) != field.source:
        best_distance = distance[row, col]
        best_cell = (row, col)
        for d_row, d_col in NEIGHBOURS_4:
            candidate = distance[row + d_row, col + d_col]
            if candidate < best_distance:
                best_distance = candidate
                best_cell = (row + d_row, col + d_col)
        row, col = best_cell
        path.append(best_cell)
    if field.source != field.start:
        path.append(field.start)        # the escape hop out of a forbidden start cell
    path.reverse()
    return path


def line_is_clear(costmap: Costmap, cell_a: tuple[int, int], cell_b: tuple[int, int],
                  max_cost: int) -> bool:
    """May the robot drive straight from cell_a to cell_b?

    Samples the segment every half cell. All sampled cells must be traversable, and none
    may be more expensive than max_cost.
    """
    longest_side = max(abs(cell_b[0] - cell_a[0]), abs(cell_b[1] - cell_a[1]))
    sample_count = 2 * longest_side + 1
    rows = np.rint(np.linspace(cell_a[0], cell_b[0], sample_count)).astype(np.intp)
    cols = np.rint(np.linspace(cell_a[1], cell_b[1], sample_count)).astype(np.intp)
    if not costmap.traversable[rows, cols].all():
        return False
    return int(costmap.cost[rows, cols].max()) <= max_cost


def simplify_path(path: list[tuple[int, int]], costmap: Costmap,
                  stride: int = 4) -> list[tuple[int, int]]:
    """Replace the staircase of grid steps by a few straight segments.

    The grid path only moves up, down, left and right. From an anchor cell we look ahead
    along the path, stride cells at a time, for as long as we could still drive there in a
    straight line. The last cell that passed is kept and becomes the next anchor.

    A shortcut must not undo the work of the costmap: it may not pass through cells that
    are more expensive than the stretch of path it replaces. Otherwise every corner would
    be cut until the path scrapes along the inflated obstacle again.

    (stride trades a little precision at the corners for speed: every look ahead is one
    line_is_clear call, and a path can be more than a thousand cells long.)
    """
    if len(path) <= 2:
        return list(path)
    cells = np.asarray(path, dtype=np.intp)
    cost_along_path = costmap.cost[cells[:, 0], cells[:, 1]]
    last = len(path) - 1
    simplified = [path[0]]
    anchor = 0
    while anchor < last:
        furthest = anchor + 1
        candidate = min(anchor + stride, last)
        while True:
            allowed_cost = int(cost_along_path[anchor:candidate + 1].max())
            if not line_is_clear(costmap, path[anchor], path[candidate], allowed_cost):
                break
            furthest = candidate
            if candidate == last:
                break
            candidate = min(candidate + stride, last)
        simplified.append(path[furthest])
        anchor = furthest
    return simplified


def path_length_cells(path: list[tuple[int, int]]) -> float:
    """Length of a path in cells (multiply by the resolution to get metres)."""
    points = np.asarray(path, dtype=np.float64)
    if len(points) < 2:
        return 0.0
    return float(np.sum(np.hypot(np.diff(points[:, 0]), np.diff(points[:, 1]))))
