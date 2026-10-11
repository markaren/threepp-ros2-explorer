"""Frontier detection: where does the known world end? Plain numpy, no ROS.

A frontier cell is a cell we know to be free that touches a cell we know nothing about.
Standing there, the robot would look into unexplored space. Exploring a building is then a
simple loop: drive to a frontier, the map grows, the frontier moves on or disappears, repeat
until none is left. (Yamauchi, "A frontier-based approach for autonomous exploration", 1997.)

Single frontier cells are no use as goals, there are hundreds of them. Cells that touch each
other form one cluster (a doorway, the open end of an aisle), and the robot aims for the
centre of a cluster.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# 8-connected: diagonal neighbours count. A frontier that runs diagonally across the grid
# is a staircase of cells that only touch at their corners, and it should stay one cluster.
NEIGHBOURS_8 = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))


@dataclass
class Frontier:
    """One cluster of frontier cells."""

    rows: np.ndarray        # row index of every cell in the cluster
    cols: np.ndarray        # column index of every cell in the cluster
    centroid_row: float     # mean of rows (a fractional cell coordinate)
    centroid_col: float     # mean of cols

    @property
    def size(self) -> int:
        """Number of cells in the cluster."""
        return int(self.rows.size)


def frontier_mask(free: np.ndarray, unknown: np.ndarray) -> np.ndarray:
    """True for every free cell with at least one unknown 4-neighbour.

    Instead of visiting each cell and looking at its neighbours, the whole unknown mask is
    shifted by one cell in each of the four directions. Cells outside the grid do not count
    as unknown.
    """
    touches_unknown = np.zeros_like(unknown)
    touches_unknown[1:, :] |= unknown[:-1, :]      # the neighbour one row below is unknown
    touches_unknown[:-1, :] |= unknown[1:, :]      # one row above
    touches_unknown[:, 1:] |= unknown[:, :-1]      # one column to the left
    touches_unknown[:, :-1] |= unknown[:, 1:]      # one column to the right
    return free & touches_unknown


def cluster_cells(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    """Group the True cells of mask into 8-connected clusters (a flood fill).

    This is the one place with a Python loop, but it runs over the frontier cells only
    (a few hundred), never over the whole grid.
    """
    rows, cols = np.nonzero(mask)
    unvisited = set(zip(rows.tolist(), cols.tolist()))
    clusters = []
    while unvisited:
        seed = unvisited.pop()
        members = [seed]
        stack = [seed]
        while stack:
            row, col = stack.pop()
            for d_row, d_col in NEIGHBOURS_8:
                neighbour = (row + d_row, col + d_col)
                if neighbour in unvisited:
                    unvisited.remove(neighbour)
                    members.append(neighbour)
                    stack.append(neighbour)
        clusters.append(members)
    return clusters


def find_frontiers(free: np.ndarray, unknown: np.ndarray, min_size: int = 8) -> list[Frontier]:
    """All frontier clusters with at least min_size cells, largest first.

    min_size filters out noise: a single never-seen cell in the middle of the floor has
    four frontier cells around it, and is not worth a trip.
    """
    frontiers = []
    for members in cluster_cells(frontier_mask(free, unknown)):
        if len(members) < min_size:
            continue
        cells = np.array(members, dtype=np.intp)
        rows = cells[:, 0]
        cols = cells[:, 1]
        frontiers.append(Frontier(rows, cols, float(rows.mean()), float(cols.mean())))
    frontiers.sort(key=lambda frontier: frontier.size, reverse=True)
    return frontiers
