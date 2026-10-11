"""Tests of frontier detection. No ROS needed."""

import numpy as np

from explorer_nav.frontiers import cluster_cells, find_frontiers, frontier_mask


def empty_masks(height: int = 40, width: int = 60) -> tuple[np.ndarray, np.ndarray]:
    """A grid where everything is unknown."""
    free = np.zeros((height, width), dtype=bool)
    unknown = np.ones((height, width), dtype=bool)
    return free, unknown


def mark_free(free: np.ndarray, unknown: np.ndarray, rows: slice, cols: slice) -> None:
    free[rows, cols] = True
    unknown[rows, cols] = False


def test_boundary_between_free_and_unknown_is_a_frontier():
    free, unknown = empty_masks()
    mark_free(free, unknown, slice(None), slice(0, 20))     # the left third is known free

    mask = frontier_mask(free, unknown)
    assert mask[:, 19].all(), 'the last free column touches unknown cells'
    assert not mask[:, :19].any(), 'cells deeper inside the free area are no frontier'
    assert not mask[:, 20:].any(), 'unknown cells themselves are no frontier'

    frontiers = find_frontiers(free, unknown, min_size=5)
    assert len(frontiers) == 1
    assert frontiers[0].size == 40
    assert abs(frontiers[0].centroid_col - 19.0) < 1e-9
    assert abs(frontiers[0].centroid_row - 19.5) < 1e-9


def test_a_wall_is_not_a_frontier():
    free, unknown = empty_masks()
    mark_free(free, unknown, slice(None), slice(0, 20))
    unknown[:, 20] = False      # column 20 is an occupied wall: known, and not free
    assert find_frontiers(free, unknown, min_size=1) == []


def test_the_grid_border_is_not_a_frontier():
    free, unknown = empty_masks()
    mark_free(free, unknown, slice(None), slice(None))      # everything known free
    assert not frontier_mask(free, unknown).any()


def test_two_openings_give_two_clusters_and_small_ones_are_dropped():
    free, unknown = empty_masks()
    # A known room with walls all around (known, not free)...
    unknown[5:35, 5:45] = False
    free[6:34, 6:44] = True
    # ...and two doors in the right wall, 10 cells and 2 cells wide.
    free[10:20, 44] = True
    free[28:30, 44] = True

    frontiers = find_frontiers(free, unknown, min_size=5)
    assert len(frontiers) == 1
    assert frontiers[0].size == 10
    assert abs(frontiers[0].centroid_row - 14.5) < 1e-9

    everything = find_frontiers(free, unknown, min_size=1)
    assert [frontier.size for frontier in everything] == [10, 2]


def test_diagonal_cells_belong_to_the_same_cluster():
    mask = np.zeros((10, 10), dtype=bool)
    mask[2, 2] = mask[3, 3] = mask[4, 4] = True     # a diagonal staircase
    mask[8, 1] = True                               # a cell far away
    clusters = cluster_cells(mask)
    assert sorted(len(cluster) for cluster in clusters) == [1, 3]
