"""Tests of the occupancy grid against a synthetic rectangular room. No ROS needed."""

import math
import time

import numpy as np

from explorer_nav.occupancy_grid import GridGeometry, LogOddsConfig, OccupancyGrid

BEAMS = 360
ANGLE_MIN = -math.pi
ANGLE_INCREMENT = 2.0 * math.pi / BEAMS
RANGE_MIN = 0.12
RANGE_MAX = 8.0


def demo_geometry() -> GridGeometry:
    """The grid of the demo: 30 x 20 m at 0.1 m per cell, 300 x 200 cells."""
    return GridGeometry(resolution=0.1, origin_x=-4.0, origin_y=-10.0, width=300, height=200)


def scan_in_room(x: float, y: float, yaw: float, x_min: float, x_max: float,
                 y_min: float, y_max: float) -> np.ndarray:
    """What an ideal lidar at (x, y, yaw) measures inside an empty rectangular room."""
    angles = yaw + ANGLE_MIN + ANGLE_INCREMENT * np.arange(BEAMS)
    direction_x = np.cos(angles)
    direction_y = np.sin(angles)
    with np.errstate(divide='ignore'):
        # Distance along each beam to the wall it is flying towards, per axis.
        to_x_wall = np.where(direction_x > 0, (x_max - x) / direction_x, (x_min - x) / direction_x)
        to_y_wall = np.where(direction_y > 0, (y_max - y) / direction_y, (y_min - y) / direction_y)
    ranges = np.minimum(to_x_wall, to_y_wall)
    ranges[ranges >= RANGE_MAX] = np.inf
    return ranges


def integrate(grid: OccupancyGrid, x: float, y: float, yaw: float, ranges: np.ndarray) -> None:
    grid.integrate_scan(x, y, yaw, ranges, ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX)


def test_geometry_round_trip():
    geometry = demo_geometry()
    row, col = geometry.world_to_cell(0.0, 0.0)
    assert (row, col) == (100, 40)
    x, y = geometry.cell_to_world(row, col)
    assert abs(x - 0.05) < 1e-9 and abs(y - 0.05) < 1e-9


def test_new_grid_is_unknown():
    grid = OccupancyGrid(demo_geometry())
    assert np.all(grid.to_occupancy_data() == -1)


def test_room_is_mapped():
    geometry = demo_geometry()
    grid = OccupancyGrid(geometry)
    # A 6 x 4 m room, the robot a little off-centre and slightly rotated.
    room = dict(x_min=0.0, x_max=6.0, y_min=-2.0, y_max=2.0)
    for _ in range(5):
        integrate(grid, 2.5, 0.3, 0.4, scan_in_room(2.5, 0.3, 0.4, **room))
    data = grid.to_occupancy_data()

    # The floor inside the room is known and free.
    for x, y in [(2.5, 0.3), (1.0, -1.0), (5.0, 1.5), (0.5, 1.5)]:
        row, col = geometry.world_to_cell(x, y)
        assert 0 <= data[row, col] < 30, f'({x}, {y}) should be free, is {data[row, col]}'

    # The walls are occupied. A beam that ends exactly on the wall falls into the cell
    # just inside or just outside the room, so look at both.
    for x, y in [(6.0, 0.0), (0.0, 0.5), (3.0, 2.0), (3.0, -2.0)]:
        row, col = geometry.world_to_cell(x, y)
        patch = data[row - 1:row + 2, col - 1:col + 2]
        assert patch.max() > 70, f'no wall found near ({x}, {y})'

    # Nothing behind the walls has been seen.
    for x, y in [(8.0, 0.0), (-2.0, 0.0), (3.0, 4.0), (3.0, -4.0)]:
        row, col = geometry.world_to_cell(x, y)
        assert data[row, col] == -1, f'({x}, {y}) is behind a wall and should be unknown'

    # About 24 m^2 of floor. The cells along the walls are occupied, hence a bit less.
    free_area = np.count_nonzero((data >= 0) & (data < 50)) * geometry.resolution ** 2
    assert 20.0 < free_area < 24.5


def test_beams_without_a_hit_clear_up_to_range_max():
    geometry = demo_geometry()
    grid = OccupancyGrid(geometry)
    integrate(grid, 10.0, 0.0, 0.0, np.full(BEAMS, np.inf))
    data = grid.to_occupancy_data()
    assert not np.any(data > 50), 'a scan that hit nothing must not create obstacles'
    row, col = geometry.world_to_cell(10.0 + 7.5, 0.0)
    assert 0 <= data[row, col] < 50
    row, col = geometry.world_to_cell(10.0 + 8.5, 0.0)
    assert data[row, col] == -1


def test_no_unseen_stripes_between_beams():
    """With several rays per beam, the disc swept by the lidar has no holes in it."""
    geometry = demo_geometry()
    grid = OccupancyGrid(geometry)
    integrate(grid, 10.0, 0.0, 0.0, np.full(BEAMS, np.inf))
    rows, cols = np.indices((geometry.height, geometry.width))
    centre_row, centre_col = geometry.world_to_cell(10.0, 0.0)
    within_7_m = np.hypot(rows - centre_row, cols - centre_col) * geometry.resolution < 7.0
    assert grid.observed[within_7_m].all()


def test_log_odds_are_clamped_and_the_map_can_change_its_mind():
    geometry = demo_geometry()
    config = LogOddsConfig()
    grid = OccupancyGrid(geometry, config)
    room = dict(x_min=0.0, x_max=6.0, y_min=-2.0, y_max=2.0)
    ranges = scan_in_room(3.0, 0.0, 0.0, **room)
    for _ in range(100):
        integrate(grid, 3.0, 0.0, 0.0, ranges)
    assert grid.log_odds.min() >= config.minimum - 1e-6
    assert grid.log_odds.max() <= config.maximum + 1e-6

    # Now a crate appears 1 m in front of the robot. After one second of scans (10 of
    # them) it must be on the map, although that cell was seen free a hundred times.
    with_crate = ranges.copy()
    with_crate[175:186] = 1.0
    for _ in range(10):
        integrate(grid, 3.0, 0.0, 0.0, with_crate)
    row, col = geometry.world_to_cell(4.0, 0.0)
    patch = grid.to_occupancy_data()[row - 1:row + 2, col - 1:col + 2]
    assert patch.max() > 70


def test_walls_seen_at_a_shallow_angle_are_not_erased():
    """Drive down a long corridor, then stand still. Both walls must stay on the map.

    From inside a corridor most beams meet the walls at a shallow angle, where the
    measured distance changes a lot from one beam to the next. A grid that lets the side
    rays of a beam run as far as the beam itself pokes through the wall with them. While
    the robot stands still they poke through the same cells scan after scan, and erase
    the wall it mapped a moment ago. (The explorer then finds "frontiers" in the holes
    and never finishes.)
    """
    geometry = demo_geometry()
    grid = OccupancyGrid(geometry)
    corridor = dict(x_min=0.0, x_max=20.0, y_min=-1.2, y_max=1.2)
    random = np.random.default_rng(7)
    drive = np.arange(1.0, 19.0, 0.1)
    stand = np.full(100, 10.0)                          # ten seconds without moving
    for x in np.concatenate([drive, drive[::-1], drive[:90], stand]):
        ranges = scan_in_room(x, 0.3, 0.0, **corridor)
        ranges += random.normal(0.0, 0.01, BEAMS)       # 1 cm of sensor noise
        integrate(grid, x, 0.3, 0.0, ranges)
    data = grid.to_occupancy_data()

    # A noisy hit on a wall lands in the cell row just inside or just outside the
    # corridor. Every 10 cm of wall must be marked in at least one of the two.
    first_col = geometry.world_to_cell(1.0, 0.0)[1]
    last_col = geometry.world_to_cell(19.0, 0.0)[1]
    for wall_y in (corridor['y_min'], corridor['y_max']):
        wall_row = geometry.world_to_cell(0.0, wall_y + 0.01)[0]
        rows = data[wall_row - 1:wall_row + 1, first_col:last_col]
        holes = np.count_nonzero(rows.max(axis=0) < 50)
        assert holes == 0, f'{holes} holes in the wall at y = {wall_y}'


def test_invalid_readings_are_ignored():
    grid = OccupancyGrid(demo_geometry())
    ranges = np.full(BEAMS, np.nan)
    ranges[:10] = 0.05          # below range_min: the lidar sees its own housing
    integrate(grid, 10.0, 0.0, 0.0, ranges)
    assert not grid.observed.any()


def test_scan_integration_is_fast_enough():
    """Budget: 10 ms per scan (the lidar sends 10 per second). Asserted with slack x3."""
    grid = OccupancyGrid(demo_geometry())
    ranges = np.full(BEAMS, np.inf)         # the worst case: every beam at full length
    ranges[::3] = 5.0
    durations = []
    for _ in range(10):
        started = time.perf_counter()
        integrate(grid, 10.0, 0.0, 0.3, ranges)
        durations.append(time.perf_counter() - started)
    best = min(durations)
    print(f'scan integration: best {best * 1e3:.2f} ms, median {np.median(durations) * 1e3:.2f} ms')
    assert best < 0.030
