"""A log-odds occupancy grid built from 2D laser scans. Plain numpy, no ROS.

The floor is cut into square cells. For every cell we keep one number: how strongly we
believe that something stands there. A laser beam tells us two things at once:

* every cell the beam flew through is probably FREE (otherwise the beam would have stopped),
* the cell where the beam ended is probably OCCUPIED (if it ended on something at all).

One beam is weak evidence, so we do not write "free" or "occupied" into the cell. We add a
small amount of evidence, and let many beams from many scans decide together.

Why log-odds? With probabilities, combining evidence means Bayes' rule, a multiplication and
a normalisation per cell. If we store L = log(p / (1 - p)) instead, the same update is a
plain addition:

    L_new = L_old + L_measurement

L = 0 means p = 0.5 (no idea), L > 0 means "probably occupied", L < 0 "probably free".
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GridGeometry:
    """Where the grid lies in the world, and how to convert between metres and cells.

    Cell (row, col) covers the square whose lower-left corner is at
    (origin_x + col * resolution, origin_y + row * resolution). Rows run along +y, columns
    along +x. This is the layout of nav_msgs/OccupancyGrid, so data[row * width + col] of
    the message is the cell (row, col) here.
    """

    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        """The (row, col) of the cell that contains the point (x, y). May be outside the grid."""
        col = int(np.floor((x - self.origin_x) / self.resolution))
        row = int(np.floor((y - self.origin_y) / self.resolution))
        return row, col

    def cell_to_world(self, row: float, col: float) -> tuple[float, float]:
        """The (x, y) of the centre of cell (row, col). Fractional cells are allowed."""
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def contains(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width


@dataclass(frozen=True)
class LogOddsConfig:
    """How much one beam counts, and how sure the map is allowed to become.

    hit is larger than -miss on purpose: many beams graze past the corner of an obstacle
    cell without hitting it, and a wall that was really seen must survive those.

    The clamps keep the map able to change its mind. Without them, a cell seen free ten
    thousand times would need ten thousand hits before a crate dropped there showed up.
    """

    hit: float = 0.85       # added to the cell where a beam ended (p = 0.70)
    miss: float = -0.4      # added to every cell a beam flew through (p = 0.40)
    minimum: float = -2.0   # lower clamp (p = 0.12)
    maximum: float = 3.5    # upper clamp (p = 0.97)


class OccupancyGrid:
    """The map: one log-odds value per cell, updated scan by scan."""

    def __init__(self, geometry: GridGeometry, config: LogOddsConfig | None = None,
                 rays_per_beam: int = 3) -> None:
        """
        rays_per_beam: a real laser beam is a narrow cone, not a line. Two neighbouring
        beams of a 360-beam lidar are 14 cm apart at 8 m, more than one 10 cm cell, so thin
        mathematical rays would leave stripes of never-seen cells between them. We therefore
        trace a few rays per beam, spread evenly over the beam's angular slot.
        """
        self.geometry = geometry
        self.config = config if config is not None else LogOddsConfig()
        self.rays_per_beam = rays_per_beam
        shape = (geometry.height, geometry.width)
        self.log_odds = np.zeros(shape, dtype=np.float32)
        # L = 0 is ambiguous: "never looked at" or "looked at, evidence cancels out".
        # The map message needs to tell them apart (-1 against 50), so we remember.
        self.observed = np.zeros(shape, dtype=bool)

    def clear(self) -> None:
        """Forget everything."""
        self.log_odds.fill(0.0)
        self.observed.fill(False)

    def integrate_scan(self, x: float, y: float, yaw: float, ranges: np.ndarray,
                       angle_min: float, angle_increment: float,
                       range_min: float, range_max: float) -> None:
        """Add the evidence of one laser scan taken from the sensor pose (x, y, yaw).

        Beam i points at yaw + angle_min + i * angle_increment. A beam that reports +inf hit
        nothing, so everything up to range_max along it is free. NaN and readings below
        range_min are ignored.

        There is no Python loop over cells or beams here. All beams are sampled at once into
        one big (rays x samples) array of points, the points are turned into cell indices,
        and two boolean masks (cells seen free, cells seen occupied) collect the result.
        A mask also guarantees that a cell crossed by five samples of the same beam is
        updated once, not five times.
        """
        geometry = self.geometry
        ranges = np.asarray(ranges, dtype=np.float64)
        beam_angles = yaw + angle_min + angle_increment * np.arange(ranges.size)

        is_hit = np.isfinite(ranges) & (ranges >= range_min) & (ranges < range_max)
        is_clear = (ranges >= range_max) & ~np.isnan(ranges)     # includes +inf
        free_length = np.zeros_like(ranges)
        free_length[is_hit] = ranges[is_hit]
        free_length[is_clear] = range_max

        # ---- cells seen free: sample points along every ray, half a cell apart ----
        # The extra rays of a beam point a little to its left and right. How far may they
        # go? Not as far as the beam itself: along a wall seen at a shallow angle, the
        # distance changes a lot from one beam to the next, and a side ray with the full
        # length would poke through the wall and slowly erase it. A side ray therefore
        # stops at the shorter of its own beam and the neighbouring beam on its side.
        slot = (np.arange(self.rays_per_beam) + 0.5) / self.rays_per_beam - 0.5
        shorter_than_previous = np.minimum(free_length, np.roll(free_length, 1))
        shorter_than_next = np.minimum(free_length, np.roll(free_length, -1))
        ray_length = np.empty((ranges.size, self.rays_per_beam))
        ray_length[:, slot < 0] = shorter_than_previous[:, np.newaxis]
        ray_length[:, slot == 0] = free_length[:, np.newaxis]
        ray_length[:, slot > 0] = shorter_than_next[:, np.newaxis]
        ray_length = ray_length.ravel()
        ray_angles = (beam_angles[:, np.newaxis] + slot * angle_increment).ravel()

        step = 0.5 * geometry.resolution
        distances = np.arange(0.0, range_max, step)
        # Stop one step short of the end, so the cell that stopped the beam is not marked free.
        along_ray = distances[np.newaxis, :] < (ray_length[:, np.newaxis] - step)
        sample_x = x + distances[np.newaxis, :] * np.cos(ray_angles)[:, np.newaxis]
        sample_y = y + distances[np.newaxis, :] * np.sin(ray_angles)[:, np.newaxis]
        free_mask = self._mask_of_points(sample_x[along_ray], sample_y[along_ray])

        # ---- cells seen occupied: the end point of every beam that hit something ----
        end_x = x + ranges[is_hit] * np.cos(beam_angles[is_hit])
        end_y = y + ranges[is_hit] * np.sin(beam_angles[is_hit])
        occupied_mask = self._mask_of_points(end_x, end_y)

        # A cell can be both: one beam ends in it, its neighbour grazes through it.
        # Being hit is the stronger statement.
        free_mask &= ~occupied_mask

        self.log_odds[free_mask] += self.config.miss
        self.log_odds[occupied_mask] += self.config.hit
        np.clip(self.log_odds, self.config.minimum, self.config.maximum, out=self.log_odds)
        self.observed |= free_mask
        self.observed |= occupied_mask

    def _mask_of_points(self, points_x: np.ndarray, points_y: np.ndarray) -> np.ndarray:
        """A boolean grid that is True in every cell containing at least one of the points."""
        geometry = self.geometry
        cols = np.floor((points_x - geometry.origin_x) / geometry.resolution).astype(np.intp)
        rows = np.floor((points_y - geometry.origin_y) / geometry.resolution).astype(np.intp)
        inside = (cols >= 0) & (cols < geometry.width) & (rows >= 0) & (rows < geometry.height)
        mask = np.zeros((geometry.height, geometry.width), dtype=bool)
        mask[rows[inside], cols[inside]] = True
        return mask

    def probabilities(self) -> np.ndarray:
        """Occupancy probability of every cell, from the log-odds: p = 1 / (1 + exp(-L))."""
        return 1.0 / (1.0 + np.exp(-self.log_odds))

    def to_occupancy_data(self) -> np.ndarray:
        """The grid in the layout of nav_msgs/OccupancyGrid.

        Returns an int8 array of shape (height, width): -1 for cells never observed,
        otherwise the occupancy probability in percent (0..100). Flatten it row by row
        (numpy's default order) to get the message's data field.
        """
        data = np.rint(100.0 * self.probabilities()).astype(np.int8)
        data[~self.observed] = -1
        return data
