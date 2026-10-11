"""Remember which cones have been found, without counting any cone twice.

The robot sees the same cone in many frames, from different places, and every
sighting lands a few centimetres from the previous one. This module turns that
stream of sightings into a short list of distinct cones.

Two ideas:

* Merging. A sighting close to a cone we already know is the same cone seen
  again. It refines the stored position (a running mean, so the noise of the
  single sightings averages out).
* Confirmation. One sighting proves little: a reflection or a bad lidar match
  can produce it. A cone is only reported once it has been seen several times
  at the same place.

No ROS, no numpy: plain Python, so the logic can be tested in isolation.
"""

import math
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class TrackedCone:
    """One cone in the map frame, and how often it has been seen."""

    x: float
    y: float
    sightings: int = 1
    # None while the cone is still a candidate. Set once, on confirmation, and
    # never changed afterwards: cone 0 stays cone 0.
    id: Optional[int] = None

    @property
    def confirmed(self) -> bool:
        return self.id is not None


class ConeMap:
    """The list of distinct cones, built up from single sightings."""

    def __init__(self, merge_radius: float = 0.6, confirmations: int = 3) -> None:
        """Create an empty map.

        merge_radius: sightings closer than this (metres) to a known cone are
            the same cone. It must be larger than the localisation noise and
            smaller than the distance between two real cones.
        confirmations: how many sightings turn a candidate into a cone.
        """
        self._merge_radius = merge_radius
        self._confirmations = confirmations
        self._cones: List[TrackedCone] = []
        self._next_id = 0

    def add_sighting(self, x: float, y: float) -> TrackedCone:
        """Record that a cone was seen at (x, y) in the map frame.

        Returns the cone the sighting was assigned to: a known one that it
        refined, or a new candidate.
        """
        nearest = self._nearest_within_radius(x, y)
        if nearest is None:
            cone = TrackedCone(x, y)
            self._cones.append(cone)
        else:
            cone = nearest
            cone.sightings += 1
            # Running mean: mean_n = mean_(n-1) + (sample - mean_(n-1)) / n.
            # It gives every sighting the same weight without storing them.
            cone.x += (x - cone.x) / cone.sightings
            cone.y += (y - cone.y) / cone.sightings

        if not cone.confirmed and cone.sightings >= self._confirmations:
            cone.id = self._next_id
            self._next_id += 1
        return cone

    def confirmed_cones(self) -> List[TrackedCone]:
        """Return the confirmed cones, ordered by id."""
        confirmed = [cone for cone in self._cones if cone.confirmed]
        confirmed.sort(key=lambda cone: cone.id)
        return confirmed

    def candidate_count(self) -> int:
        """Return how many cones are still waiting for confirmation."""
        return sum(1 for cone in self._cones if not cone.confirmed)

    def _nearest_within_radius(self, x: float, y: float) -> Optional[TrackedCone]:
        nearest: Optional[TrackedCone] = None
        nearest_distance = self._merge_radius
        for cone in self._cones:
            distance = math.hypot(x - cone.x, y - cone.y)
            if distance <= nearest_distance:
                nearest = cone
                nearest_distance = distance
        return nearest
