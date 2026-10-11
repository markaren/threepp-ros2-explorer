"""The cone map: merging repeated sightings, confirming, and stable ids."""

import pytest

from explorer_vision.cone_map import ConeMap


def test_a_cone_needs_three_sightings():
    cone_map = ConeMap(merge_radius=0.6, confirmations=3)

    cone_map.add_sighting(2.0, 1.0)
    cone_map.add_sighting(2.0, 1.0)
    assert cone_map.confirmed_cones() == []
    assert cone_map.candidate_count() == 1

    cone = cone_map.add_sighting(2.0, 1.0)
    assert cone.confirmed
    assert cone.id == 0
    assert cone_map.candidate_count() == 0


def test_nearby_sightings_merge_into_their_mean():
    cone_map = ConeMap(merge_radius=0.6, confirmations=3)

    cone_map.add_sighting(2.0, 1.0)
    cone_map.add_sighting(2.3, 1.0)
    cone_map.add_sighting(2.0, 1.3)

    cones = cone_map.confirmed_cones()
    assert len(cones) == 1
    assert cones[0].sightings == 3
    assert cones[0].x == pytest.approx(2.1)
    assert cones[0].y == pytest.approx(1.1)


def test_sightings_far_apart_are_different_cones():
    cone_map = ConeMap(merge_radius=0.6, confirmations=1)

    cone_map.add_sighting(0.0, 0.0)
    cone_map.add_sighting(0.7, 0.0)

    assert [cone.id for cone in cone_map.confirmed_cones()] == [0, 1]


def test_a_sighting_between_two_cones_goes_to_the_nearer_one():
    cone_map = ConeMap(merge_radius=0.6, confirmations=1)
    cone_map.add_sighting(0.0, 0.0)
    cone_map.add_sighting(1.0, 0.0)

    cone = cone_map.add_sighting(0.6, 0.0)

    assert cone.id == 1
    assert len(cone_map.confirmed_cones()) == 2


def test_ids_are_stable_and_follow_the_order_of_confirmation():
    cone_map = ConeMap(merge_radius=0.6, confirmations=3)

    # A is seen first but only once: it stays a candidate and takes no id.
    cone_map.add_sighting(0.0, 0.0)
    # B is confirmed first, so B is cone 0.
    for _ in range(3):
        cone_map.add_sighting(5.0, 0.0)
    # Now A catches up and becomes cone 1.
    cone_map.add_sighting(0.0, 0.0)
    cone_map.add_sighting(0.0, 0.0)
    # More sightings of B move its position a little, never its id.
    for _ in range(10):
        cone_map.add_sighting(5.1, 0.1)

    cones = cone_map.confirmed_cones()
    assert [cone.id for cone in cones] == [0, 1]
    assert cones[0].x == pytest.approx(5.0, abs=0.15)
    assert cones[1].x == pytest.approx(0.0)
