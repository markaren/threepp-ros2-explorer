"""Tests of the path follower against a simulated unicycle. No ROS needed."""

import math

import numpy as np

from explorer_nav.pure_pursuit import PurePursuit, PurePursuitConfig, wrap_angle

TIME_STEP = 0.05        # s: the control loop runs at 20 Hz


def drive(follower: PurePursuit, x: float, y: float, yaw: float,
          max_steps: int = 2000) -> tuple[np.ndarray, bool]:
    """Let an ideal unicycle follow the path. Returns the poses visited and whether it arrived."""
    poses = [(x, y, yaw)]
    for _ in range(max_steps):
        command = follower.compute(x, y, yaw)
        if command.reached:
            return np.array(poses), True
        x += command.linear * math.cos(yaw) * TIME_STEP
        y += command.linear * math.sin(yaw) * TIME_STEP
        yaw = wrap_angle(yaw + command.angular * TIME_STEP)
        poses.append((x, y, yaw))
    return np.array(poses), False


def distance_to_polyline(point: np.ndarray, polyline: np.ndarray) -> float:
    best = math.inf
    for start, end in zip(polyline[:-1], polyline[1:]):
        direction = end - start
        t = np.clip(np.dot(point - start, direction) / np.dot(direction, direction), 0.0, 1.0)
        best = min(best, float(np.linalg.norm(point - (start + t * direction))))
    return best


def test_wrap_angle():
    assert abs(wrap_angle(3.0 * math.pi) - (-math.pi)) < 1e-9
    assert abs(wrap_angle(-0.5) - (-0.5)) < 1e-9
    assert abs(wrap_angle(2.0 * math.pi + 0.25) - 0.25) < 1e-9


def test_no_path_means_no_motion():
    command = PurePursuit().compute(0.0, 0.0, 0.0)
    assert (command.linear, command.angular, command.reached) == (0.0, 0.0, False)


def test_converges_on_an_l_shaped_path():
    path = np.array([[0.0, 0.0], [4.0, 0.0], [4.0, 3.0]])
    follower = PurePursuit()
    follower.set_path(path)
    # Start beside the path and pointing the wrong way a little.
    poses, arrived = drive(follower, 0.0, 0.3, -0.5)
    assert arrived
    assert np.hypot(poses[-1, 0] - 4.0, poses[-1, 1] - 3.0) < follower.config.goal_tolerance

    # After the first metre the robot is on the path, and stays within 15 cm of it,
    # corner included (pure pursuit rounds corners off, by design).
    settled = poses[poses[:, 0] > 1.0]
    worst = max(distance_to_polyline(pose[:2], path) for pose in settled)
    assert worst < 0.15, f'strayed {worst:.2f} m from the path'


def test_commands_respect_the_speed_limits():
    config = PurePursuitConfig()
    follower = PurePursuit(config)
    follower.set_path(np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0], [0.0, 4.0]]))
    x, y, yaw = 0.0, 0.0, 2.0
    for _ in range(3000):
        command = follower.compute(x, y, yaw)
        if command.reached:
            break
        assert 0.0 <= command.linear <= config.max_linear_speed + 1e-9
        assert abs(command.angular) <= config.max_angular_speed + 1e-9
        x += command.linear * math.cos(yaw) * TIME_STEP
        y += command.linear * math.sin(yaw) * TIME_STEP
        yaw = wrap_angle(yaw + command.angular * TIME_STEP)
    assert command.reached


def test_turns_in_place_when_facing_away():
    follower = PurePursuit()
    follower.set_path(np.array([[0.0, 0.0], [3.0, 0.0]]))
    command = follower.compute(0.0, 0.0, math.pi * 0.9)     # the path is behind the robot
    assert command.linear == 0.0
    assert abs(command.angular) > 0.5
    poses, arrived = drive(follower, 0.0, 0.0, math.pi * 0.9)
    assert arrived
    assert np.abs(poses[:, 1]).max() < 0.1, 'turning on the spot must not swing the robot out'


def test_slows_down_before_the_goal():
    follower = PurePursuit()
    follower.set_path(np.array([[0.0, 0.0], [5.0, 0.0]]))
    cruising = follower.compute(1.0, 0.0, 0.0)
    braking = follower.compute(4.8, 0.0, 0.0)
    assert abs(cruising.linear - follower.config.max_linear_speed) < 1e-9
    assert 0.0 < braking.linear < 0.5 * follower.config.max_linear_speed


def test_final_heading_is_reached():
    follower = PurePursuit()
    follower.set_path(np.array([[0.0, 0.0], [2.0, 0.0]]), final_yaw=math.pi / 2.0)
    poses, arrived = drive(follower, 0.0, 0.0, 0.0)
    assert arrived
    assert abs(wrap_angle(poses[-1, 2] - math.pi / 2.0)) < follower.config.yaw_tolerance


def test_passing_near_the_goal_early_is_not_an_arrival():
    """A U-shaped path ends next to where it starts. Only the end of the path counts."""
    follower = PurePursuit()
    follower.set_path(np.array([[0.0, 0.0], [3.0, 0.0], [3.0, 0.6], [0.0, 0.6]]))
    # The robot starts 5 cm from the END of the path, which is closer than the tolerance.
    first = follower.compute(0.0, 0.55, 0.0)
    assert not first.reached
    poses, arrived = drive(follower, 0.0, 0.55, 0.0)
    assert arrived
    assert poses[:, 0].max() > 2.5, 'the robot drove the whole U'
    assert np.hypot(poses[-1, 0] - 0.0, poses[-1, 1] - 0.6) < follower.config.goal_tolerance
