import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "launch" / "hybrid_pure_pursuit.py"


def load_controller_module():
    spec = importlib.util.spec_from_file_location("hybrid_pure_pursuit", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def pose(x, y, yaw):
    return SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y),
            orientation=SimpleNamespace(
                x=0.0, y=0.0,
                z=math.sin(yaw / 2.0),
                w=math.cos(yaw / 2.0),
            ),
        )
    )


def test_path_direction_annotates_forward_and_reverse_segments():
    module = load_controller_module()

    forward = module.annotate_path_directions([pose(0, 0, 0), pose(1, 0, 0)])
    reverse = module.annotate_path_directions([pose(0, 0, 0), pose(-1, 0, 0)])

    assert forward == [(0, 0, 1), (1, 0, 1)]
    assert reverse == [(0, 0, -1), (-1, 0, -1)]


def test_downsampling_keeps_the_terminal_forward_waypoint():
    module = load_controller_module()
    points = [(0.0, 0.0, 1), (0.05, 0.0, 1), (0.08, 0.0, 1),
              (0.5, 0.0, 1)]

    sampled = module.downsample_directional_path(points, min_dist=0.1)

    assert sampled[-1] == (0.5, 0.0, 1)


def test_downsampling_preserves_a_direction_change_cusp():
    module = load_controller_module()
    points = [
        (0.0, 0.0, 1),
        (0.05, 0.0, 1),
        (0.08, 0.0, -1),
        (-0.5, 0.0, -1),
    ]

    sampled = module.downsample_directional_path(points, min_dist=0.1)

    assert (0.08, 0.0, -1) in sampled
    assert sampled[-1] == (-0.5, 0.0, -1)


def test_controller_uses_signed_speed_for_reeds_shepp_segments():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "do_transform_pose_stamped" in source
    assert "SHORT_REVERSE" not in source
    assert "signed_command_speed(cmd_speed_magnitude, desired_direction)" in source
    assert "cmd.linear.x = cmd_speed" in source


def test_signed_command_speed_follows_path_direction():
    module = load_controller_module()

    assert module.signed_command_speed(2.0, 1) == 2.0
    assert module.signed_command_speed(2.0, -1) == -2.0
    assert module.signed_command_speed(-2.0, -1) == -2.0
    assert module.signed_command_speed(math.nan, 1) == 0.0
    assert module.signed_command_speed(1.0, 0) == 0.0


def test_goal_reached_requires_position_and_wrapped_yaw_tolerance():
    module = load_controller_module()

    assert module.goal_pose_reached(
        1.0, 2.0, -math.pi + 0.05,
        1.1, 2.1, math.pi - 0.05,
        xy_tolerance=0.5,
        yaw_tolerance=0.2,
    )
    assert not module.goal_pose_reached(
        1.0, 2.0, 0.5,
        1.1, 2.1, 0.0,
        xy_tolerance=0.5,
        yaw_tolerance=0.2,
    )
    assert not module.goal_pose_reached(
        1.0, 2.0, 0.0,
        2.0, 2.0, 0.0,
        xy_tolerance=0.5,
        yaw_tolerance=0.2,
    )


def test_corner_limits_do_not_force_full_lock_steering():
    module = load_controller_module()

    steer, speed_limit = module.corner_safe_limits(
        steer=0.62,
        alpha=1.2,
        max_steer=0.6458,
    )

    assert 0.0 < steer < 0.62
    assert speed_limit < 0.8


def test_corner_limits_leave_gentle_turn_unchanged():
    module = load_controller_module()

    steer, speed_limit = module.corner_safe_limits(
        steer=-0.25,
        alpha=-0.4,
        max_steer=0.6458,
    )

    assert steer == -0.25
    assert speed_limit == 2.0
