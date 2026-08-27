import importlib.util
import math
from pathlib import Path

import pytest
from geometry_msgs.msg import PoseStamped


SCRIPT = Path(__file__).resolve().parents[1] / "launch" / "hybrid_pure_pursuit.py"


def load_controller_module():
    spec = importlib.util.spec_from_file_location("forward_anchor_controller", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def pose(x, y, yaw=0.0):
    value = PoseStamped()
    value.header.frame_id = "map"
    value.pose.position.x = x
    value.pose.position.y = y
    value.pose.orientation.z = math.sin(yaw / 2.0)
    value.pose.orientation.w = math.cos(yaw / 2.0)
    return value


def test_forward_anchor_projects_from_vehicle_front_and_preserves_heading():
    module = load_controller_module()

    anchor = module.forward_anchor_pose(1.0, 2.0, math.pi / 2.0, 4.0)

    assert anchor.header.frame_id == "map"
    assert anchor.pose.position.x == pytest.approx(1.0)
    assert anchor.pose.position.y == pytest.approx(6.0)
    assert module.yaw_from_quat(anchor.pose.orientation) == pytest.approx(math.pi / 2.0)


def test_merge_path_poses_removes_duplicate_anchor():
    module = load_controller_module()
    first = [pose(0.0, 0.0), pose(4.0, 0.0)]
    second = [pose(4.0, 0.0), pose(6.0, 1.0)]

    merged = module.merge_path_poses(first, second)

    assert [(p.pose.position.x, p.pose.position.y) for p in merged] == [
        (0.0, 0.0),
        (4.0, 0.0),
        (6.0, 1.0),
    ]


def test_controller_calls_smac_directly_without_forward_anchor():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "self.forward_anchor_distances" not in source
    assert "self.forward_anchor_profiles" not in source
    assert 'stage="direct"' in source
    assert 'planner_id="GridBased"' not in source


def test_hybrid_planner_restores_working_smac_costs():
    config = SCRIPT.parents[1] / "config" / "nav2_params.yaml"
    source = config.read_text(encoding="utf-8")

    assert 'motion_model_for_search: "REEDS_SHEPP"' in source
    assert "reverse_penalty: 2.0" in source


def test_merge_three_forward_segments_keeps_order_and_removes_joints():
    module = load_controller_module()
    merged = []
    for segment in (
        [pose(0.0, 0.0), pose(4.0, 0.0)],
        [pose(4.0, 0.0), pose(8.0, 0.0)],
        [pose(8.0, 0.0), pose(12.0, 0.0)],
    ):
        merged = module.merge_path_poses(merged, segment)

    assert [p.pose.position.x for p in merged] == [0.0, 4.0, 8.0, 12.0]
