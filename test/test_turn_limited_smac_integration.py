from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT / "config" / "nav2_params.yaml"
CONTROLLER = PACKAGE_ROOT / "launch" / "hybrid_pure_pursuit.py"


def test_small_city_uses_only_original_smac_hybrid_for_active_planning():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    params = config["planner_server"]["ros__parameters"]
    planner_plugins = params["planner_plugins"]

    assert "SmacPlannerHybrid" in planner_plugins
    assert "TurnLimitedSmacPlanner" not in planner_plugins
    assert "TurnLimitedSmacPlanner" not in params
    assert params["SmacPlannerHybrid"]["motion_model_for_search"] == "REEDS_SHEPP"


def test_small_city_controller_requests_original_smac_and_keeps_safety_logic():
    controller_source = CONTROLLER.read_text(encoding="utf-8")

    assert 'planner_id="SmacPlannerHybrid"' in controller_source
    assert 'planner_id="TurnLimitedSmacPlanner"' not in controller_source
    assert 'f"{planner_id} không trả path; status={goal_status}."' in controller_source
    assert "self.active_plan_goal.cancel_goal_async()" in controller_source
    assert "if request_id != self.plan_request_id:" in controller_source
    assert "corner_safe_limits(" in controller_source
    assert "SHORT_REVERSE" not in controller_source
    assert "cmd.linear.x = -" not in controller_source
    assert "goal_pose_reached(" in controller_source
    assert "self.goal_yaw_tolerance = 0.3" in controller_source
    assert "goal_pose=goal.pose" in controller_source
    assert "terminal_yaw = yaw_from_quat(terminal.orientation)" in controller_source
