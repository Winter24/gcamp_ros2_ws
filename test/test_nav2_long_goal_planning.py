from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT / "config" / "nav2_params.yaml"
CONTROLLER = PACKAGE_ROOT / "launch" / "hybrid_pure_pursuit.py"


def test_smac_search_restores_original_full_resolution_settings():
    source = CONFIG.read_text(encoding="utf-8")

    assert "downsample_costmap: false" in source
    assert "downsampling_factor: 1" in source
    assert "angle_quantization_bins: 72" in source
    assert "cache_obstacle_heuristic: false" in source


def test_smac_restores_working_reeds_shepp_settings():
    source = CONFIG.read_text(encoding="utf-8")
    config = yaml.safe_load(source)
    smac = config["planner_server"]["ros__parameters"]["SmacPlannerHybrid"]

    assert "analytic_expansion_ratio: 3.5" in source
    assert "analytic_expansion_max_length: 3.0" in source
    assert smac["motion_model_for_search"] == "REEDS_SHEPP"
    assert smac["reverse_penalty"] == 2.0
    assert smac["change_penalty"] == 0.2
    assert smac["non_straight_penalty"] == 1.5
    assert smac["max_iterations"] == 2500000
    assert smac["max_planning_time"] == 10.0
    assert smac["max_on_approach_iterations"] == 1000


def test_only_latest_goal_result_can_replace_waypoints():
    source = CONTROLLER.read_text(encoding="utf-8")

    assert "self.plan_request_id += 1" in source
    assert "if request_id != self.plan_request_id:" in source
    assert "self.active_plan_goal.cancel_goal_async()" in source


def test_empty_smac_path_does_not_fall_back_to_grid_planner():
    source = CONTROLLER.read_text(encoding="utf-8")

    assert 'planner_id="GridBased"' not in source
    assert "SmacForwardOnly" not in source
    assert "planning_time" in source
    assert "goal_status" in source
