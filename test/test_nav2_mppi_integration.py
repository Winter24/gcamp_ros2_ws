from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PARAMS = PACKAGE_ROOT / "config" / "nav2_params.yaml"
CAR_NAV = PACKAGE_ROOT / "launch" / "car_navigation.launch.py"
SIM_BRINGUP = PACKAGE_ROOT / "launch" / "sim_bringup.launch.py"
BT = PACKAGE_ROOT / "behavior_trees" / "navigate_to_pose_ackermann.xml"
GOAL_BRIDGE = PACKAGE_ROOT / "launch" / "goal_pose_nav2_bridge.py"


def test_nav2_config_restores_pre_mppi_controller_and_smac_settings():
    source = PARAMS.read_text()
    assert 'plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"' in source
    assert "nav2_mppi_controller::MPPIController" not in source
    assert "downsample_costmap: false" in source
    assert "angle_quantization_bins: 72" in source
    assert "analytic_expansion_max_length: 3.0" in source
    assert 'motion_model_for_search: "REEDS_SHEPP"' in source
    assert "reverse_penalty: 2.0" in source


def test_nav2_restores_chassis_frame_and_original_footprint():
    params = PARAMS.read_text()
    launch = CAR_NAV.read_text()
    assert "robot_base_frame: nav_base" not in params
    assert 'footprint: "[[0.9, -2.4], [-0.9, -2.4], [-0.9, 2.4], [0.9, 2.4]]"' in params
    assert "static_transform_publisher" not in launch
    assert "nav_base" not in launch


def test_costmaps_keep_mppi_clear_of_curbs_during_turns():
    params = yaml.safe_load(PARAMS.read_text())
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]["inflation_layer"]
    global_ = params["global_costmap"]["global_costmap"]["ros__parameters"]["inflation_layer"]

    assert local["inflation_radius"] == 2.2
    assert local["cost_scaling_factor"] == 0.8
    assert global_["inflation_radius"] == 2.2
    assert global_["cost_scaling_factor"] == 0.7


def test_local_voxel_grid_contains_simulated_lidar_origin():
    params = yaml.safe_load(PARAMS.read_text())
    voxel = params["local_costmap"]["local_costmap"]["ros__parameters"]["voxel_layer"]
    voxel_ceiling = voxel["origin_z"] + voxel["z_resolution"] * voxel["z_voxels"]

    assert voxel["z_voxels"] <= 16
    assert voxel_ceiling > 2.01


def test_car_navigation_launches_planning_only_stack():
    source = CAR_NAV.read_text()
    assert "planner_server" in source
    assert "map_server" in source
    assert "amcl" in source
    assert "controller_server" not in source
    assert "bt_navigator" not in source
    assert "velocity_smoother" not in source
    assert "goal_pose_nav2_bridge.py" not in source


def test_goal_bridge_is_not_used_by_legacy_launch():
    assert GOAL_BRIDGE.name not in CAR_NAV.read_text()


def test_simulation_launches_custom_pure_pursuit():
    source = SIM_BRINGUP.read_text()
    assert "executable='hybrid_pure_pursuit.py'" in source


def test_old_ackermann_behavior_tree_remains_available_but_unused():
    source = BT.read_text()
    assert "ComputePathToPose" in source
    assert "FollowPath" in source
    assert "SmacPlannerHybrid" in source
    assert "Spin" not in source
    assert "navigate_to_pose_ackermann.xml" not in CAR_NAV.read_text()
