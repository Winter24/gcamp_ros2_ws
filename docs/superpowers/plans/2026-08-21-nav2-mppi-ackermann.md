# Nav2 MPPI Ackermann Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run simulation navigation through Smac Hybrid and MPPI Ackermann using Nav2 `NavigateToPose`, while retaining the HMI `/goal_pose` input.

**Architecture:** A ROS 2 Python relay converts `/goal_pose` messages into replaceable `NavigateToPose` action goals. The simulation launch starts the complete Nav2 execution stack, a corrected +X-forward `nav_base` frame, MPPI, and a no-Spin behavior tree.

**Tech Stack:** ROS 2 Humble, Nav2 1.1.20, Python `rclpy`, launch, YAML, pytest

## Global Constraints

- Keep `hybrid_pure_pursuit.py` available but do not launch it in MPPI mode.
- Preserve `/goal_pose` as the HMI interface.
- Use `Ackermann` with `min_turning_r: 3.7`.
- Do not commit or push.

---

### Task 1: Configuration and launch contract

**Files:**
- Create: `test/test_nav2_mppi_integration.py`
- Modify: `config/nav2_params.yaml`
- Modify: `launch/car_navigation.launch.py`
- Modify: `launch/sim_bringup.launch.py`
- Create: `behavior_trees/navigate_to_pose_ackermann.xml`

**Interfaces:**
- Produces: active `navigate_to_pose`, `compute_path_to_pose`, and `follow_path` action servers; `cmd_vel_nav` input to velocity smoother; `/cmd_vel` output.

- [ ] **Step 1: Write failing static tests**

Test that YAML contains `nav2_mppi_controller::MPPIController`, `motion_model: "Ackermann"`, `min_turning_r: 3.7`, negative `vx_min`, no `PreferForwardCritic`, and `robot_base_frame: nav_base`; test that simulation launch starts a `-1.57079632679` static transform, full Nav2 execution nodes, relay, and does not launch `hybrid_pure_pursuit.py`; test that the BT contains `ComputePathToPose` and `FollowPath` but no `Spin`.

- [ ] **Step 2: Run the test and observe the expected failure**

Run: `pytest -q test/test_nav2_mppi_integration.py`

Expected: failures report the current Regulated Pure Pursuit plugin, absent relay/BT, and incomplete launch stack.

- [ ] **Step 3: Implement MPPI, frame, footprint, BT, and launch wiring**

Rotate the footprint to `[[2.4, 0.9], [2.4, -0.9], [-2.4, -0.9], [-2.4, 0.9]]`; configure MPPI for a 10 Hz controller, 40 time steps, `model_dt: 0.1`, batch size 1000, `vx_max: 1.5`, `vx_min: -0.8`, `wz_max: 1.0`, and the critic set defined in the design. Add the static transform and complete Nav2 nodes to the simulation bringup.

- [ ] **Step 4: Run the static test**

Run: `pytest -q test/test_nav2_mppi_integration.py`

Expected: all integration-contract tests pass.

### Task 2: Replaceable HMI goal relay

**Files:**
- Create: `launch/goal_pose_nav2_bridge.py`
- Create: `test/test_goal_pose_nav2_bridge.py`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `/goal_pose` (`geometry_msgs/msg/PoseStamped`).
- Produces: `nav2_msgs/action/NavigateToPose` goal requests; `stop()` publishes a zero Twist on `/cmd_vel` while replacing or rejecting a goal.

- [ ] **Step 1: Write failing relay tests**

Import the relay with ROS modules stubbed and verify monotonically increasing request IDs, cancellation of an active accepted goal, stale goal-response rejection, and preservation of the input pose including yaw.

- [ ] **Step 2: Run the relay test and observe the expected failure**

Run: `pytest -q test/test_goal_pose_nav2_bridge.py`

Expected: import fails because `goal_pose_nav2_bridge.py` does not yet exist.

- [ ] **Step 3: Implement the relay and install it**

Implement `GoalPoseNav2Bridge` with `/goal_pose` subscription, `NavigateToPose` action client, active goal cancellation, request-ID stale callback guards, and logging for accepted, rejected, canceled, succeeded, and failed goals. Add it to `install(PROGRAMS ...)`.

- [ ] **Step 4: Run relay and package tests**

Run: `pytest -q test/test_goal_pose_nav2_bridge.py test/test_nav2_mppi_integration.py test/test_nav2_long_goal_planning.py test/test_reverse_aware_pure_pursuit.py`

Expected: all selected tests pass; legacy Pure Pursuit tests remain green because rollback source is retained.

### Task 3: Build and runtime smoke test

**Files:**
- Verify only; no additional production files.

**Interfaces:**
- Consumes: built `gcamp_gazebo` package and Gazebo simulation.
- Produces: runtime evidence of active MPPI navigation.

- [ ] **Step 1: Build the package**

Run: `colcon build --packages-select gcamp_gazebo --symlink-install`

Expected: build exits zero and installs the relay and behavior tree.

- [ ] **Step 2: Run all package tests**

Run: `pytest -q src/gcamp_ros2_ws/test`

Expected: all tests pass.

- [ ] **Step 3: Launch and inspect lifecycle/action/plugin state**

Run simulation bringup, then verify `/navigate_to_pose`, `/compute_path_to_pose`, `/follow_path`, lifecycle node active states, and logs containing successful MPPI controller configuration. Publish one safe `/goal_pose` and verify `/plan` plus nonzero `/cmd_vel` without `hybrid_pure_pursuit.py` running.
