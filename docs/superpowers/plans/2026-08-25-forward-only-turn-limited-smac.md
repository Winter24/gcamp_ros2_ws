# Forward-Only Turn-Limited Smac Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bảo đảm `TurnLimitedSmacPlanner` chỉ trả path đi tiến, khởi hành theo hướng đầu xe và từ chối goal không thể đạt đúng yaw bằng DUBIN.

**Architecture:** Khóa custom planner vào motion model DUBIN ở lifecycle config và dynamic parameter handling, sau đó thêm forward audit độc lập lên raw path và smoothed path. DUBIN là cơ chế tìm đường chỉ-tiến; audit là lớp bảo vệ cuối cùng chống reverse do regression hoặc smoothing.

**Tech Stack:** ROS 2 Humble, Nav2 Hybrid A*, C++14, OMPL Dubins, GoogleTest, pytest, YAML.

## Global Constraints

- Chỉ `TurnLimitedSmacPlanner` dùng `DUBIN`; planner Smac gốc vẫn giữ để rollback thủ công.
- Không reverse primitive, reverse analytic segment, reverse cusp hoặc fallback sang `REEDS_SHEPP`.
- Goal phải đạt cả vị trí và yaw theo tolerance hiện tại; không chuyển thành position-only goal.
- Giữ giới hạn chuỗi rẽ liên tục `150.0` độ và reset sau `3.0` m gần thẳng.
- Raw path và smoothed path đều phải qua forward audit.
- Không commit hoặc push.

---

### Task 1: Khóa custom planner vào DUBIN

**Files:**
- Modify: `src/gcamp_turn_limited_smac/src/smac_planner_hybrid.cpp`
- Modify: `src/gcamp_turn_limited_smac/test/test_plugin_config.cpp`
- Modify: `src/gcamp_ros2_ws/config/nav2_params.yaml`
- Modify: `src/gcamp_ros2_ws/test/test_turn_limited_smac_integration.py`

**Interfaces:**
- Produces: lifecycle configure chỉ chấp nhận `_motion_model == MotionModel::DUBIN`; dynamic changes sang model khác bị từ chối.

- [ ] **Step 1: Write failing configuration tests**

Add assertions that custom YAML uses `DUBIN`, plugin configure accepts DUBIN and rejects `REEDS_SHEPP`, and runtime change away from DUBIN returns unsuccessful.

```cpp
EXPECT_NO_THROW(configureWithMotionModel("DUBIN"));
EXPECT_THROW(configureWithMotionModel("REEDS_SHEPP"), nav2_core::PlannerException);
EXPECT_FALSE(setRuntimeMotionModel("REEDS_SHEPP").successful);
```

- [ ] **Step 2: Run tests and verify RED**

```bash
colcon test --packages-select gcamp_turn_limited_smac --ctest-args -R test_plugin_config --output-on-failure
python3 -m pytest -q src/gcamp_ros2_ws/test/test_turn_limited_smac_integration.py
```

Expected: current `REEDS_SHEPP` YAML/config is accepted, so new assertions fail.

- [ ] **Step 3: Implement the DUBIN-only guard**

After parsing `motion_model_for_search`, throw a `nav2_core::PlannerException` unless it is `MotionModel::DUBIN`. In the dynamic callback reject any non-DUBIN value before mutating `_motion_model_for_search`, `_motion_model`, or rebuilding A*.

- [ ] **Step 4: Change only custom planner YAML to DUBIN**

```yaml
TurnLimitedSmacPlanner:
  motion_model_for_search: "DUBIN"
```

Leave the original `SmacPlannerHybrid` configuration unchanged.

- [ ] **Step 5: Run focused tests and build**

Expected: plugin configuration and YAML integration tests pass; both packages build.

---

### Task 2: Audit every returned path for forward-only motion

**Files:**
- Modify: `src/gcamp_turn_limited_smac/include/gcamp_turn_limited_smac/smac_planner_hybrid.hpp`
- Modify: `src/gcamp_turn_limited_smac/src/smac_planner_hybrid.cpp`
- Modify: `src/gcamp_turn_limited_smac/test/test_path_audit.cpp`
- Modify: `src/gcamp_turn_limited_smac/test/test_plugin_config.cpp`

**Interfaces:**
- Produces: `bool isForwardOnlyPath(const nav_msgs::msg::Path & path, double projection_tolerance) const` and combined returned-path validation.

- [ ] **Step 1: Write failing forward-audit tests**

Cover straight/left/right forward paths, one reverse segment, zero-length duplicate poses, reverse first segment, and smoothed-path fallback.

```cpp
EXPECT_TRUE(isForwardOnlyPath(forward_straight));
EXPECT_TRUE(isForwardOnlyPath(forward_left));
EXPECT_TRUE(isForwardOnlyPath(forward_right));
EXPECT_FALSE(isForwardOnlyPath(one_reverse_segment));
EXPECT_FALSE(isForwardOnlyPath(reverse_first_segment));
```

- [ ] **Step 2: Verify RED**

Expected: helper does not exist and current path audit checks turn history only.

- [ ] **Step 3: Implement geometric forward audit**

For each nonzero segment compute:

```cpp
const double projection =
  dx * std::cos(current_yaw) + dy * std::sin(current_yaw);
if (projection < -projection_tolerance) {
  return false;
}
```

Use a fixed numeric tolerance documented in the header. Do not infer direction from yaw delta and do not modify goal yaw.

- [ ] **Step 4: Apply audit to raw and smoothed paths**

The raw path must pass turn-limit audit and forward audit before smoothing. The smoothed path must pass both audits; otherwise restore the already-audited raw path. If raw audit fails, return an empty path and log `NO_FORWARD_ONLY_PATH`.

- [ ] **Step 5: Add production createPlan regression**

Run an actual planner fixture with smoothing enabled and assert every returned segment has nonnegative heading projection. Add a case whose only feasible connection is reverse and assert the returned path is empty.

- [ ] **Step 6: Run package build, tests and sanitizer target**

Expected: all planner targets pass; focused ASan/UBSan run reports no error.

---

### Task 3: Integration verification and documentation

**Files:**
- Modify: `src/gcamp_turn_limited_smac/README.md`
- Modify: `src/gcamp_turn_limited_smac/UPSTREAM.md`
- Modify: `src/gcamp_turn_limited_smac/test/results/small_city-turn-limit-report.md`
- Create: `src/gcamp_ros2_ws/.superpowers/sdd/forward-only-verification-report.md`

**Interfaces:**
- Produces: verified build/plugin/config evidence and a manual simulation checklist.

- [ ] **Step 1: Update behavior documentation**

Document DUBIN-only operation, exact goal-yaw enforcement, no automatic fallback, `NO_FORWARD_ONLY_PATH`, and manual rollback by changing planner ID.

- [ ] **Step 2: Fresh joint build**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select gcamp_turn_limited_smac gcamp_gazebo --symlink-install
```

Expected: two packages finish successfully.

- [ ] **Step 3: Run full automated tests**

```bash
colcon test --packages-select gcamp_turn_limited_smac --event-handlers console_direct+
colcon test-result --test-result-base build/gcamp_turn_limited_smac --verbose
python3 -m pytest -q \
  src/gcamp_ros2_ws/test/test_turn_limited_smac_integration.py \
  src/gcamp_ros2_ws/test/test_forward_anchor_planning.py \
  src/gcamp_ros2_ws/test/test_nav2_long_goal_planning.py
```

Expected: zero failures.

- [ ] **Step 4: Verify plugin wiring**

Confirm custom planner loads as `gcamp_turn_limited_smac/TurnLimitedSmacPlannerHybrid`, config is DUBIN, original Smac remains registered, and the controller requests only `TurnLimitedSmacPlanner`.

- [ ] **Step 5: Record manual small_city checks without fabricating results**

Record status for:

```text
forward straight | forward left | forward right | goal behind with loop |
goal behind without loop | exact yaw unreachable | smoothing fallback
```

If Gazebo cannot run in the verification environment, mark rows `NOT_EXECUTED` and provide exact user commands.

- [ ] **Step 6: Final diff and no-commit check**

Run whitespace checks, confirm no `/opt/ros/humble` edits, and confirm no commit/push was created.

---

## Completion Criteria

- Custom planner configuration cannot enable reverse-capable motion models.
- Every successfully returned raw or smoothed path is forward-only.
- Goal yaw remains mandatory.
- Existing 150-degree turn limit remains enforced.
- Automated tests and two-package build pass.
- Manual simulation results are reported honestly.
