# Turn-Limited Smac Planner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Xây dựng Nav2 Hybrid A* plugin cho `small_city` loại tuyệt đối mọi nhánh có chuỗi rẽ liên tục từ 150 độ và reset chuỗi sau 3 m gần thẳng.

**Architecture:** Vendor đúng phần Smac Hybrid A* từ Nav2 `1.1.20` vào package C++ riêng, đổi namespace và mở rộng graph key bằng trạng thái lịch sử rẽ. Planner mới dùng nguyên collision checking, heuristic, analytic expansion và smoothing của Smac, nhưng successor vượt giới hạn quay không được đưa vào open set.

**Tech Stack:** ROS 2 Humble, Nav2 `1.1.20`, C++14, `ament_cmake`, `pluginlib`, GoogleTest, Python/pytest integration checks.

## Global Constraints

- Không sửa `/opt/ros/humble` hoặc ghi đè `libnav2_smac_planner.so`.
- Giữ planner `nav2_smac_planner/SmacPlannerHybrid` nguyên vẹn để rollback.
- Chỉ kích hoạt planner mới cho `small_city`.
- `max_continuous_turn_deg = 150.0` là giới hạn cấm, không tự nới khi timeout.
- `straight_reset_distance = 3.0` m.
- Không fallback sang planner cho phép U-turn.
- Không sửa local controller/pure pursuit ngoài việc đổi planner ID.
- Không commit hoặc push; người dùng sẽ thực hiện một lượt khi dự án hoàn tất.

---

## File Structure

Package mới đặt tại `src/gcamp_turn_limited_smac/`:

- `package.xml`, `CMakeLists.txt`, `turn_limited_smac_plugin.xml`: build và đăng ký plugin.
- `include/gcamp_turn_limited_smac/turn_state.hpp`: luật cập nhật lịch sử rẽ độc lập với ROS.
- `include/gcamp_turn_limited_smac/node_hybrid.hpp`: node/key Hybrid A* mở rộng.
- `include/gcamp_turn_limited_smac/a_star.hpp`: vòng tìm kiếm và cancellation.
- `include/gcamp_turn_limited_smac/smac_planner_hybrid.hpp`: giao diện Nav2 lifecycle.
- `src/node_hybrid.cpp`, `src/a_star.cpp`, `src/smac_planner_hybrid.cpp`: implementation được chuyển từ Nav2 1.1.20 và patch có giới hạn.
- `test/test_turn_state.cpp`: unit test ranh giới góc/reset.
- `test/test_state_index.cpp`: chứng minh pose giống nhau nhưng lịch sử khác có key khác.
- `test/test_plugin_config.cpp`: validation tham số.

Package hiện tại:

- `src/gcamp_ros2_ws/config/nav2_params.yaml`: thêm plugin và cấu hình.
- `src/gcamp_ros2_ws/launch/hybrid_pure_pursuit.py`: yêu cầu planner ID mới.
- `src/gcamp_ros2_ws/test/test_turn_limited_smac_integration.py`: kiểm tra wiring/rollback/error handling.

---

### Task 1: Vendor Smac Hybrid 1.1.20 thành package độc lập

**Files:**
- Create: `src/gcamp_turn_limited_smac/package.xml`
- Create: `src/gcamp_turn_limited_smac/CMakeLists.txt`
- Create: `src/gcamp_turn_limited_smac/turn_limited_smac_plugin.xml`
- Create: `src/gcamp_turn_limited_smac/include/gcamp_turn_limited_smac/*.hpp`
- Create: `src/gcamp_turn_limited_smac/src/*.cpp`

**Interfaces:**
- Consumes: Nav2 Smac source version `1.1.20` matching installed Debian package.
- Produces: loadable class `gcamp_turn_limited_smac::TurnLimitedSmacPlannerHybrid` implementing `nav2_core::GlobalPlanner`.

- [ ] **Step 1: Verify the installed ABI baseline**

Run:

```bash
dpkg-query -W -f='${Version}\n' ros-humble-nav2-smac-planner
```

Expected: output starts with `1.1.20-`.

- [ ] **Step 2: Obtain the matching official source and record provenance**

Use the official `ros-navigation/navigation2` tag `1.1.20`; copy only `nav2_smac_planner` source required by Hybrid planning. Add `UPSTREAM.md` containing repository URL, tag, copied paths and original license. Do not copy build artifacts.

- [ ] **Step 3: Rename the fork without changing behavior**

Rename namespace and plugin class:

```cpp
namespace gcamp_turn_limited_smac
{
class TurnLimitedSmacPlannerHybrid : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
};
}
```

- [ ] **Step 4: Register the plugin**

`turn_limited_smac_plugin.xml` must export:

```xml
<library path="gcamp_turn_limited_smac">
  <class name="gcamp_turn_limited_smac/TurnLimitedSmacPlannerHybrid"
         type="gcamp_turn_limited_smac::TurnLimitedSmacPlannerHybrid"
         base_class_type="nav2_core::GlobalPlanner"/>
</library>
```

- [ ] **Step 5: Build the behavior-preserving fork**

Run:

```bash
colcon build --packages-select gcamp_turn_limited_smac --symlink-install
```

Expected: package builds with no undefined symbols and plugin XML is installed under `share/gcamp_turn_limited_smac`.

---

### Task 2: Implement and test turn-history transitions

**Files:**
- Create: `src/gcamp_turn_limited_smac/include/gcamp_turn_limited_smac/turn_state.hpp`
- Create: `src/gcamp_turn_limited_smac/test/test_turn_state.cpp`
- Modify: `src/gcamp_turn_limited_smac/CMakeLists.txt`

**Interfaces:**
- Produces: `TurnState updateTurnState(const TurnState &, double delta_yaw_rad, double travel_m, bool reversing, const TurnLimits &)` and `bool exceedsTurnLimit(const TurnState &, const TurnLimits &)`.

- [ ] **Step 1: Write failing boundary tests**

Tests must cover exactly:

```cpp
EXPECT_FALSE(exceedsTurnLimit(accumulateSameTurn(149.0), limits));
EXPECT_TRUE(exceedsTurnLimit(accumulateSameTurn(150.0), limits));
EXPECT_NEAR(straightFor(accumulateSameTurn(120.0), 2.9).turn_deg, 120.0, 1e-6);
EXPECT_DOUBLE_EQ(straightFor(accumulateSameTurn(120.0), 3.0).turn_deg, 0.0);
EXPECT_NEAR(turnRightAfterLeft(100.0, 20.0).turn_deg, 20.0, 1e-6);
EXPECT_NEAR(reverseAfterTurn(100.0, 1.0).turn_deg, 100.0, 1e-6);
```

- [ ] **Step 2: Run the tests and verify RED**

Run:

```bash
colcon test --packages-select gcamp_turn_limited_smac --ctest-args -R test_turn_state --output-on-failure
```

Expected: compile failure because `TurnState` is not implemented.

- [ ] **Step 3: Implement the minimal state machine**

Use explicit types:

```cpp
enum class TurnDirection : int8_t { NONE = 0, LEFT = 1, RIGHT = -1 };

struct TurnLimits {
  double max_turn_deg{150.0};
  double straight_reset_m{3.0};
  double straight_yaw_tolerance_deg{5.0};
};

struct TurnState {
  TurnDirection direction{TurnDirection::NONE};
  double turn_deg{0.0};
  double straight_m{0.0};
};
```

Normalize `delta_yaw` to `[-pi, pi]`; reversing may change travel cost but must not reset `turn_deg` or `direction`.

- [ ] **Step 4: Run test and verify GREEN**

Expected: all turn-state boundary tests pass.

---

### Task 3: Extend the Hybrid A* state key

**Files:**
- Modify: `src/gcamp_turn_limited_smac/include/gcamp_turn_limited_smac/node_hybrid.hpp`
- Modify: `src/gcamp_turn_limited_smac/src/node_hybrid.cpp`
- Create: `src/gcamp_turn_limited_smac/test/test_state_index.cpp`

**Interfaces:**
- Consumes: `TurnState` and planner quantization settings.
- Produces: `StateKey getIndex(x, y, angle_bin, turn_direction, turn_bucket, straight_bucket)` with collision-free equality/hash semantics.

- [ ] **Step 1: Write failing key-identity tests**

```cpp
EXPECT_NE(keyFor(pose, left_30), keyFor(pose, left_120));
EXPECT_NE(keyFor(pose, left_30), keyFor(pose, right_30));
EXPECT_EQ(keyFor(pose, reset_a), keyFor(pose, reset_b));
```

- [ ] **Step 2: Verify the tests fail against the pose-only key**

Expected: first two comparisons fail because upstream Smac indexes only `(x,y,yaw)`.

- [ ] **Step 3: Implement bounded buckets**

Use 5-degree turn buckets from `0` through `145`, two directions plus `NONE`, and straight-distance buckets capped at 3 m. Store the graph in a hash map keyed by the expanded struct rather than allocating the full Cartesian product.

```cpp
struct StateKey {
  uint32_t pose_index;
  uint8_t turn_bucket;
  uint8_t straight_bucket;
  int8_t turn_direction;
  bool operator==(const StateKey & other) const;
};
```

- [ ] **Step 4: Run state and key tests**

Expected: both test binaries pass; a reset state produces the canonical `NONE/0/0` key.

---

### Task 4: Apply nonlinear penalties and prune successors at 150 degrees

**Files:**
- Modify: `src/gcamp_turn_limited_smac/include/gcamp_turn_limited_smac/a_star.hpp`
- Modify: `src/gcamp_turn_limited_smac/src/a_star.cpp`
- Create: `src/gcamp_turn_limited_smac/test/test_turn_limited_expansion.cpp`

**Interfaces:**
- Produces: `double turnPenalty(double turn_deg, const TurnPenaltyParams &)` and successor expansion that never queues `turn_deg >= 150.0`.

- [ ] **Step 1: Write failing penalty/pruning tests**

```cpp
EXPECT_DOUBLE_EQ(turnPenalty(89.999, params), 0.0);
EXPECT_GT(turnPenalty(100.0, params), 0.0);
EXPECT_GT(turnPenalty(140.0, params), turnPenalty(110.0, params));
EXPECT_FALSE(expander.accepts(successorAt(150.0)));
EXPECT_TRUE(expander.accepts(successorAt(149.0)));
```

- [ ] **Step 2: Verify RED**

Expected: no turn-aware penalty/pruning API exists.

- [ ] **Step 3: Implement expansion logic**

For an accepted successor:

```cpp
const auto next_turn = updateTurnState(
  current.turnState(), delta_yaw, primitive_distance, reversing, limits_);
if (exceedsTurnLimit(next_turn, limits_)) {
  ++turn_limit_pruned_count_;
  continue;
}
tentative_g += turnPenalty(next_turn.turn_deg, penalty_params_);
```

Use zero penalty below 90 degrees, a linear light slope from 90–120 degrees, and a steeper linear slope from 120–150 degrees. Preserve upstream collision, reverse, direction-change and costmap penalties.

- [ ] **Step 4: Ensure analytic expansion cannot bypass the limit**

Sample the analytic Reeds-Shepp connection primitive-by-primitive through the same `updateTurnState`; reject analytic connections reaching 150 degrees.

- [ ] **Step 5: Run all package tests**

Expected: boundary, key, expansion and analytic-expansion tests pass.

---

### Task 5: Add ROS parameters, validation, cancellation and diagnostics

**Files:**
- Modify: `src/gcamp_turn_limited_smac/include/gcamp_turn_limited_smac/smac_planner_hybrid.hpp`
- Modify: `src/gcamp_turn_limited_smac/src/smac_planner_hybrid.cpp`
- Create: `src/gcamp_turn_limited_smac/test/test_plugin_config.cpp`

**Interfaces:**
- Produces ROS parameters named under the plugin namespace and diagnostic log `NO_PATH_WITHIN_TURN_LIMIT`.

- [ ] **Step 1: Write failing configuration tests**

Validate:

```text
0 <= light_penalty_start_deg < strong_penalty_start_deg < max_continuous_turn_deg < 180
straight_reset_distance > 0
straight_yaw_tolerance_deg >= 0
light_turn_penalty >= 0
strong_turn_penalty >= light_turn_penalty
```

- [ ] **Step 2: Implement parameter declaration and validation**

Defaults are exactly `150.0`, `3.0`, `5.0`, `90.0`, `120.0`, `1.0`, `5.0`. Throw `nav2_core::PlannerException` during configure for invalid relationships.

- [ ] **Step 3: Add cancellation checks**

Check the Nav2 cancellation callback inside the A* loop at least once per expansion batch and before analytic expansion. A canceled request returns promptly without publishing a stale path.

- [ ] **Step 4: Add deterministic failure diagnostics**

When open set is exhausted and `turn_limit_pruned_count_ > 0`, log:

```text
NO_PATH_WITHIN_TURN_LIMIT max_turn_deg=150.0 pruned=<count> expanded=<count>
```

Do not claim this reason if no branch was pruned by the turn limit.

- [ ] **Step 5: Run plugin tests and build**

Expected: valid defaults configure; each invalid boundary fails with its parameter name; cancellation test completes within its timeout.

---

### Task 6: Wire the planner into small_city without removing Smac

**Files:**
- Modify: `src/gcamp_ros2_ws/config/nav2_params.yaml`
- Modify: `src/gcamp_ros2_ws/launch/hybrid_pure_pursuit.py`
- Create: `src/gcamp_ros2_ws/test/test_turn_limited_smac_integration.py`

**Interfaces:**
- Consumes plugin ID `gcamp_turn_limited_smac/TurnLimitedSmacPlannerHybrid`.
- Produces planner request ID `TurnLimitedSmacPlanner`.

- [ ] **Step 1: Write failing wiring tests**

Assert the YAML keeps both planners and the controller requests only the new ID:

```python
assert "SmacPlannerHybrid" in planner_plugins
assert "TurnLimitedSmacPlanner" in planner_plugins
assert cfg["TurnLimitedSmacPlanner"]["max_continuous_turn_deg"] == 150.0
assert 'planner_id="TurnLimitedSmacPlanner"' in controller_source
```

- [ ] **Step 2: Run pytest and verify RED**

Run:

```bash
pytest -q src/gcamp_ros2_ws/test/test_turn_limited_smac_integration.py
```

Expected: new planner ID/config missing.

- [ ] **Step 3: Add planner configuration**

Add `TurnLimitedSmacPlanner` beside, not instead of, `SmacPlannerHybrid`. Copy the current Smac parameters and append the seven turn-limit parameters from the spec.

- [ ] **Step 4: Change direct requests and error text**

In `goal_callback`, request `planner_id="TurnLimitedSmacPlanner"`. In the empty-path branch, report the actual `planner_id` instead of hard-coded `SmacPlannerHybrid`.

- [ ] **Step 5: Run integration and existing controller tests**

Run:

```bash
pytest -q src/gcamp_ros2_ws/test/test_turn_limited_smac_integration.py src/gcamp_ros2_ws/test/test_forward_anchor_planning.py src/gcamp_ros2_ws/test/test_nav2_long_goal_planning.py
```

Expected: all pass; update legacy assertions only where they intentionally refer to the active planner ID, never delete rollback checks.

---

### Task 7: End-to-end build and small_city validation

**Files:**
- Modify only if a test exposes a defect in files from Tasks 1–6.
- Record: `src/gcamp_turn_limited_smac/test/results/small_city-turn-limit-report.md`

**Interfaces:**
- Produces evidence for ordinary turns, route-around behavior, hard rejection and runtime cost.

- [ ] **Step 1: Build the affected packages**

```bash
colcon build --packages-select gcamp_turn_limited_smac gcamp_gazebo --symlink-install
```

Expected: both packages build successfully.

- [ ] **Step 2: Run all affected tests**

```bash
colcon test --packages-select gcamp_turn_limited_smac gcamp_gazebo --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero failed tests.

- [ ] **Step 3: Validate ordinary paths in small_city**

Launch the existing simulation and submit one straight, one left-turn and one right-turn goal. Record planning time, expanded nodes, path length and maximum continuous turn. Each must succeed and remain below 150 degrees.

- [ ] **Step 4: Validate the U-turn alternative**

Submit the known goal where Smac previously chose a near-180-degree reversal. Confirm the returned path goes around the street network and its measured maximum continuous turn is below 150 degrees.

- [ ] **Step 5: Validate hard failure**

Use a constrained test map with no legal alternative. Confirm path is empty, vehicle command is zero, and log includes `NO_PATH_WITHIN_TURN_LIMIT`; confirm there is no request to `SmacPlannerHybrid` afterward.

- [ ] **Step 6: Validate goal cancellation**

While a long request is planning, send a second goal. Confirm the first goal is canceled, no stale path is installed, and the second goal is processed.

- [ ] **Step 7: Write the result report**

For each case record:

```text
case | success | planning_ms | expanded_nodes | path_m | max_continuous_turn_deg | failure_reason
```

Include a baseline row from unmodified `SmacPlannerHybrid`. Do not change the 150-degree limit to improve runtime.

---

## Final Verification

- [ ] Run `git diff --check` inside `src/gcamp_ros2_ws` and the new package repository scope.
- [ ] Confirm `/opt/ros/humble` has not changed.
- [ ] Confirm both planner plugins are discoverable with `pluginlib`.
- [ ] Confirm all successful paths have measured maximum continuous turn below 150 degrees.
- [ ] Confirm no commit or push was created.
