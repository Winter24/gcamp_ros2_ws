# Restore Original Smac Hybrid Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make simulation planning use the original Nav2 `SmacPlannerHybrid` with `REEDS_SHEPP` while preserving the current controller improvements.

**Architecture:** Keep the custom planner package on disk as an inactive fallback. Change only the active planner-server plugin list and the planner ID requested by `hybrid_pure_pursuit.py`; do not restore reverse-controller behavior or remove unrelated changes.

**Tech Stack:** ROS 2 Humble, Nav2, Python, YAML, pytest, colcon.

## Global Constraints

- Active planner ID is exactly `SmacPlannerHybrid`.
- Active Smac motion model remains exactly `REEDS_SHEPP`.
- `TurnLimitedSmacPlanner` is not loaded by `planner_server`.
- Preserve goal cancellation, stale callback rejection, goal-frame normalization, and corner-speed limiting.
- Do not delete the custom planner package.
- Do not commit or push.

---

### Task 1: Switch active planning back to original Smac Hybrid

**Files:**
- Modify: `config/nav2_params.yaml`
- Modify: `launch/hybrid_pure_pursuit.py`
- Modify: `test/test_turn_limited_smac_integration.py`

**Interfaces:**
- Consumes: `/goal_pose` and Nav2 `/compute_path_to_pose`.
- Produces: planning requests with `planner_id="SmacPlannerHybrid"`.

- [x] **Step 1: Change the integration assertions**

Assert that `planner_plugins` excludes `TurnLimitedSmacPlanner`, that `SmacPlannerHybrid.motion_model_for_search` is `REEDS_SHEPP`, and that the controller requests `SmacPlannerHybrid` while retaining cancellation/stale-result code.

- [x] **Step 2: Run the focused test and verify it fails**

Run: `python3 -m pytest -q test/test_turn_limited_smac_integration.py`

Expected: failure because the active configuration still selects `TurnLimitedSmacPlanner`.

- [x] **Step 3: Apply the minimal rollback**

Set:

```yaml
planner_plugins: ["GridBased", "SmacPlannerHybrid"]
```

Remove the inactive `TurnLimitedSmacPlanner` parameter block from the active Nav2 YAML and change the controller request to:

```python
planner_id="SmacPlannerHybrid"
```

- [x] **Step 4: Verify focused and regression tests**

Run:

```bash
python3 -m pytest -q test/test_turn_limited_smac_integration.py test/test_reverse_aware_pure_pursuit.py test/test_nav2_long_goal_planning.py
```

Expected: all tests pass and controller safety improvements remain present.

- [x] **Step 5: Build the simulation package**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select gcamp_gazebo --symlink-install
```

Expected: `gcamp_gazebo` finishes successfully.

- [x] **Step 6: Confirm no commit or push**

Run `git log -1 --oneline` and confirm HEAD is unchanged.
