# Smac Forward-Only Safety Fallback Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Reject curb-unsafe Reeds-Shepp paths and retry with a forward-only Dubin path before commanding the custom pure-pursuit controller.

**Architecture:** Configure two Smac Hybrid planner instances sharing the same global costmap. The controller subscribes to the published global occupancy grid, raster-checks its padded vehicle footprint along each candidate path, and accepts only safe candidates.

**Tech Stack:** ROS 2 Humble, Nav2 Smac Hybrid, `nav_msgs/OccupancyGrid`, Python, pytest.

## Global Constraints

- Try `SmacPlannerHybrid` (`REEDS_SHEPP`) first.
- Retry `SmacForwardOnly` (`DUBIN`) only when the first candidate is unavailable or unsafe.
- Never fall back to GridBased because it does not enforce Ackermann kinematics.
- Do not replace active waypoints with an unsafe path.
- Do not commit or push.

---

### Task 1: Planner and footprint validation

**Files:**
- Modify: `config/nav2_params.yaml`
- Modify: `launch/hybrid_pure_pursuit.py`
- Modify: `test/test_nav2_long_goal_planning.py`

- [ ] Add failing tests for both planner IDs and occupied/clear footprint validation.
- [ ] Configure the forward-only DUBIN Smac planner.
- [ ] Subscribe to the global occupancy grid and validate the padded footprint along returned paths.
- [ ] Route unsafe/empty Reeds-Shepp results to `SmacForwardOnly`; reject an unsafe/empty forward-only result.
- [ ] Run focused tests, full tests, YAML validation, build and launch parsing.
