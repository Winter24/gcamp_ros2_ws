# Small City Lane-Graph Planner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a CARLA-style, forward-only lane-graph global planner for the fixed Gazebo `small_city` world and connect it to the existing Pure Pursuit controller.

**Architecture:** A pure-Python planning core loads a validated YAML road graph, snaps start/goal to directed corridors, runs A*, generates curvature-limited path geometry, scores static-obstacle clearance, and performs a final safety audit. A ROS 2 action-server wrapper exposes `ComputePathToPose` on `/compute_lane_path_to_pose`; the existing controller selects this endpoint through `global_planner_mode`, while SMAC remains a manual rollback.

**Tech Stack:** Python 3.10, ROS 2 Humble (`rclpy`, `nav2_msgs`, `nav_msgs`, TF2), PyYAML, NumPy, pytest, colcon/ament_cmake.

## Global Constraints

- Support only `small_city_sdc_prius.world` in this version.
- Drive near the whole-road center; parked cars are static obstacles.
- Never produce reverse segments or direct turns with `abs(delta_yaw) >= 150 deg`.
- Start must align within `90 deg` of the selected directed edge.
- Ignore HMI goal orientation; terminal yaw comes from the final edge tangent.
- Minimum turning radius is `3.7 m`; waypoint spacing is `0.25 m`.
- Publish valid results in frame `map` on `/plan`.
- Fail closed with an empty path; never silently fall back to SMAC.
- Keep `global_planner_mode: smac` as a manual rollback.
- Preserve controller goal cancellation and stale-result guards.
- Do not commit or push.

---

### Task 1: Lane graph schema, loader, snapping, and A*

**Files:**
- Create: `config/small_city_lane_graph.yaml`
- Create: `launch/small_city_lane_planner_core.py`
- Create: `test/test_small_city_lane_planner_core.py`

**Interfaces:**
- Consumes: YAML with `nodes`, directed `edges`, `successors`, road widths, and parked-car rectangles.
- Produces: `load_graph(path) -> LaneGraph`, `snap_start(graph, x, y, yaw) -> Snap`, `snap_goal(graph, x, y) -> Snap`, and `astar_route(graph, start_snap, goal_snap, blocked_transitions=frozenset()) -> list[str]`.

- [ ] **Step 1: Write failing schema and topology tests**

Create tests that load the real YAML and assert road centerlines at vertical
`[-45, -15, 45, 110, 120]` and horizontal `[-45, 0, 45]`, validate all
successor IDs, and assert no successor transition changes heading by `>=150°`.

- [ ] **Step 2: Run the graph tests and confirm RED**

Run:

```bash
cd /home/winter24/ros2_ws/src/gcamp_ros2_ws
python3 -m pytest -q test/test_small_city_lane_planner_core.py
```

Expected: import/file-not-found failure because the core and graph do not exist.

- [ ] **Step 3: Add the fixed graph configuration**

Define junction nodes at intersections of the documented road centerlines and
directed edges between adjacent intersections/endpoints. Include:

```yaml
map_frame: map
minimum_turning_radius: 3.7
waypoint_spacing: 0.25
max_turn_deg: 150.0
start_heading_limit_deg: 90.0
lateral_offsets: [-0.75, -0.50, -0.25, 0.0, 0.25, 0.50, 0.75]
```

Store the fourteen `prius_parked` poses from the world as oriented rectangles
with conservative half-length/half-width plus safety margin.

- [ ] **Step 4: Implement strict loading and validation**

Use dataclasses `Node`, `Edge`, `ParkedObstacle`, `LaneGraph`, and `Snap`.
Reject duplicate IDs, missing successors, non-finite coordinates, nonpositive
width, and forbidden successor heading changes.

- [ ] **Step 5: Write failing snap/A* behavior tests**

Cover start heading selection, position-only goal snapping, straight/left/right
routes, a goal behind the vehicle that must traverse a block, cancellation via
a callback, and `NO_GRAPH_ROUTE` for disconnected input.

- [ ] **Step 6: Implement snap and A***

Project points onto edge polylines. Start score is lateral distance plus wrapped
heading error and rejects error `>pi/2`. Goal ignores input yaw. A* state is a
directed edge; cost is edge length plus configurable turn penalty and heuristic
is Euclidean distance to the goal snap.

- [ ] **Step 7: Verify Task 1**

Run the focused test file and expect all tests to pass.

---

### Task 2: Path geometry, corridor clearance, and final audit

**Files:**
- Modify: `launch/small_city_lane_planner_core.py`
- Modify: `test/test_small_city_lane_planner_core.py`
- Create: `test/test_small_city_lane_geometry.py`

**Interfaces:**
- Consumes: `LaneGraph`, route edge IDs, start/goal snaps, occupancy metadata.
- Produces: `build_path_geometry(...) -> list[PathPoint]`, `choose_clear_offset(...)`, and `audit_path(...) -> AuditResult`.

- [ ] **Step 1: Write failing straight/corner geometry tests**

Assert `0.25 m` nominal spacing, tangent yaw, forward projections, left/right
corner continuity, and measured curvature no greater than `1/3.7` plus a fixed
numeric tolerance.

- [ ] **Step 2: Run geometry tests and confirm RED**

Expected: missing geometry functions.

- [ ] **Step 3: Implement route trimming and corner interpolation**

Trim the first/last edge at snap parameters. Generate straight samples and
tangent circular fillets where possible; use a cubic Bezier only when its
sampled curvature passes the same radius audit. Return `NO_VALID_CORNER` on
failure.

- [ ] **Step 4: Write failing clearance tests**

Use synthetic occupancy grids and the real parked-obstacle list. Assert center
offset wins beside cars at `±2.55 m`, blocked candidates are rejected, smooth
offset transitions remain inside road bounds, and all-invalid candidates return
`NO_CLEAR_CORRIDOR`.

- [ ] **Step 5: Implement occupancy and oriented-box clearance**

Load PGM/YAML once at startup. Sample the Prius rectangular footprint along
candidate paths, test occupied pixels and oriented parked-car rectangles, and
maximize minimum clearance with length as tie-breaker.

- [ ] **Step 6: Write and implement final-audit tests**

Reject NaN/Inf, reverse projection, local heading change `>=150°`, excessive
curvature, collision, road-bound violation, and wrong terminal tangent. Accept
representative straight, left, and right paths.

- [ ] **Step 7: Verify Task 2**

Run both core and geometry suites and expect all tests to pass.

---

### Task 3: ROS 2 ComputePathToPose action server

**Files:**
- Create: `launch/small_city_route_planner.py`
- Create: `test/test_small_city_route_planner_node.py`
- Modify: `CMakeLists.txt`
- Modify: `package.xml`

**Interfaces:**
- Consumes: `ComputePathToPose.Goal`, TF `map -> chassis`, graph/core functions.
- Produces: action `/compute_lane_path_to_pose`, result `nav_msgs/Path`, and `/plan` publisher.

- [ ] **Step 1: Write failing source-level and callback tests**

Stub ROS modules as established in existing tests. Verify action name/type,
explicit-start handling, Prius yaw conversion, goal orientation ignored,
cancellation, empty-path abort, successful result, `/plan` publication, and
stable diagnostics.

- [ ] **Step 2: Run node tests and confirm RED**

Expected: node file missing.

- [ ] **Step 3: Implement the action wrapper**

Create an `ActionServer` using mutually exclusive request IDs/cancel checks.
Populate every pose quaternion from core tangent yaw and set planning duration
on both success and abort. Log the diagnostic stages defined in the spec.

- [ ] **Step 4: Install runtime files and dependencies**

Install both Python scripts and the graph YAML. Declare runtime dependencies
for `rclpy`, `nav2_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, `python3-yaml`,
and `python3-numpy` without removing existing dependencies.

- [ ] **Step 5: Verify Task 3**

Run node/core/geometry tests and build `gcamp_gazebo` with symlink install.

---

### Task 4: Controller mode switch and simulation launch integration

**Files:**
- Modify: `launch/hybrid_pure_pursuit.py`
- Modify: `launch/sim_bringup.launch.py`
- Create: `test/test_lane_planner_integration.py`
- Modify: `test/test_turn_limited_smac_integration.py`

**Interfaces:**
- Consumes: parameter `global_planner_mode` with values `lane_graph|smac`.
- Produces: controller action client bound to the selected endpoint and launch of the lane planner in simulation.

- [ ] **Step 1: Write failing integration tests**

Assert default `lane_graph`, action endpoint `/compute_lane_path_to_pose`, empty
planner ID for the standalone action, lane planner launch ordering, preservation
of cancel/stale guards, and `smac` mode using `/compute_path_to_pose` plus
`SmacPlannerHybrid`.

- [ ] **Step 2: Run integration tests and confirm RED**

Expected: missing mode/endpoint/node wiring.

- [ ] **Step 3: Implement controller mode selection**

Declare and validate `global_planner_mode`. Construct exactly one action client
for the selected endpoint. In lane mode retain the returned final pose/yaw from
the path instead of the HMI quaternion; in SMAC mode preserve current behavior.
Keep active-goal cancellation and request-ID filtering unchanged.

- [ ] **Step 4: Launch lane planner before the controller**

Start the lane planner after Gazebo/TF and before Pure Pursuit. Pass graph/map
paths from the installed package and `use_sim_time: true`. Keep Nav2 map/AMCL/
planner server alive for rollback mode.

- [ ] **Step 5: Verify Task 4**

Run all planner and existing controller tests, then build `gcamp_gazebo`.

---

### Task 5: Full verification and manual small_city evidence

**Files:**
- Create: `test/results/small-city-lane-planner-report.md`
- Modify: `README.md` if present, otherwise create `docs/small-city-lane-planner.md`

**Interfaces:**
- Consumes: installed lane planner/controller and Gazebo world.
- Produces: reproducible commands, measured planning times, scenario results, and rollback instructions.

- [ ] **Step 1: Run complete automated verification**

Run:

```bash
cd /home/winter24/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gcamp_gazebo --symlink-install
python3 -m pytest -q src/gcamp_ros2_ws/test
git -C src/gcamp_ros2_ws diff --check
```

Expected: build succeeds, all tests pass, and diff check is clean.

- [ ] **Step 2: Run manual Gazebo scenarios when display/runtime access permits**

Test straight routes beside every parked-car group, all turn directions, goal
behind requiring a block loop, near-curb goal snapping, replacement goal, and a
fully blocked route. Record actual planning latency and minimum observed
clearance; mark scenarios not executed rather than inventing results.

- [ ] **Step 3: Document operation and rollback**

Document launch command, diagnostics, graph limitations, and changing
`global_planner_mode` to `smac`. Confirm HEAD remains unchanged and no commit or
push occurred.
