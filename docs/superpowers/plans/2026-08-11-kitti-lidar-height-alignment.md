# KITTI LiDAR Height Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Lower the Gazebo `/points_raw` LiDAR mounting height from 2.0 m to 1.73 m while leaving the existing KITTI/simulation confidence threshold at 0.2.

**Architecture:** Make one targeted xacro configuration change at the fixed joint that mounts `velodyne` to `base_link`. Validate the source, generated robot description, package build, installed artifact, and unchanged detector threshold.

**Tech Stack:** ROS 2 Humble, xacro, colcon, Gazebo Classic, Python detector configuration

## Global Constraints

- Change only the fixed `velodyne_joint` z translation in `description/vlp_16.xacro`.
- Keep the existing HDL-64E-like scan pattern, range, update rate, and noise settings.
- Keep the KITTI/simulation confidence threshold at 0.2.
- Do not change model weights, voxel geometry, postprocessing, vehicle collision geometry, or the Unitree detection path.

---

### Task 1: Align and verify the simulated LiDAR mounting height

**Files:**
- Modify: `description/vlp_16.xacro:7`
- Verify: `launch/detect.py`

**Interfaces:**
- Consumes: `velodyne_joint` fixed transform used by robot state publication and Gazebo sensor placement.
- Produces: robot description whose `velodyne_joint` origin is `xyz="0 -0.15 1.73"`.

- [ ] **Step 1: Run the source assertion and verify it fails**

Run:

```bash
rg -n 'xyz="0 -0\.15 1\.73"' description/vlp_16.xacro
```

Expected: exit status 1 and no matching line because the current z translation is `2.0`.

- [ ] **Step 2: Change only the mounting-height value**

Replace:

```xml
<origin rpy="0 0 0" xyz="0 -0.15 2.0"/>
```

with:

```xml
<origin rpy="0 0 0" xyz="0 -0.15 1.73"/>
```

- [ ] **Step 3: Verify the source assertion passes and the old value is absent**

Run:

```bash
rg -n 'xyz="0 -0\.15 1\.73"' description/vlp_16.xacro
rg -n 'xyz="0 -0\.15 2\.0"' description/vlp_16.xacro
```

Expected: the first command reports the joint origin; the second exits with status 1 and prints nothing.

- [ ] **Step 4: Process the xacro and verify the generated joint origin**

Run:

```bash
source /opt/ros/humble/setup.bash
xacro description/robot.urdf.xacro | rg -n 'name="velodyne_joint"|xyz="0 -0.15 1.73"'
```

Expected: generated URDF contains `velodyne_joint` and `xyz="0 -0.15 1.73"`.

- [ ] **Step 5: Build the package**

Run from `/home/winter24/ros2_ws`:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select gcamp_gazebo
```

Expected: package `gcamp_gazebo` finishes successfully.

- [ ] **Step 6: Verify the installed artifact and unchanged confidence threshold**

Run:

```bash
rg -n 'xyz="0 -0\.15 1\.73"' /home/winter24/ros2_ws/install/gcamp_gazebo/share/gcamp_gazebo/description/vlp_16.xacro
rg -n "thres = 0\.5 if self\.dataset_type == 'unitree' else 0\.2" launch/detect.py
```

Expected: both commands print exactly one matching configuration line.

- [ ] **Step 7: Review the scoped diff**

Run:

```bash
git diff -- description/vlp_16.xacro
```

Expected: within this task's change, only the mounting-height value changes from `2.0` to `1.73`; pre-existing user changes in the file remain intact.
