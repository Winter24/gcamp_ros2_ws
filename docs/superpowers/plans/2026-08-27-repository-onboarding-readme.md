# Repository Onboarding README Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Provide an accurate English onboarding guide for the repository, preserve all meaningful current work, exclude generated artifacts, and push the verified `nhi` branch.

**Architecture:** Treat the ROS package manifest and executable launch graph as the source of truth. Keep the README centered on one supported simulation workflow, while repository hygiene and staged-file inspection prevent local build output from entering version control.

**Tech Stack:** Ubuntu 22.04, ROS 2 Humble, Gazebo Classic, Nav2, Python 3, CMake/ament, colcon, pytest, Git

## Global Constraints

- The primary workflow is Gazebo simulation, Nav2 global planning, and the custom hybrid pure-pursuit controller.
- The documented baseline is Ubuntu 22.04 with ROS 2 Humble and Gazebo Classic.
- Optional detection, training, dataset, robot, and joystick stacks must not be presented as required dependencies.
- Do not stage `build/`, `install/`, `log/`, Python bytecode, caches, editor files, or backup files.
- Push the named `nhi` branch to `origin` without rewriting history.

---

### Task 1: Repository hygiene

**Files:**
- Create: `.gitignore`
- Delete from Git index: `launch/__pycache__/dynamic_tf_broadcaster.cpython-310.pyc`

**Interfaces:**
- Consumes: Current ROS workspace layout and generated artifacts visible in `git status`.
- Produces: A source tree where generated files no longer appear as commit candidates.

- [ ] **Step 1: Add exact ignore rules**

Create `.gitignore` with rules for `/build/`, `/install/`, `/log/`, `__pycache__/`, `*.py[cod]`, `.pytest_cache/`, `.coverage`, editor metadata, and `*.bak`.

- [ ] **Step 2: Stop tracking the committed bytecode file**

Run:

```bash
git rm --cached launch/__pycache__/dynamic_tf_broadcaster.cpython-310.pyc
```

Expected: the bytecode file is staged as deleted but remains ignored if regenerated.

- [ ] **Step 3: Verify ignore behavior**

Run:

```bash
git status --short --ignored
git check-ignore -v build install log launch/__pycache__/detect.cpython-310.pyc launch/preprocess.py.bak
```

Expected: every generated path is marked ignored and no meaningful source path is ignored.

### Task 2: English onboarding README

**Files:**
- Modify: `README.md`
- Inspect: `package.xml`
- Inspect: `CMakeLists.txt`
- Inspect: `launch/sim_bringup.launch.py`
- Inspect: `launch/launch_sim.launch.py`
- Inspect: `launch/car_navigation.launch.py`
- Inspect: `launch/hybrid_pure_pursuit.py`

**Interfaces:**
- Consumes: Package name `gcamp_gazebo`, launch arguments, ROS topics/actions, manifest dependencies, and bundled simulation assets.
- Produces: A copy-pasteable onboarding path for a fresh ROS 2 Humble workspace.

- [ ] **Step 1: Replace the existing README**

Write these sections in English: overview, features, supported environment, prerequisites, dependency installation, workspace setup, build, simulation launch, navigation usage, verification, repository structure, additional components, troubleshooting, and license status.

- [ ] **Step 2: Document dependency installation**

Use `rosdep install --from-paths src --ignore-src -r -y` as the primary resolver. Explicitly identify ROS 2 Humble desktop, Gazebo ROS packages, Nav2 bringup, `xacro`, `tf2_geometry_msgs`, and standard Python tooling where the manifest cannot bootstrap a new machine by itself.

- [ ] **Step 3: Document the main execution path**

Include these exact commands:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch gcamp_gazebo sim_bringup.launch.py
ros2 launch gcamp_gazebo sim_bringup.launch.py gui:=true
```

Explain that headless mode is the default and that the launch sequence starts Gazebo, then Nav2 map/localization/planning, then the custom controller.

- [ ] **Step 4: Document navigation and health checks**

Provide a concrete `ComputePathToPose`/goal workflow supported by the current nodes, plus commands such as `ros2 node list`, `ros2 topic list`, and `ros2 topic echo --once /odom`. Do not document RViz as mandatory unless a checked launch command actually starts or configures it.

- [ ] **Step 5: Check documentation consistency**

Run:

```bash
rg -n "gcamp_gazebo|sim_bringup.launch.py|gui:=true|colcon build|rosdep install" README.md
git diff --check -- README.md .gitignore
```

Expected: all required setup and run concepts are present and no whitespace errors are reported.

### Task 3: Verify all meaningful current source changes

**Files:**
- Verify: `launch/*.py`
- Verify: `test/test_*.py`
- Verify: `package.xml`
- Verify: `CMakeLists.txt`
- Verify: all meaningful modified and untracked source/configuration/assets shown by Git

**Interfaces:**
- Consumes: The complete current working tree, including changes that predate the README task.
- Produces: Fresh evidence for syntax, tests, build status, and commit scope.

- [ ] **Step 1: Compile Python sources without creating repository bytecode**

Run Python compilation with `PYTHONDONTWRITEBYTECODE=1` over maintained Python files, or use an equivalent AST-based syntax check that leaves no generated output.

Expected: exit code 0 and no syntax errors.

- [ ] **Step 2: Run unit tests**

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider test
```

Expected: all collected repository tests pass. If hardware, TensorRT, CUDA, or ROS availability causes skips/failures, record exact results and resolve source regressions before committing.

- [ ] **Step 3: Build the ROS package**

From the workspace root, source ROS Humble and run:

```bash
colcon build --symlink-install --packages-select gcamp_gazebo
```

Expected: package `gcamp_gazebo` builds successfully. Build output remains outside the staged commit scope.

- [ ] **Step 4: Inspect source diffs and status**

Run:

```bash
git diff --check
git status --short
git diff --stat
```

Expected: no whitespace errors; every meaningful source/config/test/asset is understood; generated artifacts are ignored.

### Task 4: Commit and push

**Files:**
- Stage: `.gitignore`, `README.md`, and all reviewed meaningful current source/configuration/map/model/test/behavior-tree/documentation files
- Exclude: all generated artifacts named in Global Constraints

**Interfaces:**
- Consumes: Verified working tree from Tasks 1–3.
- Produces: A normal commit on `nhi` and an updated `origin/nhi`.

- [ ] **Step 1: Stage reviewed paths and audit the index**

Stage meaningful files explicitly, then run:

```bash
git diff --cached --name-status
git diff --cached --check
```

Expected: the index contains no build output, cache, bytecode, backup, credential, or unrelated temporary file.

- [ ] **Step 2: Commit**

Run:

```bash
git commit -m "feat: update autonomous navigation simulation"
```

Expected: Git creates a commit containing the reviewed existing code changes, tests, repository hygiene, and English onboarding guide.

- [ ] **Step 3: Re-run final verification**

Repeat the syntax check, unit tests, and ROS package build against committed HEAD. Confirm `git status --short --branch` contains no meaningful unstaged changes.

- [ ] **Step 4: Push without force**

Run:

```bash
git push origin nhi
```

Expected: `origin/nhi` advances to the local `nhi` HEAD and no history is rewritten.
