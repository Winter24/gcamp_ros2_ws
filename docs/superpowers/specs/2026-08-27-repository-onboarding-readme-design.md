# Repository Onboarding README Design

## Goal

Make the repository reproducible for a new developer using Ubuntu 22.04 and
ROS 2 Humble, with the primary workflow limited to Gazebo simulation, Nav2
global planning, and the custom hybrid pure-pursuit controller.

## Scope

The main README will document:

- supported operating system and ROS distribution;
- required ROS, Gazebo, Python, and build dependencies;
- workspace cloning, dependency resolution, and `colcon` build steps;
- the one-command headless simulation launch and the optional Gazebo GUI;
- how to provide a navigation goal and observe the main topics;
- test and basic troubleshooting commands; and
- a short repository layout overview.

Detection, model training, dataset playback, physical-robot launch files, and
joystick control are outside the primary setup flow. They may be identified as
additional project components without claiming that their optional dependency
stacks are installed by the main setup instructions.

## Dependency Strategy

The README will prefer `rosdep` for dependencies declared in `package.xml` and
will list explicit apt packages only where the launch files or Gazebo plugins
require them. Instructions must be derived from the current manifests, Python
imports, launch descriptions, and model plugin declarations rather than copied
from the existing README without verification.

The documented baseline is Ubuntu 22.04 with ROS 2 Humble and Gazebo Classic.
External model downloads will not be presented as mandatory when the required
Prius models are already bundled under `models/`.

## Repository Hygiene and Commit Scope

A root `.gitignore` will exclude generated ROS workspace directories, Python
bytecode and caches, pytest caches, editor files, and backup files. The final
source commit will include meaningful current source, configuration, maps,
models, tests, behavior trees, and documentation. Generated `build/`,
`install/`, `log/`, `__pycache__/`, `.pyc`, pytest cache, and backup artifacts
will not be staged.

One already tracked Python bytecode file will be removed from version control
while remaining ignored locally.

## Verification

Before the source commit and push, verification will include:

1. Python syntax compilation for maintained Python source files.
2. The repository's unit test suite.
3. A ROS package build with `colcon` when the local ROS environment supports it.
4. Inspection of the staged file list to confirm that generated artifacts are
   excluded.
5. Inspection of README commands against the actual package and launch names.

If an environment-dependent check cannot run, the final report will state the
exact command and failure instead of claiming that it passed.

## Delivery

The work will be committed on the existing `nhi` branch. After successful
verification, the branch will be pushed to its configured `origin` remote. No
force push, branch rewrite, or deletion is part of this work.
