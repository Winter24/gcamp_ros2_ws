# GCAMP ROS 2 Autonomous Navigation Simulation

This repository contains the `gcamp_gazebo` ROS 2 package for autonomous-car
simulation in Gazebo Classic. The main workflow starts a Prius in the bundled
small-city world, provides map localization and Smac Hybrid-A* planning through
Nav2, and executes planned paths with a custom hybrid pure-pursuit controller.

## Features

- Gazebo Classic small-city simulation with a Prius, camera, and 3D LiDAR
- Nav2 map server, AMCL localization, and Smac Hybrid-A* global planning
- Forward/reverse-aware pure-pursuit and PID path execution
- Headless simulation by default, with an optional Gazebo GUI
- Bundled maps, worlds, vehicle models, RViz configurations, and behavior trees
- Unit tests for planning geometry, controller behavior, goal handling, LiDAR
  processing, TensorRT routing, and dataset playback

## Supported environment

The documented setup targets:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11, installed through the ROS Gazebo packages
- Python 3.10

The package may work on other ROS 2 distributions, but the dependency names and
commands below are specific to Humble.

## Prerequisites

Install ROS 2 Humble Desktop by following the official ROS 2 installation
instructions. Then install the build tools and runtime packages used by the
main simulation:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-pip \
  python3-pytest \
  python3-rosdep \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-tf2-geometry-msgs \
  ros-humble-xacro
```

Initialize `rosdep` once per machine if it has not been initialized already:

```bash
sudo rosdep init
rosdep update
```

If `rosdep init` reports that its sources file already exists, continue with
`rosdep update`.

## Workspace setup

Clone the package into the `src` directory of a ROS 2 workspace:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Winter24/gcamp_ros2_ws.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

The Prius models and simulation worlds required by the main workflow are
included in this repository. No separate Gazebo model download is required.

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Source both setup files in every new terminal. To do this automatically, add
them to your shell configuration:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

## Run the simulation

The main launch file starts the system in this order:

1. Gazebo server and the bundled small-city world
2. Nav2 map server, AMCL, planner server, and lifecycle manager after 5 seconds
3. The custom hybrid pure-pursuit controller after 7 seconds

Run headless mode, which is the default:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch gcamp_gazebo sim_bringup.launch.py
```

Open the Gazebo client as well when a 3D view is needed:

```bash
ros2 launch gcamp_gazebo sim_bringup.launch.py gui:=true
```

Wait until the planner server is active and the pure-pursuit node reports that
it is ready before sending a goal.

## Send a navigation goal

The custom controller listens for `geometry_msgs/msg/PoseStamped` messages on
`/goal_pose`. The following example requests a goal at `(10.0, 5.0)` in the
`map` frame with zero yaw:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 10.0, y: 5.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

Choose a goal located on free, drivable space in `maps/small_city_map.pgm`.
Each new goal cancels any in-progress planning request. The controller requests
a path from the Nav2 `compute_path_to_pose` action and publishes velocity
commands on `/cmd_vel`.

RViz is optional. To inspect the map, transforms, and sensor data with the
bundled configuration, start it in a second terminal after sourcing the
workspace:

```bash
rviz2 -d "$(ros2 pkg prefix --share gcamp_gazebo)/config/main.rviz"
```

## Health checks

Useful checks while the simulation is running:

```bash
ros2 node list
ros2 topic list
ros2 action list
ros2 topic echo --once /odom
ros2 topic hz /points_raw
ros2 action info /compute_path_to_pose
```

Expected core nodes include the Gazebo ROS nodes, `map_server`, `amcl`,
`planner_server`, `lifecycle_manager_navigation`,
`dynamic_map_to_odom_broadcaster`, and `nav2_pure_pursuit_pid`.

## Tests

Run the repository unit tests from the package directory:

```bash
cd ~/ros2_ws/src/gcamp_ros2_ws
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  PYTHONDONTWRITEBYTECODE=1 \
  python3 -m pytest -p no:cacheprovider test
```

Disabling third-party pytest plugin auto-loading keeps unrelated plugins from
the user environment from affecting this test suite.

To verify the ROS package independently:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select gcamp_gazebo
```

## Repository structure

```text
gcamp_ros2_ws/
├── behavior_trees/  # Nav2 behavior-tree definitions
├── config/          # Nav2, controller, Gazebo, joystick, and RViz settings
├── description/     # URDF/Xacro robot description
├── launch/          # Launch files and ROS 2 Python nodes
├── maps/            # Small-city occupancy map
├── models/          # Bundled Gazebo vehicle models
├── test/            # Unit and integration-oriented Python tests
└── worlds/          # Gazebo simulation worlds
```

## Additional components

The repository also contains experimental object/lane detection, TensorRT and
PyTorch model code, dataset playback, joystick, mapping, and physical-robot
launch files. These components can require hardware-specific drivers, CUDA,
TensorRT, model weights, datasets, or additional ROS packages. They are not
started by `sim_bringup.launch.py` and are intentionally outside the primary
installation path documented here.

## Troubleshooting

### Package or launch file not found

Rebuild from the workspace root and source the result in the current terminal:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select gcamp_gazebo
source install/setup.bash
ros2 pkg prefix gcamp_gazebo
```

### A ROS dependency is missing

Run dependency resolution again from the workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

### Gazebo cannot find a model or plugin

Always launch the installed package through ROS 2 instead of opening the world
file directly. `launch_sim.launch.py` adds the package's `models/` directory to
`GAZEBO_MODEL_PATH` and configures its plugin search path.

### Nav2 rejects a goal or returns no path

- Wait for all lifecycle-managed Nav2 nodes to become active.
- Confirm that `/odom`, `/tf`, and `/clock` are updating.
- Ensure the goal uses the `map` frame and lies on mapped drivable space.
- Check planner output in the launch terminal for timeout or collision details.

### Gazebo GUI fails on a remote or headless machine

Use the default headless launch without `gui:=true`. If the GUI is required,
verify that the session has a working display server and hardware-accelerated
OpenGL support.

## License

The package manifest does not currently declare a finalized license. Add an
appropriate license before redistributing this project.
