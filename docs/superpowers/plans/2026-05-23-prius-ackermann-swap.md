# Prius Ackermann Car Swap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the gcamp diff-drive robot with the Prius hybrid car model (Hybrid.obj mesh) + VLP-16 on the roof, controlled via `ackermann_steering_controller`, compatible with Nav2.

**Architecture:** Create new Xacro files (`prius_core`, `prius_ros2_control`, `prius_gazebo`, `prius.urdf.xacro`) inside the existing `gcamp_gazebo` package on the `ackermann-car` branch. The `ackermann_steering_controller` from `steering_controllers_library` commands rear-wheel velocity + front steering position. The VLP-16 is inlined in `prius.urdf.xacro` at roof height (z=1.5). SDC worlds are copied in stripped of their embedded Prius spawn so our ROS2 `spawn_entity.py` handles spawning.

**Tech Stack:** ROS2 Humble, Gazebo Classic 11, ros2_control, ackermann_steering_controller, gazebo_ros2_control, xacro

---

## Prerequisite: Install ackermann controller

```bash
sudo apt install ros-humble-ackermann-steering-controller
```

---

## File Map

**Create:**
- `meshes/Hybrid.obj` + `meshes/Hybrid.mtl` — Prius body mesh (copied from SDC repo)
- `description/prius_core.xacro` — links, joints, chassis + wheel geometry
- `description/prius_ros2_control.xacro` — ros2_control hardware interfaces
- `description/prius_gazebo.xacro` — gazebo_ros2_control plugin + wheel friction
- `description/prius.urdf.xacro` — top-level entry point (includes all above + VLP-16)
- `config/prius_controllers.yaml` — ackermann_cont + joint_broad controller config
- `worlds/sdc.world` — SDC world stripped of embedded prius_hybrid spawn
- `worlds/maze_solving.world` — SDC maze world stripped of embedded prius_hybrid spawn

**Modify:**
- `CMakeLists.txt` — add `meshes` to install targets
- `launch/rsp.launch.py` — swap `robot.urdf.xacro` → `prius.urdf.xacro`
- `launch/launch_sim.launch.py` — swap `diff_cont` → `ackermann_cont`
- `config/nav2_params.yaml` — replace `robot_radius: 0.22` with Prius footprint polygon

---

## Task 1: Copy mesh files + update CMakeLists

**Files:**
- Create: `meshes/Hybrid.obj`, `meshes/Hybrid.mtl`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Copy mesh files**

```bash
mkdir -p /home/ngin/fpt-capstone/simulation/gcamp_ros2_ws/meshes
cp /home/ngin/fpt-capstone/simulation/ROS2-Self-Driving-Car-AI-using-OpenCV/self_driving_car_pkg_models/models/prius_hybrid/meshes/Hybrid.obj \
   /home/ngin/fpt-capstone/simulation/gcamp_ros2_ws/meshes/
cp /home/ngin/fpt-capstone/simulation/ROS2-Self-Driving-Car-AI-using-OpenCV/self_driving_car_pkg_models/models/prius_hybrid/meshes/Hybrid.mtl \
   /home/ngin/fpt-capstone/simulation/gcamp_ros2_ws/meshes/
```

- [ ] **Step 2: Add meshes to CMakeLists.txt install**

In `CMakeLists.txt`, change:
```cmake
install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)
```
to:
```cmake
install(
  DIRECTORY config description launch meshes worlds
  DESTINATION share/${PROJECT_NAME}
)
```

- [ ] **Step 3: Commit**

```bash
cd /home/ngin/fpt-capstone/simulation/gcamp_ros2_ws
git add meshes/ CMakeLists.txt
git commit -m "feat: add Prius Hybrid.obj mesh and update install targets"
```

---

## Task 2: Copy and strip SDC world files

**Files:**
- Create: `worlds/sdc.world`, `worlds/maze_solving.world`

The SDC worlds embed a `<include><uri>model://prius_hybrid</uri>` block that spawns the car from the Gazebo model database. We strip this because our launch file uses `spawn_entity.py` instead.

- [ ] **Step 1: Copy worlds**

```bash
cp /home/ngin/fpt-capstone/simulation/ROS2-Self-Driving-Car-AI-using-OpenCV/self_driving_car_pkg/worlds/sdc.world \
   /home/ngin/fpt-capstone/simulation/gcamp_ros2_ws/worlds/
cp /home/ngin/fpt-capstone/simulation/ROS2-Self-Driving-Car-AI-using-OpenCV/self_driving_car_pkg/worlds/maze_solving.world \
   /home/ngin/fpt-capstone/simulation/gcamp_ros2_ws/worlds/
```

- [ ] **Step 2: Strip prius_hybrid include from sdc.world**

Open `worlds/sdc.world` and delete the entire `<include>` block that contains `model://prius_hybrid`. It looks like:
```xml
<include>
  <uri>model://prius_hybrid</uri>
  ...
</include>
```
Remove the complete block (opening `<include>` through closing `</include>`).

- [ ] **Step 3: Strip prius_hybrid include from maze_solving.world**

Same edit — remove the `<include>…model://prius_hybrid…</include>` block.

- [ ] **Step 4: Commit**

```bash
git add worlds/sdc.world worlds/maze_solving.world
git commit -m "feat: add SDC worlds (sdc, maze_solving) stripped of embedded prius spawn"
```

---

## Task 3: Create `description/prius_core.xacro`

**Files:**
- Create: `description/prius_core.xacro`

This file defines all links and joints for the Prius. Coordinate system note: the SDF model faces -Y; to convert to URDF (X=forward), apply a +90° Z rotation to the mesh visual and transform positions as: `X_urdf = -Y_sdf`, `Y_urdf = X_sdf`.

- [ ] **Step 1: Write the file**

Create `description/prius_core.xacro` with the following content:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:property name="wheel_radius" value="0.31265"/>
  <xacro:property name="wheel_width"  value="0.20"/>
  <xacro:property name="front_axle_x" value="1.41"/>
  <xacro:property name="rear_axle_x"  value="-1.45"/>
  <xacro:property name="front_track_y" value="0.76"/>
  <xacro:property name="rear_track_y"  value="0.786"/>
  <xacro:property name="axle_z"       value="${wheel_radius}"/>
  <xacro:property name="max_steer"    value="0.5"/>

  <link name="base_footprint"/>

  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child  link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <link name="base_link"/>

  <joint name="chassis_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="chassis"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <link name="chassis">
    <!-- SDF chassis pose (0,-0.266,0.48) → URDF (0.266,0,0.48); mesh rotated +90° Z -->
    <visual>
      <origin xyz="0.266 0 0.48" rpy="0 0 1.5708"/>
      <geometry>
        <mesh filename="package://gcamp_gazebo/meshes/Hybrid.obj" scale="0.01 0.01 0.01"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.65" rpy="0 0 0"/>
      <geometry><box size="4.5 1.8 1.3"/></geometry>
    </collision>
    <inertial>
      <mass value="1326.0"/>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <inertia ixx="2581.13" ixy="0" ixz="0" iyy="591.31" iyz="0" izz="2681.95"/>
    </inertial>
  </link>

  <!-- ── Front left: steering parent + spinning wheel ── -->
  <joint name="front_left_steering_joint" type="revolute">
    <parent link="base_link"/>
    <child  link="front_left_steer_link"/>
    <origin xyz="${front_axle_x} ${front_track_y} ${axle_z}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-${max_steer}" upper="${max_steer}" effort="100" velocity="1.0"/>
  </joint>

  <link name="front_left_steer_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/>
    </inertial>
  </link>

  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="front_left_steer_link"/>
    <child  link="front_left_wheel"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="front_left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
      <material name="dark_grey"><color rgba="0.2 0.2 0.2 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
    </collision>
    <inertial>
      <mass value="11.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- ── Front right: steering parent + spinning wheel ── -->
  <joint name="front_right_steering_joint" type="revolute">
    <parent link="base_link"/>
    <child  link="front_right_steer_link"/>
    <origin xyz="${front_axle_x} -${front_track_y} ${axle_z}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-${max_steer}" upper="${max_steer}" effort="100" velocity="1.0"/>
  </joint>

  <link name="front_right_steer_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/>
    </inertial>
  </link>

  <joint name="front_right_wheel_joint" type="continuous">
    <parent link="front_right_steer_link"/>
    <child  link="front_right_wheel"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="front_right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
      <material name="dark_grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
    </collision>
    <inertial>
      <mass value="11.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- ── Rear left wheel (driven) ── -->
  <joint name="rear_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child  link="rear_left_wheel"/>
    <origin xyz="${rear_axle_x} ${rear_track_y} ${axle_z}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="rear_left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
      <material name="dark_grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
    </collision>
    <inertial>
      <mass value="11.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- ── Rear right wheel (driven) ── -->
  <joint name="rear_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child  link="rear_right_wheel"/>
    <origin xyz="${rear_axle_x} -${rear_track_y} ${axle_z}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="rear_right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
      <material name="dark_grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
    </collision>
    <inertial>
      <mass value="11.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

</robot>
```

- [ ] **Step 2: Commit**

```bash
git add description/prius_core.xacro
git commit -m "feat: add prius_core.xacro with chassis mesh + 4-wheel Ackermann joints"
```

---

## Task 4: Create `description/prius_ros2_control.xacro`

**Files:**
- Create: `description/prius_ros2_control.xacro`

Front steering joints use `position` command; rear driven joints use `velocity` command; front wheel joints (free spinning) have state interfaces only.

- [ ] **Step 1: Write the file**

Create `description/prius_ros2_control.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:arg name="use_ros2_control" default="true"/>
  <xacro:arg name="sim_mode"         default="false"/>

  <xacro:if value="$(arg use_ros2_control)">
    <ros2_control name="GazeboSystem" type="system">
      <hardware>
        <xacro:if value="$(arg sim_mode)">
          <plugin>gazebo_ros2_control/GazeboSystem</plugin>
        </xacro:if>
        <xacro:unless value="$(arg sim_mode)">
          <plugin>mock_components/GenericSystem</plugin>
        </xacro:unless>
      </hardware>

      <joint name="front_left_steering_joint">
        <command_interface name="position">
          <param name="min">-0.5</param>
          <param name="max">0.5</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>

      <joint name="front_right_steering_joint">
        <command_interface name="position">
          <param name="min">-0.5</param>
          <param name="max">0.5</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>

      <joint name="rear_left_wheel_joint">
        <command_interface name="velocity">
          <param name="min">-30</param>
          <param name="max">30</param>
        </command_interface>
        <state_interface name="velocity"/>
        <state_interface name="position"/>
      </joint>

      <joint name="rear_right_wheel_joint">
        <command_interface name="velocity">
          <param name="min">-30</param>
          <param name="max">30</param>
        </command_interface>
        <state_interface name="velocity"/>
        <state_interface name="position"/>
      </joint>

      <!-- Front wheel joints: state only (free spinning, no drive command) -->
      <joint name="front_left_wheel_joint">
        <state_interface name="velocity"/>
        <state_interface name="position"/>
      </joint>

      <joint name="front_right_wheel_joint">
        <state_interface name="velocity"/>
        <state_interface name="position"/>
      </joint>
    </ros2_control>
  </xacro:if>

</robot>
```

- [ ] **Step 2: Commit**

```bash
git add description/prius_ros2_control.xacro
git commit -m "feat: add prius_ros2_control.xacro with ackermann hardware interfaces"
```

---

## Task 5: Create `description/prius_gazebo.xacro`

**Files:**
- Create: `description/prius_gazebo.xacro`

- [ ] **Step 1: Write the file**

Create `description/prius_gazebo.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find gcamp_gazebo)/config/prius_controllers.yaml</parameters>
    </plugin>
  </gazebo>

  <gazebo reference="chassis">
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="front_left_wheel">
    <mu1>1.0</mu1><mu2>1.0</mu2>
    <kp>1000000.0</kp><kd>100.0</kd>
    <minDepth>0.001</minDepth><maxVel>1.0</maxVel>
  </gazebo>

  <gazebo reference="front_right_wheel">
    <mu1>1.0</mu1><mu2>1.0</mu2>
    <kp>1000000.0</kp><kd>100.0</kd>
    <minDepth>0.001</minDepth><maxVel>1.0</maxVel>
  </gazebo>

  <gazebo reference="rear_left_wheel">
    <mu1>1.0</mu1><mu2>1.0</mu2>
    <kp>1000000.0</kp><kd>100.0</kd>
    <minDepth>0.001</minDepth><maxVel>1.0</maxVel>
  </gazebo>

  <gazebo reference="rear_right_wheel">
    <mu1>1.0</mu1><mu2>1.0</mu2>
    <kp>1000000.0</kp><kd>100.0</kd>
    <minDepth>0.001</minDepth><maxVel>1.0</maxVel>
  </gazebo>

</robot>
```

- [ ] **Step 2: Commit**

```bash
git add description/prius_gazebo.xacro
git commit -m "feat: add prius_gazebo.xacro with gazebo_ros2_control plugin and wheel friction"
```

---

## Task 6: Create `description/prius.urdf.xacro`

**Files:**
- Create: `description/prius.urdf.xacro`

The VLP-16 sensor is inlined here (not re-using `vlp_16.xacro`) with the roof mount height `xyz="0 0 1.5"` appropriate for the Prius (wheel radius 0.31 + car body ~1.2m ≈ 1.5m from model origin).

- [ ] **Step 1: Write the file**

Create `description/prius.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot name="prius" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:arg name="use_ros2_control" default="true"/>
  <xacro:arg name="sim_mode"         default="false"/>

  <xacro:include filename="$(find gcamp_gazebo)/description/prius_core.xacro"/>
  <xacro:include filename="$(find gcamp_gazebo)/description/prius_ros2_control.xacro"/>
  <xacro:include filename="$(find gcamp_gazebo)/description/prius_gazebo.xacro"/>

  <!-- VLP-16 mounted on roof (z=1.5 from base_link ≈ roof of Prius) -->
  <joint name="velodyne_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 1.5"/>
    <parent link="base_link"/>
    <child  link="velodyne"/>
  </joint>

  <link name="velodyne">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1e-7" ixy="0" ixz="0" iyy="1e-7" iyz="0" izz="1e-7"/>
    </inertial>
    <visual>
      <geometry><cylinder radius="0.05" length="0.04"/></geometry>
      <material name="black"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
  </link>

  <gazebo reference="velodyne">
    <sensor name="velodyne-VLP16" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>1875</samples>
            <resolution>1</resolution>
            <min_angle>-3.1415926535897931</min_angle>
            <max_angle>3.1415926535897931</max_angle>
          </horizontal>
          <vertical>
            <samples>16</samples>
            <resolution>1</resolution>
            <min_angle>-0.2617993877991494</min_angle>
            <max_angle>0.2617993877991494</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.055</min>
          <max>140.0</max>
          <resolution>0.001</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.0</stddev>
        </noise>
      </ray>
      <plugin filename="libgazebo_ros_velodyne_laser.so" name="gazebo_ros_laser_controller">
        <topicName>/points_raw</topicName>
        <frameName>velodyne</frameName>
        <min_range>0.9</min_range>
        <max_range>130.0</max_range>
        <gaussianNoise>0.008</gaussianNoise>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

- [ ] **Step 2: Commit**

```bash
git add description/prius.urdf.xacro
git commit -m "feat: add prius.urdf.xacro entry point with VLP-16 on roof"
```

---

## Task 7: Create `config/prius_controllers.yaml`

**Files:**
- Create: `config/prius_controllers.yaml`

- [ ] **Step 1: Write the file**

Create `config/prius_controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 30

    ackermann_cont:
      type: ackermann_steering_controller/AckermannSteeringController

    joint_broad:
      type: joint_state_broadcaster/JointStateBroadcaster

ackermann_cont:
  ros__parameters:
    reference_timeout: 2.0
    front_steering: true
    open_loop: false
    velocity_rolling_window_size: 10
    position_feedback: false

    # Prius geometry (from prius_hybrid SDF, SDF→URDF coordinate transform applied)
    wheelbase: 2.86            # front_axle_x(1.41) + |rear_axle_x|(1.45)
    front_wheel_radius: 0.31265
    rear_wheel_radius: 0.31265
    front_wheels_track: 1.52   # 2 * front_track_y(0.76)
    rear_wheels_track: 1.572   # 2 * rear_track_y(0.786)

    rear_wheels_names:
      - rear_left_wheel_joint
      - rear_right_wheel_joint
    front_steering_joints_names:
      - front_left_steering_joint
      - front_right_steering_joint

    odom_frame_id: odom
    base_frame_id: base_link
    enable_odom_tf: true
```

- [ ] **Step 2: Commit**

```bash
git add config/prius_controllers.yaml
git commit -m "feat: add prius_controllers.yaml for ackermann_steering_controller"
```

---

## Task 8: Update launch files

**Files:**
- Modify: `launch/rsp.launch.py` (line 20)
- Modify: `launch/launch_sim.launch.py` (line 89–93)

- [ ] **Step 1: Swap xacro file in rsp.launch.py**

In `launch/rsp.launch.py`, change line 20 from:
```python
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
```
to:
```python
    xacro_file = os.path.join(pkg_path,'description','prius.urdf.xacro')
```

- [ ] **Step 2: Swap controller in launch_sim.launch.py**

In `launch/launch_sim.launch.py`, change the `diff_drive_spawner` node (lines 89–93) from:
```python
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
```
to:
```python
    ackermann_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_cont"],
    )
```

Also update the `return LaunchDescription([...])` block — replace `diff_drive_spawner` with `ackermann_spawner`:
```python
    return LaunchDescription([
        rsp,
        joystick,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawn_entity,
        ackermann_spawner,
        joint_broad_spawner
    ])
```

- [ ] **Step 3: Commit**

```bash
git add launch/rsp.launch.py launch/launch_sim.launch.py
git commit -m "feat: swap launch files to use prius.urdf.xacro and ackermann_cont"
```

---

## Task 9: Update nav2_params.yaml footprint

**Files:**
- Modify: `config/nav2_params.yaml`

The Prius is ~4.5m long × ~1.8m wide. Replace `robot_radius` with a rectangular `footprint`. Both occurrences of `robot_radius: 0.22` are in the `local_costmap` and `global_costmap` sections.

- [ ] **Step 1: Replace robot_radius with footprint polygon (both costmaps)**

In `config/nav2_params.yaml`, find both occurrences of:
```yaml
      robot_radius: 0.22
```
and replace each with:
```yaml
      footprint: "[[2.3, 0.9], [2.3, -0.9], [-2.25, -0.9], [-2.25, 0.9]]"
```

- [ ] **Step 2: Commit**

```bash
git add config/nav2_params.yaml
git commit -m "feat: update nav2 footprint for Prius dimensions (4.5m x 1.8m)"
```

---

## Task 9b: Add cmd_vel → AckermannDriveStamped converter for Nav2

**Files:**
- Create: `scripts/cmd_vel_to_ackermann.py`
- Modify: `CMakeLists.txt` — add scripts to install
- Modify: `launch/launch_sim.launch.py` — add converter node

Nav2 publishes `geometry_msgs/Twist` on `/cmd_vel`. The `ackermann_steering_controller` subscribes to `ackermann_msgs/AckermannDriveStamped` on `/ackermann_cont/reference`. This node bridges the gap using the Ackermann kinematic relation: `steering_angle = atan(angular.z * wheelbase / linear.x)`.

- [ ] **Step 1: Create scripts/cmd_vel_to_ackermann.py**

```bash
mkdir -p /home/ngin/fpt-capstone/simulation/gcamp_ros2_ws/scripts
```

Create `scripts/cmd_vel_to_ackermann.py`:

```python
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

WHEELBASE = 2.86  # metres (front_axle_x + |rear_axle_x|)

class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')
        self.pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cont/reference', 10)
        self.create_subscription(Twist, '/cmd_vel', self.callback, 10)

    def callback(self, msg: Twist):
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.drive.speed = msg.linear.x
        if abs(msg.linear.x) > 0.01:
            out.drive.steering_angle = math.atan(msg.angular.z * WHEELBASE / msg.linear.x)
        else:
            out.drive.steering_angle = 0.0
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(CmdVelToAckermann())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Make executable**

```bash
chmod +x /home/ngin/fpt-capstone/simulation/gcamp_ros2_ws/scripts/cmd_vel_to_ackermann.py
```

- [ ] **Step 3: Install scripts in CMakeLists.txt**

In `CMakeLists.txt`, change:
```cmake
install(
  DIRECTORY config description launch meshes worlds
  DESTINATION share/${PROJECT_NAME}
)
```
to:
```cmake
install(
  DIRECTORY config description launch meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/cmd_vel_to_ackermann.py
  DESTINATION lib/${PROJECT_NAME}
)
```

- [ ] **Step 4: Add converter node to launch_sim.launch.py**

In `launch/launch_sim.launch.py`, add this node definition after `ackermann_spawner`:
```python
    cmd_vel_to_ackermann = Node(
        package='gcamp_gazebo',
        executable='cmd_vel_to_ackermann.py',
        output='screen',
    )
```

And add `cmd_vel_to_ackermann` to the `return LaunchDescription([...])` list:
```python
    return LaunchDescription([
        rsp,
        joystick,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawn_entity,
        ackermann_spawner,
        joint_broad_spawner,
        cmd_vel_to_ackermann,
    ])
```

- [ ] **Step 5: Commit**

```bash
git add scripts/cmd_vel_to_ackermann.py CMakeLists.txt launch/launch_sim.launch.py
git commit -m "feat: add cmd_vel→AckermannDriveStamped converter for Nav2 compatibility"
```

---

## Task 10: Build and verify

- [ ] **Step 1: Build**

```bash
cd /home/ngin/fpt-capstone/simulation/gcamp_ros2_ws
colcon build --packages-select gcamp_gazebo
```
Expected: build succeeds with no errors.

- [ ] **Step 2: Verify xacro processes cleanly**

```bash
source install/setup.bash
xacro description/prius.urdf.xacro use_ros2_control:=true sim_mode:=true > /tmp/prius_check.urdf
echo "Exit code: $?"
```
Expected: exit code 0, no xacro errors.

- [ ] **Step 3: Validate URDF**

```bash
check_urdf /tmp/prius_check.urdf
```
Expected: "Successfully Parsed" with robot name "prius".

- [ ] **Step 4: Launch simulation**

```bash
ros2 launch gcamp_gazebo launch_sim.launch.py
```
Expected: Gazebo opens with Prius mesh visible, VLP-16 cylinder on roof. Check with:
```bash
ros2 topic list | grep -E "points_raw|odom|cmd_vel"
```
Expected topics: `/points_raw`, `/ackermann_cont/odom`, `/ackermann_cont/cmd_vel`.

- [ ] **Step 5: Test drive**

```bash
ros2 topic pub /ackermann_cont/reference ackermann_msgs/msg/AckermannDriveStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, drive: {speed: 1.0, steering_angle: 0.0}}" --once
```
Expected: car moves forward in Gazebo.

- [ ] **Step 5b: Test Nav2 cmd_vel bridge**

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```
Expected: converter translates to `/ackermann_cont/reference`, car moves forward.

- [ ] **Step 6: Final commit**

```bash
git add -p   # review any uncommitted changes
git commit -m "feat: verify prius ackermann simulation working in Gazebo"
```
