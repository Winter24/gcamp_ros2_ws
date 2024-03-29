This is the project i'm working on forked from <code>joshnewans/articubot_one</code>:

<h1>ROS2 web app for Slam and Navigation</h1>
Dependencies(a lot):
- Ros2 control
- Ros2 controllers
- gazebo-ros2-control

Doc: 

- https://control.ros.org/master/index.html

- https://github.com/ros-controls/roadmap/blob/master/design_drafts/hardware_access.md

Install Ros2 control: 
```bash
sudo apt install ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-gazebo-ros2-control
```

- Ros2 Robot pose publisher

Link: https://github.com/MilanMichael/robot_pose_publisher_ros2

- Ros2 web application:

  - Python, JavaScript, HTML, CSS, sqlite, Flask(v2.2.2)
  - Rosbridge Server:

    Doc: https://index.ros.org/p/rosbridge_server/

    Install Rosbridge Server: <code>sudo apt install ros-foxy-rosbridge-server</code>

There's also a lot of libraries and packages i couldn't remember while doing this project. If you encounter any errors, use apt, pip, npm to install it on your own (* ¯ ▽ ¯) b, or else contact me through the information i provide on my git profile.

Run:

To run the simulation world and robot:
```bash
ros2 launch gcamp_gazebo launch_sim.launch.py
```

To run the web-app:
- On the first terminal:
```bash
cd ~/gcamp_ros2_ws/ros2-web-bridge/
python3 app.py
```
- On the second terminal:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
- On the third terminal:
```bash
ros2 launch robot_pose_publisher_ros2 robot_pose_publisher_launch.py
```
Localhost:8080

=========================================================================

<h1>LIO-Sam on Ros2 Foxy</h1>

![image](https://github.com/Winter24/gcamp_ros2_ws/assets/70335675/f14465d2-648c-432e-b08c-c1fed29f9416)
![image](https://github.com/Winter24/gcamp_ros2_ws/assets/70335675/3beba068-6ec2-4330-b5a1-69253b84ef87)

Using <code>CloudCompare</code> to view the point cloud data

Things i added: 
- Lidar 3d (VLP - 16)
- An IMU sensor for the robot model
  
*Update: TF_OLD_DATA ignoring data from the past for frame base_link at time 746.980000 according to authority Authority undetectable 
(Actually it not really a bug, everything go well but it's funny that went i use the joystick to drive the robot the wheel of the robot seperate from the chassis and go opposite direction :))))))). As an OCD person, i'll try fixed it)

To run LIO-SAM: 
- First make sure you comment the 2d lidar xacro file <code>lidar.xacro</code> in the <code>./description/robot.urdf.xacro</code> file and enable the <code>vlp_16.xacro</code> and the <code>imu.xacro</code>.
  
  Things will look like this:
  
![image](https://github.com/Winter24/gcamp_ros2_ws/assets/70335675/58606398-ec42-4110-949e-47cec8aae775)

- Next, building the package again (just for sure):
```bash
cd gcamp_ros2_ws/
colcon build
source install/setup.bash
```
- Last step:
  On your first terminal:
```bash
ros2 launch gcamp_gazebo launch_sim.launch.py 
```
  On the second terminal:
```bash
ros2 launch lio_sam run.launch.py 
```
  On the 3rd terminal run:
```bash
rviz2 
```
  We still not done yet. On Rviz2 add these to see the 3d point cloud display. When you finish mapping just turn off (<code>ctrl + c</code>) the map will auto save in <code>/Downloads/LOAM/</code>. You can change it by edit the file <code>./LIO-SAM-ros2/config/params.yaml</code> at line 24 <code>savePCDDirectory: "/Downloads/LOAM/"</code>
  ![image](https://github.com/Winter24/gcamp_ros2_ws/assets/70335675/e6943756-5af4-4de8-b0e2-6838fb305d32)



