#install dependency:
- These should be installed:
```
sudo apt install ros-humble-rqt* -y
sudo apt install ros-humble-image-view -y 
sudo apt install ros-humble-joint-state-publisher-gui -y
sudo apt install ros-humble-xacro -y
sudo apt install -y python3-pip
pip3 install -U argcomplete
sudo apt install ros-humble-gazebo-ros-pkgs -y
```
- Ros2 control and diff drive (for moving the robot, must bee installed)
```
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-diff-drive-controller
sudo apt install ros-humble-joint-state-broadcaster
sudo apt install ros-humble-joint-trajectory-controller
```
- For SLAM and Navigation (optiional):
```
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
```
- For 3d lidar (not reaally use in this simulation but you can install it if you want):
```
sudo apt install ros-humble-pointcloud-to-laserscan
sudo apt install ros-humble-pcl-conversions
sudo apt install ros-humble-laser-geometry

sudo apt install ros-humble-perception-pcl
sudo apt install ros-humble-velodyne

sudo apt install ros-humble-ouster-driver
```
model file
https://drive.google.com/file/d/1YAregqyx0IYaNxJuJUjsCAcGJp42SQzS/view?usp=sharing


Extract the model.zip file into your /.gazebo/models/ folder like this:
![image](https://github.com/user-attachments/assets/df51cfff-b771-44e8-a850-17f2e9e904f9)










