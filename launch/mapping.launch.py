import launch
import os
import yaml
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,ExecuteProcess,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression,Command
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    bringup_tarkbot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('tarkbot_robot')),'launch','robot.launch.py'))
    )

    bringup_ld19lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('ldlidar_stl_ros2')),'launch', 'tarkbot_ld19.launch.py'))
    )

    bringup_rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rplidar_ros')),'launch','rplidar_tarkbot.launch.py'))
    )

    bringup_m1ct = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('m1ct_d2')),'launch','tarkbot_m1ct.launch.py'))
    )

    bringup_mapping_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('tarkbot_slam')),'launch','include','online_async_launch.py'))
    )

    ld.add_action(bringup_tarkbot_driver)
    ld.add_action(bringup_rplidar)
    ld.add_action(bringup_m1ct)
    # ld.add_action(bringup_ld19lidar)
    ld.add_action(bringup_mapping_node)

    return ld

