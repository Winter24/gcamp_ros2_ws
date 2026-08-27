#!/usr/bin/env python3
"""
sim_bringup.launch.py — one-command simulation bringup.

Replaces the three-terminal workflow:
  1. ros2 launch gcamp_gazebo launch_sim.launch.py
  2. ros2 launch gcamp_gazebo car_navigation.launch.py
  3. Custom hybrid pure-pursuit path execution

Startup is sequenced with TimerAction so each layer has what it needs:
  t=0s   Gazebo (gzserver; gzclient only if gui:=true) -> /clock, TF, sensors
  t=5s   Nav2 map/localization/Smac planner + custom path controller

Headless by default (gui:=false) to save RAM/VRAM for the webapp.
Run with gui:=true to also open the Gazebo 3D window for debugging.

Usage:
  ros2 launch gcamp_gazebo sim_bringup.launch.py
  ros2 launch gcamp_gazebo sim_bringup.launch.py gui:=true
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_launch = os.path.join(
        get_package_share_directory('gcamp_gazebo'), 'launch'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Open the Gazebo 3D client window (gzclient). '
                    'Default false = headless (saves RAM/VRAM for the webapp).'
    )

    # t=0s : Gazebo (gzserver always; gzclient gated by gui inside launch_sim).
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_launch, 'launch_sim.launch.py')
        ),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items()
    )

    # t=5s : Nav2 planning stack and custom Smac path follower.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_launch, 'car_navigation.launch.py')
        )
    )
    nav2_delayed = TimerAction(period=5.0, actions=[nav2])

    hybrid_pure_pursuit = Node(
        package='gcamp_gazebo',
        executable='hybrid_pure_pursuit.py',
        name='nav2_pure_pursuit_pid',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    controller_delayed = TimerAction(
        period=7.0,
        actions=[hybrid_pure_pursuit],
    )

    return LaunchDescription([
        declare_gui,
        gazebo,
        nav2_delayed,
        controller_delayed,
    ])
