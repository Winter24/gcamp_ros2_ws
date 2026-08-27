from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "data_root", default_value="/home/winter24/ros2_ws/Data_1/fused/9"
        ),
        DeclareLaunchArgument("sequence", default_value="both"),
        DeclareLaunchArgument("playback_rate", default_value="1.0"),
        DeclareLaunchArgument("loop", default_value="false"),
        DeclareLaunchArgument("pointcloud_topic", default_value="/fused_points"),
        DeclareLaunchArgument("publish_raw_image", default_value="false"),
    ]
    playback = Node(
        package="gcamp_gazebo",
        executable="sq_dataset_playback.py",
        name="sq_dataset_playback",
        output="screen",
        parameters=[
            {
                "data_root": LaunchConfiguration("data_root"),
                "sequence": LaunchConfiguration("sequence"),
                "playback_rate": LaunchConfiguration("playback_rate"),
                "loop": LaunchConfiguration("loop"),
                "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                "publish_raw_image": LaunchConfiguration("publish_raw_image"),
            }
        ],
    )
    return LaunchDescription(arguments + [playback])
