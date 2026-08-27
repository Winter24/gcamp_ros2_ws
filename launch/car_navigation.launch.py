import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("gcamp_gazebo")
    params_file = os.path.join(pkg_share, "config", "nav2_params.yaml")
    map_file = os.path.join(pkg_share, "maps", "small_city.yaml")

    # Nav2 only provides map, localization and Smac global planning.
    # The custom hybrid pure-pursuit node executes the returned path.
    lifecycle_nodes = ["map_server", "amcl", "planner_server"]

    return LaunchDescription([
        Node(
            package="gcamp_gazebo",
            executable="dynamic_tf_broadcaster.py",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[params_file, {"yaml_filename": map_file}],
            output="screen",
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[
                {"use_sim_time": True},
                {"autostart": True},
                {"node_names": lifecycle_nodes},
            ],
            output="screen",
        ),
    ])
