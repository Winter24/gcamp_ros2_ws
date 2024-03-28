import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    tarkbot_r10_akm_urdf_file = os.path.join(
        get_package_share_directory('tarkbot_description'),'urdf','tarkbot_r10_akm.urdf'
    )

    with open(tarkbot_r10_akm_urdf_file, 'r') as infp:
        robot_akm = infp.read()

    tarkbot_r10_fwd_urdf_file = os.path.join(
        get_package_share_directory('tarkbot_description'),'urdf','tarkbot_r10_fwd.urdf'
    )

    with open(tarkbot_r10_fwd_urdf_file, 'r') as infp:
        robot_fwd = infp.read()

    tarkbot_r10_mec_urdf_file = os.path.join(
        get_package_share_directory('tarkbot_description'),'urdf','tarkbot_r10_mec.urdf'
    )

    with open(tarkbot_r10_mec_urdf_file, 'r') as infp:
        robot_mec = infp.read()

    tarkbot_r10_omni_urdf_file = os.path.join(
        get_package_share_directory('tarkbot_description'),'urdf','tarkbot_r10_omni.urdf'
    )

    with open(tarkbot_r10_omni_urdf_file, 'r') as infp:
        robot_omni = infp.read()

    tarkbot_r10_twd_urdf_file = os.path.join(
        get_package_share_directory('tarkbot_description'),'urdf','tarkbot_r10_twd.urdf'
    )

    with open(tarkbot_r10_twd_urdf_file, 'r') as infp:
        robot_twd = infp.read()

    tarkbot_r20_akm_urdf_file = os.path.join(
        get_package_share_directory('tarkbot_description'),'urdf','tarkbot_r20_akm.urdf'
    )

    with open(tarkbot_r20_akm_urdf_file, 'r') as infp:
        robot_akm = infp.read()

    tarkbot_r20_fwd_urdf_file = os.path.join(
        get_package_share_directory('tarkbot_description'),'urdf','tarkbot_r20_fwd.urdf'
    )

    with open(tarkbot_r20_fwd_urdf_file, 'r') as infp:
        robot_fwd = infp.read()

    tarkbot_r20_mec_urdf_file = os.path.join(
        get_package_share_directory('tarkbot_description'),'urdf','tarkbot_r20_mec.urdf'
    )

    with open(tarkbot_r20_mec_urdf_file, 'r') as infp:
        robot_mec = infp.read()

    tarkbot_r10_akm = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': robot_akm}],
            arguments=[tarkbot_r10_akm_urdf_file]
        )
        
    tarkbot_r10_fwd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': robot_fwd}],
            arguments=[tarkbot_r10_fwd_urdf_file]
        )

    tarkbot_r10_mec = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': robot_mec}],
            arguments=[tarkbot_r10_mec_urdf_file]
        )

    tarkbot_r10_omni = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': robot_omni}],
            arguments=[tarkbot_r10_omni_urdf_file]
        )

    tarkbot_r10_twd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': robot_twd}],
            arguments=[tarkbot_r10_twd_urdf_file]
        )

    tarkbot_r20_akm = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': robot_akm}],
            arguments=[tarkbot_r20_akm_urdf_file]
        )

    tarkbot_r20_fwd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': robot_fwd}],
            arguments=[tarkbot_r20_fwd_urdf_file]
        )

    tarkbot_r20_mec = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': robot_mec}],
            arguments=[tarkbot_r20_mec_urdf_file]
        )

    ld = LaunchDescription()

    ld.add_action(tarkbot_r20_mec)

    return ld



