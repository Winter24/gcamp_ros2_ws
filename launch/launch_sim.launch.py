import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='gcamp_gazebo' #<--- CHANGE ME 
    world_file_name = "small_city_sdc_prius.world" #<--- CHANGE ME small_house
    # publish_rate = 

    pkg_path = os.path.join(get_package_share_directory(package_name))
    world_path = os.path.join(pkg_path, "worlds", world_file_name)

    # Make Gazebo prefer the Prius model bundled with this package.
    # This model has the VLP-16 3D LiDAR mounted on the roof.
    gcamp_model_path = os.path.join(pkg_path, "models")
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = gcamp_model_path + (os.pathsep + existing_model_path if existing_model_path else '')

    # Ensure Gazebo can find libgazebo_ros_velodyne_laser.so from the workspace install.
    ws_install_path = os.path.abspath(os.path.join(pkg_path, '..', '..', '..'))
    velodyne_plugin_path = os.path.join(ws_install_path, 'velodyne_gazebo_plugins', 'lib')
    existing_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    gazebo_plugin_path = velodyne_plugin_path + (os.pathsep + existing_plugin_path if existing_plugin_path else '')

    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=gazebo_plugin_path
    )

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world_path,
            # 'publish_rate': publish_rate
        }.items()
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        launch_arguments={
            # 'publish_rate': publish_rate
        }.items()
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    # twist_mux = Node(
    #         package="twist_mux",
    #         executable="twist_mux",
    #         parameters=[twist_mux_params, {'use_sim_time': True}],
    #         remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    #     )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot'
                    # '-x', '0',      # x-coordinate
                    # '-y', '0',      # y-coordinate
                    # '-z', '0.1',    # z-coordinate
                    # '-Y', '-1.57',      # Yaw in radians(π/2 radians =  90 degrees)
                    # '-R', '0',      # Roll in radians
                    # '-P', '0'       # Pitch in radians
                ],
        output='screen')


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )


    # Static TF for the VLP-16 LiDAR mounted on the Prius roof.
    # Gazebo publishes /points_raw with frame_id=velodyne, but it does not
    # publish the chassis -> velodyne transform for this SDF-only model.
    velodyne_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='chassis_to_velodyne_tf',
        arguments=[
            '0', '0.25', '1.58',      # x y z: same pose as velodyne link in model.sdf
            '0', '0', '0',           # roll pitch yaw
            'chassis', 'velodyne'
        ],
        output='screen'
    )

    base_to_chassis_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_chassis_tf',
        arguments=[
            '0', '0', '0',   # X, Y, Z (Giả sử chassis và base_link trùng nhau)
            '0', '0', '0',   # Roll, Pitch, Yaw
            'base_link', 'chassis'  # <--- Nếu Bước 1 ra 'base_footprint', hãy thay 'base_link' bằng 'base_footprint'
        ],
        output='screen'
    )


    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner



    # Launch them all!
    return LaunchDescription([
        set_gazebo_model_path,
        set_gazebo_plugin_path,
        velodyne_tf_pub,
        # base_to_chassis_tf,
        # rsp,
        # joystick,
        # twist_mux,
        # gazebo,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        # spawn_entity,
        # diff_drive_spawner,
        # joint_broad_spawner
    ])
