import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
# from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
# from launch.actions import RegisterEventHandler
# from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# import xacro


def generate_launch_description():

    # Package Names
    pkg_name = 'park_bot'
    pkg_park_bot = get_package_share_directory(pkg_name)

    # Robot State Publisher
    node_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_park_bot, 'launch', 'rsp_sim.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items())

    # Gazebo
    # launch_ignition = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             [os.path.join(get_package_share_directory('ros_ign_gazebo'),
    #                           'launch', 'ign_gazebo.launch.py')]),
    #         launch_arguments=[('gz_args', [' -r src/park_bot/worlds/park.world'])])

    # Spawn
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot',
                   '-world', 'src/park_bot/worlds/park.world'],
        output='screen')
    
    fws_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fws_controller"]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    # State Controller
    # load_joint_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'joint_state_broadcaster'],
    #     output='screen')

    # Trajectory Controller
    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'fws_controller'],
    #     output='screen')

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock]ignition.msgs.Clock'],
        output='screen')

    # gz_joint_controller = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=ignition_spawn_entity,
    #         on_exit=[load_joint_state_controller],))

    # gz_trajectory_controller = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=load_joint_state_controller,
    #         on_exit=[load_joint_trajectory_controller],))

    # Launch
    return LaunchDescription([
        node_robot_state_publisher,
        # launch_ignition,
        ignition_spawn_entity,
        fws_controller_spawner,
        joint_state_broadcaster_spawner,
        bridge,
        # gz_joint_controller,
        # gz_trajectory_controller,
    ])

    # Bridge
    # camera_info_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image',
    #                '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
    #     output='screen'
    # )

    # laser_scan_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
    #                '-r /lidar', 'laser_scan'],
    #     output='screen'
    # )

    # cmd_vel_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image',
    #                '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
    #     output='screen'
    # )
