import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# import xacro


def generate_launch_description():

    # Package Names
    pkg_name = 'park_bot'
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # pkg_ign_ros2_control = get_package_share_directory('ign_ros2_control_demos')

    # Use Sim Time
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Robot State Publisher
    node_robot_state_publisher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items(),
    )

    # Gazebo
    # gz_params_file = os.path.join(get_package_share_directory(pkg_name),'config','gz_params.yaml')
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_ros_gz_sim, 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[('gz_args', ['-r empty.sdf'])])
            # launch_arguments=[('gz_args', ['-r src/park_bot/worlds/park.world'])])
    
    # Spawn
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-topic', 'robot_description',
                    '-entity', 'my_bot'],
        output='screen')
    
    # State Controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Trajectory Controller
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'fws_controller'],
        output='screen'
    )
    
    gz_joint_controller = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
    )
    
    gz_trajectory_controller = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
    )

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
    
    # Launch
    return LaunchDescription([
        node_robot_state_publisher,
        launch_gazebo,
        spawn_entity,
        gz_joint_controller,
        gz_trajectory_controller,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])