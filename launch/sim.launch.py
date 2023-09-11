import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process URDF File
    pkg_name = 'park_bot'
    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}

    # Robot State Publisher Node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params])

    # Launch Gazebo
    launch_ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                          'launch', 'ign_gazebo.launch.py')]),
        launch_arguments=[('gz_args', [' -r empty.sdf'])])
        # launch_arguments=[('gz_args', [' -r src/park_bot/worlds/park.world'])])

    # Spawn Robot in Gazebo
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot'],
        output='screen')
    
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen')

    # Controller Spawner
    fws_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fws_controller"])
    
    steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_position_controller"])
    
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_velocity_controller"])

    steering_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_trajectory_controller"])
    
    wheel_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_trajectory_controller"])

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"])

    # Launch Arguments
    sim_time_args = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true')

    ros2_control_args = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control if true')
    
    # Steering
    steer = Node(
        package='park_bot',
        executable='steer.py',
        output='screen',
        arguments=["0.5"])

    # Launch
    return LaunchDescription([sim_time_args,
                              ros2_control_args,
                              node_robot_state_publisher,
                              launch_ignition,
                              ignition_spawn_entity,
                              bridge,
                              # fws_controller_spawner,
                              steering_controller_spawner,
                              wheel_controller_spawner,
                              # steering_trajectory_controller_spawner,
                              # wheel_trajectory_controller_spawner,
                              joint_broad_spawner,
                              # steer
                              ])