import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Package Names
    pkg_name = 'park_bot'
    pkg_park_bot = get_package_share_directory(pkg_name)

    # Launch Robot State Publisher
    node_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_park_bot, 'launch', 'rsp.launch.py')]),
            launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items())

    # Launch Gazebo
    launch_ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                          'launch', 'ign_gazebo.launch.py')]),
        launch_arguments=[('gz_args', [' -r empty.sdf'])])
    
    # Spawn Robot in Gazebo
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.5'],
        output='screen')
    
    # Bridge
    pose_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/dynamic_pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V --ros-args -r /world/empty/dynamic_pose/info:=/pose'],
        output='screen')
    
    odom_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        output='screen')
    
    # Controller Spawner
    steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_position_controller"])

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_velocity_controller"])

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"])

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        launch_ignition,
        ignition_spawn_entity,
        pose_bridge,
        odom_bridge,
        steering_controller_spawner,
        wheel_controller_spawner,
        joint_broad_spawner])
