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

    # Robot State Publisher
    node_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_park_bot, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items())

    # Gazebo
    launch_ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('gz_args', [' -r src/park_bot/worlds/park.world'])])

    # Spawn
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot'],
        output='screen')

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        launch_ignition,
        ignition_spawn_entity
    ])

