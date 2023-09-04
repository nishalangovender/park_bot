import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Package Name
    pkg_name = 'park_bot'
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Robot State Publisher
    node_robot_state_publisher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','rsp.launch.py')]), 
                    launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # Gazebo
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_ros_gz_sim, 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[('gz_args', ['-r src/park_bot/worlds/park.world'])])

    # Spawn
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-topic', 'robot_description',
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.065',
                    '-entity', 'my_bot'],
        output='screen')

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        launch_gazebo,
        spawn_entity
    ])

