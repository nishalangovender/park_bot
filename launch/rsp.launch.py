import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check Use Sim Time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process URDF File
    pkg_path = os.path.join(get_package_share_directory('park_bot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Robot State Publisher
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments={'gz_args': '-r empty.sdf'}.items(),
        )
    
    # Launch
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        robot_state_publisher,
        gazebo,
    ])

    # Spawn Entity
    # spawn = Node(package='ros_gz_sim', executable='create',
    #                 arguments=[
    #                     'name', 'my_park_bot',
    #                     '-x', '0',
    #                     '-y', '0',
    #                     '-z', '0',
    #                     '-topic', 'robot_description',
    #                     '-entity', 'my_bot'],
    #                 output='screen')