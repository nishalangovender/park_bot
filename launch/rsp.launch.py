import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check Use Sim Time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process URDF File
    pkg_name = 'park_bot'
    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': use_sim_time}

    print(params)
    
    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # Launch
    return LaunchDescription([
        node_robot_state_publisher,
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, 
                              description='If true, use simulated clock'),
    ])