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
    robot_description_config = Command(
        ['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    params = {'robot_description': robot_description_config,
              'use_sim_time': use_sim_time}

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
        # launch_arguments=[('gz_args', [' -r empty.sdf'])])
        launch_arguments=[('gz_args', [' -r src/park_bot/worlds/park.sdf'])])

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
    laser_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /scan:=/laser_scan'],
        output='screen')
    
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

    # Publisher Node
    # publisher = Node(
    #     package='fws_publisher',
    #     executable='publisher',
    #     output='screen')

    # Action Node
    inverse_kinematics = Node(
        package='fws_controller',
        executable='fws_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])
    
    # Odometry
    odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('rf2o_laser_odometry'),
                          'launch', 'rf2o_laser_odometry.launch.py')]))

    # Joy
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_path, 'launch', 'joystick.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items())
    
    # Static Transform
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['--frame-id', 'laser_frame',
                   '--child-frame-id','robot/base_link/laser'])
    
    # SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('slam_toolbox'),
                          'launch', 'online_async_launch.py')]),
        launch_arguments=[('slam_params_file', ['./src/park_bot/config/slam.yaml']),
                          ('use_sim_time', use_sim_time)])

    # Launch Arguments
    sim_time_args = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true')

    ros2_control_args = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control if true')

    # Launch
    return LaunchDescription([sim_time_args,
                              ros2_control_args,
                              node_robot_state_publisher,
                              launch_ignition,
                              ignition_spawn_entity,
                              laser_bridge,
                              pose_bridge,
                              odom_bridge,
                              steering_controller_spawner,
                              wheel_controller_spawner,
                              joint_broad_spawner,
                              inverse_kinematics,
                              odometry,
                              joystick,
                              static_tf,
                              slam])
