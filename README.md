## Autonomous Parking of a 4WS Vehicle

# Description

Implemention and testing of the simulation for my final-year mechatronic project "Autonomous Parking of a 4WS Vehicle".

This project follows the Mechanical and Mechatronic Engineering Department module Mechatronics Project 488 at Stellenbosch University. <br/>
Student: Nishalan Govender <br/>
Supervisor: Prof J Engelbrecht

## Setup

This package is designed for ROS2 Humble and Ignition Gazebo Fortress. <br/>
The testing was completed on the virtual machine Parallels with a MacBook Pro M1 (arm64) running Ubuntu 22.04 LTS.

# Packages
Required packages include:
- xxx
- (ros2 humble)
- (ros ign)
- (ros2 control)
- (ros2 control ign)

# Instructions
1. Make directory from ${HOME} <br/>
   `mkdir -p ros2_ws/dev_ws/src`
2. Go into directory <br/>
   `cd ros2_ws/dev_ws/src`
3. Clone packages `park_bot` and `fws_controller` <br/>
   `gh repo clone nishalangovender/park_bot` <br/>
   `gh repo clone nishalangovender/fws_controller`
4. Move into workspace <br/>
   `cd ..`
5. Build with colcon <br/>
   `colcon build --symlink-install`
6. Source environment variable <br/>
   `source install/setup.bash`

## Methodology

# Robot State Publisher
1. Test `robot_state_publisher` with launch file <br/>
   Terminal 1: `ros2 launch park_bot rsp.launch.py` <br/>
   Terminal 2: `ros2 run joint_state_publisher_gui joint_state_publisher_gui` <br/>
   Terminal 3: `rviz2 -d view_bot.rviz`

# Ignition Gazebo
2. Test Ignition Gazebo Spawn with launch file <br/>
   `ros2 launch park_bot rsp_sim.launch.py`

# Simulation
3. Test full simulation in Ignition Gazbeo and ROS2 Control with launch file <br/>
   Terminal 1: `ros2 launch park_bot sim.launch.py` <br/>
   Terminal 2: `ros2 topic pub /steering_position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0, 1.0, 1.0, 1.0]"` <br/>
   Terminal 3: `ros2 topic pub /wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0, 1.0, 1.0, 1.0]"`

## RViz

1. Set fixed frame to `world`.
2. Add `RobotModel` display <br/>
   Topic `/robot_description` <br/>
   Alpha 0.8
3. Add `TF`

## Notes

Using Robot Package Template by Josh Newans. <br/>
If you are learning robotics, ROS2, Gazebo etc., I strongly recommend Articulated Robotics (https://articualtedrobotics.xyz) and YouTube channel (https://www.youtube.com/@ArticulatedRobotics).