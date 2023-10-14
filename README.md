# Autonomous Parking of a 4WS Vehicle

## Description

Implemention and testing of the simulation for my final-year mechatronic project "Autonomous Parking of a 4WS Vehicle".

This project follows the Mechanical and Mechatronic Engineering Department module Mechatronics Project 488 at Stellenbosch University.

Student: Nishalan Govender <br/>
Supervisor: Prof J Engelbrecht

# Setup

Software:
- ROS2 Humble
- Ignition Gazebo (Fortress)
- Ubuntu 22.04 LTS
- Parallels VM (virtual machine)

Hardware:
- ARM64 (MacBook Pro M1)

## Packages

Required packages include:
- ros-humble-desktop
- python3-colcon-common-extensions
- ros-humble-ros-gz
- ros-humble-ros2-control
- ros-humble-nav2-amcl
- ros-humble-nav2-bringup

Helpful:
- ros-humble-ign-ros2-control-demos

## Instructions

1. Make directory from ${HOME} <br/>
   `mkdir -p ros2_ws/dev_ws/src`
2. Go into directory <br/>
   `cd ros2_ws/dev_ws/src`
3. Clone packages `park_bot`, `fws_controller` and `rf2o_laser_odometry`<br/>
   `gh repo clone nishalangovender/park_bot` <br/>
   `gh repo clone nishalangovender/fws_controller` <br />
   `gh repo clone nishalangovender/rf2o_laser_odometry`
4. Move into workspace <br/>
   `cd ..`
5. Build with colcon <br/>
   `colcon build --symlink-install`
6. Source environment variable <br/>
   `source install/setup.bash`

# Methodology

## Robot State Publisher

1. Test `robot_state_publisher` with launch file <br/>
   Terminal 1: `ros2 launch park_bot rsp.launch.py` <br/>
   Terminal 2: `ros2 run joint_state_publisher_gui joint_state_publisher_gui` <br/>
   Terminal 3: `rviz2 -d view_bot.rviz`

## Ignition Gazebo

2. Test Ignition Gazebo Spawn with launch file <br/>
   `ros2 launch park_bot rsp_sim.launch.py`

## Simulation

3. Test full simulation in Ignition Gazebo, using ROS2 Control, SLAM and NAV2 with launch file <br/>
   Terminal 1: `ros2 launch park_bot sim.launch.py` <br/>
   Terminal 2: `rviz2 -d src/park_bot/config/main.rviz` <br/>
   Terimnal 3: `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true`

4. Set `Goal Pose` and watch the park_bot move!

## Notes

Using Robot Package Template by Josh Newans. <br/>
If you are learning robotics, ROS2, Gazebo etc., I strongly recommend Articulated Robotics (https://articualtedrobotics.xyz) and YouTube channel (https://www.youtube.com/@ArticulatedRobotics).
