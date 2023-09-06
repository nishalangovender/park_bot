## Autonomous Parking of a 4WS Vehicle

Using Robot Package Template by Josh Newans.
If you are learning robotics, ROS2, Gazebo etc., I strongly recommend Articulated Robotics (https://articualtedrobotics.xyz) and YouTube channel (https://www.youtube.com/@ArticulatedRobotics).

## Setup

This package is designed for ROS2 Humble and Ignition Gazebo Fortress.
The testing was completed on the virtual machine Parallels with a MacBook Pro M1 (arm64) running Ubuntu 22.04 LTS.

Required packages include:
- xxx

1. Make directory from ${HOME}
   `mkdir -p ros2_ws/dev_ws/src`
2. Go into directory
   `cd ros2_ws/dev_ws/src`
3. Clone packages `park_bot` and `fws_controller`
   `gh repo clone nishalangovender/park_bot`
   `gh repo clone nishalangovender/fws_controller`
4. Move into workspace
   `cd ..`
5. Build with colcon
   `colcon build --symlink-install`
6. Source environment variable
   `source install/setup.bash`

## Methodology

1. Test `robot_state_publisher` with launch file
   Terminal 1: `ros2 launch park_bot rsp.launch.py`
   Terminal 2: `ros2 run joint_state_publisher_gui joint_state_publisher_gui`
   Terminal 3: `rviz2 -d view_bot.rviz`
2. Test Ignition Gazebo Spawn with launch file
   `ros2 launch park_bot rsp_sim.launch.py`
3. Test full simulation in Ignition Gazbeo and ROS2 Control with launch file
   `ros2 launch park_bot sim.launch.py`

## RViz

1. Set fixed frame to `world`.
2. Add `RobotModel` display
   Topic `/robot_description`
   Alpha 0.8
3. Add `TF`