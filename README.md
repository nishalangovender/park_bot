## Autonomous Parking of a 4WS Vehicle

Using Robot Package Template by Josh Newans

Note that each directory currently has at least one file in it to ensure that git tracks the files (and, consequently, that a fresh clone has directories present for CMake to find). These example files can be removed if required (and the directories can be removed if `CMakeLists.txt` is adjusted accordingly).

## Methodology

1. Build the package with colcon.
2. Launch the `robot_state_publisher` launch file with `ros2 launch urdf_example rsp.launch.py`.
3. Launch `joint_state_publisher` with `ros2 run joint_state_publisher_gui joint_state_publisher_gui`.
4. Launch RViz with `rviz2`.

## RViz

1. Set fixed frame to `world`.
2. Add `RobotModel` display
   Topic `/robot_description`
   Alpha 0.8
3. Add `TF`