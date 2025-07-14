# Custom-DWA-planner-for-Differential-Drive-Robot
Implemented a custom Dynamic Window Approach (DWA) local planner for a TurtleBot in Gazebo using ROS2 Humble.

SETUP INSTRUCTIONS-

-Paste the files in the base directory

-Go to dwa_custom/src/ in the terminal

-In the terminal, execute:
  colcon build
  
-source the workspace, execute:
  source install/local_setup.bash
  
-Now execute:
  ros2 run my_dwa_planner dwa_node
  
-In a new terminal, execute:
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
  
  (in case you havent downloaded turtlebot3, first execute: sudo apt install ros-humble-turtlebot3*)
  
-In another new terminal, execute:
  rviz2
  
-In rviz, File -> open config, direct to rviz_config.rviz in my_dwa_planner/src/rviz_config

- Now to use '2D Goal Pose' in Rviz window to create a goal pose, which the turtlebot will move to using the DWA local planner

