# Custom Dynamic Window Approach based local path planner, used by a Differential Drive Robot in a custom world

This is a custom Dynamic Window Approach (DWA) local planner for a TurtleBot in Gazebo Classic using ROS2 Humble.

## Pre-requisites:
- Installed ROS2 Humble and Gazebo Classic in Linux Ubuntu. Use official sites.

Gazebo - https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install

ROS2 Humble - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html


- Turtlebot3 waffle needs to be installed and sourced. 

Turtlebot3 - https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#download-and-install-ubuntu-on-pc

After installing turtlebot3, paste the (command: export TURTLEBOT3_MODEL=waffle) in your bash file '.bashrc', and source it (command: source ~/.bashrc)

- Install tf transformations (command: sudo apt install ros-humble-tf-transformations)


## SETUP INSTRUCTIONS:

- After downloading and extracting, go to root directory 'planner_ws' and use (command: colcon build --symlink-install)

- Now use the launch file (command: ros2 launch dwa_planner display.launch.py). This will open Gazebo, Rviz, turtlebot, and the planning nodes.

- Now use '2D Goal Pose' in Rviz window to create a goal pose, to which the turtlebot will move, using the DWA local planner.

