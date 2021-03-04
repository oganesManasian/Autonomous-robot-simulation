# Autonomous robot simulation
Project does simulation of environment where robot have to navigate in known place but with dynamic objects.

## Features
1. ROS1 (Noetic)
2. Gazebo 11
3. Omnidirectional robot
4. Localisation
5. Dynamic environment (people)
  

## Run
1.  `cd ~/catkin_ws/src`
2.  `git clone https://github.com/REPOSITORY_NAME`
3.  `cd ~/catkin_ws`
4.  `catkin_make`
5.  `source ~/catkin_ws/devel/setup.bash`
6.  `source ~/.bashrc`
7.  `roslaunch omni3ros_pkg velocity_controller.launch `- launch model simulation with controllers
8.  `rosrun teleop_keyboard_omni3 teleop_keyboard_omni3.py `- control model with keyboard
