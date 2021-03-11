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
7.  `roslaunch omni3ros_pkg gazebo_rviz_controllers.launch`- launch model simulation (Gazebo + Rviz) with controllers
8.  `rosrun teleop_keyboard_omni3 teleop_keyboard_omni3.py`- control model with keyboard
9.  `rosrun teleop_keyboard_omni3 odom_tf_publisher.py` - run odometry broadcasting
10. `roslaunch omni3ros_pkg gmapping.launch`- launch gmapping module o build the map
11. `rosrun map_server map_saver -f map` - save map of environment
