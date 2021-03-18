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

I Scenario: SLAM
7.  `roslaunch omni3ros_pkg gazebo_rviz_controllers.launch`- launch model simulation (Gazebo + Rviz) with controllers
8.  `rosrun teleop_keyboard_omni3 teleop_keyboard_omni3.py`- send Twist control commands to model with keyboard
8.5 `rosrun teleop_keyboard_omni3 wheel_controller.py`- convert Twist control commands to motor velocity commands
9.  `rosrun teleop_keyboard_omni3 odom_publisher.py` - run odometry broadcasting
10. `roslaunch omni3ros_pkg gmapping.launch`- launch gmapping module o build the map
11. `rosrun map_server map_saver -f map` - save map of environment
12. `roslaunch omni3ros_pkg robot_localization.launch` - Run EKF wheel odometry and IMU sensor fusion
13. `rosrun rqt_gui rqt_gui` - Run rqt to vilusalize node graph, topics and tf 

II Scenario: Localization
7.  `roslaunch omni3ros_pkg gazebo_rviz_controllers.launch`- launch model simulation (Gazebo + Rviz) with controllers
8.  `rosrun teleop_keyboard_omni3 teleop_keyboard_omni3.py`- send Twist control commands to model with keyboard
8.5 `rosrun teleop_keyboard_omni3 wheel_controller.py`- convert Twist control commands to motor velocity commands
9.  `rosrun teleop_keyboard_omni3 odom_publisher.py` - run odometry broadcasting
10. `roslaunch omni3ros_pkg amcl.launch` - Run AMCL localization
11. `roslaunch omni3ros_pkg robot_localization.launch` - Run EKF wheel odometry and IMU sensor fusion
12. `rosrun rqt_gui rqt_gui` - Run rqt to vilusalize node graph, topics and tf 
