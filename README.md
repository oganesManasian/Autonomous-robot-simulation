# Autonomous robot simulation
Project does simulation of environment where robot have to navigate in known place with dynamic objects.

## Features
1. ROS1 (Noetic)
2. Gazebo 11
3. Omnidirectional robot
4. Scenarios: Static environment (Done) / Static environment with dynamic objects (TODO) / Dynamic environment with Dynamic objects (TODO)
5. Localisation
6. Autonomous navigation
  

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
9. `rosrun teleop_keyboard_omni3 wheel_controller.py`- convert Twist control commands to motor velocity commands
10.  `rosrun teleop_keyboard_omni3 odom_publisher.py` - run odometry broadcasting
11. `roslaunch omni3ros_pkg gmapping.launch`- launch gmapping module o build the map
12. `rosrun map_server map_saver -f map` - save map of environment
13. `roslaunch omni3ros_pkg robot_localization.launch` - Run EKF wheel odometry and IMU sensor fusion
14. `roslaunch omni3ros_pkg move_base.launch` - For autonomous navigation based on goal set with use of Rviz
15. `rosrun rqt_gui rqt_gui` - Run rqt to vilusalize node graph, topics and tf 

II Scenario: Localization
7.  `roslaunch omni3ros_pkg gazebo_rviz_controllers.launch`- launch model simulation (Gazebo + Rviz) with controllers
8.  `rosrun teleop_keyboard_omni3 teleop_keyboard_omni3.py`- send Twist control commands to model with keyboard
9. `rosrun teleop_keyboard_omni3 wheel_controller.py`- convert Twist control commands to motor velocity commands
10.  `rosrun teleop_keyboard_omni3 odom_publisher.py` - run odometry broadcasting
11. `roslaunch omni3ros_pkg amcl.launch` - Run AMCL localization
12. `roslaunch omni3ros_pkg robot_localization.launch` - Run EKF wheel odometry and IMU sensor fusion
13. `roslaunch omni3ros_pkg move_base.launch` - For autonomous navigation based on goal set with use of Rviz
14. `rosrun rqt_gui rqt_gui` - Run rqt to vilusalize node graph, topics and tf 

## Autonomous navigation
Goal poses can be set with use of Rviz. 

## Releases
1.0:
* Omnidirectional platform with IMU and 2D laser sensors. EKF does fusion of wheel odometry with IMU data. Than AMCL takes the result and predicts the correct position taking also into account laser scans. Standard ROS path planning algorithm based on set goal position provides global and local paths and control commands to follow them. Platform could be either controlled by these commands to reach the goal pose or with manual keyboard interface.