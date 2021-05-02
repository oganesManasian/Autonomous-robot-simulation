#!/usr/bin/env python

"""
This code does:
Controling of each wheel motor listening Twist commands sent with either keyboard or move_base 
"""

from __future__ import print_function

import rospy
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3
import tf
import sys, select, termios, tty

# # Usage: (left_wheel_velocity, back_wheel_velocity, right_wheel_velocity) = np.matmul(TWIST2WHEEL_VELOCITY, (x_velocity, y_velocity, turn_velocity))
TWIST2WHEEL_VELOCITY = np.array([[-1/3, -1/np.sqrt(3), 1/3],
                                 [2/3, 0, 1/3],
                                 [-1/3, 1/np.sqrt(3), 1/3]])


ROBOT_RADIUS = 0.7 # Based on +-0.04 rim shifts in main.urdf.xacro
ROBOT_WHEEL_RADIUS = 0.08 # Based on +-0.0055 roller z-axis positions in rim.urdf.xacro  

PUB_LEFT = rospy.Publisher('/open_base/left_joint_velocity_controller/command', Float64, queue_size=1)
PUB_BACK = rospy.Publisher('/open_base/back_joint_velocity_controller/command', Float64, queue_size=1)
PUB_RIGHT = rospy.Publisher('/open_base/right_joint_velocity_controller/command', Float64, queue_size=1)

def publish_commands(data):
    vx = data.linear.x
    vy = data.linear.y
    vth = data.angular.z * ROBOT_RADIUS # Back to linear speed from angular speed of robot

    # Since we rotated model 90 degrees clockwise we need to recompute TWIST2WHEEL_VELOCITY. But so far let's just substitute x and y
    vx, vy = -vy, vx 
    (left_wheel_velocity, back_wheel_velocity, right_wheel_velocity) = np.matmul(TWIST2WHEEL_VELOCITY, (vx, vy, vth))

    vell = Float64(left_wheel_velocity / ROBOT_WHEEL_RADIUS) # Velocity controller takes angular speed of wheel
    velb = Float64(back_wheel_velocity / ROBOT_WHEEL_RADIUS)
    velr = Float64(right_wheel_velocity / ROBOT_WHEEL_RADIUS)

    print("Published commands: ", vell, velb, velr)
    PUB_LEFT.publish(vell)
    PUB_BACK.publish(velb)
    PUB_RIGHT.publish(velr)
    
def listener():
    rospy.init_node('wheel_controller')

    rospy.Subscriber("cmd_vel", Twist, publish_commands)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
