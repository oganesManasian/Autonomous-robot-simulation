#!/usr/bin/env python

"""
This code does:
1) Teleoperation of robot with use of keyboard
2) Publishing of Twist messages for odom_tf_publisher.py which will use it to publish tf transform base_link -> odom
"""

from __future__ import print_function

import rospy
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3
import tf
import sys, select, termios, tty

ROBOT_WHEEL_RADIUS = 0.011 # Based on +-0.0055 roller z-axis positions in rim.urdf.xacro  
ROBOT_RADIUS = 0.08 # Based on +-0.04 rim shofts in main.urdf.xacro
BASE_SPEED = 0.1 # Initial speed

help_info = """
Use keyboard to set move directions:
    w
a   s   d
r - rotate clockwise
e - rotate backwards

anything else : stop

q/z : increase/decrease max speeds by 10%

CTRL-C to quit
"""

# From keyboard to twist
MOVE_BINDINGS = {
    'w':"UP",
    'a':"LEFT",
    's':"DOWN",
    'd':"RIGHT",
    'e':"COUNTER_CLOCKWISE",
    'r':"CLOCKWISE",
}

# From Twist to each motor command 
MOTOR_BINDINGS = {
    "UP":(-0.57, 0, 0.57),
    "DOWN":(0.57, 0, -0.57),
    "LEFT":(0.33, -0.67, 0.33),
    "RIGHT":(-0.33, 0.67, -0.33),
    "COUNTER_CLOCKWISE":(0.33, 0.33, 0.33),
    "CLOCKWISE":(-0.33, -0.33, -0.33),
}

# TWIST is represented as (vx, vy, vth)
MOVE_DIRECTION2TWIST = {
    "UP": (0, 1, 0),
    "LEFT": (-1, 0, 0),
    "RIGHT": (1, 0, 0),
    "DOWN": (0, -1, 0),
    "COUNTER_CLOCKWISE": (0, 0, -1),
    "CLOCKWISE": (0, 0, 1),
}

# # Usage: (left_wheel_velocity, back_wheel_velocity, right_wheel_velocity) = np.matmul(DIRECTION_2_WHEEL_VELOCITY_MATRIX, (forward_velocity, right_velocity, turn_velocity))
# DIRECTION_2_WHEEL_VELOCITY_MATRIX = np.matrix([[1/3, -1/np.sqrt(3), 1/3],
#                                                 [-2/3, 0, 1/3],
#                                                 [1/3, 1/np.sqrt(3), 1/3]])

SPEED_BINDINGS={
        'q':1.1,
        'z':.9,
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_speed(speed):
    return "Currently:\tspeed %s " % (speed)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('velocity_publisher')
    publ = rospy.Publisher('/open_base/left_joint_velocity_controller/command', Float64, queue_size=1)
    pubb = rospy.Publisher('/open_base/back_joint_velocity_controller/command', Float64, queue_size=1)
    pubr = rospy.Publisher('/open_base/right_joint_velocity_controller/command', Float64, queue_size=1)

    pub_twist = rospy.Publisher('/open_base/twist', Twist, queue_size=1)

    speed = BASE_SPEED
    speed_l = speed_b = speed_r = 0 # Motor speeds
    vx = vy = vth = 0 # Direction speeds

    try:
        while True:
            # Publish control commands
            print(help_info)
            print(print_speed(speed))

            key = getKey()
            if key in MOVE_BINDINGS.keys():
                move_direction = MOVE_BINDINGS[key]

                speed_l = MOTOR_BINDINGS[move_direction][0]
                speed_b = MOTOR_BINDINGS[move_direction][1]
                speed_r = MOTOR_BINDINGS[move_direction][2]

                vx, vy, vth = MOVE_DIRECTION2TWIST[move_direction]
            elif key in SPEED_BINDINGS.keys():
                speed = speed * SPEED_BINDINGS[key]
            else: # Any other key => stop
                speed_l = speed_b = speed_r = 0
                vx = vy = vth = 0
                if (key == '\x03'):
                    break
            
            vell = Float64(speed_l * speed / ROBOT_WHEEL_RADIUS) # Devide by wheel radius since commands are for angular wheel speed (rad/s)
            velb = Float64(speed_b * speed / ROBOT_WHEEL_RADIUS)
            velr = Float64(speed_r * speed / ROBOT_WHEEL_RADIUS)
            
            print("Published commands: ", vell, velb, velr)
            publ.publish(vell)
            pubb.publish(velb)
            pubr.publish(velr)

            # Publish twist message
            twist_message = Twist(Vector3(vx * speed, vy * speed, 0), Vector3(0, 0, -vth * speed / ROBOT_RADIUS)) # Divide by robot radius since vth so far was linear speed
            pub_twist.publish(twist_message)

    except Exception as e:
        print(e)

    finally:
        vell = Float64()
        velb = Float64()
        velr = Float64()
        
        vell = 0.0
        velb = 0.0
        velr = 0.0

        pubb.publish(vell)
        publ.publish(velb)
        pubr.publish(velr)

        twist_message = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        pub_twist.publish(twist_message)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
