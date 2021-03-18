#!/usr/bin/env python

"""
This code does:
Converting keyboard control commands to Twist messages which will be used in both
1) Controling of each wheel motor by wheel_controller.py
2) Publishing tf transform base_link -> odom and odometry messages by odom_publisher.py 
"""

from __future__ import print_function

import rospy
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3
import tf
import sys, select, termios, tty

ROBOT_RADIUS = 0.07 # Based on +-0.04 rim shifts in main.urdf.xacro
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

# TWIST is represented as (vx, vy, vth)
MOVE_DIRECTION2TWIST = {
    "UP": (0, 1, 0),
    "LEFT": (-1, 0, 0),
    "RIGHT": (1, 0, 0),
    "DOWN": (0, -1, 0),
    "COUNTER_CLOCKWISE": (0, 0, -1),
    "CLOCKWISE": (0, 0, 1),
}

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

if __name__== "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_keyboard')
    pub_twist = rospy.Publisher('/open_base/twist', Twist, queue_size=1)

    speed = BASE_SPEED
    vx = vy = vth = 0 # Direction speeds

    try:
        while True:
            # Publish control commands
            print(help_info)
            print(print_speed(speed))

            key = getKey()
            if key in MOVE_BINDINGS.keys():
                move_direction = MOVE_BINDINGS[key]

                vx, vy, vth = MOVE_DIRECTION2TWIST[move_direction]
            elif key in SPEED_BINDINGS.keys():
                speed = speed * SPEED_BINDINGS[key]
            else: # Any other key => stop
                vx = vy = vth = 0
                if (key == '\x03'):
                    break

            # Publish twist message
            twist_message = Twist(Vector3(vx * speed, vy * speed, 0), Vector3(0, 0, -vth * speed / ROBOT_RADIUS)) # Divide by robot radius since vth so far was linear speed
            pub_twist.publish(twist_message)

    except Exception as e:
        print(e)

    finally:
        twist_message = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        pub_twist.publish(twist_message)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
