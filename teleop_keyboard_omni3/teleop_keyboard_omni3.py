#!/usr/bin/env python

from __future__ import print_function

import rospy

from std_msgs.msg import Float64

import sys, select, termios, tty

help_info = """
Reading from the keyboard !
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >


anything else : stop

q/z : increase/decrease max speeds by 10%

CTRL-C to quit
"""

MOVE_BINDINGS = {
        'i':(-1,0,1),
        'o':(-1,1,0),
        'j':(1,1,1),
        'l':(-1,-1,-1),
        'u':(-1,0,1),
        ',':(1,0,-1),
        '.':(0,1,-1),
        'm':(1,-1,0),  

        'I':(-1,0,1),
        'O':(-1,1,0),
        'J':(1,-2,1),
        'L':(-1,2,-1),
        'U':(0,-1,1),
        '<':(1,0,-1),
        '>':(0,1,-1),
        'M':(1,-1,0),  
    }

SPEED_BINDINGS={
        'q':1.1,
        'z':.9,
    }

PUBLISH_RATE = 2

def getKey():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_speed(speed):
    return "Currently:\tspeed %s " % (speed)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('vel_Publisher')
    publ = rospy.Publisher('/open_base/left_joint_velocity_controller/command', Float64, queue_size=1)
    pubb = rospy.Publisher('/open_base/back_joint_velocity_controller/command', Float64, queue_size=1)
    pubr = rospy.Publisher('/open_base/right_joint_velocity_controller/command', Float64, queue_size=1)


    speed = 1.0
    rate = rospy.Rate(PUBLISH_RATE)

    try:
        while True:
            print(help_info)
            print(print_speed(speed))

            key = getKey()
            if key in MOVE_BINDINGS.keys():
                x = MOVE_BINDINGS[key][0]
                y = MOVE_BINDINGS[key][1]
                z = MOVE_BINDINGS[key][2]
            elif key in SPEED_BINDINGS.keys():
                speed = speed * SPEED_BINDINGS[key]
            else: # Any other key => stop
                x = 0
                y = 0
                z = 0
                if (key == '\x03'):
                    break
            
            vell = Float64()
            velb = Float64()
            velr = Float64()
            
            vell = x * speed
            velb = y * speed
            velr = z * speed
            
            publ.publish(vell)
            pubb.publish(velb)
            pubr.publish(velr)

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

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
