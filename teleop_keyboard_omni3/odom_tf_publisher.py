#!/usr/bin/env python

"""
This code does:
1) Publishing of tf transform base_link -> odom with use of Twist messages published by teleop_keyboard_omni3.py
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
import tf


PUBLISH_RATE = 10 # 10 Hz
VX = VY = VTH = 0 # Current speeds # TODO think about making them not global

def update_odom(data):
    global VX, VY, VTH
    
    VX = data.linear.x
    VY = data.linear.y
    VTH = data.angular.z

    print(data, "\n New speeds are:", (VX, VY, VTH))

if __name__== "__main__":

    rospy.init_node('odom_tf_publisher')

    rospy.Subscriber("/open_base/twist", Twist, update_odom)
    odom_broadcaster = tf.TransformBroadcaster()

    # vx = vy = vth = 0 # Current speeds # Made global for now
    x = y = th = 0 # Current position
    current_time = last_time = rospy.Time.now()

    rate = rospy.Rate(PUBLISH_RATE)

    while not rospy.is_shutdown():
        # Publish odom transformation
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()

        delta_x = (VX * np.cos(th) - VY * np.sin(th)) * dt #/ ERROR_FACTOR_TRANSLATION
        delta_y = (VX * np.sin(th) + VY * np.cos(th)) * dt #/ ERROR_FACTOR_TRANSLATION
        delta_th = VTH * dt #/ ERROR_FACTOR_ROTATION

        x += delta_x
        y += delta_y
        th += delta_th

        print("New position is: ", (x, y, th))

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_link", # Maybe it's better to use "origin_link"?
            "odom"
        )

        last_time = current_time
        rate.sleep()
