#!/usr/bin/env python

"""
This code does:
1) Publishing of tf transform base_link -> odom with use of Twist messages published by teleop_keyboard_omni3.py
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
import tf


# CURRENT_TIME = None
# LAST_TIME = None
# ODOM_BROADCASTER = tf.TransformBroadcaster()
# X = Y = TH = 0.0

# def publish_odom(data):
#     global CURRENT_TIME, LAST_TIME, ODOM_BROADCASTER, X, Y, TH

#     if CURRENT_TIME is None: # First initialization
#         CURRENT_TIME = LAST_TIME = rospy.Time.now()
#         return

#     CURRENT_TIME = rospy.Time.now()

#     vx = data.linear.x
#     vy = data.linear.y
#     vth = data.angular.z

#     print(CURRENT_TIME, LAST_TIME)
#     dt = (CURRENT_TIME - LAST_TIME).to_sec()
#     delta_x = (vx * np.cos(TH) - vy * np.sin(TH)) * dt
#     delta_y = (vx * np.sin(TH) + vy * np.cos(TH)) * dt
#     delta_th = vth * dt

#     X += delta_x
#     Y += delta_y
#     TH += delta_th
#     print(data, "\n New position is:", (X, Y, TH))

#     # since all odometry is 6DOF we'll need a quaternion created from yaw
#     odom_quat = tf.transformations.quaternion_from_euler(0, 0, TH)

#     ODOM_BROADCASTER.sendTransform(
#         (X, Y, 0.),
#         odom_quat,
#         CURRENT_TIME,
#         "base_link", # Maybe it's better to use "origin_link"?
#         "odom"
#     )

#     LAST_TIME = CURRENT_TIME

# def listen():
#     global CURRENT_TIME, LAST_TIME

#     rospy.init_node('odom_tf_publisher')
 
#     rospy.Subscriber("/open_base/twist", Twist, publish_odom)
 
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

# if __name__== "__main__":
#     listen()

PUBLISH_RATE = 10 # 10 Hz
VX = VY = VTH = 0 # Current speeds # TODO think about making them not global
ERROR_FACTOR = 75 # Experimentally checked difference between distance really traveled and distance teoretically travelled # TODO solve

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

        delta_x = (VX * np.cos(th) - VY * np.sin(th)) * dt / ERROR_FACTOR
        delta_y = (VX * np.sin(th) + VY * np.cos(th)) * dt / ERROR_FACTOR
        delta_th = VTH * dt / ERROR_FACTOR

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
