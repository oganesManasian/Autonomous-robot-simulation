#!/usr/bin/env python

"""
This code does:
1) Publishing of tf transform base_link -> odom with use of Twist messages published by teleop_keyboard_omni3.py
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
import tf

# Constants
PUBLISH_RATE = 50 # 10 Hz
TRANSLATION_POSITION_VARIANCE = 1
ROTATION_POSITION_VARIANCE = 1 # yaw
TRANSLATION_VELOCITY_VARIANCE = 0.5
ROTATION_VELOCITY_VARIANCE = 1

# odom -> base_link will be published by EKF (robot_localization node) => false
PUBLISH_TF = False # odom -> base_link 

# Global variable
VX = VY = VTH = 0 # Current speeds # TODO think about making them not global

def update_odom(data):
    global VX, VY, VTH
    
    VX = data.linear.x
    VY = data.linear.y
    VTH = data.angular.z

    print(data, "\n New speeds are:", (VX, VY, VTH))

if __name__== "__main__":
    rospy.init_node('odom_publisher')

    rospy.Subscriber("cmd_vel", Twist, update_odom)
    odom_broadcaster = tf.TransformBroadcaster()
    odom_publisher = rospy.Publisher("odom", Odometry, queue_size=50)

    # vx = vy = vth = 0 # Current speeds # Made global for now
    x = y = th = 0 # Current position
    current_time = last_time = rospy.Time.now()

    rate = rospy.Rate(PUBLISH_RATE)

    while not rospy.is_shutdown():
        # Publish odom transformation
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()

        delta_x = (VX * np.cos(th) - VY * np.sin(th)) * dt
        delta_y = (VX * np.sin(th) + VY * np.cos(th)) * dt
        delta_th = VTH * dt

        x += delta_x
        y += delta_y
        th += delta_th

        print("New position is: ", (x, y, th))

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        if PUBLISH_TF:
            odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

        # Next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
        odom.pose.covariance = [
            TRANSLATION_POSITION_VARIANCE, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, TRANSLATION_POSITION_VARIANCE, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, ROTATION_POSITION_VARIANCE
        ]

        # set the velocity
        odom.twist.twist = Twist(Vector3(VX, VY, 0), Vector3(0, 0, VTH))
        odom.twist.covariance = [
            TRANSLATION_VELOCITY_VARIANCE, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, TRANSLATION_VELOCITY_VARIANCE, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, ROTATION_VELOCITY_VARIANCE
        ]
        # publish the message
        odom_publisher.publish(odom)

        last_time = current_time
        rate.sleep()
