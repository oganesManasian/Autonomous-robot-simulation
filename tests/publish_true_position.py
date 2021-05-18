#! /usr/bin/env python


"""
In this script we request ground truth device position from GAzebo simulator and then publish Odometry on PUBLISHED_TOPIC_NAME topic and odom -> base_link transform on tf topic 
"""


import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import tf

# False - for assesing localization, True - for assesing path planners
PUBLISH_TF = False


DEVICE_NAME = 'open_base'
SERVICE_NAME = '/gazebo/get_model_state'
PUBLISHED_TOPIC_NAME = '/base_pose_ground_truth'

rospy.init_node('true_position_publisher')

odom_pub = rospy.Publisher (PUBLISHED_TOPIC_NAME, Odometry, queue_size=10)
if PUBLISH_TF:
    odom_broadcaster = tf.TransformBroadcaster()

rospy.wait_for_service (SERVICE_NAME)
get_model_srv = rospy.ServiceProxy(SERVICE_NAME, GetModelState)

# Prefill
odom = Odometry()
header = Header()
header.frame_id = 'odom'
odom.child_frame_id = "base_link"

model = GetModelStateRequest()
model.model_name = DEVICE_NAME

r = rospy.Rate(100)
while not rospy.is_shutdown():
    result = get_model_srv(model)

    if PUBLISH_TF:
        x = result.pose.position.x
        y = result.pose.position.y
        quat = (result.pose.orientation.x, result.pose.orientation.y, result.pose.orientation.z, result.pose.orientation.w)
        current_time = rospy.Time.now()

        odom_broadcaster.sendTransform(
            (x, y, 0.),
            quat,
            current_time,
            "base_link",
            "odom"
      )

    # publish on topic
    odom.pose.pose = result.pose
    odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()
    odom.header = header

    odom_pub.publish(odom)

    r.sleep()