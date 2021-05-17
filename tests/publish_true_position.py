# #!/usr/bin/env python

# import rospy
# # from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
# from geometry_msgs.msg import PoseStamped
# from gazebo_msgs.srv import GetModelState
# from nav_msgs,msg import Odometry
# # import numpy as np
# # import matplotlib.pyplot as plotter
# import time

# POSITION_SERVICE_NAME = "/gazebo/get_model_state"
# POSITION_PUBLISH_TOPIC_NAME = "true_position"
# PUBLISH_RATE = 100
  
# def position_request():
#   print("Waiting for request")
#   rospy.wait_for_service(POSITION_SERVICE_NAME)
#   try:
#       print("Sending request")
#       get_position = rospy.ServiceProxy(POSITION_SERVICE_NAME, GetModelState)
#       position = get_position(model_name="open_base")
#       return position
#   except rospy.ServiceException as e:
#       print("Service call failed: %s"%e)

# def publish_position():
#   position_pub = rospy.Publisher(POSITION_PUBLISH_TOPIC_NAME, Odometry, queue_size=10)
#   rospy.init_node("true_position_publisher")
#   global position
  
#   # rospy.

#   # topic_name = "/move_base/TebLocalPlannerROS/teb_feedback" # define feedback topic here!
#   # rospy.Subscriber(topic_name, FeedbackMsg, feedback_callback, queue_size = 1) 

#   # rospy.loginfo("Visualizing velocity profile published on '%s'.",topic_name) 
#   # rospy.loginfo("Make sure to enable rosparam 'publish_feedback' in the teb_local_planner.")

#   # # two subplots sharing the same t axis
#   # fig, (ax_v, ax_omega) = plotter.subplots(2, sharex=True)
#   # plotter.ion()
#   # plotter.show()
  

#   r = rospy.Rate(PUBLISH_RATE) # define rate here
#   while not rospy.is_shutdown():
    

#     position = position_request()
#     print("Position is:\n", position)
        
#     pub.publish(hello_str)

#     r.sleep()

# if __name__ == '__main__': 
#   try:
#     publish_position()
#   except rospy.ROSInterruptException:
#     pass


#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import tf

PUBLISH_TF = True

rospy.init_node('true_position_publisher')

odom_pub = rospy.Publisher ('/base_pose_ground_truth', Odometry, queue_size=10)
if PUBLISH_TF:
    odom_broadcaster = tf.TransformBroadcaster()

rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom = Odometry()
header = Header()
header.frame_id = 'odom'
odom.child_frame_id = "base_link"

model = GetModelStateRequest()
model.model_name='open_base'

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

    odom.pose.pose = result.pose
    odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()
    odom.header = header

    odom_pub.publish (odom)

    r.sleep()