#! /usr/bin/env python


"""
In this script we subscribe on tf topic base_link <- odom <- map transformations provided by EKF + AMCL 
and compare themn with ground truth localization data provided by publish_true_position.py script on base_pose_ground_truth topic
"""

import rospy
from nav_msgs.msg import Odometry
import tf
import numpy as np

GROUND_TRUTH_POSITION_TOPIC = "/base_pose_ground_truth"
ASSESING_RATE = 2 # Twice per second

ground_truth_odometry = None

def update_gt_pose(data):
    global ground_truth_odometry
    
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    rot_quat = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    theta = tf.transformations.euler_from_quaternion(rot_quat)[-1] # take only yaw
    ground_truth_odometry = (x, y, theta)

def normalize_angle(angle):
    if angle < -np.pi:
        return angle + 2 * np.pi
    return angle

rospy.init_node('localization_assessor')

tf_listener = tf.TransformListener() # Listen EKF + AMCL position estimations
rospy.Subscriber(GROUND_TRUTH_POSITION_TOPIC, Odometry, update_gt_pose) # Listen ground truth position

differences = [] # list of (difference_x, difference_y, difference_theta) tuples calculated ASSESING_RATE times in a second

r = rospy.Rate(ASSESING_RATE)
while not rospy.is_shutdown():
    try:
        print("\n\n\n")
        """
        Formula for calculations of position resulting from two transformations is the same for both methods
        x* = x1 + x2 * cos(theta1) - y2 * sin(theta1)
        y* = y1 + x2 * sin(theta1) + y2 * cos(theta1)
        theta* = theta1 + theta2
        where x1, y1, theta1 are parameters of map -> odom transform transformation, x2, y2, theta2 are parameters of odom -> base_link transform transformation
        """
        # Calculate ground truth position
        if ground_truth_odometry is None:
            continue

        # print("Ground truth odometry is:", ground_truth_odometry)
        theta1 = -1.57 # -1.57 it's initial transform done by Gazebo when publishing odometry data
        x_gt = ground_truth_odometry[0] * np.cos(theta1) - ground_truth_odometry[1] * np.sin(theta1)
        y_gt = ground_truth_odometry[0] * np.sin(theta1) + ground_truth_odometry[1] * np.cos(theta1)
        theta_gt = theta1 + ground_truth_odometry[2] 
        theta_gt = normalize_angle(theta_gt)
        ground_truth_position = (x_gt, y_gt, theta_gt)
        print("Ground truth position is:", ground_truth_position)

        # Calculate estimated position
        try:
            (trans_map_odom, rot_map_odom) = tf_listener.lookupTransform('/map', '/odom', rospy.Time(0))
            rot_map_odom = tf.transformations.euler_from_quaternion(rot_map_odom)[-1] # take only yaw
            transform_map_odom = (trans_map_odom[0], trans_map_odom[1], rot_map_odom)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Failed to find base_link -> odom transform")
            continue

        try:
            (trans_odom_base, rot_odom_base) = tf_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            rot_odom_base = tf.transformations.euler_from_quaternion(rot_odom_base)[-1] # take only yaw
            transform_odom_base = (trans_odom_base[0], trans_odom_base[1], rot_odom_base)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Failed to find odom -> base_link transform")
            continue

        # print("map -> odom transform is:", transform_map_odom)
        # print("odom -> base_link transform is:", transform_odom_base)

        x_est = transform_map_odom[0] + transform_odom_base[0] * np.cos(transform_map_odom[2]) - transform_odom_base[1] * np.sin(transform_map_odom[2])
        y_est = transform_map_odom[1] + transform_odom_base[0] * np.sin(transform_map_odom[2]) + transform_odom_base[1] * np.cos(transform_map_odom[2])
        theta_est = transform_map_odom[2] + transform_odom_base[2]
        theta_est = normalize_angle(theta_est)
        estinated_position = (x_est, y_est, theta_est)
        print("Estimated position is:", estinated_position)

        # Collect differences
        difference = [np.abs(gt - est) for (gt, est) in zip(ground_truth_position, estinated_position)]
        differences.append(difference)
        print("Difference was: ", difference)
        r.sleep()

    except rospy.exceptions.ROSInterruptException:
        print("\nFinishing assesment")
        difference_x, difference_y, difference_angle = list(zip(*differences))
        difference_xy = difference_x + difference_y
        print(f"Mean error was: {np.mean(difference_xy)}, std: {np.std(difference_xy)} for (x, y) position")
        print(f"Mean error was: {np.mean(difference_angle)}, std: {np.std(difference_angle)} for orientation")