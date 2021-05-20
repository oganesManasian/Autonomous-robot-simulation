#! /usr/bin/env python


"""
In this script we subscribe on tf topic base_link <- odom <- map transformations provided by EKF + AMCL 
and compare themn with ground truth localization data provided by publish_true_position.py script on base_pose_ground_truth topic
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import tf
import numpy as np
from time import perf_counter
import pickle
import os
import re

GROUND_TRUTH_ODOMETRY_TOPIC = "/base_pose_ground_truth"
NEW_GOAL_TOPIC = "/move_base_simple/goal"
GOAL_RESULT_TOPIC = "/move_base/status"
EXPERIMENT_RESULT_FOLDER = "experiments"
EXPERIMENT_FILENAME = "path_planning_data_"
EXPERIMENT_NAME = "rollout planner"

ground_truth_odometry = []
collect_data = False # Collect only when navigating to the goal (between "goal is sent" and "goal is reached" messages)
start_time = None
end_time = None
    
def sort_human(l):
    convert = lambda text: float(text) if text.isdigit() else text
    alphanum = lambda key: [convert(c) for c in re.split('([-+]?[0-9]*\.?[0-9]*)', key)]
    l.sort(key=alphanum)
    return l

def update_gt_odometry(data):
    if collect_data:
        global ground_truth_odometry
        
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        rot_quat = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(rot_quat)[-1] # take only yaw

        dx = data.twist.twist.linear.x
        dy = data.twist.twist.linear.y
        dtheta = data.twist.twist.angular.z

        time = perf_counter()

        ground_truth_odometry.append((x, y, theta, dx, dy, dtheta, time))

        # print("Got message:", ground_truth_odometry[-1])

def update_goal_start_time(data):
    global start_time, collect_data, ground_truth_odometry
    
    start_time = perf_counter() 
    collect_data = True
    print("Updated start time:", start_time)

def update_goal_end_time(data):
    global start_time, end_time, collect_data
    
    if start_time is not None and data.status_list and data.status_list[-1].status == 3: # Last goal is reached
        end_time = perf_counter() 
        collect_data = False
        print("Updated end time:", end_time)

        # Process stats data
        # calculate_navigation_stats()
        save_data()

        # Clean results and prepare for next test 
        start_time = end_time = None
        ground_truth_odometry = []

def save_data():
    global start_time, end_time, ground_truth_odometry
    data = {"start_time": start_time, "end_time": end_time, "odometry": ground_truth_odometry, "experiment_name": EXPERIMENT_NAME}

    if os.path.isdir(EXPERIMENT_RESULT_FOLDER):
        existing_experiments = os.listdir(EXPERIMENT_RESULT_FOLDER)
        if existing_experiments:
            last_experiment_filename = sort_human(existing_experiments)[-1]
            counter = int(last_experiment_filename.split(".")[0].split("_")[-1]) + 1
        else:
            counter = 1

    else:
        os.mkdir(EXPERIMENT_RESULT_FOLDER)
        counter = 1

    filename = EXPERIMENT_FILENAME + str(counter) + ".pickle"

    with open(os.path.join(EXPERIMENT_RESULT_FOLDER, filename), "wb") as f:
        pickle.dump(data, f) 
        print("Saved data to", filename)

def calculate_navigation_stats():
    global start_time, end_time, ground_truth_odometry
    print("Calculating navigation statistics")

    x, y, theta, dx, dy, dtheta, time = list(zip(*ground_truth_odometry))
    print("Mean linear speed was:", np.mean(dx))
    print("Mean angular speed was was:", np.mean(dtheta))
    spent_time = end_time - start_time
    print("Spent time on navigation", spent_time) # Get from difference in "set goal" command and "goal reached" feedback


def listener():
    rospy.init_node('local_path_planning_assessor')

    rospy.Subscriber(GROUND_TRUTH_ODOMETRY_TOPIC, Odometry, update_gt_odometry) # Listen ground truth odometry
    rospy.Subscriber(NEW_GOAL_TOPIC, PoseStamped, update_goal_start_time) # Listen when new goal position will be published
    rospy.Subscriber(GOAL_RESULT_TOPIC, GoalStatusArray, update_goal_end_time) # Listen when the goal position is reached


    rospy.spin()


if __name__ == '__main__':
    listener()
