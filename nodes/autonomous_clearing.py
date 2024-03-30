#!/usr/bin/env python3

import rospy
import csv
import rospkg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import time
from geometry_msgs.msg import PoseStamped, Quaternion, Point

def read_csv_file(csv_file):
    goals = []
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        next(reader)
        for row in reader:
            x, y, theta = map(float, row)
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position = Point(x, y, 0.0)
            goal.pose.orientation = Quaternion(0.0, 0.0, theta, 1.0)
            goals.append(goal)
            print(goals)
    return goals

def move_base_client():
    rospy.init_node('autonomous_clearing')
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('snow_removal')
    csv_file = package_path + '/pose_data.csv'  # Path to your CSV file
    goals = read_csv_file(csv_file)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
    for goal in goals:
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose = goal
        client.send_goal(move_base_goal)
        client.wait_for_result()

if __name__ == '__main__':
    try:
        move_base_client()
    except rospy.ROSInterruptException:
        pass
