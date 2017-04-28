#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from requests import post
from bayes_people_tracker.msg import PeopleTracker
from visualization_msgs.msg import Marker
import actionlib
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from sound_play.libsoundplay import SoundClient

import json

import tf

import math
import random
import statistics
import numpy

rospy.init_node('robot_move')


client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

while not rospy.is_shutdown():

    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0.2
    goal.target_pose.pose.position.y = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    
    # #rospy.sleep(3)
   
    rospy.sleep(3)



rospy.spin()
