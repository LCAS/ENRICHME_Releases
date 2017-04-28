#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from requests import post
from bayes_people_tracker.msg import PeopleTracker
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA

import json

import tf

import math
import statistics

def people_cb(msg):
	global publisherGoal
	new_msg = PoseStamped()
	new_msg.header = msg.header
	new_msg.pose   = msg.poses[0]
        new_msg.pose.orientation.w = 1.0

	publisherGoal.publish(new_msg)

rospy.init_node('virtual_hbba')

publisherGoal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
subscriberPeople = rospy.Subscriber('/people_tracker/positions', PeopleTracker, people_cb)



rospy.sleep(1.0)
