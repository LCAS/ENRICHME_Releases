#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from requests import post
from bayes_people_tracker.msg import PeopleTracker
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

import json

import tf

import math
import statistics

rospy.init_node('virtual_odom')

publisher = rospy.Publisher('/odom', Odometry,queue_size=10)
rate=rospy.Rate(5)


while not rospy.is_shutdown():
    j = Odometry()
    j.header = Header()
    #j.header.stamp = {'secs' : datetime.datetime.now().time().second , 'nsecs' : datetime.datetime.now().time().microsecond}
    now = rospy.Time.now()
    j.header.stamp.secs = now.secs
    j.header.stamp.nsecs = now.nsecs
    j.header.frame_id = '/odom_combined'
    
    pose1 = Pose()
    pose1.position.x = 0
    pose1.position.y = 0

    
    j.pose.pose = pose1
    
    publisher.publish(j)


rospy.spin()
