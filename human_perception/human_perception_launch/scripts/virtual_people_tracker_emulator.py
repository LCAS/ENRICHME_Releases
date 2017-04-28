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

rospy.init_node('virtual_people_tracker')

publisher = rospy.Publisher('/human/transformed', PoseStamped, queue_size=10)
rate=rospy.Rate(30)
counter=0
people_inside=False

while not rospy.is_shutdown():
    j = PoseStamped()
    j.header = Header()
    #j.header.stamp = {'secs' : datetime.datetime.now().time().second , 'nsecs' : datetime.datetime.now().time().microsecond}
    now = rospy.Time.now()
    j.header.stamp.secs = now.secs
    j.header.stamp.nsecs = now.nsecs
    j.header.frame_id = '/odom_combined'
    
    pose1 = Pose()
    pose1.position.x = 1
    pose1.position.y = 1

    #if (counter % 500)==0:
    #    if people_inside:
    #        people_inside=False
    #    else:
    #        people_inside=True

    people_inside=True
    if people_inside:        
        j.pose = pose1
    #print(j.header.seq)

    publisher.publish(j)
    counter = counter + 1
    rate.sleep()


rospy.spin()
