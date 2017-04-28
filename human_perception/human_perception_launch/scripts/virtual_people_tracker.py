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

publisher = rospy.Publisher('/people_tracker/positions', PeopleTracker, queue_size=10)
publisher_marker = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
publisher_marray = rospy.Publisher('/people_tracker/marker_array', MarkerArray, queue_size=10)
rate=rospy.Rate(30)
counter=0
people_inside=False

while not rospy.is_shutdown():
    j = PeopleTracker()
    j.header = Header()
    #j.header.stamp = {'secs' : datetime.datetime.now().time().second , 'nsecs' : datetime.datetime.now().time().microsecond}
    now = rospy.Time.now()
    j.header.stamp.secs = now.secs
    j.header.stamp.nsecs = now.nsecs
    j.header.frame_id = '/odom'
    
    pose1 = Pose()
    pose1.position.x = 0
    pose1.position.y = 5
    pose2 = Pose()
    pose2.position.x = 1
    pose2.position.y = -1

    markerPeople = Marker()
    markerPeople.header = Header()
    markerPeople.header.stamp.secs = now.secs
    markerPeople.header.stamp.nsecs = now.nsecs
    markerPeople.header.frame_id = '/odom'
    markerPeople.ns='people_tracker'
    markerPeople.id=1
    markerPeople.type=markerPeople.SPHERE
    scalePeople = Vector3()
    scalePeople.x=0.2
    scalePeople.y=0.2
    scalePeople.z=0.2
    markerPeople.scale=scalePeople
    colorPeople = ColorRGBA()
    colorPeople.r=0.0
    colorPeople.g=1.0
    colorPeople.b=0.0
    colorPeople.a=1.0
    markerPeople.color = colorPeople
    markerPeople.lifetime.secs=1
    markerPeople.lifetime.nsecs=0

    markerArrayPeople = MarkerArray()
    
    
#    if (counter % 500)==0:
#        if people_inside:
#            people_inside=False
#        else:
    people_inside=True

    if people_inside:        
        j.poses.append(pose1)
 #       j.poses.append(pose2)
        markerPeople.pose=pose1
        markerArrayPeople.markers.append(markerPeople)
    #print(j.header.seq)

    publisher.publish(j)
    publisher_marker.publish(markerPeople)
    publisher_marray.publish(markerArrayPeople)
    counter = counter + 1
    rate.sleep()


rospy.spin()
