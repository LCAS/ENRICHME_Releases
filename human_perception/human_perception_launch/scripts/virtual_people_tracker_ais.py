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

from nav_msgs.msg import Odometry

import json

import tf

import math
import statistics

class VirtualPeopleTrackerAIS:
    def __init__(self):
        self.publisher = rospy.Publisher('/people_tracker/positions', PeopleTracker, queue_size=10)
        self.publisher_marker = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.publisher_marray = rospy.Publisher('/people_tracker/marker_array', MarkerArray, queue_size=10)

        self.sub_odom = rospy.Subscriber("/person/odom", Odometry, self.odom_cb)

        self.counter=0
        self.people_inside=False

    def odom_cb(self, msg):
        j = PeopleTracker()
        j.header = msg.header
        pose1 = msg.pose.pose

        pose2 = Pose()
        pose2.position.x = 1
        pose2.position.y = -1

        markerPeople = Marker()
        markerPeople.header = msg.header
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

        self.publisher.publish(j)
        self.publisher_marker.publish(markerPeople)
        self.publisher_marray.publish(markerArrayPeople)
        self.counter = self.counter + 1

if __name__ == "__main__":
    rospy.init_node('virtual_people_tracker')
    node = VirtualPeopleTrackerAIS()

    rospy.spin()
