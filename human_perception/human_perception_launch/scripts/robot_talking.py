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
from sound_play.libsoundplay import SoundClient

import random

import json

import tf

import math
import statistics

rospy.init_node('robot_talking')


rate=rospy.Rate(5)

soundhandle = SoundClient()

speechs=["Hello. I am the enrich me robot.", "I will assist you in your daily life.","I can monitor your health status twenty four seven","I can remind the events in your schedule","Do you know about the article in The Times newspaper on Adele's new album?"]

while not rospy.is_shutdown():
    #global speech_paths
    idx = random.randint(0,len(speechs)-1)
    soundhandle.say(speechs[idx])
    rospy.sleep(7)


rospy.spin()
