#!/usr/bin/env python
import rospy
import math
from bayes_people_tracker.msg import PeopleTracker
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from st_anomaly_detector.msg import StringArray
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import statistics
import numpy


rospy.init_node('auto_suggest')

def callback(msg):
    #print msg
    if len(msg.data)>0:
        rospy.loginfo("Anomaly Detected! Triggering Suggest Activity...")
        pub_msg = Empty()
        #suggest_pub.publish(pub_msg)
    else:
        rospy.loginfo("No Anomaly Detected!")


s = rospy.Subscriber('/lt_anomaly', String, callback)

suggest_pub = rospy.Publisher('/suggest_activity', Empty, queue_size=10)

rospy.spin()
