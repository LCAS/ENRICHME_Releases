#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

height=0
face=0
shoulder=0
distance=0
volFace=0
volBreast=0
volBelly=0


rospy.init_node('reidentifier_print')
bridge = CvBridge()

def callback_img(img):
    global height
    global face
    global shoulder
    global distance
    global volFace
    global volBreast
    global volBelly

    cv_image = bridge.imgmsg_to_cv2(img, "rgb8")
    #cv2.putText(cv_image, "Reidentifier: " + 'H:{:0.2f}'.format(height) + ' S:{:0.2f}'.format(shoulder)+ ' F:{:0.2f}'.format(face) + ' D:{:0.2f}'.format(distance), (10, 450), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 0), 2)
    cv2.putText(cv_image, 'H:{:0.2f}'.format(height), (500, 360), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 0), 2)
    cv2.putText(cv_image, 'S:{:0.2f}'.format(shoulder), (500, 380), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 0), 2)
    cv2.putText(cv_image, 'F:{:0.2f}'.format(face) , (500, 400), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 0), 2)
    cv2.putText(cv_image, 'D:{:0.2f}'.format(distance), (500, 420), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 0), 2)
    cv2.putText(cv_image, 'F:{:0.2f}'.format(volFace), (500, 440), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 0), 2)
    cv2.putText(cv_image, 'Br:{:0.2f}'.format(volBreast), (500, 460), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 0), 2)
    cv2.putText(cv_image, 'Be:{:0.2f}'.format(volBelly) , (500, 480), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 0), 2)


    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "rgb8"))

def callback(msg):

    now = rospy.Time.now()
    global height
    global face
    global shoulder
    global distance
    global volFace
    global volBreast
    global volBelly

    if len(msg.data)>0:
        #print msg.data
        height = msg.data[0]
        shoulder = msg.data[1]
        face = msg.data[2]
        distance = msg.data[3]
        volFace = msg.data[4]
        volBreast = msg.data[5]
        volBelly = msg.data[6]

        

        
    
    


s = rospy.Subscriber('/reidentifier/features', Float32MultiArray, callback)

s = rospy.Subscriber('/reidentifier/imageDebug',Image, callback_img)

image_pub = rospy.Publisher("/reidentifier/imageFeatures", Image, queue_size=10)

rospy.spin()
