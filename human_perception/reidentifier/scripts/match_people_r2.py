#!/usr/bin/env python  

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from requests import post
from bayes_people_tracker.msg import PeopleTracker
from upper_body_detector.msg import UpperBodyDetector
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy
import json

import tf

import math
import statistics

global K_kinect
global pix_kinect
global pix_kinect_array
global det_kinect
global det_kinect_array
global matchedIDs
global ridentify_result
global target_frame
target_frame = '/k2/depth_frame'
K_kinect = numpy.matrix('367.1581115722656, 0.0, 260.6588134765625; 0.0, 367.1581115722656, 209.61529541015625; 0.0, 0.0, 1.0')
pix_kinect = numpy.zeros([3, 1])
det_kinect = numpy.zeros([4, 1])
pix_kinect_array = []
det_kinect_array = []
matchedIDs = []
ridentify_result = []

def callback(msg):
    global K_kinect
    global pix_kinect
    global pix_kinect_array
    poses = PoseArray()
    poses_kinect = PoseArray()
    pixels_kinect = PoseArray()
    people_kinect = PeopleTracker()
    # print len(msg.poses)
    if len(msg.poses)>0:

        poses = msg.poses
        pix_kinect_array =[]

        for prs in range(0,len(msg.poses)):
            # print prs

            pose = PoseStamped()
            pose.pose = poses[prs]
            pose.header = Header()
            pose.header = msg.header
            pose_kinect = PoseStamped()
            #print poses[prs]

            # listener.waitForTransform('/odom_combined','/k2/depth_frame',rospy.Time.now(),rospy.Duration(0.1))
            try:
                pose_kinect = listener.transformPose(target_frame,pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            # print pose_kinect
            #poses_kinect.append(pose_kinect.poses)

            pt_kinect = numpy.zeros([3, 1])
            pt_kinect[0] = pose_kinect.pose.position.x
            pt_kinect[1] = -0.4
            pt_kinect[2] = pose_kinect.pose.position.z
            pt_kinect /= pt_kinect[2]

            pix_kinect = K_kinect*pt_kinect

            pix_kinect_array.append(pix_kinect)

            pix_kinect_pose = Pose()
            pix_kinect_pose.position.x = pix_kinect[0]
            pix_kinect_pose.position.y = pix_kinect[1]
            pix_kinect_pose.position.z = pix_kinect[2]
            # print pix_kinect_pose
            pixels_kinect.poses.append(pix_kinect_pose)


            # new_orientation = tf.transformations.euler_from_quaternion([pose_kinect.pose.orientation.x,pose_kinect.pose.orientation.y,pose_kinect.pose.orientation.z,pose_kinect.pose.orientation.w])

            
            # print (new_orientation[1]/math.pi)*180
            # print prs
            # print pose_kinect.pose.position

            # markerPeople = Marker()
            # markerPeople.header = Header()
            # markerPeople.header = msg.header
            # markerPeople.ns='people_tracker'
            # markerPeople.id=1
            # markerPeople.type=markerPeople.SPHERE
            # scalePeople = Vector3()
            # scalePeople.x=0.2
            # scalePeople.y=0.2
            # scalePeople.z=0.2
            # markerPeople.scale=scalePeople
            # colorPeople = ColorRGBA()
            # colorPeople.r=0.0
            # colorPeople.g=1.0
            # colorPeople.b=1.0
            # colorPeople.a=1.0
            # markerPeople.color = colorPeople
            # markerPeople.lifetime.secs=1
            # markerPeople.lifetime.nsecs=0
            # markerArrayPeople = MarkerArray()
            # markerPeople.pose=pose_kinect.pose
            # markerArrayPeople.markers.append(markerPeople)

    # people_kinect.header = Header()
    # people_kinect.header = msg.header
    # people_kinect.header.frame_id = '/k2/depth_frame'
    # people_kinect.poses = poses_kinect

    # people_kinect_msg.publish(people_kinect)    

    pixels_kinect.header = Header()
    pixels_kinect.header = msg.header
    pixels_kinect.header.frame_id = target_frame
    pixels_kinect_msg.publish(pixels_kinect) 


    #publisher_marray.publish(markerArrayPeople)


def callbackDet(msg):
    global det_kinect
    global det_kinect_array
    global pix_kinect
    global pix_kinect_array
    global matchedIDs
    

    # print len(msg.poses)
    if len(msg.pos_x)>0:
        
        det_kinect_array =[]

        matchedIDs = numpy.zeros([len(msg.pos_x), 1])
        for detNo in range(0,len(msg.pos_x)):
            # print prs
            det_kinect[0] = msg.pos_x[detNo]
            det_kinect[1] = msg.pos_y[detNo]
            det_kinect[2] = msg.width[detNo]
            det_kinect[3] = msg.height[detNo]

            detpix_kinect = numpy.zeros([3, 1])
            detpix_kinect[0] = msg.pos_x[detNo]
            detpix_kinect[1] = msg.pos_y[detNo]
            detpix_kinect[2] = 1.0

            #print poses[prs]

            det_kinect_array.append(det_kinect)

            matchedIDs[detNo] = -1

            minDist = 100000
            for prsNo in range(0,len(pix_kinect_array)):

            
                dif = detpix_kinect - pix_kinect_array[prsNo]
                dist = numpy.linalg.norm(dif)   
                # distToPositions.pushBack(dist);

                if (dist < minDist):
                    minDist = dist
                    matchedIDs[detNo] = prsNo



    matchedIDs_msg = Int32MultiArray()
    if len(matchedIDs)>0:
        matchedIDs_msg.data = matchedIDs
    matchedIDs_pub.publish(matchedIDs_msg)




def callbackImage(msg):

    # print type(msg)
    global det_kinect_array
    global pix_kinect_array
    cv_image = bridge.imgmsg_to_cv2(msg, "16UC1")

    img = numpy.empty((cv_image.shape[0],cv_image.shape[1],1),dtype = numpy.uint16)
    image = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

    cvuint8 = cv2.convertScaleAbs(image)
    # cv_image8u = cv2.

    if (len(pix_kinect_array)>0):
        for pNo in range(0,len(pix_kinect_array)):
            # print (pNo,int(round(pix_kinect_array[pNo][0])),pix_kinect_array[pNo][1])
            cv2.circle(cvuint8,(int(round(pix_kinect_array[pNo][0])),int(round(pix_kinect_array[pNo][1]))),5,(255,0,255),5)

    if (len(det_kinect_array)>0):
        for pNo in range(0,len(det_kinect_array)):
            # print (pNo,int(round(det_kinect_array[pNo][0])),det_kinect_array[pNo][1])
            cv2.rectangle(cvuint8, (det_kinect_array[pNo][0], det_kinect_array[pNo][1]), (det_kinect_array[pNo][0] + det_kinect_array[pNo][2] , det_kinect_array[pNo][1] + det_kinect_array[pNo][3]), (255, 0,255), 2)

    image_pub.publish(bridge.cv2_to_imgmsg(cvuint8,'rgb8'))

def callbackRI(msg):

    global ridentify_result
    global matchedIDs

    ridentify_result = msg.data
    recognizedID = -1
    if len(matchedIDs)>0:
        if msg.data>-1:
            recognizedID = matchedIDs[msg.data]

            
    recognizedID_msg = Int32()
    recognizedID_msg.data = recognizedID
    matched_pub.publish(recognizedID_msg)

def callbackCamInfo(msg):
    global K_kinect
    global target_frame
    K_kinect = numpy.matrix(msg.K).reshape((3,3))
    target_frame = msg.header.frame_id

if __name__ == '__main__':
    rospy.init_node('match_people')

    listener = tf.TransformListener()

    people_kinect_msg = rospy.Publisher('/people_tracker/positions_kinect', PeopleTracker,queue_size=10)
    pixels_kinect_msg = rospy.Publisher('/people_tracker/pixels_kinect', PoseArray,queue_size=10)
    publisher_marray = rospy.Publisher('/people_tracker/positions_kinect_array', MarkerArray, queue_size=10)
    people_msg = rospy.Subscriber('/people_tracker/positions', PeopleTracker,callback)
    ridentify_msg = rospy.Subscriber('/reidentifier/recognizedID', Int32,callbackRI)

    image_msg = rospy.Subscriber('/people_tracker/image_depth', Image,callbackImage)
    caminfo_msg = rospy.Subscriber('/people_tracker/camera_info_depth', CameraInfo,callbackCamInfo)
    image_pub = rospy.Publisher("/detections_kinect", Image, queue_size=10)

    det_msg = rospy.Subscriber('/upper_body_detector/detections', UpperBodyDetector,callbackDet)

    matchedIDs_pub = rospy.Publisher("/reidentifier/matchedIDs", Int32MultiArray, queue_size=10)
    matched_pub = rospy.Publisher("/reidentifier/result", Int32, queue_size=10)

    bridge = CvBridge()

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        #try:
        #    (trans,rot) = listener.lookupTransform('/odom_combined', '/k2/depth_frame', rospy.Time(0))
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #    continue


        
        #angular = 4 * math.atan2(trans[1], trans[0])
        #linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        


        rate.sleep()
