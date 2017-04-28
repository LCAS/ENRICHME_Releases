#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

global camInf_pub
global depth_pub
global rgb_pub

def imgcb(msg0):
    global camInf_pub
    global depth_pub

    msg=CameraInfo()
    msg.K= [366.37579345703125, 0.0 , 257.82269287109375, 0.0 , 366.37579345703125, 209.122802734375,0.0, 0.0, 1.0 ]
    msg.height = msg0.height
    msg.width = msg0.width

    #depthHeader = msg0.header
    #msg.header = depthHeader
    msg.header.stamp = rospy.Time.now()
    camInf_pub.publish(msg)

    msg0.header.stamp = rospy.Time.now()
    depth_pub.publish(msg0)

def rgbcb(msg0):
    global rgb_pub

    msg0.header.stamp = rospy.Time.now()
    rgb_pub.publish(msg0)

if __name__ == '__main__':
    rospy.init_node("cameraInfo")
    camInf_pub = rospy.Publisher('/camera/depth/camera_info', CameraInfo, queue_size=10)
    depth_pub = rospy.Publisher('/camera/depth/image_rect', Image, queue_size=10)
    rgb_pub = rospy.Publisher('/camera/rgb/image_rect_color', Image, queue_size=10)
    depth_sub = rospy.Subscriber('/head/kinect2/k2_depth/image', Image, imgcb)
    rgb_sub = rospy.Subscriber('/head/kinect2/k2_rgb/image', Image, rgbcb)
    rospy.spin()
