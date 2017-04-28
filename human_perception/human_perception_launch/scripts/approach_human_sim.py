#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from requests import post
from bayes_people_tracker.msg import PeopleTracker
from visualization_msgs.msg import Marker
import actionlib
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from sound_play.libsoundplay import SoundClient
from mary_tts.msg import maryttsAction
from mary_tts.msg import maryttsActionGoal
from mary_tts.msg import maryttsGoal

import json

import tf

import math
import random
import statistics
import numpy

rospy.init_node('approach_human')

publisher = rospy.Publisher('/robot_pos', Marker,queue_size=10)
publisher2 = rospy.Publisher('/robot_pose', Marker,queue_size=10)
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

soundhandle = SoundClient()
speechClient = actionlib.SimpleActionClient('speak',maryttsAction)

speechs=["Hello. I am the enrich me robot.", "I will assist you in your daily life.","I can monitor your health status twenty four seven","I can remind the events in your schedule","Do you know about the article in The Times newspaper on Adele's new album?"]

xpos_array=[]
ypos_array=[]
cur_robposx=0
cur_robposy=0
prev_nsec_peopletracker=0
prev_sec_peopletracker=0

trajs = numpy.array(range(200)).reshape(10,2,10)
records = numpy.zeros((1,10))
cur_msg=[]

randomwalk_space=1

buffer_full=False
processed_prs=[]

robot_busy=False

def do_randomwalk():
    global randomwalk_space
    x_pos = random.randint(-randomwalk_space,randomwalk_space)
    y_pos = random.randint(-randomwalk_space,randomwalk_space)
    angle = random.randint(-5,5)
    print((x_pos,y_pos,angle*36))
    new_quaternion = tf.transformations.quaternion_from_euler(0,0,angle*36)

    #client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x_pos
    goal.target_pose.pose.position.y = y_pos
    goal.target_pose.pose.orientation.x = new_quaternion[0]
    goal.target_pose.pose.orientation.y = new_quaternion[1]
    goal.target_pose.pose.orientation.z = new_quaternion[2]
    goal.target_pose.pose.orientation.w = new_quaternion[3]

    global robot_busy
    robot_busy=True
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(7.0))
    rospy.sleep(3)
    robot_busy=False

def callback_odom(msg):
    j = Pose()
    j = msg.pose.pose
    global cur_robposx
    cur_robposx = j.position.x
    global cur_robposy
    cur_robposy = j.position.y

def callback(msg):
    
    j = Pose()
    global prev_nsec_peopletracker
    global cur_msg
    cur_msg = msg
    # print('seq: %d, secs: %d, nsecs: %d' % (msg.header.seq,msg.header.stamp.secs,msg.header.stamp.nsecs))
    # print('seq: %d' % msg.header.seq)
    if len(msg.poses)>0:
    
        
        #print(msg.header.seq,msg.header.stamp.nsecs,prev_nsec_peopletracker,msg.header.stamp.nsecs-prev_nsec_peopletracker)
        # print( msg.header.stamp.nsecs, prev_nsec_peopletracker, msg.header.stamp.secs, prev_sec_peopletracker)
        if math.fabs(msg.header.stamp.secs - prev_sec_peopletracker) >= 0:
            prev_sec_peopletracker = msg.header.stamp.secs
            if math.fabs(msg.header.stamp.nsecs-prev_nsec_peopletracker)>=180000000:
                prev_nsec_peopletracker = msg.header.stamp.nsecs

                # print('seq: %d - Tracking %d person(s)...' % (msg.header.seq,len(msg.poses)))
                print('%d.%d - Tracking %d person(s)...' % (msg.header.stamp.secs, msg.header.stamp.nsecs / 1000000, len(msg.poses)))
                # global cur_msg
                # cur_msg=msg
                for prs in range(0,len(msg.poses)):
                    j = msg.poses[prs]
                    xpos = j.position.x
                    ypos = j.position.y
                    global trajs
                    global records

                    #print(prs)
                    #print(records)
                    trajs[prs][0][records[0,prs]] = xpos
                    trajs[prs][1][records[0,prs]] = ypos

                    records[0,prs] = records[0,prs] + 1
                    # global buffer_full
                    # buffer_full=False

                    #xpos_array.append(xpos)

                    #ypos_array.append(ypos)


                    if records[0,prs]>9:
                        records[0,prs] = 0
                        # buffer_full=True

    else:
        global prev_sec_peopletracker
        global robot_busy
        if math.fabs(msg.header.stamp.secs-prev_sec_peopletracker)>=3:
            prev_sec_peopletracker = msg.header.stamp.secs
            if robot_busy==False:
                # print('seq: %d - Could not detect anybody... Keep looking...' % (msg.header.seq))
                print('%d.%d - Could not detect anybody... Keep looking...' % (msg.header.stamp.secs, msg.header.stamp.nsecs / 1000000))
                do_randomwalk()
                robot_busy=False
                rospy.sleep(3)

def approach(event):
    global trajs
    global records
    global cur_msg
    global processed_prs
    #print(len(records[0,:]))
    #print(records[0,:])
    print records

    for prs in range(0,len(records[0,:])):
        
        #print (prs,processed_prs)
        # if prs == processed_prs:
        #     continue
        # global buffer_full
        #print('records:')
        #print(records)
        print((prs,records[0,prs]))
        if records[0,prs]>=7:
            if len(cur_msg.poses) > prs:

                j = cur_msg.poses[prs]
                xpos = j.position.x
                ypos = j.position.y
                global xpos_array
                global ypos_array
                xpos_array = trajs[prs][0]
                ypos_array = trajs[prs][1]
                # print('xpos_array:')
                # print(xpos_array)
                # print('ypos_array:')
                # print(ypos_array)

                xpos_std = statistics.stdev(xpos_array)
                ypos_std = statistics.stdev(ypos_array)
                xpos_array=[]
                ypos_array=[]
                #records[0,prs] = 0
                # print('records:')
                # print(records)

                stopped=False
                if xpos_std<0.005 and ypos_std<0.005:
                    # print('seq: %d - Person %d stopped!' % (cur_msg.header.seq,prs))
                    print('%d.%d - Person %d stopped!' % (cur_msg.header.stamp.secs, cur_msg.header.stamp.nsecs / 10000000, prs))
                    stopped=True
                else:
                    # print('seq: %d - Person %d moving...' % (cur_msg.header.seq,prs))
                    print('%d.%d - Person %d moving...' % (cur_msg.header.stamp.secs, cur_msg.header.stamp.nsecs / 1000000, prs))
                    stopped=False

                #print((xpos_std,ypos_std))

                if stopped:
                    #global processed_prs
                    processed_prs = prs
                    quaternion = (j.orientation.x,j.orientation.y,j.orientation.z,j.orientation.w)
                    euler = tf.transformations.euler_from_quaternion(quaternion)
                    roll = euler[0]
                    pitch = euler[1]
                    yaw = euler[2]
                    #angle = math.pi-yaw

                    xdif = math.fabs(xpos-cur_robposx)
                    ydif = math.fabs(ypos-cur_robposy)
                    if xpos>cur_robposx:
                        if ypos>cur_robposy:
                            angle = math.atan(ydif/xdif)
                        else:
                            angle = (2*math.pi) - math.atan(ydif/xdif)
                    else:
                        if ypos>cur_robposy:
                            angle = math.pi-math.atan(ydif/xdif)
                        else:
                            angle = math.pi + math.atan(ydif/xdif)


                    #print((xpos,ypos,cur_robposx,cur_robposy,(angle/math.pi)*180))
                    print((prs,xpos,ypos))

                    xd = math.fabs(math.cos(angle)*0.65)
                    yd = math.fabs(math.sin(angle)*0.65)


                    #print(((angle/math.pi)*180, xpos, ypos, xd, yd))
                    if angle>=0 and angle<math.pi:
                        yrob = ypos - yd
                        if angle<=math.pi/2:
                            xrob = xpos - xd
                        else:
                            xrob = xpos + xd
                    else:
                        yrob = ypos + yd
                        if angle<=(3*math.pi/2):
                            xrob = xpos + xd
                        else:
                            xrob = xpos - xd

                    marker = Marker()
                    marker.header = cur_msg.header
                    marker.ns = 'ROBOT'
                    marker.id = 0
                    marker.type = marker.SPHERE
                    marker.action = marker.ADD
                    marker.lifetime.secs = 0
                    marker.lifetime.nsecs = 500000000
                    marker.scale.x = 0.2
                    marker.scale.y = 0.2
                    marker.scale.z = 0.2
                    marker.color.a = 1.0
                    marker.color.b = 1.0
                    marker.color.r = 1.0
                    marker.pose.orientation.w = 1.0
                    marker.pose.position.x = xrob
                    marker.pose.position.y = yrob
                    marker.pose.position.z = 0
                    publisher.publish(marker)


                    new_angle = angle#-math.pi
                    #print((angle/math.pi)*180, xpos, ypos, xrob, yrob,(new_angle/math.pi)*180)

                    marker2 = Marker()
                    marker2.header = cur_msg.header
                    marker2.ns = 'ROBOT'
                    marker2.id = 0
                    marker2.type = marker2.ARROW
                    marker2.action = marker2.ADD
                    marker2.lifetime.secs = 0
                    marker2.lifetime.nsecs = 500000000
                    marker2.scale.x = 1.0
                    marker2.scale.y = 0.1
                    marker2.scale.z = 0.1
                    marker2.color.a = 1.0
                    marker2.color.b = 0.5
                    marker2.color.r = 1.0

                    new_quaternion = tf.transformations.quaternion_from_euler(0,0,new_angle)
                    marker2.pose.orientation.x = new_quaternion[0]
                    marker2.pose.orientation.y = new_quaternion[1]
                    marker2.pose.orientation.z = new_quaternion[2]
                    marker2.pose.orientation.w = new_quaternion[3]

                    marker2.pose.position.x = xrob
                    marker2.pose.position.y = yrob
                    marker2.pose.position.z = 0
                    publisher2.publish(marker2)

                    #print(rospy.Duration())


                    client.wait_for_server()
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "odom"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = xrob
                    goal.target_pose.pose.position.y = yrob
                    goal.target_pose.pose.orientation.x = new_quaternion[0]
                    goal.target_pose.pose.orientation.y = new_quaternion[1]
                    goal.target_pose.pose.orientation.z = new_quaternion[2]
                    goal.target_pose.pose.orientation.w = new_quaternion[3]
                    client.send_goal(goal)
                    client.wait_for_result(rospy.Duration.from_sec(5.0))

                    # #rospy.sleep(3)
                    idx = random.randint(0,len(speechs)-1)
                    #soundhandle.say(speechs[idx])
                    #rospy.sleep(3)
                    # speechClient.wait_for_server()
                    #speechGoal = maryttsActionGoal()
                    #speechGoal.header.frame_id = ''
                    #speechGoal.header.stamp = rospy.Time.now()
                    # sGoal = maryttsGoal()
                    # sGoal.text = speechs[idx]
                    #print(sGoal)
                    #speechGoal.goal = sGoal
                    #speechClient.send_goal(sGoal)
                    # speechClient.wait_for_result(rospy.Duration.from_sec(3.0))
                    rospy.sleep(3)
                    do_randomwalk()
        
    # else:
    #     global prev_nsec_peopletracker
    #     prev_nsec_peopletracker = msg.header.stamp.nsecs


s = rospy.Subscriber('/people_tracker/positions', PeopleTracker, callback)

s2 = rospy.Subscriber('/odom', Odometry, callback_odom)

timer = rospy.Timer(rospy.Duration(0.25), approach)


rospy.spin()
