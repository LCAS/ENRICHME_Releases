#!/usr/bin/env python
import rospy
import math
from bayes_people_tracker.msg import PeopleTracker
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from st_anomaly_detector.msg import StringArray
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import statistics
import numpy

prev_x=[1,2,3,4,5,6,7,8,9,10]
prev_y=[1,2,3,4,5,6,7,8,9,10]
start=[1,2,3,4,5,6,7,8,9,10]
status=["stopped","stopped","stopped","stopped","stopped","stopped","stopped","stopped","stopped","stopped"]
speed=[1,2,3,4,5,6,7,8,9,10]
prev_stopped_time=[1,2,3,4,5,6,7,8,9,10]
prev_walking_time=[1,2,3,4,5,6,7,8,9,10]
prev_running_time=[1,2,3,4,5,6,7,8,9,10]
anomalyStatus=["NaN","NaN","NaN","NaN","NaN","NaN","NaN","NaN","NaN","NaN"]

Headprev_x=[1,2,3,4,5,6,7,8,9,10]
Headprev_y=[1,2,3,4,5,6,7,8,9,10]
Headstart=[1,2,3,4,5,6,7,8,9,10]
Headstatus=["stopped","stopped","stopped","stopped","stopped","stopped","stopped","stopped","stopped","stopped"]
Headspeed=[1,2,3,4,5,6,7,8,9,10]
Headavgspeed=[1,2,3,4,5,6,7,8,9,10]
Headprev_stopped_time=[1,2,3,4,5,6,7,8,9,10]
Headprev_moving_time=[1,2,3,4,5,6,7,8,9,10]
HeadanomalyStatus=["NaN","NaN","NaN","NaN","NaN","NaN","NaN","NaN","NaN","NaN"]
HeadspeedAll = numpy.zeros((10,30))
HeadcounterAvg=0

SHLcur_x=[1,2,3,4,5,6,7,8,9,10]
SHLcur_y=[1,2,3,4,5,6,7,8,9,10]
SHLprev_x=[1,2,3,4,5,6,7,8,9,10]
SHLprev_y=[1,2,3,4,5,6,7,8,9,10]
SHLspeed=[1,2,3,4,5,6,7,8,9,10]
SHLavgspeed=[1,2,3,4,5,6,7,8,9,10]
SHLspeedAll = numpy.zeros((10,30))
SHRcur_x=[1,2,3,4,5,6,7,8,9,10]
SHRcur_y=[1,2,3,4,5,6,7,8,9,10]
SHRprev_x=[1,2,3,4,5,6,7,8,9,10]
SHRprev_y=[1,2,3,4,5,6,7,8,9,10]
SHRspeed=[1,2,3,4,5,6,7,8,9,10]
SHRavgspeed=[1,2,3,4,5,6,7,8,9,10]
SHRspeedAll = numpy.zeros((10,30))
Torsostart=[1,2,3,4,5,6,7,8,9,10]
Torsoprev_stopped_time=[1,2,3,4,5,6,7,8,9,10]
Torsoprev_moving_time=[1,2,3,4,5,6,7,8,9,10]
Torsostatus=["stopped","stopped","stopped","stopped","stopped","stopped","stopped","stopped","stopped","stopped"]
TorsoanomalyStatus=["NaN","NaN","NaN","NaN","NaN","NaN","NaN","NaN","NaN","NaN"]
TorsocounterAvg=0

volHeadstd=[1,2,3,4,5,6,7,8,9,10]
volHeadAll = numpy.zeros((10,30))
volBreaststd=[1,2,3,4,5,6,7,8,9,10]
volBreastAll = numpy.zeros((10,30))
volBellystd=[1,2,3,4,5,6,7,8,9,10]
volBellyAll = numpy.zeros((10,30))
Volstart=[1,2,3,4,5,6,7,8,9,10]
Volprev_stopped_time=[1,2,3,4,5,6,7,8,9,10]
Volprev_moving_time=[1,2,3,4,5,6,7,8,9,10]
Volstatus=["stopped","stopped","stopped","stopped","stopped","stopped","stopped","stopped","stopped","stopped"]
VolanomalyStatus=["NaN","NaN","NaN","NaN","NaN","NaN","NaN","NaN","NaN","NaN"]
VolcounterAvg=0

numberOfPeople=0

rospy.init_node('st_anomaly_detector')
bridge = CvBridge()

def callback_img(img):
    global speed
    global status
    global anomalyStatus

    cv_image = bridge.imgmsg_to_cv2(img, "rgb8")
    cv2.putText(cv_image, "Activity Level Detector: ", (10, 20), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 2)
    for prs in range(0,numberOfPeople): 
        cv2.putText(cv_image, "P" + '{:2d}'.format(prs) + " Global: " + status[prs] + ", " + anomalyStatus[prs], (10, 40+(prs)*100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 200, 255), 2)
        cv2.putText(cv_image, "Head: " + Headstatus[prs] + ", " + HeadanomalyStatus[prs], (40, 60+(prs)*100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 200, 255), 2)
        cv2.putText(cv_image, "Torso: " + Torsostatus[prs] + ", " + TorsoanomalyStatus[prs], (40, 80+(prs)*100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 200, 255), 2)
        cv2.putText(cv_image, "Body: " + Volstatus[prs] + ", " + VolanomalyStatus[prs], (40, 100+(prs)*100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 200, 255), 2)
        cv2.putText(cv_image, "S:" + '{:05.2f}'.format(speed[prs]) + "," + '{:05.3f}'.format(Headavgspeed[prs]) + "," + '{:05.3f}'.format(SHLavgspeed[prs]) + "," + '{:05.3f}'.format(SHRavgspeed[prs]) + "," + '{:05.3f}'.format(volHeadstd[prs]) + "," + '{:05.3f}'.format(volBreaststd[prs]) + "," + '{:05.3f}'.format(volBellystd[prs]), (10, 450+(prs-1)*25), cv2.FONT_HERSHEY_PLAIN, 1, (0, 200, 255), 1)

    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "rgb8"))

def callbackPT(msg):
    #print type(t)
    pub_msg = StringArray()
    anomalypub_msg = StringArray()
    speed_msg = Float32MultiArray()
    #print msg.poses
    now = rospy.Time.now()
    global start
    global prev_x
    global prev_y
    global speed
    global status
    global prev_stopped_time
    global prev_walking_time
    global prev_running_time
    global anomalyStatus
    global numberOfPeople

    if len(msg.poses)>0:
        numberOfPeople = len(msg.poses)

        for prs in range(0,len(msg.poses)):

            if start[prs]==0:
                prev_x[prs] = msg.poses[prs].position.x
                prev_y[prs] = msg.poses[prs].position.y
                speed[prs] = 0
                start[prs]=1
            else:
                dif_x = abs(prev_x[prs] - msg.poses[prs].position.x)
                dif_y = abs(prev_y[prs] - msg.poses[prs].position.y)
                speed[prs] = math.sqrt((dif_x*dif_x) + (dif_y*dif_y))
                prev_x[prs] = msg.poses[prs].position.x
                prev_y[prs] = msg.poses[prs].position.y

            if (speed[prs]<0.02):
                status[prs] = "stopped"
                if prev_stopped_time[prs]>0:
                    dif = now.secs - prev_stopped_time[prs]
                    prev_walking_time[prs] = 0
                    prev_running_time[prs] = 0
                    if dif >=5:
                        anomalyStatus[prs] = "Not Active"
                else:
                    prev_stopped_time[prs] = now.secs
            elif (speed[prs]>0.02) & (speed[prs]<0.20):
                status[prs] = "walking"
                if prev_walking_time[prs]>0:
                    prev_stopped_time[prs] = 0
                    prev_running_time[prs] = 0
                    dif = now.secs - prev_walking_time[prs]
                    if dif >=5:
                        anomalyStatus[prs] = "Active"
                else:
                    prev_walking_time[prs] = now.secs
            elif (speed[prs]>0.20):
                status[prs] = "running"
                if prev_running_time[prs]>0:
                    prev_walking_time[prs] = 0
                    prev_stopped_time[prs] = 0
                    dif = now.secs - prev_running_time[prs]
                    if dif >=5:
                        anomalyStatus[prs] = "Highly Active"
                else:
                    prev_running_time[prs] = now.secs
            else:
                status[prs] = "NaN"
        #print (datetime.datetime.fromtimestamp(t.secs).strftime("%Y-%m-%d %H:%M:%S"),t.nsecs,speed,status)
        #print (speed,status)
        pub_msg.data = status
        speed_msg.data = speed
        status_pub.publish(pub_msg)
        speed_pub.publish(speed_msg)
        anomalypub_msg.data = anomalyStatus
        anomaly_pub.publish(anomalypub_msg)


def callbackH(msg):
    #print type(t)
    pub_msg = StringArray()
    anomalypub_msg = StringArray()
    #print msg.poses
    now = rospy.Time.now()
    global Headstart
    global Headprev_x
    global Headprev_y
    global Headspeed
    global Headavgspeed
    global Headstatus    
    global Headprev_stopped_time
    global Headprev_moving_time    
    global HeadanomalyStatus
    global numberOfPeople
    global HeadspeedAll
    global HeadcounterAvg

    if len(msg.poses)>0:
        numberOfPeople = len(msg.poses)

        for prs in range(0,len(msg.poses)):

            if Headstart[prs]==0:
                Headprev_x[prs] = msg.poses[prs].position.x
                Headprev_y[prs] = msg.poses[prs].position.y
                Headspeed[prs] = 0
                Headstart[prs]=1
            else:
                dif_x = abs(Headprev_x[prs] - msg.poses[prs].position.x)
                dif_y = abs(Headprev_y[prs] - msg.poses[prs].position.y)
                Headspeed[prs] = math.sqrt((dif_x*dif_x) + (dif_y*dif_y))
                HeadspeedAll[prs][HeadcounterAvg]=Headspeed[prs]
                HeadcounterAvg+=1
                Headprev_x[prs] = msg.poses[prs].position.x
                Headprev_y[prs] = msg.poses[prs].position.y

            if HeadcounterAvg==30:                
                HeadcounterAvg=0
            Headavgspeed[prs] = statistics.mean(HeadspeedAll[prs])

            if (Headavgspeed[prs]<0.02):
                Headstatus[prs] = "stopped"
                if Headprev_stopped_time[prs]>0:
                    dif = now.secs - Headprev_stopped_time[prs]
                    Headprev_moving_time[prs] = 0
                    if dif >=5:
                        HeadanomalyStatus[prs] = "Not Active"
                else:
                    Headprev_stopped_time[prs] = now.secs
            elif (Headavgspeed[prs]>0.02 ):
                Headstatus[prs] = "moving"
                if Headprev_moving_time[prs]>0:
                    Headprev_stopped_time[prs] = 0                    
                    dif = now.secs - Headprev_moving_time[prs]
                    if dif >=5:
                        HeadanomalyStatus[prs] = "Active"
                else:
                    Headprev_moving_time[prs] = now.secs
            else:
                Headstatus[prs] = "NaN"
        #print (datetime.datetime.fromtimestamp(t.secs).strftime("%Y-%m-%d %H:%M:%S"),t.nsecs,speed,status)
        #print (speed,status)
        pub_msg.data = Headstatus
        statusHead_pub.publish(pub_msg)
        anomalypub_msg.data = HeadanomalyStatus
        anomalyHead_pub.publish(anomalypub_msg)

def callbackSHR(msg):
    
    global SHRcur_x
    global SHRcur_y    

    if len(msg.poses)>0:
        for prs in range(0,len(msg.poses)):
            SHRcur_x[prs] = msg.poses[prs].position.x
            SHRcur_y[prs] = msg.poses[prs].position.y

def callbackSHL(msg):
    
    global SHLcur_x
    global SHLcur_y    

    if len(msg.poses)>0:
        for prs in range(0,len(msg.poses)):
            SHLcur_x[prs] = msg.poses[prs].position.x
            SHLcur_y[prs] = msg.poses[prs].position.y

def callbackT(msg):
    #print type(t)
    pub_msg = StringArray()
    anomalypub_msg = StringArray()
    #print msg.poses
    now = rospy.Time.now()    
    global SHRcur_x
    global SHRcur_y  
    global SHRprev_x
    global SHRprev_y    
    global SHRspeed
    global SHRavgspeed
    global SHRspeedAll
    global SHLcur_x
    global SHLcur_y  
    global SHLprev_x
    global SHLprev_y    
    global SHLspeed
    global SHLavgspeed
    global SHLspeedAll
    global Torsostart
    global Torsostatus 
    global TorsoanomalyStatus        
    global Torsoprev_stopped_time
    global Torsoprev_moving_time        
    global numberOfPeople
    global TorsocounterAvg

    if numberOfPeople>0:

        for prs in range(0,numberOfPeople):

            if Torsostart[prs]==0:
                SHRprev_x[prs] = SHRcur_x[prs]
                SHRprev_y[prs] = SHRcur_y[prs]
                SHRspeed[prs] = 0
                SHLprev_x[prs] = SHLcur_x[prs]
                SHLprev_y[prs] = SHLcur_y[prs]
                SHLspeed[prs] = 0
                Torsostart[prs]=1
            else:
                SHRdif_x = abs(SHRprev_x[prs] - SHRcur_x[prs])
                SHRdif_y = abs(SHRprev_y[prs] - SHRcur_y[prs])
                SHLdif_x = abs(SHLprev_x[prs] - SHLcur_x[prs])
                SHLdif_y = abs(SHLprev_y[prs] - SHLcur_y[prs])
                SHRspeed[prs] = math.sqrt((SHRdif_x*SHRdif_x) + (SHRdif_y*SHRdif_y))
                SHLspeed[prs] = math.sqrt((SHLdif_x*SHLdif_x) + (SHLdif_y*SHLdif_y))
                SHRspeedAll[prs][TorsocounterAvg]=SHRspeed[prs]
                SHLspeedAll[prs][TorsocounterAvg]=SHLspeed[prs]
                TorsocounterAvg+=1
                SHRprev_x[prs] = SHRcur_x[prs]
                SHRprev_y[prs] = SHRcur_y[prs]
                SHLprev_x[prs] = SHLcur_x[prs]
                SHLprev_y[prs] = SHLcur_y[prs]

            if TorsocounterAvg==30:                
                TorsocounterAvg=0
            SHRavgspeed[prs] = statistics.mean(SHRspeedAll[prs])
            SHLavgspeed[prs] = statistics.mean(SHLspeedAll[prs])

            if ((SHRavgspeed[prs]<0.02) | (SHLavgspeed[prs]<0.02)):
                Torsostatus[prs] = "stopped"
                if Torsoprev_stopped_time[prs]>0:
                    dif = now.secs - Torsoprev_stopped_time[prs]
                    Torsoprev_moving_time[prs] = 0
                    if dif >=5:
                        TorsoanomalyStatus[prs] = "Not Active"
                else:
                    Torsoprev_stopped_time[prs] = now.secs
            elif ((SHRavgspeed[prs]>0.02) | (SHLavgspeed[prs]>0.02)):
                Torsostatus[prs] = "moving"
                if Torsoprev_moving_time[prs]>0:
                    Torsoprev_stopped_time[prs] = 0                    
                    dif = now.secs - Torsoprev_moving_time[prs]
                    if dif >=5:
                        TorsoanomalyStatus[prs] = "Active"
                else:
                    Torsoprev_moving_time[prs] = now.secs
            else:
                Torsostatus[prs] = "NaN"
        #print (datetime.datetime.fromtimestamp(t.secs).strftime("%Y-%m-%d %H:%M:%S"),t.nsecs,speed,status)
        #print (speed,status)
        pub_msg.data = Torsostatus
        statusTorso_pub.publish(pub_msg)
        anomalypub_msg.data = TorsoanomalyStatus
        anomalyTorso_pub.publish(anomalypub_msg)


def callbackV(msg):
    #print type(t)
    pub_msg = StringArray()
    anomalypub_msg = StringArray()
    #print msg.poses
    now = rospy.Time.now()
    global volHeadstd
    global volHeadAll
    global volBreaststd
    global volBreastAll
    global volBellystd
    global volBellyAll
    global Volstart
    global Volstatus 
    global VolanomalyStatus        
    global Volprev_stopped_time
    global Volprev_moving_time        
    global numberOfPeople
    global VolcounterAvg

    if ((numberOfPeople>0) & (len(msg.data)>0)):

        for prs in range(0,numberOfPeople):
            volHeadAll[prs][VolcounterAvg]=msg.data[prs*7+4]
            volBreastAll[prs][VolcounterAvg]=msg.data[prs*7+5]
            volBellyAll[prs][VolcounterAvg]=msg.data[prs*7+6]
            VolcounterAvg+=1

            if VolcounterAvg==30:
                VolcounterAvg=0
            volHeadstd[prs] = statistics.stdev(volHeadAll[prs])
            volBreaststd[prs] = statistics.stdev(volBreastAll[prs])
            volBellystd[prs] = statistics.stdev(volBellyAll[prs])
            # print(volHeadstd)
            # print(volBreaststd)
            # print(volBellystd)

            if ((volHeadstd[prs]<0.002) | (volBreaststd[prs]<0.002) | (volBellystd[prs]<0.002)):
                Volstatus[prs] = "stopped"
                if Volprev_stopped_time[prs]>0:
                    dif = now.secs - Volprev_stopped_time[prs]
                    Volprev_moving_time[prs] = 0
                    if dif >=5:
                        VolanomalyStatus[prs] = "Not Active"
                else:
                    Volprev_stopped_time[prs] = now.secs
            elif ((volHeadstd[prs]>0.002) | (volBreaststd[prs]>0.002) | (volBellystd[prs]>0.002)):
                Volstatus[prs] = "moving"
                if Volprev_moving_time[prs]>0:
                    Volprev_stopped_time[prs] = 0                    
                    dif = now.secs - Volprev_moving_time[prs]
                    if dif >=5:
                        VolanomalyStatus[prs] = "Active"
                else:
                    Volprev_moving_time[prs] = now.secs
            else:
                Volstatus[prs] = "NaN"
        #print (datetime.datetime.fromtimestamp(t.secs).strftime("%Y-%m-%d %H:%M:%S"),t.nsecs,speed,status)
        #print (speed,status)
        pub_msg.data = Volstatus
        statusBody_pub.publish(pub_msg)
        anomalypub_msg.data = VolanomalyStatus
        anomalyBody_pub.publish(anomalypub_msg)
                                


s = rospy.Subscriber('/people_tracker/positions', PeopleTracker, callbackPT)

s = rospy.Subscriber('/reidentifier/head3DPt', PoseArray, callbackH)

s = rospy.Subscriber('/reidentifier/shR3DPt', PoseArray, callbackSHR)

s = rospy.Subscriber('/reidentifier/shL3DPt', PoseArray, callbackSHL)

s = rospy.Subscriber('/reidentifier/features', Float32MultiArray, callbackV)

timer = rospy.Timer(rospy.Duration(0.1), callbackT)

s = rospy.Subscriber('/reidentifier/image',Image, callback_img)
#s = rospy.Subscriber('/camera/rgb/image_rect_color',Image, callback_img)

status_pub = rospy.Publisher('/st_anomaly_detector/person_status', StringArray, queue_size=10)
statusTorso_pub = rospy.Publisher('/st_anomaly_detector/torso_status', StringArray, queue_size=10)
statusHead_pub = rospy.Publisher('/st_anomaly_detector/head_status', StringArray, queue_size=10)
statusBody_pub = rospy.Publisher('/st_anomaly_detector/body_status', StringArray, queue_size=10)
speed_pub = rospy.Publisher('/st_anomaly_detector/person_speed', Float32MultiArray, queue_size=10)
image_pub = rospy.Publisher("/st_anomaly_detector/image_status", Image, queue_size=10)
anomaly_pub = rospy.Publisher('/st_anomaly_detector/person_anomaly', StringArray, queue_size=10)
anomalyTorso_pub = rospy.Publisher('/st_anomaly_detector/torso_anomaly', StringArray, queue_size=10)
anomalyHead_pub = rospy.Publisher('/st_anomaly_detector/head_anomaly', StringArray, queue_size=10)
anomalyBody_pub = rospy.Publisher('/st_anomaly_detector/body_anomaly', StringArray, queue_size=10)

rospy.spin()
