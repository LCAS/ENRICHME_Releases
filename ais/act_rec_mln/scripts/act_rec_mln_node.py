#!/usr/bin/env python


import rospy
from ros_alchemy.srv import InferenceSrv, InferenceSrvResponse
from std_msgs.msg import String
from binarySensor import BinarySensor
from magnitudeSensor import MagnitudeSensor
import datetime
from tzlocal import get_localzone  # $ pip install tzlocal
import pytz
from datetime import timedelta
import collections

# Node class.
class ActivityRecognition():

    def  createSensorHandlers(self):
        self.sensors=[]
        for sensorName in self.sensorList:
            if 'Contact' in sensorName:
                sensor=BinarySensor(sensorName,self.mongoServer,self.mongoPort,self.mongoDBName,self.mongoCollName)
            elif 'Presence' in sensorName:
                sensor = BinarySensor(sensorName, self.mongoServer, self.mongoPort, self.mongoDBName,self.mongoCollName)
            elif 'Humid' in sensorName:
                sensor = MagnitudeSensor(sensorName, self.mongoServer, self.mongoPort, self.mongoDBName,self.mongoCollName)
            elif 'Lux' in sensorName:
                sensor = MagnitudeSensor(sensorName, self.mongoServer, self.mongoPort, self.mongoDBName,self.mongoCollName)
            elif 'Temp' in sensorName:
                sensor = MagnitudeSensor(sensorName, self.mongoServer, self.mongoPort, self.mongoDBName,self.mongoCollName)
            elif 'UV' in sensorName:
                sensor = MagnitudeSensor(sensorName, self.mongoServer, self.mongoPort, self.mongoDBName,self.mongoCollName)
            elif 'Power' in sensorName:
                sensor = MagnitudeSensor(sensorName, self.mongoServer, self.mongoPort, self.mongoDBName,self.mongoCollName)
            else:
                rospy.logerr("Don't know how to handle sensor named %d" % sensorName)
            self.sensors.append(sensor)

    def  updateSensorData(self):
        self.endTime=datetime.datetime.now(get_localzone())
        d = timedelta(seconds=self.tevidence)
        self.startTime=self.endTime -d

        for sensor in self.sensors:
            sensor.updateData(self.tevidence,self.endTime)
         # maybe each sensor object should update by themselves...

    def  getSensorEvidences(self):
        evidenceList=[]
        for sensor in self.sensors:
            evidenceList.append(sensor.getEvidence())
            rospy.logdebug(sensor)
        return evidenceList

    # class constructor.
    def __init__(self):
        # load parameters
        # mongo server location, database and collection
        # They are global config params, see global.yaml in ais_rosparam
        self.mongoServer=rospy.get_param("/ais_hostname", "192.168.1.182")
        self.mongoPort= int(rospy.get_param('/mongoPort', '27017'))
        self.mongoDBName=str(rospy.get_param('/domoticDBName','openhab'))
        self.mongoCollName=str(rospy.get_param('/domoticCollName','openhab'))

        # sensor names on database

        #self.sensorList =['External_Door_Contact','Entry_Multi_Contact','Fridge_Door_Contact','Toilet_Door_Contact','Workshop_Multi_Contact','Office1_Multi_Humid','Entry_Multi_Lux','Kitchen_Multi_Lux','Lounge_Multi_Lux','Office2_Multi_Lux','Workshop_Multi_Lux','Entry_Multi_Presence','Kitchen_Multi_Presence','Kitchen_Plug_Power','Lounge_Multi_Presence','Office1_Multi_Presence','Printer_Plug_Power','Workshop_Multi_Presence','Entry_Multi_Temp','Kitchen_Multi_Temp','Lounge_Multi_Temp','Office2_Multi_Temp','Workshop_Multi_Temp','Office1_Multi_UV']
        #self.sensorList = ['Fridge_Door_Contact','Kitchen_Plug_Power','Kitchen_Multi_Presence','Toilet_Door_Contact','Entry_Multi_Presence','External_Door_Contact','Entry_Multi_Contact','Lounge_Multi_Presence','Office1_Multi_Presence','Office2_Multi_Presence','Workshop_Multi_Presence','Printer_Plug_Power']
        self.sensorList = rospy.get_param('~sensors')
        self.staticEvidence = rospy.get_param('~staticEvidence')

        # evidence building time ranges
        self.tevidence = float(rospy.get_param('~tevidence', '60.0'))

        # sample time: time between inferences in secs
        self.tsample=float(rospy.get_param('~tsample', '10.0'))

        # Topic name where inferences are published
        self.inferTopicName=str(rospy.get_param('~inferTopicName','/mln_location'))


        self.model = rospy.get_param('~mlnModelName')
        self.queries = rospy.get_param('~queries')

        # .........................................................................................................
        # Create subscriber to inference service
        rospy.loginfo("Waiting for MLN inference service ")
        rospy.wait_for_service('probcog_infer')
        self.inf_srvcall = rospy.ServiceProxy('probcog_infer', InferenceSrv)
        rospy.loginfo("Connected to MLN inference service ")
        # Create publisher for current mln
        self.act_pub=rospy.Publisher(self.inferTopicName, String, queue_size=10)

        #  create sensor handlers (connect to database, build evidences...)
        self.createSensorHandlers()

        # Main while loop.
        r = rospy.Rate(1/self.tsample)
        rospy.loginfo("Activity recognition node started ")
        while not rospy.is_shutdown():
            # update sensor data
            rospy.logdebug("Updating sensor data")
            self.updateSensorData()

            # gather sensor evidences
            rospy.logdebug("Gathering evidences between "+self.startTime.isoformat(' ')+" and "+self.endTime.isoformat(' '))
            evidence=self.getSensorEvidences()+self.staticEvidence

            #precook evidence for testing
            #evidence=['level(Low,Lounge_Multi_Presence)','level(Low,Kitchen_Multi_Presence)','level(Mid,Kitchen_Plug_Power)','level(Mid,Toilet_Door_Contact)',
            #          'level(Mid,Entry_Multi_Contact)','level(Low,Entry_Multi_Presence)','level(Mid,Kitchen_Multi_Lux)','level(Mid,Entry_Multi_Temp)',
            #          'level(Mid,Kitchen_Multi_Temp)','level(Mid,Lounge_Multi_Lux)','level(Mid,Lounge_Multi_Temp)','level(Mid,Entry_Multi_Lux)']


            rospy.logdebug("")
            rospy.logdebug("Evidence from sensors is ")
            for ev in evidence:
                rospy.logdebug(ev)
            rospy.logdebug("")

            # ask inference service
            try:
                rospy.logdebug("Connecting to inference service")
                self.lastInferenceResult=self.inf_srvcall(self.model,evidence,self.queries)

                # sort inferences by probability
                sortedInferences=[]
                for inference in self.lastInferenceResult.results:
                    sortedInferences.append([inference.probability,inference.functionName])

                sortedInferences.sort(key=lambda x:(x[0] * -1, x[1]))

                ans="\nEvidences between "+self.startTime.isoformat(' ')+" and "+self.endTime.isoformat(' ')+'\n'
                for ev in evidence:
                    ans = ans + ev + '\n'
                ans = ans + "\nInference on Network model "+self.model+"\n"
                for inference in sortedInferences:
                    ans=ans+inference[1]+'=('+"{0:>2.2f}".format(inference[0])+')\n'

                rospy.logdebug("Publishing inference")
                rospy.logdebug(ans)
                
                ans=""     
                for inference in sortedInferences:
                    ans=ans+inference[1]+','+"{0:>2.2f}".format(inference[0])+','
                
                self.act_pub.publish(ans[0:-1])

            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s" % e)

            # Sleep for a while after publishing new messages
            r.sleep()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('act_rec_mln',log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ar = ActivityRecognition()
    except rospy.ROSInterruptException: pass
