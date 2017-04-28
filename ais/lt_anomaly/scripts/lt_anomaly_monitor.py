#!/usr/bin/env python


'''
    Long term anomaly detector.
    Version 1: based only on ambient sensors.

    Connects to database to get information about sensors.
    TODO for act_mln:   timezones management
                        ranges management
'''

import rospy
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import String
from binarySensor import BinarySensor
from magnitudeSensor import MagnitudeSensor
from ncp_event_notifier.srv import *
from ncp_event_notifier.msg import *
from math import log
import numpy as np
import yaml

# Node class.

class lt_anom():
    def sendLestlessnessReport(self):
        self.anom_pub.publish("Alarm:" + "Restlessness" + ":" +"{0:> 2.2f}".format(self.houseEntropy/self.maxSystemEntropy) )
        self.postNCPEvent()

    def sendInactivityReport(self):
        self.anom_pub.publish("Alarm:" + "Inactivity" + ":" + "{0:> 2.2f}".format(self.houseEntropy/self.maxSystemEntropy))
        self.postNCPEvent()

    def loadROSParams(self):
        self.MIN_ACTIVATION = 0.001

        # mongo server location, database and collection
        self.mongoServer=rospy.get_param("/ais_hostname", "192.168.1.182")
        self.mongoPort= int(rospy.get_param('/mongoPort', '27017'))
        self.mongoDBName=str(rospy.get_param('/domoticDBName','openhab'))
        self.mongoCollName=str(rospy.get_param('/domoticCollName','openhab'))
        self.targetCD = str(rospy.get_param('/targetCD', '123456'))
        #self.sensorsFileName= str(rospy.get_param('~usedSensorsFileName',"../cfg/lt_anomaly.yaml"))

        # load list of presence sensors to monitor restlessness
        #f = open(self.sensorsFileName, 'r')
        #data = yaml.load(f)

        #self.sensorList = data['sensors']
        self.sensorList = rospy.get_param('~sensors')

                           #  ['Kitchen_Multi_Presence',
                           # 'Entry_Multi_Presence',
                           # 'Lounge_Multi_Presence',
                           # 'Office1_Multi_Presence',
                           # 'Office2_Multi_Presence',
                           # 'Workshop_Multi_Presence']

        # sensor data time ranges
        self.tevidence = float(rospy.get_param('~tevidence', '600.0'))

        # entropy warning ranges..
        # TODO find a better thresholding like average entropies on system or so
        self.maxSystemEntropy=log(len(self.sensorList))

        self.warningEntropyHigh = float(rospy.get_param('~warningEntropyHighPercent', '75')) * self.maxSystemEntropy/100.0

        self.warningEntropyLow  = float(rospy.get_param('~warningEntropyLowPercent',  '5')) * self.maxSystemEntropy/100.0

        # sample time: time between inferences in secs
        self.tsample = float(rospy.get_param('~tsample', '10.0'))

        # anomaly topic name
        self.anomTopicName=rospy.get_param('~anomTopicName', '/lt_anomaly')

    def createROSconns(self):

        # create long term anomaly publisher
        self.anom_pub = rospy.Publisher(self.anomTopicName, String, queue_size=10)

        rospy.loginfo('Waiting NCP event notification service to be ready')
        rospy.wait_for_service('ncp_event_operation')
        self.ens = rospy.ServiceProxy('ncp_event_operation', NCP_Event_operation)
        rospy.loginfo('Connected to NCP event notification service')

        # TODO debug topics
        '''
        Each sensor:
            sensor name
            check interval
            points used
            av active time
            entropy
        System
            Entropy delta
            Total entropy
            Max entropy
        '''


        # TODO create connection to ROBOT OUTPUT V2

    def createSensorHandlers(self):
        self.sensors = []
        for sensorName in self.sensorList:
            if 'Presence' in sensorName:
                sensor = BinarySensor(sensorName, self.mongoServer, self.mongoPort, self.mongoDBName,
                                      self.mongoCollName,self.tevidence,self.tsample,self.maxSystemEntropy)
            elif 'Contact' in sensorName:
                sensor = BinarySensor(sensorName, self.mongoServer, self.mongoPort, self.mongoDBName,
                                      self.mongoCollName,self.tevidence,self.tsample,self.maxSystemEntropy)                                      
            else:
                rospy.logerr("Don't know how to handle sensor named %d" % sensorName)
            self.sensors.append(sensor)


    # TODO this is a work in progress: get to know if we have already sended event to NCP
    def getEvent(self):
        # get event
        # y = Event_operation()
        verb = 'GET'
        payload = NCP_Event()

        payload.ID = self.lastNCP_event.ID
        payload.TargetCD = self.targetCD
        #payload.srcDomain = 'ULINAAL'
        #payload.srcId = 'house01'
        rospy.loginfo('getting previous event')

        rospy.sleep(1)

        try:
            resp1 = self.ens(verb, payload)
            rospy.loginfo("Response is %s", resp1.feedback)
            self.receivedNCPevent=resp1.response
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))




    def postNCPEvent(self):
        rospy.loginfo("NCP notification disabled by now...")

    def postNCPEvent_ORIG(self):
        '''
        Send a new NCP event only if we haven't already sent it
        :return: -
        '''

        #if not self.alarmWasSent:
        try:
            verb = 'POST'
            # Payload

            # this is a temp value. Real one is filled by NCP
            self.lastNCP_event.ID = self.DUMMY_ID
            # user code
            self.lastNCP_event.TargetCD = self.targetCD
            # User Has Abnormal Behaviour
            self.lastNCP_event.TypeCD = 'UHAB'
            self.lastNCP_event.Begin = rospy.Time.now()
            # Static value to ULINAAL for the AAL server.
            self.lastNCP_event.srcDomain = 'ULINAAL'
            # Universal ID  identifying   the  event and invariable   from the installation  of  the AAL  server.
            self.lastNCP_event.srcId = 'house01'
            resp1 = self.ens(verb, self.lastNCP_event)

            # todo ensure transmission? store id for further use?...
            if resp1.wasOk:
                self.lastNCP_event.ID = resp1.response.ID
                #self.alarmWasSent = True

        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))


    # class constructor.
    def __init__(self):

        self.DUMMY_ID='00000000-0000-0000-0000-000000000000'
        # load config params
        self.loadROSParams()

        # message for NCP
        self.lastNCP_event = NCP_Event()
        self.lastNCP_event.ID = self.DUMMY_ID

        # Create ros subscribers/publishers and/or services
        self.createROSconns()

        #  create sensor handlers (connect to database, build evidences...)
        # must be called AFTER creating ROS connections/service proxies
        self.createSensorHandlers()

        # TODO create connection to FREMEN, or other pattern system.... V3

        r = rospy.Rate(1/self.tsample)
        #main loop
        while not rospy.is_shutdown():
            self.updateEntropy()
            rospy.logdebug("******************************************")
            rospy.logdebug("House Entropy "+"{0:> 2.2f}".format(100.0 * self.houseEntropy/self.maxSystemEntropy)+" %")
            #for sensor in self.sensors:
                #rospy.logdebug(sensor)


            if self.houseEntropy>self.warningEntropyHigh:
                self.sendLestlessnessReport()
                rospy.logdebug("Restlessnes!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" )
            elif self.houseEntropy<self.warningEntropyLow:
                self.sendInactivityReport()
                rospy.logdebug("Inactivity!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

            rospy.logdebug(" ")
            
            
            # Sleep for a while after publishing new messages
            r.sleep()

    def updateEntropy(self):
        totalActiveTime = 0
        self.houseEntropy = 0

        # all sensors are binary. Hence average value in interval is equal to percentage of time active.
        # sensor activations may have overlappings, so totalActiveTime may be more than tevicence...
        for sensor in self.sensors:
            #sensors inactive are given a minimum activation so they have entropy...
            if sensor.avValueAtInterval<=0:
                sensor.avValueAtInterval=self.MIN_ACTIVATION
            totalActiveTime += sensor.avValueAtInterval

        for sensor in self.sensors:
            sensor.prob = sensor.avValueAtInterval / totalActiveTime
            sensor.sensorEntropy = -log(sensor.prob) * sensor.prob
            self.houseEntropy += sensor.sensorEntropy


    def entropy(self,ar):
        total=float(np.array(ar).sum())
        ar[ar == 0] += self.MIN_ACTIVATION
        ar_norm=ar

        for i in range(0,len(ar)):
            e=ar[i]/total
            ar_norm[i]=-e*log(e)
        return ar_norm



# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('lt_anomaly_monitor_node',log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        nodo = lt_anom()
    except rospy.ROSInterruptException:
        pass
