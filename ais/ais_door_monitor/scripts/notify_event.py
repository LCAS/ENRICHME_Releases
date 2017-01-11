#!/usr/bin/env python


'''
'''

import rospy
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import String
from ncp_event_notifier.srv import *
from ncp_event_notifier.msg import *


# Node example class.
class notifier():

    def report(self,evt):

        #event = KeyValue()
        #event.key = "Environmental"
        #event.value = 'Main door open for 30 secs now.'

        #self.notif_pub .publish(event)

        self.notif_pub.publish("Alarm:" + self.doorSensorName + ":" + "{0:> 2.2f}".format(self.openedSince.to_sec()))
        self.postNCPEvent()

        if self.alarmWasSent:
           rospy.logdebug('Event send')
           self.timer.shutdown()
           self.alarmWasSent = False

    def openhab_callback(self,evt):
        rospy.logdebug("Openhab update received (%s)",evt.key)
        if evt.key==self.doorSensorName:
            rospy.logdebug('Entry door event received')
            if (evt.value == 'OPEN' and self.prev_event.value == 'CLOSED'):
                self.timer = rospy.Timer(rospy.Duration(self.t_alarm), self.report)
                self.openedSince=rospy.Time.now()
                rospy.logdebug('Programed a timer')
            elif (evt.value == 'CLOSED' and self.prev_event.value == 'OPEN'):
                self.timer.shutdown()
                rospy.logdebug('Cancelled a timer')
            else:
                pass
            self.prev_event=evt
       
    # class constructor.
    def __init__(self):
        self.alarmWasSent = False
        
        # Report door open after 30 seconds
        self.t_alarm = int(rospy.get_param('~t_alarm', '30'))

        # Topic to publish alarms to
        self.notifTopicName= rospy.get_param('~notifTopicName', '/ais_notifications')

        # Monitored door sensor name
        self.doorSensorName= rospy.get_param('~sensorName', 'Entry_Door_Contact')

        # message for NCP
        self.targetCD = str(rospy.get_param('/targetCD', '123456'))
        self.lastNCP_event = NCP_Event()
        self.lastNCP_event.ID = '00000000-0000-0000-0000-000000000000'
        rospy.logdebug('Waiting for ncp event notifier service to be active')
        rospy.wait_for_service('ncp_event_operation')
        self.ens = rospy.ServiceProxy('ncp_event_operation', NCP_Event_operation)
        rospy.logdebug('Connected to ncp event notifier service ')
        # Create openhab subscriber
        listOfTopics = rospy.get_published_topics()
        topicName = '/openhab_updates'
        for tup in listOfTopics:
            if 'openhab' in tup[0]:
                topicName = '/openhab_updates'
                break
            if 'iot' in tup[0]:
                topicName = '/iot__updates'
                break
        rospy.logdebug('Exporting values on topic %s', topicName)
        # Create openhab updates subscriber
        self.openhab_sub = rospy.Subscriber(topicName,KeyValue,self.openhab_callback)

        #create notification publisher
        #self.notif_pub = rospy.Publisher(self.notifTopicName, KeyValue, queue_size=10)
        self.notif_pub = rospy.Publisher(self.notifTopicName, String, queue_size=10)

        #publish data
        self.timer = rospy.Timer(rospy.Duration(self.t_alarm), self.report)
        self.timer.shutdown()

        #previous data
        #TODO this should be loaded from mongo!!
        self.prev_event=KeyValue()
        self.prev_event.value ='CLOSED'

        rospy.logdebug('Waiting data')
        #that's all
        rospy.spin()

    def postNCPEvent(self):
        rospy.loginfo('NCP notification disabled by now')


    def postNCPEvent_ORIG(self):
        '''
        Send a new NCP event only if we haven't already sent it
        :return: -
        '''

        if not self.alarmWasSent:
         try:
             verb = 'POST'
             # Payload
 
             # this is a temp value. Real one is filled by NCP
             self.lastNCP_event.ID = '00000000-0000-0000-0000-000000000000'
             # user code
             self.lastNCP_event.TargetCD = self.targetCD
             # User Has Abnormal Behaviour
             self.lastNCP_event.TypeCD = 'AHAB'
             self.lastNCP_event.Begin = rospy.Time.now()
             # Static value to ULINAAL for the AAL server.
             self.lastNCP_event.srcDomain = 'ULINAAL'
             # Universal ID  identifying   the  event and invariable   from the installation  of  the AAL  server.
             self.lastNCP_event.srcId = 'house01'
             resp1 = self.ens(verb, self.lastNCP_event)

             # todo ensure transmission? store id for further use?...
             if resp1.wasOk:
                 self.lastNCP_event.ID = resp1.response.ID
                 self.alarmWasSent = True
 
         except rospy.ServiceException as exc:
             rospy.logerr("Service did not process request: " + str(exc))


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ais_door_monitor_node',log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        note = notifier()
    except rospy.ROSInterruptException: pass
