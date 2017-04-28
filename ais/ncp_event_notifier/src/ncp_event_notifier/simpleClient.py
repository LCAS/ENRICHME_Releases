#!/usr/bin/env python

import os
from ncp_event_notifier.srv import *
from ncp_event_notifier.msg import *
import rospy
import xml.etree.ElementTree as ET
import requests
import json
import uuid

#TODO ALLOW CLASSES TO BE LOADED FROM FILE or param server
types = {'ADOR', 'AHAB', \
         'UCDO', 'URCT', 'UCDU', 'UNCT', 'UHAB', 'URFO', \
         'UPPB', \
         'DPSM', 'DRSM', 'UHLH', 'URTH', \
         'DIBR'}




# Server Class ........................................................................................................
class notifyClient():


    def getEvent(self):
        # get event
        # y = Event_operation()
        verb = 'GET'
        payload = NCP_Event()

        payload.ID = '9d35f770-8c78-44f1-9a95-5440fa92c5e9'  # from  https://secure.tesanonline.it/pac/
        payload.TargetCD = '123456'
        #payload.srcDomain = 'ULINAAL'
        #payload.srcId = 'house01'

        rospy.loginfo('dummy event ready')

        rospy.sleep(2)

        rospy.loginfo("Doing a %s operation", verb)

        try:
            resp1 = self.ens(verb, payload)
            rospy.loginfo("Response is %s", resp1.feedback)
            self.pub.publish(resp1.response)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def postEvent(self):
        # Post event
        # y = Event_operation()
        verb = 'POST'
        payload = NCP_Event()

        payload.ID = '74bafd06-d500-4319-9ecc-8524769bafbd'  # overriden!
        payload.TargetCD = '123456'
        payload.TypeCD = 'AHAB'
        payload.Begin = rospy.Time.now()
        #payload.End = rospy.Time.now()  # overriden ... i hope
        #payload.Shown = rospy.Time.now()  # overriden ... i hope
        # payload.Conclusion
        # payload.Shown
        # payload.Disable
        payload.srcDomain = 'ULINAAL'
        payload.srcId='house01'
        # payload.Ext=''

        rospy.loginfo('dummy event ready')

        #while not rospy.is_shutdown():
        # Sleep before publishing new messages
        rospy.sleep(2)

        rospy.loginfo("Doing a %s operation", verb)
        # rospy.loginfo("With ID (%s)", payload.ID)

        try:
            resp1 = self.ens(verb, payload)
            rospy.loginfo("Response is %s", resp1.feedback)
            self.pub.publish(resp1.response)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def putEvent(self):
        # Post event
        # y = Event_operation()
        verb = 'PUT'
        payload = NCP_Event()

        payload.ID = '110bae9-aaa5-4961-b063-c658fa89b4c0'  # overriden!
        payload.TargetCD = '123456'
        payload.TypeCD = 'AHAB'          # overriden ... i hope
        payload.Begin = rospy.Time.now() # overriden ... i hope
        payload.End = rospy.Time.now()  # overriden ... i hope
        payload.Shown = rospy.Time.now()  # overriden ... i hope
        # payload.Conclusion
        # payload.Shown
        # payload.Disable
        payload.srcDomain = 'ULINAAL'
        payload.srcId = 'house01'
        # payload.Ext=''

        rospy.loginfo('dummy event ready')

        # while not rospy.is_shutdown():
        # Sleep before publishing new messages
        rospy.sleep(2)

        rospy.loginfo("Doing a %s operation", verb)
        # rospy.loginfo("With ID (%s)", payload.ID)

        try:
            resp1 = self.ens(verb, payload)
            rospy.loginfo("Response is %s", resp1.feedback)
            self.pub.publish(resp1.response)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def postGetWithXML(self):

        # Create XML to attach
        currDir=os.getcwd()
        tree = ET.parse(currDir+'/exampleData.xml')
        root = tree.getroot()

        #getting an xml object from a text string
        #root = ET.fromstring(example_data_as_string)

        #casting a xml object to a text string
        example_data_as_string=ET.tostring(root,encoding="us-ascii",method="xml")


        rospy.loginfo('dummy event ready')

        try:
            verb = 'POST'
            # Payload
            payload = NCP_Event()
            payload.ID = '62697465-6d79-7368-696e-696d6574616c'   # overriden
            payload.TargetCD = '123456'
            payload.TypeCD = 'ADOR'
            payload.Begin = rospy.Time.now()
            payload.srcDomain = 'ULINAAL'
            payload.srcId = 'house01'
            payload.Ext = example_data_as_string

            rospy.loginfo("Performing %s operation",verb)
            resp1 = self.ens(verb, payload)

            if resp1.wasOk:
                rospy.loginfo("Transmission correct!!!!!!")
            rospy.loginfo("Response is %s", resp1.feedback)
            postedID=resp1.response.ID

            rospy.loginfo(".....................................................................................")
            verb = 'GET'
            # Payload
            payload = NCP_Event()
            payload.ID=postedID

            rospy.loginfo("Performing %s operation", verb)
            resp2 = self.ens(verb, payload)
            if resp2.wasOk:
                rospy.loginfo("Transmission correct!!!!!!")
            rospy.loginfo("Response is %s", resp2.feedback)



        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def test3(self):

        # Create XML to attach

        tree = ET.parse(
            './exampleData.xml')
        root = tree.getroot()

        # getting an xml object from a text string
        # root = ET.fromstring(example_data_as_string)

        # casting a xml object to a text string
        example_data_as_string = ET.tostring(root, encoding="us-ascii", method="xml")

        rospy.loginfo('dummy event ready')

        try:
            verb = 'PUT'
            # Payload
            payload = NCP_Event()
            payload.ID = 'ca412f55-e5d3-4d9b-8c14-535ddfc1f722'
            payload.TargetCD = '123456'
            payload.TypeCD = 'ADOR'
            payload.Begin = rospy.Time.now()
            payload.srcDomain = 'ULINAAL'
            payload.srcId = 'house01'
            payload.Ext = example_data_as_string

            rospy.loginfo("Performing %s operation", verb)
            resp1 = self.ens(verb, payload)

            if resp1.wasOk:
                rospy.loginfo("Transmission correct!!!!!!")
            rospy.loginfo("Response is %s", resp1.feedback)
            postedID = resp1.response.ID

            rospy.loginfo(
                ".....................................................................................")


        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    # class constructor.
    def __init__(self):
        global baseURL

        rospy.init_node('simpleNCPclient')
        rospy.loginfo('Waiting for NCP Event notifier server to be active')
        rospy.wait_for_service('ncp_event_operation')
        self.ens = rospy.ServiceProxy('ncp_event_operation', NCP_Event_operation)

        self.pub = rospy.Publisher('NCPEvent', NCP_Event, queue_size=10)

        rospy.loginfo('Event notifier server now active')

        # Main while loop.
        #self.getEvent()
        self.postEvent()
        #self.putEvent() #<- does not work
        #self.postGetWithXML()


# Main function........................................................................................................
if __name__ == "__main__":
    notifyClient()
