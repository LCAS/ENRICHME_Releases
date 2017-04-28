#!/usr/bin/env python


import rospy
from ros_alchemy.srv import InferenceSrv, InferenceSrvResponse
from std_msgs.msg import String
import datetime
from tzlocal import get_localzone  # $ pip install tzlocal
import pytz
from datetime import timedelta
import collections


# Node class.
class ros_alch_test():

    # class constructor.
    def __init__(self):
        # .........................................................................................................
        # Create subscriber to inference service
        rospy.loginfo("Waiting for MLN inference service ")
        rospy.wait_for_service('probcog_infer')
        self.inf_srvcall = rospy.ServiceProxy('probcog_infer', InferenceSrv)
        rospy.loginfo("Connected to MLN inference service ")

        # Main while loop.
        r = rospy.Rate(1 / 5.0)
        rospy.loginfo("Activity recognition node started ")
        while not rospy.is_shutdown():
            # gather sensor evidences

            # precook evidence for testing
            evidence=[
            'sensorAt(Entry_Door_Contact, Entry)',
            'sensorAt(Fridge_Door_Contact, Kitchen)',
            'sensorAt(Kitchen_Plug_Power, Kitchen)',
            'sensorAt(Kitchen_Multi_Presence, Kitchen)',
            'sensorAt(Kitchen_Multi_Lux, Kitchen)',
            'sensorAt(Kitchen_Multi_Temp, Kitchen)',
            'sensorAt(Bathroom_Multi_Presence, Bathroom)',
            'sensorAt(Bathroom_Multi_Lux, Bathroom)',
            'sensorAt(Bathroom_Multi_Temp, Bathroom)',
            'sensorAt(Bedroom_Multi_Presence, Bedroom)',
            'sensorAt(Bedroom_Multi_Lux, Bedroom)',
            'sensorAt(Bedroom_Multi_Temp, Bedroom)',
            'sensorAt(Livingroom_Multi_Presence, Livingroom)',
            'sensorAt(Livingroom_Multi_Lux, Livingroom)',
            'sensorAt(Livingroom_Multi_Temp, Livingroom)',
            'level(Low,Entry_Door_Contact)',
            'level(Low,Fridge_Door_Contact)',
            'level(Low,Kitchen_Plug_Power)',
            'level(Hig,Kitchen_Multi_Presence)',    'level(Low,Kitchen_Multi_Lux)',    'level(Low,Kitchen_Multi_Temp)',
            'level(Mid,Bathroom_Multi_Presence)',   'level(Mid,Bathroom_Multi_Lux)',   'level(Low,Bathroom_Multi_Temp)',
            'level(Low,Bedroom_Multi_Presence)',    'level(Low,Bedroom_Multi_Lux)',    'level(Low,Bedroom_Multi_Temp)',
            'level(Low,Livingroom_Multi_Presence)', 'level(Low,Livingroom_Multi_Lux)', 'level(Low,Livingroom_Multi_Temp)'
            ]


            model = 'LOCATION-FDG'
            queries = ['currPlace']

            rospy.logdebug("")
            rospy.logdebug("Evidence is ")
            for ev in evidence:
                rospy.logdebug(ev)
            rospy.logdebug("")

            # ask inference service
            try:
                rospy.logdebug("Connecting to inference service")
                self.lastInferenceResult = self.inf_srvcall(model, evidence, queries)

                # sort inferences by probability
                sortedInferences = []
                for inference in self.lastInferenceResult.results:
                    sortedInferences.append([inference.probability, inference.functionName])

                sortedInferences.sort(key=lambda x: (x[0] * -1, x[1]))

                ans = ""
                for ev in evidence:
                    ans = ans + ev + '\n'
                ans = ans + "\nInference on Network model " + model + "\n"
                for inference in sortedInferences:
                    ans = ans + inference[1] + '=(' + str(inference[0]) + ')\n'

                rospy.logdebug("Publishing inference")
                rospy.logdebug(ans)

                ans = ""
                for inference in sortedInferences:
                    ans = ans + inference[1] + ',' + str(inference[0])+','
                ans=ans[:-1]
                rospy.logdebug(ans)

            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s" % e)

            # Sleep for a while after publishing new messages
            r.sleep()


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ros_alch_test' ,log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ar = ros_alch_test()
    except rospy.ROSInterruptException:
        pass
