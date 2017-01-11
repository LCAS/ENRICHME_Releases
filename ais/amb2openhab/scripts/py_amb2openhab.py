#!/usr/bin/env python


'''
'''

import rospy
from env_sensor.msg import Environment
from diagnostic_msgs.msg import KeyValue



# Node example class.
class Amb2openhab():
    def env_callback(self,data):
        self.env_data=data
        self.newData=True

    def periodicPublish(self,evt):
        rospy.logdebug("Periodic Openhab update")
        if self.newData:
            self.iot_msg.key = "Env_cO"
            self.iot_msg.value = "{0:>2.2f}".format(self.env_data.cO)
            self.iot_set_pub.publish(self.iot_msg)

            self.iot_msg.key = "Env_particleCount"
            self.iot_msg.value = "{0:>2.2f}".format(self.env_data.particleCount)
            self.iot_set_pub.publish(self.iot_msg)
    
            self.iot_msg.key = "Env_vOCResistance"
            self.iot_msg.value = "{0:>2.2f}".format(self.env_data.vOCResistance)
            self.iot_set_pub.publish(self.iot_msg)
    
            self.iot_msg.key = "Env_temp"
            self.iot_msg.value = "{0:>2.2f}".format(self.env_data.temp)
            self.iot_set_pub.publish(self.iot_msg)

            self.iot_msg.key = "Env_humidity"
            self.iot_msg.value = "{0:>2.2f}".format(self.env_data.humidity)
            self.iot_set_pub.publish(self.iot_msg)

            self.iot_msg.key = "Env_light"
            self.iot_msg.value = "{0:>2.2f}".format(self.env_data.light)
            self.iot_set_pub.publish(self.iot_msg)

            self.iot_msg.key = "Env_timestamp"
            self.iot_msg.value = "{0:>2.2f}".format(self.env_data.header.stamp.secs)
            self.iot_set_pub.publish(self.iot_msg)
            self.newData=False
        else:
            rospy.logdebug("No new data arrived!")

    # class constructor.
    def __init__(self):
        self.newData=False    
        # Publish data every second...
        self.tsample = float(rospy.get_param('~tsample', '30.0'))
        rospy.loginfo('tsample = %d', self.tsample)


        # Create environment sensor subscriber
        self.env_sub = rospy.Subscriber("/environment_sensor",Environment,self.env_callback)

        # Create iot publisher with correct name
        listOfTopics=rospy.get_published_topics()
        topicName='/openhab_set'
        for tup in listOfTopics:            
            if 'openhab' in tup[0]:
                topicName='/openhab_set'                
                break
            if 'iot' in tup[0]:
                topicName = '/iot_set'
                break
        rospy.logdebug('Exporting values on topic %s',topicName)
        self.iot_set_pub = rospy.Publisher(topicName, KeyValue,queue_size=10)

        # Create message object
        self.iot_msg = KeyValue()

        #periodically publish data
        timer = rospy.Timer(rospy.Duration(self.tsample), self.periodicPublish)

        #that's all
        rospy.spin()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('py_amb2openhab_node')#,log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        a2o = Amb2openhab()
    except rospy.ROSInterruptException: pass
