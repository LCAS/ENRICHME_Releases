#!/usr/bin/env python


'''
'''
import random
import requests
import rospy
from env_sensor.msg import Environment
from diagnostic_msgs.msg import KeyValue


# Node example class.
class randomHouse():

    def env_callback(self, data):
        self.env_data = data
        self.newData = True

    def periodicPublish(self, evt):
        for sensor in self.sensorList:
            self.iot_msg.value = str(self.generateRandomFor(sensor))
            if self.iot_msg.value !='invalid':
                self.iot_msg.key = sensor['name']
                self.iot_set_pub.publish(self.iot_msg)
                #rospy.logdebug(self.iot_msg.key+": "+self.iot_msg.value)

    def generateRandomFor(self,sensor):
        ans='invalid'
        sensorName=sensor['name']
        if ('Contact' in sensorName) and ('ContactItem' in sensor['type']):
            ans=random.choice( ['CLOSED', 'OPEN'])

        if ('Presence' in sensorName) and ('ContactItem' in sensor['type']):
            ans = random.choice(['CLOSED', 'OPEN'])
        if ('Presence' in sensorName) and ('NumberItem' in sensor['type']):
            ans=random.choice( ['0', '1'])

        if 'Power' in sensorName:
            ans = random.random() * 1000
        if 'Power' in sensorName:
            ans = random.random() * 1000
        if 'Lux' in sensorName:
            ans = random.random() * 200
        if 'Temp' in sensorName:
            ans = random.random() * 15 +15

        return ans

    def getOpenHABItems(self):
        itemList = list()
        openhab_host = '127.0.0.1'
        openhab_port = '8080'

        url = 'http://%s:%s/rest/items' % (openhab_host, openhab_port)
        payload = {'type': 'json'}
        ph = {
            "Accept": "application/json"
        }

        try:
            req = requests.get(url, params=payload, headers=ph)
            if req.status_code != requests.codes.ok:
                req.raise_for_status()
            # Try to parse JSON response
            items = req.json()["item"]
            for item in items:
                if item['type'] != 'GroupItem':
                    itemList.append(item)
        except:
            pass
        return sorted(itemList)

    def createSensorList(self):
        self.sensorList=self.getOpenHABItems()



    # class constructor.
    def __init__(self):
        rospy.loginfo('Waiting openhab to be running')
        rospy.sleep(5)


        self.newData = False
        # Publish data every minute...
        self.tsample = int(rospy.get_param('~tsample', '5'))
        rospy.loginfo('tsample = %d', self.tsample)

        # Create iot publisher with correct name
        listOfTopics = rospy.get_published_topics()
        topicName = '/openhab_set'
        for tup in listOfTopics:
            if 'openhab' in tup[0]:
                topicName = '/openhab_set'
                break
            if 'iot' in tup[0]:
                topicName = '/iot_set'
                break

        rospy.logdebug('Forwarding values to topic %s', topicName)
        self.iot_set_pub = rospy.Publisher(topicName, KeyValue, queue_size=10)


        rospy.logdebug('Creating sensor list')
        # get a list of thing to made up:
        self.createSensorList()

        # Create message object
        self.iot_msg = KeyValue()

        # Initial publish
        rospy.logdebug('Initial publish')
        self.periodicPublish('')

        # periodically publish data
        timer = rospy.Timer(rospy.Duration(self.tsample), self.periodicPublish)
        rospy.logdebug('Ready to generate random house values')
        # that's all
        rospy.spin()


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('py_amb2openhab_node',log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        rh = randomHouse()
    except rospy.ROSInterruptException:
        pass
