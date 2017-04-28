#!/usr/bin/env python

import yaml
import rospy
import operator
from std_msgs.msg import String
from sensor_server.srv import envService, envServiceRequest, envServiceResponse
from diagnostic_msgs.msg import KeyValue
import rosnode

import requests
import base64

def polling_header(self):
    """ Header for OpenHAB REST request - polling """
    # self.auth = base64.encodestring('%s:%s'%(self.username, self.password)).replace('\n', '')
    return {
    #    "Authorization" : "Basic %s" % self.cmd.auth,
        "X-Atmosphere-Transport" : "long-polling",
    #    "X-Atmosphere-tracking-id" : self.atmos_id,
        "X-Atmosphere-Framework" : "1.0",
        "Accept" : "application/json"}

def basic_header(self):
    """ Header for OpenHAB REST request - standard """
    #self.auth = base64.encodestring('%s:%s' %(self.username, self.password) ).replace('\n', '')
    return {
     #       "Authorization" : "Basic %s" %self.auth,
            "Content-type": "text/plain"}




print getList()

exit()




def parseName( strMongoName):
    ans = ''
    elements = strMongoName.split('_')
    if (elements[0] == 'Env'):
        elements[0] = 'Robot'
    if (elements[1] == 'cO' or elements[1] == 'vOCResistance' or elements[1] == 'particleCount' or elements[
        1] == 'timestamp'):
        elements[1] = 'air'
    if (elements[1] == 'Plug'):
        elements[1] = 'Power'

    ans = elements[0] + '-' + elements[1]
    return ans


def parseNameMongo( strName):
    ans = list()
    elements = strName.split('-')
    if (elements[0] == 'Robot'):
        elements[0] = 'Env'

    if (elements[1] == 'air'):
        elements[1] = 'cO'
        ans.append(elements[0] + '_' + elements[1])
        elements[1] = 'vOCResistance'
        ans.append(elements[0] + '_' + elements[1])
        elements[1] = 'particleCount'
        ans.append(elements[0] + '_' + elements[1])

    elif (elements[1] == 'Multi'):
        elements[1] = 'Multi_Presence'
        ans.append(elements[0] + '_' + elements[1])
        elements[1] = 'Multi_Temp'
        ans.append(elements[0] + '_' + elements[1])
        elements[1] = 'Multi_Lux'
        ans.append(elements[0] + '_' + elements[1])

    elif (elements[1] == 'Door'):
        elements[1] = 'Door_Contact'
        ans.append(elements[0] + '_' + elements[1])

    elif (elements[1] == 'Power'):
        elements[1] = 'Plug_Power'
        ans.append(elements[0] + '_' + elements[1])
    else:
        ans.append(elements[0] + '_' + elements[1])

    return ans



ohList=['Env_cO', 'Env_light', 'Livingroom_Multi_Temp', 'Env_temp', 'Env_humidity', 'Bedroom_Multi_Lux', 'Bedroom_Multi_Alarm', 'Livingroom_Multi_Battery', 'Bathroom_Multi_Battery', 'Bedroom_Multi_Presence', 'Kitchen_Plug_Energy', 'Dining_Multi_Presence', 'Dining_Multi_Lux', 'Bathroom_Multi_Presence', 'Livingroom_Multi_Presence', 'Bedroom_Multi_Temp', 'Livingroom_Multi_Lux', 'Toilet_Door_Battery', 'Livingroom_Multi_Alarm', 'Env_vOCResistance', 'Kitchen_Plug_Switch', 'Bathroom_Multi_Alarm', 'Dining_Multi_Alarm', 'Toilet_Door_Contact', 'Bedroom_Multi_Battery', 'Dining_Multi_Battery', 'Bathroom_Multi_Temp', 'Dining_Multi_Temp', 'Env_timestamp', 'Toilet_Plug_Energy', 'Entry_Door_Contact', 'Kitchen_Plug_Power', 'Toilet_Plug_Power', 'Entry_Door_Battery', 'Env_particleCount', 'Toilet_Plug_Switch', 'Bathroom_Multi_Lux']



sensorsList = list()
for sensorName in ohList:
    newName = parseName(sensorName)
    if newName not in sensorsList:
        sensorsList.append(newName)

sensorsList=sorted(sensorsList)

print sensorsList
print "..........................................."

reverseSensorsList = list()
for sensorName in sensorsList:
    newNames = parseNameMongo(sensorName)
    for n in newNames:
        if n not in reverseSensorsList:
            reverseSensorsList.append(n)

reverseSensorsList=(reverseSensorsList)

print reverseSensorsList
print "..........................................."
print sorted(set(ohList)-set(reverseSensorsList))


print sep.join([str(0),'{0:.1f}'.format(23.55),str(345)  ])
