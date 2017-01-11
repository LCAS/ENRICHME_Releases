#!/usr/bin/env python

import yaml
import rospy
import operator
from std_msgs.msg import String
from sensor_server.srv import sensorService, sensorServiceRequest, sensorServiceResponse
from diagnostic_msgs.msg import KeyValue
import rosnode
import requests
import json
import numpy as np
import math
import collections


class sensor_server_node():

    def percentFormat(self, strNum):
        return  '{:.1f}'.format(100. *float( strNum))

    def decimalFormat(self, strNum):
        return  '{0:.1f}'.format( float( strNum))

    def sensorServiceCallback(self,sensorServiceReq):
        receivedAction = sensorServiceReq.action    #  str.lower(sensorServiceReq.action)
        receivedPayload = sensorServiceReq.payload  # str.lower(sensorServiceReq.payload)
        rospy.logdebug("Received service request.")
        rospy.logdebug("Action: %s", receivedAction)
        rospy.logdebug("Payload: %s",receivedPayload)

        if   receivedAction== 'list':
            ans=self.performListAct(receivedPayload )
        elif receivedAction== 'get':
            ans=self.performGetAct(receivedPayload )
        else:
            ans=self.createErrorResponse('Unknown action: '+ sensorServiceReq.action)

        self.periodicPublish('')

        return ans

    def periodicPublish(self,evt):
        #forward last data from openhab
        houseTopicData=list()
        robotTopicData =list()
        sep=','
        for sensorName in self.getSensorList():
            values=self.getSensorVals(sensorName)
            if 'Robot' in sensorName:
                robotTopicData.append(sensorName)
                robotTopicData.append(values)
            else:
                houseTopicData.append(sensorName)
                houseTopicData.append(values)

        self.house_sens_pub.publish(sep.join(houseTopicData))
        self.robot_sens_pub.publish(sep.join(robotTopicData))

        rospy.Timer(rospy.Duration(self.publishTime), self.periodicPublish,oneshot=True)

    def performListAct(self,payload):
        '''
         Returns a list of sensors at available locations
         :param payload: 'sensors'
         :return: filled srv response with proper list
         '''

        if payload == 'sensors':
            ans = self.createOkResponse(self.getSensorList())
        else:
            ans = self.createErrorResponse('Unknown payload for list action:' + payload)
        return ans

    def performGetAct(self,payload):
        if payload not in self.getSensorList():
            ans = self.createErrorResponse('Sensor not found in server:' + payload)
        else:
            strValue = self.getSensorVals(payload)
            ans = self.createOkResponse(strValue)
        return ans


    def getSensorVals(self,sensorName):
        ans='0'
        sep=','
        ohSensorNames=self.parseNameMongo(sensorName)
        # Multi has Presence,Temp,Lux
        if 'Multi' in sensorName:
            try:
                ans = sep.join([str(self.openHABdata[ohSensorNames[0]]),
                                '{0:.1f}'.format(float(self.openHABdata[ohSensorNames[1]])),
                                '{0:.0f}'.format(float(self.openHABdata[ohSensorNames[2]]))
                                ])
            except KeyError:
                ans = '0,0.0,0'
        elif 'air' in sensorName:
            try:
                ans = self.getAirQuality()
            except KeyError:
                ans = 'safe'
        elif 'Power' in sensorName:
            try:
                ans = '{0:.1f}'.format(float(self.openHABdata[ohSensorNames[0]]))
            except KeyError:
                ans = '0.0'
        else:
            try:
                ans = str(self.openHABdata[ohSensorNames[0]])
            except KeyError:
                ans = '0'
        return ans

    def getAirQuality(self):
        # air may be comfort,safe,alarm
        ans='comfort'

        coQuality=self.getQuality('Env_cO','ambient-CO')

        partQuality=self.getQuality('Env_particleCount','ambient-particleCount')

        vocQuality=self.getQuality('Env_vOCResistance','ambient-VOC')

        if (coQuality == 'alarm') or  (partQuality == 'alarm') or (vocQuality == 'alarm'):
            ans='alarm'
        elif (coQuality == 'safe') or  (partQuality == 'safe') or (vocQuality == 'safe'):
            ans = 'safe'

        return ans


    def getAirQualityValue(self,ohName):
        ans=float(self.openHABdata[ohName])

        if 'particle' in ohName:
            humidity=float(self.openHABdata['Env_humidity'])
            if humidity<5.0:
                humidity=50
            ans=self.pcs2ugm3(ans,humidity)
        if 'Resistance' in ohName:
            if self.rav!=0.0:
                ans = 100.0 * float(self.openHABdata[ohName]) / self.rav
            else:
                ans = 79
        return ans

    def getQuality(self,ohEntry,NCPentry):
        # air may be comfort,safe,alarm
        ans = 'alarm'

        ohValue=self.getAirQualityValue(ohEntry)


        safe_minRange=float(self.NCPRangesDict[NCPentry][0])
        safe_maxRange=float(self.NCPRangesDict[NCPentry][1])
        comfort_minRange=float(self.NCPRangesDict[NCPentry][2])
        comfort_maxRange=float(self.NCPRangesDict[NCPentry][3])

        # we asume comfort ranges INSIDE safe ranges
        #   RANGE:           safe_minRange     comfort_minRange  comfort_maxRange         safe_maxRange
        #                        |                    |                 |                     |
        #   OUTPUT: ALARM........[....S.A.F.E.........[...C.O.M.F.O.R.T.].....S.A.F.E.........]....ALARM...
        #

        if (ohValue >= comfort_minRange) and (ohValue < comfort_maxRange):
            ans = 'comfort'
        elif (ohValue>=safe_minRange) and (ohValue<safe_maxRange):
            ans = 'safe'
        else:
            ans='alarm'

        #rospy.logdebug('Sensor ['+ohEntry+'] has value:'+str(ohValue))
        #rospy.logdebug('Comfort range: '+str(comfort_minRange)+' - '+str(comfort_maxRange))
        #rospy.logdebug('Safe range: '+str(safe_minRange)+' - '+str(safe_maxRange))
        #rospy.logdebug('STATUS is ......................................................>' + ans)

        return ans

    def loadDataNCP(self):
        prefix = 'ambient-'  # environmental parameters have this prefix on config file
        # login ...............................................................................................................

        myHeader = {'Content-Type': 'application/json', 'Accept': 'application/json'}
        loginData = {'user': self.userName, 'password': self.myPass}

        # (remember to get cookie for transactions)
        r = requests.post(self.urlLogIn, data=json.dumps(loginData), headers=myHeader, verify=False)

        rospy.logdebug('Login ........' + '\n')
        rospy.logdebug(str(r.status_code))
        if (r.status_code != requests.codes.ok):
            rospy.logerr("Cant log into " + self.urlLogIn)
            exit()
        else:
            rospy.logdebug('Login successful: ' + str(r.cookies))
            cuki = r.cookies

        # getting latest care plan  ...........................................................................................
        rospy.logdebug('Getting latest care plan ........')

        r = requests.get(self.urlCareplan, headers=myHeader, verify=False, cookies=cuki)
        rospy.logdebug(str(r.status_code))
        carePlan = r.json()
        rospy.logdebug(r.json())
        revision = carePlan['care-plan']['revision']
        rospy.logdebug('On revision ' + revision)

        # getting values .......................................................................................................
        rospy.logdebug('Getting ambient ranges........')
        self.NCPRangesDict = dict()

        for entry in carePlan['care-plan']['threshold']:
            if prefix in entry['name']:
                minSafe = self.parseEntry(entry, 'safe-min', -np.Inf)
                maxSafe = self.parseEntry(entry, 'safe-max')
                minComfort = self.parseEntry(entry, 'comfort-min', -np.Inf)
                maxComfort = self.parseEntry(entry, 'comfort-max')
                name = entry['name']
                self.NCPRangesDict[name]=(minSafe,maxSafe,minComfort,maxComfort)


    def pcs2ugm3(self,concentration_pcs, humidity_percent):
        '''
        Casts particles per cubic feet to micrograms per cubic meter

        Based on ,,Preliminary Screening System for Ambient Air Quality in Southeast Philadelphia,,
        http://www.cleanair.org/sites/default/files/Drexel%20Air%20Monitoring_-_Final_Report_-_Team_19_0.pdf
        :param concentration_pcs: concentration in  particles/0.01 cf
        :return: concentration in  ug/m3
        '''

        # 1. get mass:
        # All particles are spherical, with a density of 1.65E12 ug/m3
        # The average radius of an spherical particle in the PM2.5 channel is .44 um
        # mass = density * 4/3 * pi r^3

        mass25 = 1.65 * (4 / 3) * math.pi * pow(4.4, 3) * math.pow(10, -9)  # ug/particle

        cubicFootToMeters = 3531.5  # 0.01cf / m^3

        concentration_ugm3 = cubicFootToMeters * mass25 * concentration_pcs  # 0.01cf/m3 * ug/particle * particle/0.01cf

        # apply correction factor
        correctionFactor = {39: 13, 49: 9, 59: 6, 69: 9, 79: 9, 89: 9, 100: 1}
        correctionFactor = collections.OrderedDict(sorted(correctionFactor.items()))

        correction = -1
        for (refHumidity, val) in correctionFactor.iteritems():
            correction = val
            if (humidity_percent < refHumidity):
                break

        return concentration_ugm3 * humidity_percent * correction / 100



    def parseEntry(self,dict, key, defaultVal=np.Inf):
        val = defaultVal
        try:
            val = int(dict[key])
        except KeyError:
            pass
        return val

    def createOkResponse(self, data):
        '''
        Embeds a sequence of strings to a ROL srv response
        :param data: sequence to be joined and stored in srv response
        :return: filled srv response
        '''

        separator = ","

        ans = sensorServiceResponse()
        if (type(data)==list):
            ans.response = separator.join(data)
        else:
            ans.response = (data)
        ans.wasOk = True
        ans.feedback = ''

        return ans

    def createErrorResponse(self, data):
        '''
        Returns a srv response describing an error
        :param data: error description
        :return: filled srv response
        '''

        ans = sensorServiceResponse()
        ans.response = ''
        ans.wasOk = False
        ans.feedback = data

        return ans

    def rosSetup(self):

        self.houseTopic=rospy.get_param('houseTopic','house_sensors')
        self.robotTopic=rospy.get_param('robotTopic','robot_sensors')
        self.publishTime=int(rospy.get_param('publishPeriod','10'))

        # NCP parameters
        self.targetCD=str(rospy.get_param('/targetCD','123456'))
        self.userName = rospy.get_param('/NCPuserName', 'manuel.carmona')
        self.myPass = rospy.get_param('/NCPPass', 'Prova!2016')
        self.baseURL = rospy.get_param('/NCPbaseURL', 'https://secure.tesanonline.it/pac/api')

        self.urlLogIn = self.baseURL + '/account/login'
        self.urlLogout = self.baseURL + '/account'
        self.urlCareplan = self.baseURL + '/careplan/' + self.targetCD + '/ENRI'

    def waitForTopic(self,topicFullName):
        isRunning = False

        while (not isRunning):
            currentTopics=rospy.get_published_topics()
            # flatten them
            currentTopics=[item for sublist in currentTopics for item in sublist]
            print currentTopics
            isRunning= (topicFullName in currentTopics)
            if not isRunning:
                rospy.loginfo('Topic '+topicFullName+' not found. Waiting ...')
                rospy.sleep(2.)
        return

    def waitForNode(self,nodeName):
        isRunning = False

        while (not isRunning):
            currentNodes=    rosnode.get_node_names()
            #currentNodes=[item for sublist in currentNodes for item in sublist]
            print currentNodes
            isRunning= (nodeName in currentNodes)
            if not isRunning:
                rospy.loginfo('Node '+nodeName+' not found. Waiting ...')
                rospy.sleep(2.)
        return


    def openhabUpdatesCB(self,data):
        #rospy.logdebug('Recived update from %s, and I am %s',data.key,self.sensorName)
        if data.value=='OPEN':
            self.openHABdata[data.key]=1
        elif data.value=='CLOSED':
            self.openHABdata[data.key]=0
        else:
            self.openHABdata[data.key] = data.value

        if 'Resistance' in data.key:
            self.updateAverageVOCR(float(data.value))

    def updateAverageVOCR(self,ri):
        tnow=rospy.get_time()
        tinc=tnow-self.tprev
        rinc=ri-self.rprev

        # rospy.logdebug("...............................................")
        # rospy.logdebug("X(t-1)="+str(self.rprev)+"("+str(self.tprev)+")")
        # rospy.logdebug("av x (t-1) =" + str(self.rav))
        # rospy.logdebug("")
        # rospy.logdebug("X(t)="+str(ri)+"("+str(tnow)+")")
        # rospy.logdebug("AX(At)="+str(rinc)+"("+str(tinc)+")")
        # rospy.logdebug("")

        if self.tprev!=-1:
            self.rav=(self.rav*self.tprev+tinc*rinc)/tnow
        else:
            self.rav=ri

        # rospy.logdebug("av x (t) =" + str(self.rav))
        # rospy.logdebug("...............................................")


        self.tprev=tnow
        self.rprev=ri

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
        self.openHABdata=dict()

        # subscribe to openhab updates
        self.openHABupdates_sub=rospy.Subscriber("/openhab_updates", KeyValue, self.openhabUpdatesCB)

        # fill in the list: rostopic pub /openhab_command diagnostic_msgs/KeyValue ROS_COMMAND REFRESH
        self.openHABcommand_pub = rospy.Publisher("/openhab_command", KeyValue, queue_size=10)
        command=KeyValue()
        command.key='ROS_COMMAND'
        command.key = 'REFRESH'
        self.openHABcommand_pub.publish(command)

        # create a sensor list
        self.sensorList=list()
        itemList=self.getOpenHABItems()
        for item in itemList:
            newName=self.parseName(item['name'])
            if ('Door' in newName) or ('Power' in newName) or ('Multi' in newName) or ('Robot' in newName):
                if not (newName in self.sensorList):
                    self.sensorList.append(newName)
        self.sensorList=sorted(self.sensorList)

    def getSensorList(self):
        return self.sensorList



    def parseName(self,strMongoName):
        ans=''
        elements = strMongoName.split('_')
        if (elements[0] == 'Env'):
            elements[0] = 'Robot'

        if (elements[1] == 'cO' or elements[1] == 'vOCResistance' or elements[1] == 'particleCount' or elements[1] == 'timestamp'):
            elements[1]='air'
        elif (elements[1] == 'Plug' ):
            elements[1]='Power'

        ans = elements[0] + '-' + elements[1]



        return ans


    def parseNameMongo(self,strName):
        ans=list()
        elements = strName.split('-')
        if (elements[0] == 'Robot'):
            elements[0] = 'Env'

        if (elements[1] == 'Multi'):
            elements[1] = 'Multi_Presence'
            ans.append(elements[0] + '_' + elements[1])
            elements[1] = 'Multi_Temp'
            ans.append(elements[0] + '_' + elements[1])
            elements[1] = 'Multi_Lux'
            ans.append(elements[0] + '_' + elements[1])

        elif (elements[1] == 'air'):
            elements[1] = 'cO'
            ans.append(elements[0] + '_' + elements[1])
            elements[1] = 'vOCResistance'
            ans.append(elements[0] + '_' + elements[1])
            elements[1] = 'particleCount'
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

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.tprev = -1
        self.rprev = 0
        self.rav = 0

        rospy.loginfo("Advertising environmental data service ")

        #sleep for 2 seconds to be sure tag nodes are running
        rospy.sleep(2.)

        #check that node iot_bridge is running and topic  /openhab_updates exits
        openHABTopic = '/openhab_updates'
        self.waitForTopic(openHABTopic)

        #check that env_sensor is running and topic  /environment_sensor exits
        envTopic = '/environment_sensor'
        self.waitForTopic(envTopic)

        #check that amb2openhab is running
        requestedNode='/amb2openhab_node'
        self.waitForNode(requestedNode)


        #get sensor list from iot_bridge
        self.createSensorList()

        # get other parameters from ros
        self.rosSetup()

        # load NCP care plan
        self.loadDataNCP()

        # create topics for service requests
        self.house_sens_pub = rospy.Publisher(self.houseTopic, String, queue_size=10)
        self.robot_sens_pub = rospy.Publisher(self.robotTopic, String, queue_size=10)


        # start service callback
        self.s=rospy.Service('sensor_server', sensorService, self.sensorServiceCallback)


        rospy.loginfo("Ready...")
        rospy.spin()


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('sensor_server', log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        r_s = sensor_server_node()
    except rospy.ROSInterruptException: pass


