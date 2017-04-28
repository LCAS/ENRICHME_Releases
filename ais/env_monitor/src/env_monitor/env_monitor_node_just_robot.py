#!/usr/bin/env python

from env_monitor.srv import *
from env_sensor.msg import Environment
import collections
from std_msgs.msg import String
import rospy
import math
from datetime import date


'''

        # see http://www.medicinenet.com/script/main/art.asp?articlekey=21837
        # values are PPM
        self.cOMagnitude= magnitude()
        self.cOMagnitude.setSafeRanges(0,70)
        self.cOMagnitude.setAlarmRanges(71, Inf)

        #  http: // ec.europa.eu / environment / air / quality / standards.htm
        # these limits are in ug/m3 and sensor reports in particles/0.01cf!!!
        self.particleCountMagnitude= magnitude()
        self.particleCountMagnitude.setSafeRanges(0,25)
        self.particleCountMagnitude.setAlarmRanges(26, Inf)

        # www.imagesco.com/catalog/sensors/files/TGS2602_datasheet.pdf
        # VOC are reported as kilohms in Rs. It should be always lower than resistance in clean air Ro (20deg 65RH).
        # Any drop bigger than 0.85 should indicate cleaner indication, thus a single cigarrete drop to 0.65
        # Checking values normally we have 80khOhms so 0.85 is about 65khoms
        self.vOCResMagnitude= magnitude()
        self.vOCResMagnitude.setSafeRanges(65, 80)
        self.vOCResMagnitude.setAlarmRanges(0, 65)

	# ‘Cold Weather Plan’   https://www.gov.uk/government/uploads/system/uploads/attachment_data/file/464858/KWKW_2015.pdf
        # minimum of 18, recommended 21
        # Healthy temperatures under 24  http://ieh.cranfield.ac.uk/ukieg/restricted/191004pres/Rudge.pdf
        #
        self.tempMagnitude= magnitude()
        self.tempMagnitude.setSafeRanges(18, 24)
        self.tempMagnitude.setAlarmRanges(-Inf, 18)
        self.tempMagnitude.setAlarmRanges(25, Inf)

        # http://www.doh.wa.gov/portals/1/Documents/Pubs/333-044.pdf
        self.humidityMagnitude= magnitude()
        self.humidityMagnitude.setSafeRanges(30, 60)
        self.humidityMagnitude.setAlarmRanges(0, 30)
        self.humidityMagnitude.setAlarmRanges(61,100)

	# http://www.cibse.org/getmedia/8faea1b4-1cee-4fcb-93f0-37df26acab2d/Lighting-for-Residential-Buildings-(LG9)_2015.pdf.aspx
	# http://www.noao.edu/education/QLTkit/ACTIVITY_Documents/Safety/LightLevels_outdoor+indoor.pdf
	# no alarm but recommended between 100-250 luxes
        self.lightMagnitude= magnitude()
        self.lightMagnitude.setSafeRanges(100, 250)

'''






def pcs2ugm3(concentration_pcs,humidity_percent):
    '''
    Based on "Preliminary Screening System for Ambient Air Quality in Southeast Philadelphia
    http://www.cleanair.org/sites/default/files/Drexel%20Air%20Monitoring_-_Final_Report_-_Team_19_0.pdf
    :param concentration_pcs: concentration in  particles/0.01 cf
    :return: concentration in  μg/m3
    '''

    #1. get mass:
    #All particles are spherical, with a density of 1.65E12 µg/m3
    #The average radius of an spherical particle in the PM2.5 channel is .44 µm
    # mass = density * 4/3 * pi r^3

    mass25 = 1.65 * (4/3) * math.pi * pow (4.4, 3) * math.pow (10, -9)  # ug/particle

    cubicFootToMeters = 3531.5 # 0.01cf / m^3

    concentration_ugm3= cubicFootToMeters * mass25 * concentration_pcs # 0.01cf/m3 * ug/particle * particle/0.01cf

    #apply correction factor
    correctionFactor = {39: 13, 49: 9, 59: 6, 69: 9, 79: 9, 89: 9, 100:1}
    correctionFactor =collections.OrderedDict(sorted(correctionFactor.items()))

    correction = -1
    for (refHumidity,val) in correctionFactor.iteritems():
        correction = val
        if (humidity_percent<refHumidity):
            break



    return concentration_ugm3*humidity_percent*correction/100




class Magnitude:
    # TODO UNCOMPLETE CLASS
    def __init__(self,name):
        self.name=name
        self.values=dict()
        self.alarm.status=False
        self.warning.status= False

    def addVal(self,val,timestamp):
        self.values[timestamp]=val
        self.checkRanges()

    def setSafeRanges(self,minRange,maxRange):
        self.safe.minRange=minRange
        self.safe.maxRange=maxRange

    def setAlarmRanges(self,minRange,maxRange):
        self.alarm.minRange=minRange
        self.alarm.maxRange=maxRange

    def getAlarm(self):
        return self.alarm.status

    def getWarning(self):
        return self.warning.status

    def checkRanges(self):
        self.alarm.status   = True
        self.warning.status = True

# Server Class ........................................................................................................
class env_monitor():

    # class constructor.
    def __init__(self):
        global alarm

        rospy.init_node('env_monitor')

        magnitudeNamesList  =[ 'CO',                 'particleCount',      'VOC',     'temp',                                 'humidity',        'light']
        safeRangesList      =[ [[0,70]],             [[0,25]],             [[65,80]], [[18,24]],                              [[30,60]],         [[100,250]]]
        alarmRangesList     =[ [[70, float('Inf')]], [[25, float('Inf')]], [[0, 65]], [[float('-Inf'),18],[24,float('Inf')]], [[0,30],[60,100]], []]
        self.magnitudesDict=dict()
        for i in range(0,len(magnitudeNamesList)):
            name=magnitudeNamesList[i]
            m = Magnitude(name)
            safeRangesM=safeRangesList[i]
            for safeRange in safeRangesM:
                m.setSafeRanges(safeRange[0],safeRange[1])
            alarmRangesM = alarmRangesList[i]
            for alarmRange in alarmRangesM:
                m.setAlarmRanges(alarmRange[0], alarmRange[1])
            self.magnitudesDict[name]=m

        #s = rospy.Service('env_monitor_config', Env_Monitor_Configure, self.service_request)
        rospy.Subscriber("environment_sensor", Environment, self.envDataCallback)
        self.pub = rospy.Publisher('/env_alarms', String, queue_size=10)

        rospy.loginfo('Environment monitor server now active')
        rospy.spin()

    # CALLBACKS.............................................................................................................
    def envDataCallback(self,msg):
        alarm=False
        timestamp = msg.header.stamp
        self.magnitudesDict['CO'].addVal(msg.cO, timestamp)
        self.magnitudesDict['particleCount'].addVal(msg.particleCount, timestamp)
        self.magnitudesDict['VOC'].addVal(msg.vOCResistance, timestamp)
        self.magnitudesDict['temp'].addVal(msg.temp, timestamp)
        self.magnitudesDict['humidity'].addVal(msg.humidity, timestamp)
        self.magnitudesDict['light'].addVal(msg.light, timestamp)

        for m in self.magnitudesDict:
            if m.getAlarm():
                self.pub("Alarm value in "+m.name)




    def service_request(self,req):
        if req.verb.lower() == 'getParamsList':
            response = self.getParamRangeCallback(req)
        elif req.verb.lower() == 'setParamRange':
            response = self.setParamRangeCallback(req)
        elif req.verb.lower() == 'getParamRange':
            response = self.getParamRangeCallback(req)
        else:
            response = self.createResponse("Unknown verb \"" + req.verb + "\"", False)

        return response

    def getParamRangeCallback(req):
        #TODO UNCOMPLETE METHOD
        return response

    def setParamRangeCallback(req):
        # TODO UNCOMPLETE METHOD
        return response

    def getParamRangeCallback(req):
        # TODO UNCOMPLETE METHOD
        return response

    def createResponse(message, status):
        response = Env_Monitor_ConfigureResponse()
        response.response = ""
        response.wasOk = status
        response.feedback = message

        if not status:
            rospy.logerr(response.feedback)

        return response


# Main function........................................................................................................
if __name__ == "__main__":
    env_monitor()


