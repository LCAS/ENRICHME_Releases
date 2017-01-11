from ncp_event_notifier.srv import *
from ncp_event_notifier.msg import *
from diagnostic_msgs.msg import KeyValue
from pymongo import MongoClient
from std_msgs.msg import String
import rospy
import math
import collections



class Magnitude:

    def __init__(self,name,checkPeriod,targetCD):
        '''
        Monitors a magnitude in openhab and stored in mongo database. Magnitude should be inside comfort/safe ranges, otherwise is notified to ROS.
        If magnitude goes out of safe ranges a report is also sent.
        Mongo support is included to keep track of posible statistics and to offer unit casting in particle sesnor.
        :param name: magnitude name
        :param checkPeriod: period in seconds between value checks
        '''

        self.name=name
        self.targetCD=targetCD
        self.values=dict()
        self.comfortStatus=True
        self.safeStatus= True
        self.alarmWasSent = False
        self.checkPeriod=checkPeriod
        self.isMongoConfigured=False
        self.isAParticleCountSensor=False

        self.lastNCP_event = NCP_Event()
        self.lastNCP_event.ID = '00000000-0000-0000-0000-000000000000'

        self.openhab_sub = rospy.Subscriber("/openhab_updates", KeyValue, self.openhab_callback)

    def openhab_callback(self, data):
        #rospy.logdebug('Recived update from %s, and I am %s',data.key,self.sensorName)
        if data.key == self.sensorName:
            rospy.logdebug('Update from my sensor')
            #updateValues, but from topic
            self.lastValue = data.value
            self.particleSensorCast()
            # go on with habitual process
            self.updateAlarms()
            self.reactToAlarms()



    def configureMongo(self,sensorName, mongoServer, mongoPort, mongoDBName, mongoCollName):
        '''
        Sets up all mongodb parameters. SHOULD BE CALLED BEFORE calling start!!!
        :param sensorName: mangitude name in mongo database
        :param mongoServer: ip address of mongodb
        :param mongoPort: database port
        :param mongoDBName: database name
        :param mongoCollName: collection name
        :return: -
        '''
        self.sensorName=sensorName
        self.mongoServer=mongoServer
        self.mongoPort=mongoPort
        self.mongoDBName=mongoDBName
        self.mongoCollName=mongoCollName

        self.client = MongoClient(host=self.mongoServer, port=self.mongoPort)

        self.db = self.client[self.mongoDBName]
        self.coll = self.db[self.mongoCollName]
        self.isMongoConfigured = True

    def start(self):
        '''
        Starts monitoring a magnitude
        :return: -
        '''
        if self.isMongoConfigured:
            rospy.wait_for_service('ncp_event_operation')
            self.ens = rospy.ServiceProxy('ncp_event_operation', NCP_Event_operation)
            self.pub = rospy.Publisher('/env_monitor_status', String, queue_size=10)
            rospy.Timer(rospy.Duration(self.checkPeriod), self.periodicCheck)
        else:
            rospy.logerr("Magnitude can't start before configuring mongo database!")

    def setComfortRanges(self,minRange,maxRange):
        '''
        Optimal ranges. Subset of Safe ranges
        :param minRange: minimum comfortable value
        :param maxRange: maximum comfortable value
        :return: -
        '''
        self.comfort_minRange=minRange
        self.comfort_maxRange=maxRange

    def setSafeRanges(self,minRange,maxRange):
        '''
        Safe ranges.
        :param minRange: lower safe limit
        :param maxRange:  upper safe limit
        :return: -
        '''
        self.safe_minRange=minRange
        self.safe_maxRange=maxRange

    def updateValues(self):
        '''
        Connects to mongo database to retrieve latest value.
        :return: -
        '''

        cursor = self.coll.find({"item": self.sensorName}).sort([("timestamp", -1)]).limit(1)
        try:
            document = cursor.next()
            self.lastValue = document['value']
            self.particleSensorCast()
        except StopIteration:
            self.lastValue = float('NaN')
            rospy.logerr("Mongo database does not contain entries for item: "+self.sensorName)

    def particleSensorCast(self):
        if self.isAParticleCountSensor:
            cursor = self.coll.find({"item": self.humiditySensorName}).sort([("timestamp", -1)]).limit(1)
            document = cursor.next()
            humidityValue = document['value']
            self.lastValue=self.pcs2ugm3(self.lastValue,humidityValue)


    def updateAlarms(self):
        '''
        Tests if data is inside ranges, updating safe/comfort flags
        :return: -
        '''

        if (self.lastValue>=self.safe_minRange) and (self.lastValue<self.safe_maxRange):
            self.safeStatus = True
            self.alarmWasSent = False
        else:
            self.safeStatus = False
            rospy.logdebug('Unsafe value (%s). Sensor %s safe range is (%s,%s)',str(self.lastValue),self.sensorName,str(self.safe_minRange),str(self.safe_maxRange))

        # we assume confort ranges are inside safe values, de do not need to reset report flag
        if (self.lastValue >= self.comfort_minRange) and (self.lastValue < self.comfort_maxRange):
            self.comfortStatus = True
        else:
            self.comfortStatus= False
            rospy.logdebug('Uncomfortable value (%s). Sensor %s comfort range is (%s,%s)',str(self.lastValue),self.sensorName,str(self.comfort_minRange),str(self.comfort_minRange))


    def setHumidityMongoSensor(self,mongoSensorHumidity):
        self.isAParticleCountSensor=True
        self.humiditySensorName=mongoSensorHumidity

    def sendAlarmReport(self):
        '''
        Sends to NCP an Alarm and publishses an Alarm in a ROS topic
        :return: -
        '''
        self.postNCPEvent()
        self.publishAlarm()

    def sendOutOfComfortReport(self):
        '''
        Publishes a warning in a ROS topic as an indication of discomfort
        :return: -
        '''
        self.publishOutOfConfort()

    def publishAlarm(self):
        '''
        Builds alarm message to be written on ros topic
        :return:
        '''
        rospy.logdebug("Sending alarm to robot")
        self.rosPublish("Alarm:"+self.name+":"+str(self.lastValue))

    def publishOutOfConfort(self):
        '''
        Builds warning message to be written on ros topic
        :return:
        '''
        rospy.logdebug("Sending Warning to robot")
        self.rosPublish("Warning:" + self.name + ":" + str(self.lastValue))

    def rosPublish(self,message):
        '''
        publishes a string in ros indicating alarm/warning
        :param message: string with format [Alarm|Warning]:magnitudeName:value
        :return: -
        '''
        self.pub.publish(message)

    def postNCPEvent(self):
        '''
        Send a new NCP event only if we haven't already sent it
        :return: -
        '''

        if not self.alarmWasSent:
            try:
                rospy.logdebug("Reporting to NCP")
                verb = 'POST'
                # Payload

                # this is a temp value. Real one is filled by NCP
                self.lastNCP_event.ID = '00000000-0000-0000-0000-000000000000'
                #user code
                self.lastNCP_event.TargetCD = self.targetCD
                # Ambient Data is Out-of-Range -
                self.lastNCP_event.TypeCD = 'ADOR'
                self.lastNCP_event.Begin = rospy.Time.now()
                # Static value to ULINAAL for the AAL server.
                self.lastNCP_event.srcDomain = 'ULINAAL'
                # Universal ID  identifying   the  event and invariable   from the installation  of  the AAL  server.
                self.lastNCP_event.srcId = 'house01'
                resp1 = self.ens(verb, self.lastNCP_event )

                # todo ensure transmission? store id for further use?...
                if resp1.wasOk:
                    self.lastNCP_event.ID = resp1.response.ID
                    self.alarmWasSent=True

            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))

    def periodicCheck(self,evt):
        '''
        Called every checkPeriod seconds, triggers data and flags updates and perform necessary message transmissions.
        :return: -
        '''
        self.updateValues()
        self.updateAlarms()
        self.reactToAlarms()

    def reactToAlarms(self):
        if (not self.safeStatus and not self.comfortStatus):
            self.sendAlarmReport()
        if (self.safeStatus and not self.comfortStatus):
            self.sendOutOfComfortReport()
        if (not self.safeStatus and self.comfortStatus):
            rospy.logdebug("An unsafe value is not suposed to be confortable...")

    def pcs2ugm3(self,concentration_pcs, humidity_percent):
        '''
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
