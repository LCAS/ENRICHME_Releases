
from pymongo import MongoClient
import pytz
from tzlocal import get_localzone  # $ pip install tzlocal
import datetime
import rospy

class Sensor:
    def __init__(self,sensorName, mongoServer, mongoPort, mongoDBName, mongoCollName,t_evidence,t_sample,maxSystemEntr):
        self.maxSystemEntropy=maxSystemEntr
        self.sensorName=sensorName
        self.mongoServer=mongoServer
        self.mongoPort=mongoPort
        self.mongoDBName=mongoDBName
        self.mongoCollName=mongoCollName
        self.tevidence=t_evidence
        self.tsample=t_sample

        self.client = MongoClient(host=self.mongoServer, port=self.mongoPort)

        self.db = self.client[self.mongoDBName]
        self.coll = self.db[self.mongoCollName]

        self.timer = rospy.Timer(rospy.Duration(self.tsample), self.periodicReport)

    def periodicReport(self,evt):
        self.updateDataNowDefault()

    def updateDataNowDefault(self):
        self.updateDataNow(self.tevidence)

    def updateDataNow(self, timeRangeSecs):
        now = self.createOffsetAwareDate(datetime.datetime.utcnow())

        self.updateData( timeRangeSecs, now )

    def updateData(self, timeRangeSecs, endDate):
        print(' SENSOR.updateData: THIS METHOD SHOULD BE OVERRIDEN!!!')
        exit()


    def createOffsetAwareDate(self, naiveDate):

        try:
            utc = pytz.timezone('UTC')
            awareDate = utc.localize(naiveDate)
        except:
            awareDate = naiveDate.tz_convert(pytz.UTC)

        return awareDate
