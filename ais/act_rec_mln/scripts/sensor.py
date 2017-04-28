
from pymongo import MongoClient
import pytz
from tzlocal import get_localzone  # $ pip install tzlocal
import datetime

class Sensor:
    def __init__(self,sensorName, mongoServer, mongoPort, mongoDBName, mongoCollName):
        self.sensorName=sensorName
        self.mongoServer=mongoServer
        self.mongoPort=mongoPort
        self.mongoDBName=mongoDBName
        self.mongoCollName=mongoCollName

        self.client = MongoClient(host=self.mongoServer, port=self.mongoPort)

        self.db = self.client[self.mongoDBName]
        self.coll = self.db[self.mongoCollName]

    def updateData(self, timeRangeSecs):
        now = self.createOffsetAwareDate(datetime.datetime.utcnow())
        self.updateData(self, timeRangeSecs, now)

    def updateData(self, timeRangeSecs, endDate):
        print(' SENSOR.updateData: THIS METHOD SHOULD BE OVERRIDEN!!!')
        exit()

    def getDiscreteLevel(self):
        print(' SENSOR.getDiscreteLevel: THIS METHOD SHOULD BE OVERRIDEN!!!')
        exit()
        return 0

    def getEvidence(self):
        evidence=''.join(['level(',self.getDiscreteLevel(),',',self.sensorName,')'])
        return evidence



    def utc_to_local(self,utc_dt):
        local_tz = get_localzone()
        local_dt = utc_dt
        local_dt = local_dt.replace(tzinfo=pytz.utc).astimezone(local_tz)
        return local_dt

    def local_to_utc(self, local_dt):
        utc_tz = pytz.utc
        utc_dt = local_dt
        utc_dt   = utc_dt.replace(tzinfo=pytz.utc).astimezone(utc_tz)
        return utc_dt


    def createOffsetAwareDate(self, naiveDate):
        try:
            utc = pytz.timezone('UTC')
            awareDate = utc.localize(naiveDate)
        except:
            awareDate = naiveDate.tz_convert(pytz.UTC)

        return awareDate