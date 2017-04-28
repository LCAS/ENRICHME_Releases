import pandas as pd
import datetime
from datetime import timedelta
from sensor import Sensor
import rospy
from tzlocal import get_localzone  # $ pip install tzlocal
import pytz
import math

class MagnitudeSensor(Sensor):
    def __init__(self,sensorName, mongoServer, mongoPort, mongoDBName, mongoCollName,t_evidence,t_sample,maxSystemEntr):
        Sensor.__init__(self,sensorName, mongoServer, mongoPort, mongoDBName, mongoCollName,t_evidence,t_sample,maxSystemEntr)
        self.avValueAtInterval = float('nan')
        self.avValue= float('nan')
        self.isOld = False
        self.prob=0


    def castValue(self,val):
        return val


    def updateData(self, timeRangeSecs, endDate):
        d = timedelta(seconds=timeRangeSecs)
        self.endDate = endDate
        self.startDate =self.endDate - d

        dbEndDate = (self.endDate)
        dbStartDate = (self.startDate)

        val = []
        tm = []

        # first element: last entry before start date
        cursor = self.coll.find({"$and": [{"item": self.sensorName}, {"timestamp": {"$lte": dbStartDate}}]}).sort(
            [("timestamp", -1)]).limit(1)

        # database is not that old...
        if cursor.count() == 0:
            rospy.logerr('Database started after requested start date...')
            return

        # our first entry will be value at starting time, either because is on database or we asume it's kept
        currVal = self.castValue(cursor.next()['value'])
        currDate = (dbStartDate)
        val.append(currVal)
        tm.append(currDate)

        # rest of entries ...
        cursor = self.coll.find({"$and": [{"item": self.sensorName}, {"timestamp": {"$gte": dbStartDate, "$lte": dbEndDate}}]}).sort(
            [("timestamp", -1)])

        for document in cursor:
            currVal = self.castValue(document['value'])
            currDate = self.createOffsetAwareDate(pd.Timestamp(document['timestamp']))
            val.append(currVal)
            tm.append(currDate)

        # if we don't have data on endDate, we duplicate last entry on end date
        # also, if no document was found on the interval, we add again the value previous to interval
        if currDate != dbEndDate:
            currVal = val[-1]
            val.append(currVal)
            tm.append(pd.Timestamp(endDate))

        self.timeSerie = pd.Series(val, tm)

        data = self.timeSerie.resample('1S', fill_method="backfill")

        self.avValueAtInterval=data.mean()


    def __str__(self):
        ans='[No data]'
        try:
            timeStart = self.timeSerie.keys()[0]
            timeEnd = self.timeSerie.keys()[self.timeSerie.count() - 1]

            timeStart=timeStart.tz_convert(pytz.UTC)
            timeEnd=timeEnd.tz_convert(pytz.UTC)
            dif=timeEnd-timeStart
            localzone=get_localzone()

            ans =  '{0: <26}'.format("[" + self.sensorName + "]:")
            ans += " Rel. Entr. % " + "{0:> 2.2f}".format(100.0*self.sensorEntropy/self.maxSystemEntropy)
            ans += " Prob. % " + "{0:> 2.2f}".format(100.0*self.prob)
            #ans += " - Av. val "  + "{0:> 2.2f}".format(self.avValueAtInterval)
            ans += " - Act. time "  + "{0:> 2.2f}".format(self.avValueAtInterval*dif.seconds)
            ans += ' - Time range ' + str(dif.seconds) + ' secs.' 
            ans += ' starting at ' + localzone.normalize(timeStart).strftime("%d/%m/%y - %H:%M:%S") 

        except AttributeError:
            pass

        return ans


    #def debug(self):
        '''
        Sends to connection debug data from this sensed magnitude:
            [sensor name, av active time, entropy, interval begin, interval end, points used, points]
        :return:
        '''

        #vector=[self.sensorName,self.avValueAtInterval,self.sensorEntropy
