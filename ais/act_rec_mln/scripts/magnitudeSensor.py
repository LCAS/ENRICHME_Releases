import pandas as pd
import datetime
from datetime import timedelta
from sensor import Sensor
import rospy
import pytz
from tzlocal import get_localzone  # $ pip install tzlocal

class MagnitudeSensor(Sensor):
    def __init__(self,sensorName, mongoServer, mongoPort, mongoDBName, mongoCollName):
        Sensor.__init__(self,sensorName, mongoServer, mongoPort, mongoDBName, mongoCollName)

        self.avValueAtInterval = 0
        self.isOld = False

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
        
        #rospy.logdebug(".......................")
        #rospy.logdebug("%s: ",self.sensorName)
        #rospy.logdebug("From: %s",dbStartDate)
        #rospy.logdebug("To:   %s",dbEndDate)
        
        # first element: last entry before start date
        cursor = self.coll.find({"$and": [{"item": self.sensorName}, {"timestamp": {"$lte": dbStartDate}}]}).sort(
            [("timestamp", -1)]).limit(1)

        # database is not that old...
        if cursor.count() == 0:
            rospy.logerr('Database started after requested start date...')
            return

        #rospy.logdebug("Prev state was at: %s",cursor[0]['timestamp'])
		
        # our first entry will be value at starting time, either because is on database or we asume it's kept
        currVal = self.castValue(cursor.next()['value'])                
        currDate =self.createOffsetAwareDate(pd.Timestamp(dbStartDate))
        val.append(currVal)
        tm.append(currDate)
		
        #rospy.logdebug("Value:             %s",str(currVal))
        
        # rest of entries ...
        cursor = self.coll.find({"$and": [{"item": self.sensorName}, {"timestamp": {"$gte": dbStartDate, "$lte": dbEndDate}}]}).sort([("timestamp", -1)])

        #rospy.logdebug("Rest of entries are:")
        for document in cursor:
            currVal = self.castValue(document['value'])
            currDate = self.createOffsetAwareDate(pd.Timestamp(document['timestamp']))
            #rospy.logdebug("entry[%s]=%s",currDate,currVal)
            val.append(currVal)
            tm.append(currDate)

        # if we don't have data on endDate, we duplicate last entry on end date
        # also, if no document was found on the interval, we add again the value previous to interval
        if currDate != dbEndDate:
            currVal = val[-1]
            currDate=self.createOffsetAwareDate(pd.Timestamp(endDate))
            val.append(currVal)
            tm.append(currDate)            
            #rospy.logdebug("Assuring last entry[%s]=%s",endDate,currVal)

        self.timeSerie = pd.Series(val, tm)

        data = self.timeSerie.resample('1S', fill_method="backfill")

        self.avValueAtInterval=data.mean()
        
        
    def updateStats(self):
        cursorMax = self.coll.find({"item": self.sensorName}, {'value': 1, 'timestamp':1,'_id': 0}).sort([('value', -1)])
        self.maxVal = cursorMax[0]['value']              
        self.totalPoints = cursorMax.count()        
        self.minVal = cursorMax[self.totalPoints-1]['value']

        val=[]
        tm=[]
        for document in cursorMax:
            currVal = self.castValue(document['value'])
            currDate = self.createOffsetAwareDate(pd.Timestamp(document['timestamp']))
            #rospy.logdebug("entry[%s]=%s",currDate,currVal)
            val.append(currVal)
            tm.append(currDate)	
        
        self.avValue = pd.Series(val, tm).mean()

    def getDiscreteLevel(self):
        # not sure about updating internal stats every time...
        self.updateStats()

        discreteLevel='Low'
        # TODO these conversion thresholds should go to file...
        if self.avValueAtInterval>0.5*self.avValue:
            discreteLevel = 'Hig'
        elif self.avValueAtInterval>0.4*self.avValue:
            discreteLevel = 'Mid'        

        return discreteLevel

    def __str__(self):
        ans =  '{0: <26}'.format("[" + self.sensorName + "]:")
        try:
           timeStart = self.timeSerie.keys()[0]
           timeEnd = self.timeSerie.keys()[self.timeSerie.count() - 1]
        
        
        
           timeStart=timeStart.tz_convert(pytz.UTC)
           timeEnd=timeEnd.tz_convert(pytz.UTC)
           localzone=get_localzone()
		
           ans += " Av val(#" + "{0:> 2.0f}".format(self.timeSerie.count()) + ") " + "{0:> 2.2f}".format(self.avValueAtInterval) + " from " + "{0:> 2.6f}".format(self.avValue) +'('+"{0:> 3.0f}".format(self.totalPoints)+')'+ " == " +self.getDiscreteLevel()

           dif=timeEnd-timeStart
           ans += ' - Time range ' + localzone.normalize(timeStart).strftime("%d/%m/%y - %H:%M:%S") + ' to ' + localzone.normalize(timeEnd).strftime("%d/%m/%y - %H:%M:%S") + ' = ' + str(dif.seconds) + ' secs. '
        except AttributeError:
            ans += "[ no data ]"

        return ans
