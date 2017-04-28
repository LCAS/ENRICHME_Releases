"""
Bar chart demo with probs and entropies
"""

import numpy as np
import matplotlib.pyplot as plt
import pytz
from pymongo import MongoClient
import datetime
import pandas as pd
from datetime import timedelta

def getEntropy(active_times):
    delta=0.001
    probs = np.zeros(active_times.size+1)
    probs[0:-1] = active_times+delta
    probs[-1]=np.sum(probs[0:-1] )

    if probs[-1] >0:
        probs = np.array(probs) / probs[-1]
    else:
        print 'No activations'

    max_entropy = np.log(len(probs[0:-1]))

    rel_entropies = (-probs * np.log(probs)) / max_entropy
    rel_entropies[-1] = sum(rel_entropies[0:-1])
    return (probs,rel_entropies)


def plot_data(active_times,locations):

        #active_times = [20.0, 5.0, 3.0, 35.0]

        (probs,rel_entropies)=getEntropy(active_times)

        #...........................................................

        locations.append('House')

        plot_groups = len(probs)

        fig, ax = plt.subplots()

        index = np.arange(plot_groups)
        bar_width = 0.35

        opacity = 0.4
        error_config = {'ecolor': '0.3'}


        rects1 = plt.bar(index, 100.0*probs, bar_width, alpha=opacity, color='b', label='Location Probability')

        rects2 = plt.bar(index+bar_width, 100.0*rel_entropies, bar_width, alpha=opacity, color='g', label='Relative Entropy')

        plt.xlabel('Locations')
        plt.ylabel('(%)')
        #plt.title('Probabilities and Relative entropy')
        plt.xticks(index + bar_width, locations)
        plt.legend(loc=2)
        ax.grid(True)

        plt.tight_layout()
        plt.show()

        #...............................................................................


#........................................................................

def castValue(currVal):

    if isinstance(currVal, unicode):
        if currVal == 'CLOSED':
            currVal = 0
        elif currVal == 'OPEN':
            currVal = 1
        else:
            print("Don't know how to handle ",currVal, " values")
            exit()

    return currVal


def createOffsetAwareDate( naiveDate):
    try:
        utc = pytz.timezone('UTC')
        awareDate = utc.localize(naiveDate)
    except:
        awareDate = naiveDate.tz_convert(pytz.UTC)

    return awareDate


def getData(sensorName, startDate, timeRangeMins):
    d = timedelta(minutes=timeRangeMins)
    endDate = startDate + d

    dbEndDate = (endDate)
    dbStartDate = (startDate)

    val = []
    tm = []

    # first element: last entry before start date
    cursor = coll.find({"$and": [{"item": sensorName}, {"timestamp": {"$lte": dbStartDate}}]}).sort(
        [("timestamp", -1)]).limit(1)

    # database is not that old...
    if cursor.count() == 0:
        print('Database started after requested start date...')
        exit()

    # our first entry will be value at starting time, either because is on database or we asume it's kept
    currVal = castValue(cursor.next()['value'])
    currDate = (dbStartDate)
    val.append(currVal)
    tm.append(currDate)

    # rest of entries ...
    cursor = coll.find(
        {"$and": [{"item": sensorName}, {"timestamp": {"$gte": dbStartDate, "$lte": dbEndDate}}]}).sort(
        [("timestamp", -1)])

    for document in cursor:
        currVal = castValue(document['value'])
        currDate = (pd.Timestamp(document['timestamp']))
        val.append(currVal)
        tm.append(currDate)

    # if we don't have data on endDate, we duplicate last entry on end date
    # also, if no document was found on the interval, we add again the value previous to interval
    if currDate != dbEndDate:
        currVal = val[-1]
        val.append(currVal)
        tm.append(pd.Timestamp(endDate))

    timeSerie = pd.Series(val, tm)

    data = timeSerie.resample('15S').backfill()
    return data
    #return timeSerie

# ..........................................................................................

def showCases():
    client = MongoClient(host='127.0.0.1', port=27017)

    #startDate = datetime.datetime(2016, 11, 9, 14, 20, 0, 0)
    timeRangeMins=30
    startDate = datetime.datetime(2016, 11, 10, 8, 00, 0, 0)

    active_times=[]
    datas=[]

    db = client['openhab']
    coll = db['openhab']

    itemNames = coll.distinct('item')

    presenceItems = list()
    for item in itemNames:
        if 'Presence' in item:
            presenceItems.append(item)

    presenceItems = ['Livingroom_Multi_Presence',
                     'Kitchen_Multi_Presence',
                     'Bedroom_Multi_Presence',
                     'Bathroom_Multi_Presence',
                     'Corridor_Multi_Presence']

    print 'Rooms ',presenceItems[0:4]

    for t in range(0,60):
        d2 = timedelta(minutes=timeRangeMins/2)
        startDate=startDate+d2
        active_times = []
        datas = []
        print 'Start Time: ', startDate
        for i in range(0,4):
            sensorName = presenceItems[i]
            #print 'Processing  ', sensorName
            data=getData(sensorName,startDate,timeRangeMins)
            datas.append(data)
            active_times.append( data.mean())

        actTime=np.array(active_times) * timeRangeMins

        (probs, rel_entropies) = getEntropy(actTime)
        print 'Active times ', actTime
        print 'Probs ', probs
        print 'Relative entropies ', rel_entropies
        print '........................'


#.........................................................................


client = MongoClient(host='127.0.0.1', port=27017)


timeRangeMins=30

# Low entropy 2016-11-09 15:50:00
startDateL = datetime.datetime(2016, 11, 9, 15, 50, 0, 0)
# High entropy 2016-11-09 16:20:00
startDateH = datetime.datetime(2016, 11, 9, 16, 20, 0, 0)
# Normal entropy 2016-11-10 09:30:00
startDateN = datetime.datetime(2016, 11, 10, 9, 30, 0, 0)

dateList=[]
dateList.append(startDateN )
dateList.append(startDateH )
dateList.append(startDateL )

active_times=[]
datas=[]

db = client['openhab']
coll = db['openhab']

presenceItems = ['Kitchen_Multi_Presence',
                 'Bathroom_Multi_Presence',
                 'Bedroom_Multi_Presence',
                 'Livingroom_Multi_Presence'
                 ]

locations=[ 'Kitchen', 'Bathroom', 'Bedroom','Livingroom']


print 'Rooms ',locations

for startDate in dateList:
    active_times = []
    datas = []
    print 'Start Time: ', startDate
    for i in range(0,4):
        sensorName = presenceItems[i]
        #print 'Processing  ', sensorName
        data=getData(sensorName,startDate,timeRangeMins)
        datas.append(data)
        active_times.append( data.mean())

    actTime=np.array(active_times) * timeRangeMins

    (probs, rel_entropies) = getEntropy(actTime)
    print 'Active times ', actTime
    print 'Probs ', probs
    print 'Relative entropies ', rel_entropies
    print '........................'
    plot_data(actTime, locations)
