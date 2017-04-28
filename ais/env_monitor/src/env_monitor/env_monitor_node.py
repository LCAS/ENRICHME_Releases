#!/usr/bin/env python

from env_monitor.srv import *
from env_sensor.msg import Environment
from datetime import date
import requests
import json
import numpy as np
import rospy
from magnitude import Magnitude
import yaml

'''
    This package checks ambient sensors and reports values out of ranges

          RANGES
     Comfort   Safe        Action
       In       In       None. Everything ok
       Out      In       Acceptable. Robot suggestion. Do not report to NCP
       In       Out      Impossible. Potential failure (Are you unsafely confortable??)
       Out      Out      Alarm situation

    To ensure a good configuration comfort should be a subrange INSIDE safe ranges

        Sensor: CO
        Unit: PPM
        Ranges:
        - Comfort (0, 60)
        - Safe    (0, 70)
        See: http://www.medicinenet.com/script/main/art.asp?articlekey=21837

        Sensor: Particles
        Unit: ug/m3 ( sensor reports in particles/0.01cf!!! )
        Ranges:
        - Comfort (0, 20)
        - Safe    (0, 25)
        See: http://ec.europa.eu/environment/air/quality/standards.htm

        Sensor:  VOC
        Unit: KOhms
        Ranges:
        - Comfort (85, 115)
        - Safe    (65, 135)
        See: www.imagesco.com/catalog/sensors/files/TGS2602_datasheet.pdf
             It should be always lower than resistance in clean air Ro (20deg 65RH).
             Any drop bigger than 85% should indicate cleaner indication, thus a single cigarrete drop to 65%
             Checking values normally we have 80khOhms so 85% is about 65khoms

        Sensor:  Temperature
        Unit:
        Ranges:
        - Comfort  (18, 24)
        - Safe     (16, 24)
        See:  ,,Cold Weather Plan,,
              https://www.gov.uk/government/uploads/system/uploads/attachment_data/file/464858/KWKW_2015.pdf
              http://ieh.cranfield.ac.uk/ukieg/restricted/191004pres/Rudge.pdf


        Sensor:  Humidity
        Unit:  %
        Ranges:
        - Comfort   (30, 50)
        - Safe      (30, 60)
        See:  http://www.doh.wa.gov/portals/1/Documents/Pubs/333-044.pdf

        Sensor:  Light
        Unit:  Lux
        Ranges:
        - Comfort (100, 250)
        - Safe    (-Inf,Inf)   ????
        See:  http://www.cibse.org/getmedia/8faea1b4-1cee-4fcb-93f0-37df26acab2d/Lighting-for-Residential-Buildings-(LG9)_2015.pdf.aspx
	          http://www.noao.edu/education/QLTkit/ACTIVITY_Documents/Safety/LightLevels_outdoor+indoor.pdf

'''




# Server Class ........................................................................................................
class env_monitor():
    def parseEntry(self,dict, key, defaultVal=np.Inf):
        val = defaultVal
        try:
            val = int(dict[key])
        except KeyError:
            pass
        return val


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
            rospy.logerr("Cant log into " + self.urlLogout)
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
        numThresholds = len(carePlan['care-plan']['threshold'])
        self.safeRangesList = []
        self.comfortRangesList = []
        self.magnitudeNamesList = []

        for entry in carePlan['care-plan']['threshold']:
            if prefix in entry['name']:
                minSafe = self.parseEntry(entry, 'safe-min', -np.Inf)
                maxSafe = self.parseEntry(entry, 'safe-max')
                minComfort = self.parseEntry(entry, 'comfort-min', -np.Inf)
                maxComfort = self.parseEntry(entry, 'comfort-max')
                updateTime = self.parseEntry(entry, 'updatePeriod')
                name = entry['name'][len(prefix):]

                self.safeRangesList.append([minSafe, maxSafe])
                self.comfortRangesList.append([minComfort, maxComfort])
                self.magnitudeNamesList.append(name)

    #def loadYAML(self):
    #    '''
    #    Association between sensors and magnitudes are described in this file.
    #    Each sensor entry contains three elements:
    #        {magnitude: NCP associated ambient threshold configuration. This links to sensor ranges
    #        updateTime: How often are magnitudes checked.
    #        mongoNames: List of mongodb item values that use this magnitude configuration.
    #        }
    #    :return:
    #    '''
       # yDict = dict()
       # self.sensorConfig=[]


       # with open(self.sensorConfigFile) as stream:
        #    try:
        #        yDict = (yaml.load(stream))
        #    except yaml.YAMLError as exc:
        #        print(exc)

       # for sensor in yDict[0]['sensorConfig']:
        #    self.sensorConfig.append(sensor)




    # class constructor.
    def __init__(self):
        rospy.init_node('env_monitor',log_level=rospy.DEBUG)

        # load ros parameters
        # mongo server location, database and collection
        # They are global config params, see global.yaml in ais_rosparam
        self.mongoServer=rospy.get_param("/ais_hostname", "192.168.1.182")
        self.mongoPort= int(rospy.get_param('/mongoPort', '27017'))
        self.mongoDBName=str(rospy.get_param('/domoticDBName','openhab'))
        self.mongoCollName=str(rospy.get_param('/domoticCollName','openhab'))
        #self.sensorConfigFile=str(rospy.get_param('~yamlConfigFile','../../cfg/sensorConfigFDG.yaml'))
        self.sensorConfig=rospy.get_param('~sensorConfig')

        #NCP parameters
        self.targetCD=str(rospy.get_param('/targetCD','123456'))
        self.userName = rospy.get_param('/NCPuserName','manuel.carmona')
        self.myPass = rospy.get_param('/NCPPass','Prova!2016')
        self.baseURL =rospy.get_param('/NCPbaseURL', 'https://secure.tesanonline.it/pac/api')

        self.urlLogIn = self.baseURL + '/account/login'
        self.urlLogout = self.baseURL + '/account'
        self.urlCareplan = self.baseURL + '/careplan/' + self.targetCD + '/ENRI'

        # load yaml parameters
        # data previously hardcoded
        # mongoSensorNameList =[ ['Env_cO'], ['Env_particleCount'],  ['Env_vOCResistance'],        ['Env_temp','Kitchen_Multi_Temp','Entry_Multi_Temp','Lounge_Multi_Temp','Workshop_Multi_Temp','Office1_Multi_Temp','Office2_Multi_Temp'],          ['Office1_Multi_Humid','Env_humidity'],         ['Env_light','Kitchen_Multi_Lux','Entry_Multi_Lux','Lounge_Multi_Lux','Workshop_Multi_Lux','Office1_Multi_Lux','Office2_Multi_Lux' ]      ]
        # updatePeriodList    =[ 10,          10,                     10,        30,          120,         300]
        #self.loadYAML()

        # load data from NCP
        # data previously hardcoded
        # magnitudeNamesList  =[ 'CO',        'particleCount',       'VOC',     'temp',      'humidity', 'light']
        # comfortRangesList   =[ [0, 60],    [0, 20],                [85,115],   [18,24],     [30,50],    [100,250] ]
        # safeRangesList      =[ [0, 70],    [0, 25],                [65,135],   [16,25],     [30,60],    [float('-Inf'),float('Inf')] ]
        self.loadDataNCP()


        self.magnitudesDict=dict()

        for i in range(0,len(self.magnitudeNamesList)):
            # According to data from NCP
            name=self.magnitudeNamesList[i]
            comfortRange=self.comfortRangesList[i]
            safeRange=self.safeRangesList[i]

            # Get updatePeriod and mongoList for magnitude
            for entry in self.sensorConfig:
                if entry['magnitude']==name:
                    updatePeriod=entry['updateTime']
                    mongoSensorNameL=entry['mongoNames']

                    for sensorName in mongoSensorNameL:
                        m = Magnitude(name, updatePeriod, self.targetCD)
                        m.setComfortRanges(comfortRange[0], comfortRange[1])
                        m.setSafeRanges(safeRange[0], safeRange[1])
                        # UEM sensor provides particle count in pp/0.01CF but most regulations offer values in ug/m3
                        # this tells magnitude class to cast values provided by low level sensor
                        if name is 'particleCount':
                            m.setHumidityMongoSensor('Env_humidity')
                        m.configureMongo(sensorName, self.mongoServer, self.mongoPort, self.mongoDBName,
                                         self.mongoCollName)
                        m.start()
                        self.magnitudesDict[sensorName] = m


        rospy.loginfo('Environment monitor server now active')
        rospy.spin()


# Main function........................................................................................................
if __name__ == "__main__":
    env_monitor()


