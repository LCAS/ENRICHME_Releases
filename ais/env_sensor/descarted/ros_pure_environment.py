#!/usr/bin/env python

# based on https://github.com/LCAS/ENRICHME/blob/master/codes/ros_pure/scripts/ros_pure_bridge.py
#
# reads from the http webservice and publishes it into a custom msg, every 20 secs
# this is a draft:
#    - is this all the environmental info OK
#    - is web address correct   OK

#	 - is update frequency  reasonable?  Maybe check if timestamp has changed before publishing?



import rospy
from std_msgs.msg import Header
from ros_pure.msg import Environment
from requests import post
from requests import get

from xml.etree import ElementTree
import xmltodict

import json
import tf

#only used for filebased-Airbox
from ftplib import FTP
import os, errno
from datetime import datetime
import time

# functions . . . . . . . . . . . . . . . .  . . . . . . . . . . . . . . . . 
def timer_cb(evt):
    global url_env
    global got_first
    
    req = get(url_env)
    parsed_json = req.json()
   
    if (not got_first):
        print("Connection OK.")
        got_first = True

	#check this...
    msg = Environment()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "environment_sensor"
    msg.cO = parsed_json['cO']
    msg.particleCount = parsed_json['particleCount']
    msg.vOCResistance = parsed_json['vOCResistance']
    msg.temp = parsed_json['temp']
    msg.humidity = parsed_json['humidity']
    msg.light = parsed_json['light']
    msg.datetime = parsed_json['datetime']

    pub_env.publish(msg)

# . . . . . . . . . . . . . . . .  . . . . . . . . . . . . . . . . 

def getSecondField(twoFields,fieldSep):
    sepFields=twoFields.split(fieldSep)
    return float(sepFields[1])

# thanks to stackoverflow...
def silentremove(filename):
    try:
        os.remove(filename)
    except OSError as e: # this would be "except OSError, e:" before Python 2.6
        if e.errno != errno.ENOENT: # errno.ENOENT = no such file or directory
            raise # re-raise exception if a different error occured

# gets file from windows pc and deletes it ... each time
def timer_cb_old(evt):
	HOST   = '192.168.1.3'
	UNAME  = 'kompai'
	PASSWD = 'ros'
	FILE   = 'Log.txt'
	fieldDelimiter=' / '
	keyValueDelimiter =' : '

	datePos=0
	COPos=1
	ParticlePos=2
	VOCPos=3
	TempPos=4
	HumidityPos=5
	LightPos=6

	silentremove(FILE)
	ftp = FTP(HOST)
	ftp.login(user=UNAME, passwd=PASSWD)

	try:
	   ftp.retrbinary("RETR " + FILE ,open(FILE, 'wb').write)
        except  Exception as e:
	   print 'No log file found. Is Airbox running?'
	   #rospy.sleep(2.5)
	   return 

	ftp.delete(FILE)
	ftp.close()

	with open(FILE , 'r') as file :
	    for lastLine in file:
		#print lastLine
		fields=lastLine.split(fieldDelimiter)
		msg = Environment()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "environment_sensor"
		msg.cO = getSecondField(fields[COPos],keyValueDelimiter)
		msg.particleCount = getSecondField(fields[ParticlePos],keyValueDelimiter)
		msg.vOCResistance = getSecondField(fields[VOCPos],keyValueDelimiter)
		msg.temp = getSecondField(fields[TempPos],keyValueDelimiter)
		msg.humidity = getSecondField(fields[HumidityPos],keyValueDelimiter)
		msg.light = getSecondField(fields[LightPos],keyValueDelimiter)

		date=fields[datePos]
		msg.datetime = rospy.Time.from_sec((datetime.strptime(date, '%d/%m/%Y %H:%M:%S')-datetime(1970,1,1)).total_seconds())

		pub_env.publish(msg)

	silentremove(FILE)

#similar to previous one but does not delete file on windows machine
def timer_cb_old_nondestroy(evt):
	global lastReadLine
	HOST   = '192.168.1.3'
	UNAME  = 'kompai'
	PASSWD = 'ros'
	FILE   = 'Log.txt'
	fieldDelimiter=' / '
	keyValueDelimiter =' : '

	datePos=0
	COPos=1
	ParticlePos=2
	VOCPos=3
	TempPos=4
	HumidityPos=5
	LightPos=6

	silentremove(FILE)
	ftp = FTP(HOST)
	ftp.login(user=UNAME, passwd=PASSWD)

	try:
	   ftp.retrbinary("RETR " + FILE ,open(FILE, 'wb').write)
        except  Exception as e:
	   print 'No log file found. Is Airbox running?'
	   #rospy.sleep(2.5)
	   return 

	
	ftp.close()

	#with handles closing our local file...
	with open(FILE , 'r') as file :
	    for i, lastLine in enumerate(file):
			if i>lastReadLine:
				fields=lastLine.split(fieldDelimiter)
				msg = Environment()
				msg.header.stamp = rospy.Time.now()
				msg.header.frame_id = "environment_sensor"
				msg.cO = getSecondField(fields[COPos],keyValueDelimiter)
				msg.particleCount = getSecondField(fields[ParticlePos],keyValueDelimiter)
				msg.vOCResistance = getSecondField(fields[VOCPos],keyValueDelimiter)
				msg.temp = getSecondField(fields[TempPos],keyValueDelimiter)
				msg.humidity = getSecondField(fields[HumidityPos],keyValueDelimiter)
				msg.light = getSecondField(fields[LightPos],keyValueDelimiter)

				date=fields[datePos]
				msg.datetime = rospy.Time.from_sec((datetime.strptime(date, '%d/%m/%Y %H:%M:%S')-datetime(1970,1,1)).total_seconds())
				
				if (msg.datetime>msg.header.stamp): 
					secondsAhead=(msg.datetime-msg.header.stamp).to_sec()
					hoursAhead=int(secondsAhead)/3600				
					#print 'ROS       time is %s' % time.strftime('%Y-%m-%d %H:%M:%S', time.localtime( msg.header.stamp.secs ))
					msg.datetime = msg.datetime - rospy.Duration(hoursAhead*60*60) 
					#print 'NOW UiEnv time is %s' % time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(msg.datetime.secs))
					
				pub_env.publish(msg)
				lastReadLine=i

	silentremove(FILE)




# . . . . . . . . . . . . . . . .  . . . . . . . . . . . . . . . . 

rospy.init_node('ros_pure_environment')
pub_env = []
pub_env = rospy.Publisher("environment_sensor", Environment, queue_size=1)

usingNewAPI=False

if usingNewAPI:
	headers = {'content-type': 'application/json'}

	
	host = rospy.get_param('~host', "192.168.1.105")

	url_env = "http://%s:7007/Devices/Airbox" % host

	print("Connecting to PURE host at " + url_env + "...")
	got_first = False
	timer = rospy.Timer(rospy.Duration(0.05), timer_cb)
else:
	lastReadLine=-1
	print("Retrieving text Logs from tablet pc...")
	timer = rospy.Timer(rospy.Duration(5), timer_cb_old_nondestroy)

rospy.spin()
