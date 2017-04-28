#!/usr/bin/env python


#
# reads from the http webservice and publishes it into a custom msg, every 10 secs
#	 - is update frequency  reasonable?  Maybe check if timestamp has changed before publishing?
#
# See here
#   http://wiki.ros.org/ROSNodeTutorialPython
#   for more info about dinamic reconfiguring.
#   Followed same structure
'''
UEMV measures
Carbon Monoxide                       in ppm
Particulates of 0.5 microns in air    in particles/0.01CF
Volatile Organic compounds            in KOhms
Temperature                           in Celsius degrees
Humidity                              in %
Light                                 in Lux
Measures are far from being accurate. 
CO Measures based on CO-AF should be corrected (sensor decais on time) and there is no info about how to do it
'''


import rospy
#from std_msgs.msg import Header not creating headers, just accessing
from env_sensor.msg import Environment
from datetime import datetime
import serial
import sys
import math
import io










# Node example class.
class Environment_sensor():

    # blocks until airbox is connected
    def ensureActiveAirbox(self):
        while self.disconnected:
            rospy.sleep(self.tsample)
            if ( self.connectAirbox()):
                self.disconnected = not self.setActive()

    # connects serial port. returns true on success
    def connectAirbox(self):
        try:
            self.ser = serial.Serial(self.AIRBOX_PORT, self.AIRBOX_BAUDRATE, timeout=self.AIRBOX_TIMEOUT)
            ans=True
        except serial.SerialException as e:
            rospy.logerr("Can't find %s. Did you plug Airbox and run install.sh?", self.AIRBOX_PORT)
            ans = False

        return ans

    #checks connection and sends to Airbox
    def sendCommandToAirbox(self,command):
        self.ensureActiveAirbox()
        return self.sendCommandToAirboxRaw(command)

    # returns string containing answer from Airbox without checking connection
    #  format is echoing command and aftewards the response
    def sendCommandToAirboxRaw(self, command):
        try:
            self.ser.write(command + '\r')

            char=self.ser.read(1)
            line=""
            keepReading=True
            while keepReading:
                line=line+(char)
                char = self.ser.read(1)
                keepReading=(char != '\r') & (char != '')
            self.ser.flushOutput()
        except serial.SerialException as e:
            rospy.logerr("Can't communicate with Airbox")
            self.disconnected=True
            line='0'
        return line

    # sets airbox active. returns true on succeed
    def setActive(self):
        ans=self.sendCommandToAirboxRaw('Mode 1')
        return ('Mode 1' in ans)

    # retrieves a string value from airbox
    def getStringVal(self,command):
        ans = self.sendCommandToAirbox(command)
        fields=ans.split()
        if len(fields)>2:
            ans=fields[2]
        else:
            rospy.logerr("Command response to '%s' is not properly formed: %s",command,ans)
            self.disconnected = True
            ans=0
        return ans

    # retrieves a float value from airbox
    def getFloatVal(self,command):
        return float(self.getStringVal(command))

    # updates our environmental message
    def readDataFromAirbox(self):

        self.env_msg.header.stamp = rospy.Time.now()
        self.env_msg.header.frame_id = "environment_sensor"

        #read Runsensor data
        self.env_msg.temp = self.getFloatVal('GetRunSensor 1')
        self.env_msg.humidity = self.getFloatVal('GetRunSensor 4')
        self.env_msg.light = self.getFloatVal('GetRunSensor 7')

        #read averaged sensor data
        self.env_msg.cO = self.getFloatVal('GetAverage 1')
        #self.env_msg.cO = self.getFloatVal('GetAverage 1')/(0.0283168*0.01)  #these are particles per cubic meter
        self.env_msg.cO = abs(self.getFloatVal('GetAverage 1'))  # Values should not be negative. This means sensor is not properly calibrated. 
        self.env_msg.particleCount = self.getFloatVal('GetAverage 4')
        self.env_msg.vOCResistance = self.getFloatVal('GetAverage 8')*1000.0
        if self.env_msg.vOCResistance!=0:
            self.env_msg.vOCResistance = self.getFloatVal('GetAverage 7') / self.env_msg.vOCResistance

        #date and time according to airbox
        DMY=self.sendCommandToAirbox('Date')
        HMS=self.sendCommandToAirbox('Time')
        date=DMY+' '+HMS
        if len(date)>8:
			try:
				timestamp=datetime.strptime(date, 'Date %d/%m/%Y Time %H:%M:%S') - datetime(1970, 1, 1)
				self.env_msg.datetime = rospy.Time.from_sec(timestamp.total_seconds())

				# Apparently Airbox is a couple of hours ahead...
				if (self.env_msg.datetime > self.env_msg.header.stamp):
					secondsAhead = (self.env_msg.datetime - self.env_msg.header.stamp).to_sec()
					hoursAhead = int(secondsAhead) / 3600
					self.env_msg.datetime = self.env_msg.datetime - rospy.Duration(hoursAhead * 60 * 60)
			except ValueError:
				self.env_msg.datetime = self.env_msg.header.stamp



        # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):


        # "Constants" or variables you should really not change
        self.AIRBOX_PORT = '/dev/ttyUSB0' #'/dev/airbox' # use install script to fix this device name
        self.AIRBOX_BAUDRATE=115200
        self.AIRBOX_TIMEOUT=1            #  seconds to timeout. 0 is non-blocking mode

        # Get the ~private namespace parameters from command line or launch file.
        self.tsample = float(rospy.get_param('~tsample', '1.0'))
        self.envTopic = rospy.get_param('~EnvSensorTopic', 'environment_sensor')

        rospy.loginfo('tsample = %d', self.tsample)
        rospy.loginfo('EnvSensorTopic = %s', self.envTopic)

        # Init Airbox
        self.disconnected=True
        if (self.connectAirbox()):
            self.disconnected = not self.setActive()
        if self.disconnected:
            sys.exit(2)

        # Create publishers
        self.env_pub = rospy.Publisher(self.envTopic, Environment,queue_size=10)

        # Create our message
        self.env_msg = Environment()

        # Main while loop.
        r = rospy.Rate(1/self.tsample)
        while not rospy.is_shutdown():
            #Get data from Airbox
            self.readDataFromAirbox()

            # publish data
            self.env_pub.publish(self.env_msg)

            # Sleep for a while after publishing new messages
            r.sleep()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('env_sensor')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = Environment_sensor()
    except rospy.ROSInterruptException: pass
