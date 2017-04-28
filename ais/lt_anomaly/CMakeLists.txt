cmake_minimum_required(VERSION 2.8.3)
project(lt_anomaly)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	diagnostic_msgs
    rospy
)

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs
)

#############
## Install ##
#############

 install(PROGRAMS
   scripts/lt_anomaly_monitor.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
