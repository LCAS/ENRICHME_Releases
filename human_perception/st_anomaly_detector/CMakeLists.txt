cmake_minimum_required(VERSION 2.8.3)
project(st_anomaly_detector)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  cv_bridge
  bayes_people_tracker
  rospy
  message_generation
)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  StringArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs cv_bridge bayes_people_tracker rospy message_runtime
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/st_anomaly_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
