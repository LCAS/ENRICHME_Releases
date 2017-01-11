cmake_minimum_required(VERSION 2.8.3)
project(user_status_server)

find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  optris_reader
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  UserStatus.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   optris_reader
 )


###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES ais_services
  CATKIN_DEPENDS message_runtime
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
   scripts/user_status_server.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

