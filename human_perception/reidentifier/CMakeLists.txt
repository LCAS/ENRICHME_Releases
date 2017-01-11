cmake_minimum_required(VERSION 2.8.3)
project(reidentifier)

find_package(catkin REQUIRED COMPONENTS 
    cv_bridge 
    geometry_msgs 
    ground_plane_estimation
    image_transport 
    message_filters 
    message_generation
    roscpp 
    sensor_msgs 
    std_msgs 
    visualization_msgs
    upper_body_detector
    dynamic_reconfigure
    )

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-O3")        ## Optimize
endif()

#######################################
## Declare ROS messages and services ##
#######################################

# add_message_files(
#  FILES
#  UpperBodyDetector.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

generate_dynamic_reconfigure_options(
  cfg/reidentifier.cfg
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    cv_bridge 
    geometry_msgs 
    ground_plane_estimation
    image_transport 
    message_filters 
    roscpp 
    sensor_msgs 
    std_msgs 
    visualization_msgs
    dynamic_reconfigure
)

###########
## Build ##
###########

include_directories( include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(reidentifier 
    src/main.cpp 
    src/Camera.cpp 
    src/Globals.cpp 
    src/KConnectedComponentLabeler.cpp 
    src/Math.cpp 
    src/pointcloud.cpp 
    src/ROI.cpp 
    src/CPoint.cpp 
    src/reidentifier.cpp 
    src/AncillaryMethods.cpp 
    src/Hypo.cpp 
    src/FrameInlier.cpp
) 

add_executable(train 
    src/train.cpp 
    src/Camera.cpp 
    src/Globals.cpp 
    src/KConnectedComponentLabeler.cpp 
    src/Math.cpp 
    src/pointcloud.cpp 
    src/ROI.cpp 
    src/CPoint.cpp 
    src/reidentifier.cpp 
    src/AncillaryMethods.cpp 
    src/Hypo.cpp 
    src/FrameInlier.cpp
) 

add_dependencies(reidentifier ${reidentifier_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(reidentifier ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_dependencies(train ${reidentifier_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(train ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(reidentifier
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${QT_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
)

target_link_libraries(train
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${QT_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
)

#############
## Install ##
#############

install(TARGETS reidentifier
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
