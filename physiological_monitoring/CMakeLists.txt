cmake_minimum_required(VERSION 2.8.3)
project(physiological_monitoring)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  sensor_msgs
  cv_bridge
  message_generation
)

add_message_files(
  FILES
  PhysiologicalData.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(physiological_monitoring
  src/physiological_monitoring.cpp
  )

add_dependencies(physiological_monitoring
  irimager
  irimageprocessing
  ircore
  optris_drivers_generate_messages_cpp
  )

target_link_libraries(physiological_monitoring
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  irimager
  irimageprocessing
  ircore
  udev
  gsl
  gslcblas
  )
