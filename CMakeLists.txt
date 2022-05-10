cmake_minimum_required(VERSION 3.0.2)
project(state_estimation)

add_definitions(-std=c++11 -g -O3 )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_filters
  #mav_msgs
  geometry_msgs
  sensor_msgs
  gazebo_msgs
  tf
  message_generation
)

find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GenericRanging.msg
  Ranging.msg
  UWBRanging.msg
  DWRanging.msg
  PozyxRanging.msg
  RangingDiff.msg
  PozyxRangingWithCir.msg
  # ESP32S2FTMFrame.msg
  # ESP32S2FTMRanging.msg
  # ESP32S2FTMRangingExtra.msg
  RadarCube.msg
  RadarRangeAzimuth.msg
  RadarRangeDoppler.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${catkin_INCLUDE_DIRS})
catkin_package()


# add_executable(offboard src/offboard.cpp  )
# target_link_libraries(offboard ${catkin_LIBRARIES})

add_executable(state_esti src/state_esti.cpp )
target_link_libraries(state_esti ${catkin_LIBRARIES})

add_executable(state_mean src/state_mean.cpp)
target_link_libraries(state_mean ${catkin_LIBRARIES})