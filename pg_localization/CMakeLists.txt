cmake_minimum_required(VERSION 3.0.2)
project(pg_localization)

add_compile_options(-std=c++11)

add_definitions(-Wall -Werror)

set(ODOMETRY_LIB odometry)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pg_localization
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf tf2_ros
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${ODOMETRY_LIB}
  src/odometry.cpp
)

add_executable(odometry_node src/odometry_node.cpp)

target_link_libraries(odometry_node
  ${ODOMETRY_LIB}
  ${catkin_LIBRARIES}
)

