cmake_minimum_required(VERSION 3.0.2)
project(pg_hardware)

add_compile_options(-std=c++11)

add_definitions(-Wall -Werror)

set(IMU_LIB imu_driver)
set(MOTOR_DRIVER_LIB motor_driver)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2_ros
  pg_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pg_hardware
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${MOTOR_DRIVER_LIB}
  src/motor_driver.cpp
)

add_library(${IMU_LIB}
  src/imu.cpp
)

add_executable(motor_driver_node src/motor_driver_node.cpp)
add_executable(imu_node src/imu_node.cpp)

target_link_libraries(motor_driver_node
  ${MOTOR_DRIVER_LIB}
  ${catkin_LIBRARIES}
)

target_link_libraries(imu_node
  ${IMU_LIB}
  ${catkin_LIBRARIES}
)
