cmake_minimum_required(VERSION 3.0.2)
project(pg_hardware)

add_compile_options(-std=c++11)

add_definitions(-Wall -Werror)

set(IMU_LIB imu_driver)
set(MOTOR_DRIVER_LIB motor_driver)
set(DRIBBLER_DRIVER_LIB dribbler_driver)
set(KICKER_LIB kicker)

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

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

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

add_library(${DRIBBLER_DRIVER_LIB}
  src/dribbler_driver.cpp
)

add_library(${KICKER_LIB}
  src/kicker.cpp
)

add_executable(motor_driver_node src/motor_driver_node.cpp)
add_executable(imu_node src/imu_node.cpp)
add_executable(dribbler_driver_node src/dribbler_driver_node.cpp)
add_executable(kicker_node src/kicker_node.cpp)

target_link_libraries(motor_driver_node
  ${MOTOR_DRIVER_LIB}
  ${catkin_LIBRARIES}
)

target_link_libraries(imu_node
  ${IMU_LIB}
  ${catkin_LIBRARIES}
)

if (${ARCHITECTURE} MATCHES "aarch64")
	target_link_libraries(dribbler_driver_node
	  ${DRIBBLER_DRIVER_LIB}
	  ${catkin_LIBRARIES}
	  -lwiringPi
	  -lm
	  -lpthread
	  -lcrypt
	  -lrt
	)
else()
	target_link_libraries(dribbler_driver_node
	  ${DRIBBLER_DRIVER_LIB}
	  ${catkin_LIBRARIES}
	)
endif()

if (${ARCHITECTURE} MATCHES "aarch64")
	target_link_libraries(kicker_node
	  ${KICKER_LIB}
	  ${catkin_LIBRARIES}
	  -lwiringPi
	  -lm
	  -lpthread
	  -lcrypt
	  -lrt
	)
else()
	target_link_libraries(kicker_node
	  ${KICKER_LIB}
	  ${catkin_LIBRARIES}
	)
endif()

