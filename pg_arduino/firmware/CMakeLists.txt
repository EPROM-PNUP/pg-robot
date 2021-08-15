cmake_minimum_required(3.0.2)

set(PG_LIB_DIR lib)

include_directories(
  ${PG_LIB_DIR}
  ${ROS_LIB_DIR}
)

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

generate_arduino_firmware(main
  BOARD mega2560 
  PORT /dev/ttyACM0
  SRCS src/main.cpp
  ${PG_LIB_DIR}/cmps12/cmps12.cpp
  ${PG_LIB_DIR}/motor/motor.cpp
  ${PG_LIB_DIR}/rotary_encoder/rotary_encoder.cpp
)
