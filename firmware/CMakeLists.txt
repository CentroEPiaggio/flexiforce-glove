cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# We are using micro or nano
add_definitions(-DUSB_CON)

generate_arduino_firmware(flexiforce
  SRCS flexiforce_publisher.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD micro
  PORT /dev/ttyACM0
)