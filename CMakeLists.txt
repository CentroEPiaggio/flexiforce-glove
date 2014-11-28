cmake_minimum_required(VERSION 2.8.3)
project(sensor_gloves)

find_package(catkin REQUIRED
	rosserial_arduino
	rosserial_client
	std_msgs
  sensor_msgs
  roscpp
)

catkin_package(
)

###########
## Build ##
###########

include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_executable(flexiforce_joint_state_publisher 
  src/flexiforce_joint_state_publisher.cpp
)

target_link_libraries(flexiforce_joint_state_publisher
  ${catkin_LIBRARIES}
)

###################
## Arduino Build ##
###################

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware flexiforce ALL)
rosserial_add_client_target(firmware flexiforce-upload)