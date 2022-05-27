cmake_minimum_required(VERSION 3.0.2)
project(massrobotics_amr_sender)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
)


###################################
## catkin specific configuration ##
###################################

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    rospy
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(
  DIRECTORY
  	scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(
  PROGRAMS
    scripts/massrobotics_amr_sender_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
