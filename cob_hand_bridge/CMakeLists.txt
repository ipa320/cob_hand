cmake_minimum_required(VERSION 2.8.3)
project(cob_hand_bridge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
  JointValues.msg
  Status.msg
)

add_service_files(
  FILES
  InitFinger.srv
  InitPins.srv
  SetPWM.srv
  UpdatePins.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cob_hand_bridge
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
