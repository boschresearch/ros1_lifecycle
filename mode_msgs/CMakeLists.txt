cmake_minimum_required(VERSION 2.8.3)
project(mode_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation actionlib_msgs)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Mode.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   ListModes.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES 
  ModeChange.action
#   Action2.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mode_msgs
  CATKIN_DEPENDS std_msgs actionlib_msgs
#  DEPENDS system_lib
)


