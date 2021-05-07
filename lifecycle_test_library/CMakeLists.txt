cmake_minimum_required(VERSION 2.8.3)
project(lifecycle_test_library)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  lifecycle_msgs
  lifecycle_python
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  AddTwoInts.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS lifecycle_msgs lifecycle_python rospy
)
