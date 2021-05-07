cmake_minimum_required(VERSION 2.8.3)
project(lm_monitor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib_msgs
  lifecycle_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS lifecycle_msgs rospy std_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_lm_monitor.launch)
endif()
