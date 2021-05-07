cmake_minimum_required(VERSION 2.8.3)
project(lifecycle_python)

find_package(catkin REQUIRED COMPONENTS rospy actionlib actionlib_msgs lifecycle_msgs)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS actionlib lifecycle_msgs rospy
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_lifecycle_client.launch)
  add_rostest(test/test_manager.launch)
endif()
