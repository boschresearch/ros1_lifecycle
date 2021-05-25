cmake_minimum_required(VERSION 2.8.3)
project(lifecycle)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  rostest
  rosunit
  actionlib
  lifecycle_msgs
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lifecycle
  CATKIN_DEPENDS roscpp lifecycle_msgs actionlib
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(lifecycle
  src/${PROJECT_NAME}/manager.cpp
  src/${PROJECT_NAME}/managed_node.cpp
  src/${PROJECT_NAME}/lifecycle_model.cpp
  src/${PROJECT_NAME}/broadcaster.cpp
  src/${PROJECT_NAME}/client.cpp
)

add_dependencies(lifecycle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lifecycle_example_lcm examples/example_lifecycle_manager.cpp)

add_executable(lifecycle_example_managed_node examples/example_managed_node.cpp)

add_executable(lifecycle_example_client examples/example_lifecycle_client.cpp)

target_link_libraries(lifecycle
  ${catkin_LIBRARIES}
)

target_link_libraries(lifecycle_example_lcm
  lifecycle
  ${catkin_LIBRARIES}
)

target_link_libraries(lifecycle_example_managed_node
  lifecycle
  ${catkin_LIBRARIES}
)

target_link_libraries(lifecycle_example_client
  lifecycle
  ${catkin_LIBRARIES}
)

install(PROGRAMS
  scripts/change_lifecycle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS lifecycle
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/lifecycle/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*~" EXCLUDE
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}-test test/test_lifecycle.launch test/test_lifecycle.cpp)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_rostest_gtest(lifecycle_client-test test/test_lifecycle_client.launch test/test_lifecycle_client.cpp)
  target_link_libraries(lifecycle_client-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
