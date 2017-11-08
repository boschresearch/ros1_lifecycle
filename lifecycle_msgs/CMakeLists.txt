cmake_minimum_required(VERSION 2.8.3)
project(lifecycle_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
        COMPONENTS
        message_generation
        actionlib_msgs
        geometry_msgs
        )


add_message_files(
        DIRECTORY msg 
        FILES 
        Lifecycle.msg
        lm_events.msg
)

# actions
add_action_files(
        DIRECTORY action
        FILES
        Lifecycle.action
)

generate_messages(
        DEPENDENCIES
        actionlib_msgs
        std_msgs
)

catkin_package(
)

