cmake_minimum_required(VERSION 3.0.2)
project(marty_sim)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        tvmc_msg
        message_generation
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        tvmc_msg
)


catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime tvmc_msg
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)