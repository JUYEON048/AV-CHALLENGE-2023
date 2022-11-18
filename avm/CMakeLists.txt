cmake_minimum_required(VERSION 2.8.3)
project(avm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


