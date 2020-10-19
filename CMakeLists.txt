cmake_minimum_required(VERSION 2.8.3)
project(fy03_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  sensor_msgs
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime sensor_msgs
)

#install(
#   PROGRAMS scripts/fy03_localization_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

