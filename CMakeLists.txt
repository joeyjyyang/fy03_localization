cmake_minimum_required(VERSION 2.8.3)
project(fy03_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  visualization_msgs
  geometry_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs nav_msgs tf2_ros visualization_msgs geometry_msgs
)

