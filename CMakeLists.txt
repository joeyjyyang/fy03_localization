cmake_minimum_required(VERSION 2.8.3)
project(fy03_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  sensor_msgs
  nav_msgs
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime sensor_msgs nav_msgs
)

catkin_install_python(
   PROGRAMS scripts/fy03_localization_node.py
	    scripts/imu_position_estimation_node.py
	    scripts/plotter_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

