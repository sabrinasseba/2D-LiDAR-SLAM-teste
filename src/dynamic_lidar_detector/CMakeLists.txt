cmake_minimum_required(VERSION 2.8.3)
project(dynamic_lidar_detector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/dynamic_detector_node.py
  scripts/ideal_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
