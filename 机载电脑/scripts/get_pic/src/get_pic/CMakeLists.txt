cmake_minimum_required(VERSION 3.0.2)
project(get_pic)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/save_one_image.py
  scripts/record_20s_open3d.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
