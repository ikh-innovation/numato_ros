cmake_minimum_required(VERSION 2.8.3)
project(numato_ros)

find_package(catkin REQUIRED COMPONENTS)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/numato_ros.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
