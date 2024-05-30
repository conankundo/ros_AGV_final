cmake_minimum_required(VERSION 3.0.2)
project(stage_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)
catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
   src/stage_controller.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
   src/draw.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
   src/listen.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

