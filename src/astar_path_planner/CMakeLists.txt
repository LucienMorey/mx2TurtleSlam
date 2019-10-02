cmake_minimum_required(VERSION 2.8.3)
project(astar_path_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy nav_msgs

)

###########
## Build ##
###########

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  nodes/map_writer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
