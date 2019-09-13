cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_move)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  rostime
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot3_move
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}

)
add_executable(turtlebot3_move_node src/turtlebot3_move.cpp)
target_link_libraries(turtlebot3_move_node ${catkin_LIBRARIES})
add_dependencies(turtlebot3_move_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS turtlebot3_move_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
