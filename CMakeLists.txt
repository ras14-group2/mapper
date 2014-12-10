cmake_minimum_required(VERSION 2.8.3)
project(mapper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ir_reader
  nav_msgs
  roscpp
  message_filters
  message_generation
)

add_message_files(
	FILES
	PathToUnknown.msg
)

add_service_files(
  FILES
  WallInFront.srv
  PathToObject.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mapper
#  CATKIN_DEPENDS geometry_msgs ir_reader nav_msgs roscpp
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(mapper_node src/mapper_node.cpp src/OGMapper.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(mapper_node mapper_generate_messages_cpp)
add_dependencies(mapper_node object_finder_generate_messages_cpp)
add_dependencies(mapper_node robo_ctrl_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(mapper_node
  ${catkin_LIBRARIES}
)
