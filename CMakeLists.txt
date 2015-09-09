cmake_minimum_required(VERSION 2.8.3)
project(designator_relay)

find_package(catkin REQUIRED COMPONENTS
  designator_integration_cpp
  designator_integration_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS designator_integration_cpp designator_integration_msgs roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(relay_publisher src/node.cpp)

add_dependencies(relay_publisher designator_relay_generate_messages_cpp)

target_link_libraries(relay_publisher
  ${catkin_LIBRARIES}
)
