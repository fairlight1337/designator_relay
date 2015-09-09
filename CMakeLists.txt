cmake_minimum_required(VERSION 2.8.3)
project(designator_relay)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  tf
  designator_integration_cpp
  designator_integration_msgs
  geometry_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS designator_integration_cpp designator_integration_msgs roscpp geometry_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(relay_publisher src/node.cpp)

add_dependencies(relay_publisher designator_relay_generate_messages_cpp)

target_link_libraries(relay_publisher
  ${catkin_LIBRARIES}
)
