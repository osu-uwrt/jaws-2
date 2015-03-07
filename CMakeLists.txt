cmake_minimum_required(VERSION 2.8.3)
project(jaws_arbotix)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  jaws_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(arbotix_node src/arbotix_node.cpp)

target_link_libraries(arbotix_node
  ${catkin_LIBRARIES}
)
