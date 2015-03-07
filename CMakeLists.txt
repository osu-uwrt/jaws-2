cmake_minimum_required(VERSION 2.8.3)
project(jaws-controls)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  joy
  jaws_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(controls_node src/controls_node.cpp)

target_link_libraries(controls_node
  ${catkin_LIBRARIES}
)
