cmake_minimum_required(VERSION 2.8.3)
project(jaws2_hardware)

find_package(catkin REQUIRED COMPONENTS roscpp jaws2_msgs message_filters)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(serial_port src/serial_port.cpp)
add_dependencies(serial_port jaws2_msgs_gencpp) 
target_link_libraries(serial_port ${catkin_LIBRARIES})

add_executable(thrust_cal src/thrust_cal.cpp)
add_dependencies(thrust_cal jaws2_msgs_gencpp)
target_link_libraries(thrust_cal ${catkin_LIBRARIES})
