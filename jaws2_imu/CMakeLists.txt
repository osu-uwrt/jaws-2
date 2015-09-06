cmake_minimum_required(VERSION 2.8.3)
project(jaws2_imu)

find_package(catkin REQUIRED COMPONENTS
#  imu_3dm_gx4/
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jaws2_imu
#  CATKIN_DEPENDS imu_3dm_gx4/
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
