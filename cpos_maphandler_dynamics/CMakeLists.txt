cmake_minimum_required(VERSION 3.0.2)
project(cpos_maphandler_dynamics)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  roslib  # ros/package
  pcl_conversions
  pcl_ros
  crabster_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES virtual_sensor
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

link_libraries(${catkin_LIBRARIES})

#
add_executable(contact_terrain_node src/main_contact_terrain.cpp)
add_executable(contact_terrain_realtime_node src/main_contact_terrain_realtime.cpp)

# CRABSTER
add_executable(contact_terrain_crabster_node src/main_contact_terrain_crabster.cpp)