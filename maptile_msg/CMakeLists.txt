cmake_minimum_required(VERSION 3.0.2)
project(maptile_msg)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
    FILES
    mapTileMsg.msg
)

generate_messages(
  DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES maptile_msg
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

install(TARGETS
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)