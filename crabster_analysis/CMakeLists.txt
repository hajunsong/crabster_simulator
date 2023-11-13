cmake_minimum_required(VERSION 3.0.2)
project(crabster_analysis)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  crabster_msgs
  actionlib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crabster_analysis
#  CATKIN_DEPENDS roscpp roslib std_msgs
#  DEPENDS system_lib
)

find_package(Eigen3 REQUIRED)

file(GLOB SOURCES src/*.cpp)

include_directories(
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(${PROJECT_NAME}_node ${SOURCES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} -ljsoncpp)
install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})