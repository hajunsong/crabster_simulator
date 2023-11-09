cmake_minimum_required(VERSION 3.0.2)
project(crabster_analysis)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crabster_analysis
#  CATKIN_DEPENDS roscpp roslib std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

find_package(Eigen3 REQUIRED)

file(GLOB SOURCES src/*.cpp)
file(GLOB HEADERS include/*.h)

include_directories(
	${EIGEN3_INCLUDE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(${PROJECT_NAME}_single ${SOURCES} main_src/KRISO_CRABSTER.cpp ${HEADERS})
add_dependencies(${PROJECT_NAME}_single ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(${PROJECT_NAME}_single ${catkin_LIBRARIES} -ljsoncpp)
install(TARGETS ${PROJECT_NAME}_single RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(${PROJECT_NAME}_node ${SOURCES} main_src/KRISO_CRABSTER_NODE.cpp ${HEADERS})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} -ljsoncpp)
install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})