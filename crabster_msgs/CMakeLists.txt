cmake_minimum_required(VERSION 3.0.2)
project(crabster_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	message_generation
	genmsg
	actionlib_msgs
	actionlib
)

# Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY msg
	FILES CrabsterPose.msg
)

# Generate actions in the 'action' folder
add_action_files(
	DIRECTORY action
	FILES CrabsterSimulation.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES std_msgs actionlib_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(
	# include
	# ${catkin_INCLUDE_DIRS}
)
