cmake_minimum_required(VERSION 2.8.3)
project(server_multiple_client_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin 
	REQUIRED
		actionlib_msgs 
		std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate actions in the 'action' folder
add_action_files(
	FILES
		smc_dummy_server.action
		smc_test.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
 	DEPENDENCIES
		actionlib_msgs
		std_msgs 
)

catkin_package(
	CATKIN_DEPENDS 
		actionlib_msgs 
		std_msgs
)
