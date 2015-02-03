cmake_minimum_required(VERSION 2.8.3)
project(server_multiple_client)

find_package( catkin 
	REQUIRED 
	COMPONENTS
		server_multiple_client_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC -g")

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
	${catkin_INCLUDE_DIRS}
	include 
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)