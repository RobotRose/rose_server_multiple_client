cmake_minimum_required(VERSION 2.8.3)
project(server_multiple_client)

find_package( catkin 
	REQUIRED 
	COMPONENTS
		server_multiple_client_msgs
)

include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC -g")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES server_multiple_client
)

target_link_libraries(server_multiple_client ${catkin_LIBRARIES})
add_dependencies(server_multiple_client ${PROJECT_NAME}_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)