cmake_minimum_required(VERSION 2.8.3)
project(server_multiple_client)

find_package(catkin 
    REQUIRED 
      COMPONENTS
        actionlib
        genmsg
        roscpp
        rose_common
        rose_conversions
        server_multiple_client_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC -g")

catkin_package(
    INCLUDE_DIRS 
  	    include
    LIBRARIES
    CATKIN_DEPENDS 
        actionlib
        genmsg
        roscpp
        rose_common
        rose_conversions
        server_multiple_client_msgs
    DEPENDS 
)

include_directories(
	${catkin_INCLUDE_DIRS}
	include 
)

# Build test nodes
# SMC A
add_executable(smc_test_a ${PROJECT_SOURCE_DIR}/test/smc_a.cpp)
add_dependencies(smc_test_a ${PROJECT_NAME}_generate_messages_cpp)      
target_link_libraries(smc_test_a ${catkin_LIBRARIES})

# SMC B
add_executable(smc_test_b ${PROJECT_SOURCE_DIR}/test/smc_b.cpp)
add_dependencies(smc_test_b ${PROJECT_NAME}_generate_messages_cpp)      
target_link_libraries(smc_test_b ${catkin_LIBRARIES})

# SMC C
add_executable(smc_test_c ${PROJECT_SOURCE_DIR}/test/smc_c.cpp)
add_dependencies(smc_test_c ${PROJECT_NAME}_generate_messages_cpp)      
target_link_libraries(smc_test_c ${catkin_LIBRARIES})

# SMC D
add_executable(smc_test_d ${PROJECT_SOURCE_DIR}/test/smc_d.cpp)
add_dependencies(smc_test_d ${PROJECT_NAME}_generate_messages_cpp)      
target_link_libraries(smc_test_d ${catkin_LIBRARIES})

# SMC D
add_executable(smc_test_pub_to_b ${PROJECT_SOURCE_DIR}/test/pub_to_b.cpp)
add_dependencies(smc_test_pub_to_b ${PROJECT_NAME}_generate_messages_cpp)        
target_link_libraries(smc_test_pub_to_b ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
)