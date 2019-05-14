cmake_minimum_required(VERSION 2.8.3)
project(motion)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
# qt_build
 message_generation
 std_msgs
 roscpp
)
set(INCLUDE_DIR
        common/
)
include_directories(${catkin_INCLUDE_DIRS} ${INCLUDE_DIR})

catkin_package(CATKIN_DEPENDS
 message_runtime
 std_msgs
 roscpp
)
#add_library(csslLib cssl/cssl.h cssl/cssl.h cssl/uty.c)
foreach(subdir
		common
		Attack_motion
# Defend_motion
#		test
		)
	add_subdirectory(${subdir})
endforeach()

add_executable(Crc get_crc.cpp)
target_link_libraries(Crc ${catkin_LIBRARIES} crcLib)
##target_link_libraries(motion_test qnode)
#target_link_libraries(motion_attack csslLib)

