cmake_minimum_required(VERSION 2.8.3)
project(motion)

find_package(catkin REQUIRED COMPONENTS
 qt_build
 message_generation
 std_msgs
 roscpp
)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS
 message_runtime
 std_msgs
 roscpp
)
#add_library(csslLib cssl/cssl.h cssl/cssl.h cssl/uty.c)
foreach(subdir
		common
		Attack_motion
		Defend_motion
		test
		)
	add_subdirectory(${subdir})
endforeach()

#add_executable(motion_attack motion.cpp MotorControl.h)
#target_link_libraries(motion_attack ${catkin_LIBRARIES})
##target_link_libraries(motion_test qnode)
#target_link_libraries(motion_attack csslLib)

