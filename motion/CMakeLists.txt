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
    ${PROJECT_SOURCE_DIR}/include
# common/
# common/cssl
)
# message(FIND PATH ${PROJECT_SOURCE_DIR})
# message(FIND INCLUDE ${INCLUDE_DIR})

include_directories(${catkin_INCLUDE_DIRS} ${INCLUDE_DIR})

catkin_package(CATKIN_DEPENDS
 message_runtime
 std_msgs
 roscpp
)

add_library(csslLib src/cssl/cssl.c)
add_library(crcLib src/crc_16.cpp)
# foreach(subdir
#         src
#       Defend_motion
#		test
#		)
# 	add_subdirectory(${subdir})
# endforeach()
add_subdirectory(src)

##target_link_libraries(motion_test qnode)
#target_link_libraries(motion_attack csslLib)

