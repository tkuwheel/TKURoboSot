cmake_minimum_required(VERSION 2.8.3)

project(motion)

set(INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)
include_directories(${catkin_INCLUDE_DIRS} ${INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
# qt_build
 message_generation
 std_msgs
 roscpp
)

catkin_package(CATKIN_DEPENDS
 message_runtime
 std_msgs
 roscpp
)

foreach(subdir
        lib
        src
  	)
	add_subdirectory(${subdir})
endforeach()

##target_link_libraries(motion_test qnode)
#target_link_libraries(motion_attack csslLib)

