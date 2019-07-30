cmake_minimum_required(VERSION 2.8.3)

add_definitions(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS} ${INCLUDE_DIR})

add_library(csslLib cssl/cssl.c)

add_library(crcLib crc/crc_16.cpp)

add_library(nodeLib node/node_handle.cpp)
target_link_libraries(nodeLib ${catkin_LIBRARIES})

add_library(baseLib base/base_control.cpp)
target_link_libraries(baseLib csslLib crcLib)

