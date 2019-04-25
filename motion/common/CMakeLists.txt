cmake_minimum_required(VERSION 2.8.3)
project(common)

add_library(csslLib SHARED cssl/cssl.h cssl/cssl.c cssl/uty.c)
add_library(motorDataLib SHARED motor_data.cpp motor_data.h)
#target_link_libraries(csslLib ${catkin_LIBRARIES})
#target_link_libraries(motorDataLib ${catkin_LIBRARIES})

