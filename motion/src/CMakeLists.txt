cmake_minimum_required(VERSION 2.8.3)

set(INCLUDE_DIR 
    include)
include_directories(${catkin_INCLUDE_DIRS} ${INCLUDE_DIR})

add_library(csslLib cssl/cssl.c)
add_library(crcLib crc_16.cpp)
add_library(nodeLib node_handle.cpp)
target_link_libraries(nodeLib ${catkin_LIBRARIES})
add_library(baseLib base_control.cpp)
target_link_libraries(nodeLib csslLib crcLib)

foreach(subdir
        test
#        fira5_v_01
        fira6_v_05
        fira7_v_05
)
add_subdirectory(${subdir})
endforeach()

add_executable(Crc
                get_crc.cpp
)
target_link_libraries(Crc  crcLib)

