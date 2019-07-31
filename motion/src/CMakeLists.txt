cmake_minimum_required(VERSION 2.8.3)

include_directories(${catkin_INCLUDE_DIRS} ${INCLUDE_DIR})

add_library(csslLib cssl/cssl.c)
add_library(crcLib crc_16.cpp)

foreach(subdir
        test
        fira6
# fira7
)
add_subdirectory(${subdir})
endforeach()

add_executable(Crc
                get_crc.cpp
)
target_link_libraries(Crc  crcLib)

