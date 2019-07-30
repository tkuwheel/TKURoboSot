cmake_minimum_required(VERSION 2.8.3)

add_definitions(-std=c++11)

foreach(subdir
        test
#        fira5_v_01
        fira6_v_05
#        fira7_v_05
)
add_subdirectory(${subdir})
endforeach()

