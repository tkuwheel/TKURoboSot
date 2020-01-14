add_executable(bin_reader main.cpp)
target_link_libraries(bin_reader ${catkin_LIBRARIES})
add_dependencies(bin_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(bin_reader ${OpenCV_LIBRARIES})
