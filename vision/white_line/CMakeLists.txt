add_executable(white_line main.cpp white_line.cpp nodehandle.cpp)
target_link_libraries(white_line ${catkin_LIBRARIES})
add_dependencies(white_line ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(white_line ${OpenCV_LIBRARIES})
