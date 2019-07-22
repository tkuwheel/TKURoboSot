add_executable(black_item main.cpp black_item.cpp nodehandle.cpp)
target_link_libraries(black_item ${catkin_LIBRARIES})
add_dependencies(black_item ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(black_item ${OpenCV_LIBRARIES})
