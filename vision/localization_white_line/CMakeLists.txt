add_executable(localization_white_line main.cpp white_line.cpp nodehandle.cpp)
target_link_libraries(localization_white_line ${catkin_LIBRARIES})
add_dependencies(localization_white_line ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(localization_white_line ${OpenCV_LIBRARIES})
