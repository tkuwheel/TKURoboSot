cmake_minimum_required(VERSION 2.8.3)
add_executable(image_publisher main.cpp)
target_link_libraries(image_publisher ${catkin_LIBRARIES})
add_dependencies(image_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(image_publisher ${OpenCV_LIBRARIES})
