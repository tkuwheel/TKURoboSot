add_executable(obstacle main.cpp obstacle.cpp nodehandle.cpp)
target_link_libraries(obstacle ${catkin_LIBRARIES})
add_dependencies(obstacle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(obstacle ${OpenCV_LIBRARIES})
