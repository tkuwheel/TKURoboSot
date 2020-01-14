add_executable(tracking main.cpp tracking.cpp nodehandle.cpp)
target_link_libraries(tracking ${catkin_LIBRARIES})
add_dependencies(tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(tracking ${OpenCV_LIBRARIES})
