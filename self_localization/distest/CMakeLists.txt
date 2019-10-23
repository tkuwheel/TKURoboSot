add_executable(distest main.cpp)
target_link_libraries(distest ${catkin_LIBRARIES})
add_dependencies(distest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(distest ${OpenCV_LIBRARIES})
