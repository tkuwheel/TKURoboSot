add_executable(passing_interface main.cpp interface.cpp nodehandle.cpp)
target_link_libraries(passing_interface ${catkin_LIBRARIES} )
add_dependencies(passing_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(passing_interface ${OpenCV_LIBRARIES})
