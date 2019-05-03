add_executable(interface main.cpp interface.cpp nodehandle.cpp)
target_link_libraries(interface ${catkin_LIBRARIES} )
add_dependencies(interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(interface ${OpenCV_LIBRARIES})
