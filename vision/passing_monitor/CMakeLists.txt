add_executable(passing_monitor main.cpp monitor.cpp nodehandle.cpp)
target_link_libraries(passing_monitor ${catkin_LIBRARIES} ${OpenMP_LIBS} )
add_dependencies(passing_monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(passing_monitor PRIVATE ${OpenMP_FLAGS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(passing_monitor ${OpenCV_LIBRARIES})
