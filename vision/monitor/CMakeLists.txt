add_executable(monitor main.cpp monitor.cpp nodehandle.cpp)
target_link_libraries(monitor ${catkin_LIBRARIES} ${OpenMP_LIBS} )
add_dependencies(monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(monitor PRIVATE ${OpenMP_FLAGS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(monitor ${OpenCV_LIBRARIES})
