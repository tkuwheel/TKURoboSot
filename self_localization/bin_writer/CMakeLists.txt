add_executable(bin_writer main.cpp)
target_link_libraries(bin_writer ${catkin_LIBRARIES} ${OpenMP_LIBS})
add_dependencies(bin_writer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(bin_writer PRIVATE ${OpenMP_FLAGS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(bin_writer ${OpenCV_LIBRARIES})
