add_executable(self_localization main.cpp src/mcl.cpp src/localization.cpp src/fMatrix/fVector.cpp)
target_link_libraries(self_localization ${catkin_LIBRARIES})
add_dependencies(self_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(self_localization ${OpenCV_LIBRARIES})
