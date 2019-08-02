cmake_minimum_required(VERSION 2.8.3)


add_executable(fira7
				fira7.cpp 
                node_handle.cpp
                base_control.cpp 
)
target_link_libraries(fira7 ${catkin_LIBRARIES} csslLib crcLib)

