cmake_minimum_required(VERSION 2.8.3)


add_executable(fira6
				motion.cpp 
                motion_nodeHandle.cpp
                base_control.cpp 
)
target_link_libraries(fira6 ${catkin_LIBRARIES} csslLib crcLib)

