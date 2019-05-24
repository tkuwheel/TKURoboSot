cmake_minimum_required(VERSION 2.8.3)

add_executable(Attack_motion_node 
				Attack_motion/motion.cpp 
				Attack_motion/motion_nodeHandle.cpp
                Attack_motion/base_control.cpp 
)
target_link_libraries(Attack_motion_node ${catkin_LIBRARIES} csslLib crcLib)

add_executable(Crc
                get_crc.cpp
)
target_link_libraries(Crc  crcLib)
