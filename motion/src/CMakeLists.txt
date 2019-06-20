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

add_executable(Single
                Attack_motion/single_motor.cpp
                Attack_motion/base_control.cpp
)
target_link_libraries(Single  ${catkin_LIBRARIES} crcLib csslLib)

add_executable(Triple
                Attack_motion/three_motor.cpp
                Attack_motion/base_control.cpp
)
target_link_libraries(Triple  ${catkin_LIBRARIES} crcLib csslLib)

add_executable(Step
                Attack_motion/test_motor_step.cpp
                Attack_motion/base_control.cpp
)
target_link_libraries(Step  ${catkin_LIBRARIES} crcLib csslLib)

add_executable(TX_Test
                tx_test.cpp
)
target_link_libraries(TX_Test  ${catkin_LIBRARIES} csslLib)
