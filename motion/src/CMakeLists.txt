cmake_minimum_required(VERSION 2.8.3)

set(BASE_CONTROL Attack_motion/base_control.cpp)
add_executable(Attack_motion_node 
				Attack_motion/motion.cpp 
				Attack_motion/motion_nodeHandle.cpp
#                Attack_motion/base_control.cpp 
                ${BASE_CONTROL}
)
target_link_libraries(Attack_motion_node ${catkin_LIBRARIES} csslLib crcLib)

add_executable(Crc
                get_crc.cpp
)
target_link_libraries(Crc  crcLib)

add_executable(Single
                test/single_motor.cpp
                ${BASE_CONTROL}
)
target_link_libraries(Single  ${catkin_LIBRARIES} crcLib csslLib)

add_executable(Triple
                test/three_motor.cpp
                ${BASE_CONTROL}
)
target_link_libraries(Triple  ${catkin_LIBRARIES} crcLib csslLib)

add_executable(Step
                test/test_motor_step.cpp
                ${BASE_CONTROL}
)
target_link_libraries(Step ${catkin_LIBRARIES} crcLib csslLib)

add_executable(Cos
                test/test_motor_cos.cpp
                ${BASE_CONTROL}
)
target_link_libraries(Cos  ${catkin_LIBRARIES} crcLib csslLib)

add_executable(Robot_Test
                test/robot_test.cpp
                ${BASE_CONTROL}
)
target_link_libraries(Robot_Test  ${catkin_LIBRARIES} crcLib csslLib)

add_executable(TX_Test
                test/tx_test.cpp
)
target_link_libraries(TX_Test  ${catkin_LIBRARIES} csslLib)
