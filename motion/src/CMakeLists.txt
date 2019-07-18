cmake_minimum_required(VERSION 2.8.3)

set(BASE_CONTROL_DIR fira6_v_05)
include_directories(${BASE_CONTROL_DIR})
set(BASE_CONTROL fira6_v_05/base_control.cpp)

add_executable(fira6
				fira6_v_05/motion.cpp 
                fira6_v_05/motion_nodeHandle.cpp
                fira6_v_05/base_control.cpp 
)
target_link_libraries(fira6 ${catkin_LIBRARIES} csslLib crcLib)

add_executable(Crc
                get_crc.cpp
)
target_link_libraries(Crc  crcLib)

add_executable(Single_n
                test/single_motor.cpp
                ${BASE_CONTROL}
)
target_link_libraries(Single_n  ${catkin_LIBRARIES} crcLib csslLib)

add_executable(Triple
                test/three_motor.cpp
                ${BASE_CONTROL}
)
target_link_libraries(Triple  ${catkin_LIBRARIES} crcLib csslLib)

#add_executable(Step_n
#                test/test_motor_step.cpp
#                ${MOTOR_CONTROL}
#)
#target_link_libraries(Step_n ${catkin_LIBRARIES} crcLib csslLib)
#
#add_executable(Sin_n
#                test/test_motor_sin.cpp
#                ${MOTOR_CONTROL}
#)
#target_link_libraries(Sin_n  ${catkin_LIBRARIES} crcLib csslLib)
#add_executable(Robot_Test
#                test/robot_test.cpp
#                ${BASE_CONTROL}
#)
#target_link_libraries(Robot_Test  ${catkin_LIBRARIES} crcLib csslLib)

add_executable(TX_Test
                test/tx_test.cpp
)
target_link_libraries(TX_Test  ${catkin_LIBRARIES} csslLib)

add_executable(RX_Test
                test/rx_test.cpp
)
target_link_libraries(RX_Test  ${catkin_LIBRARIES} csslLib)
