cmake_minimum_required(VERSION 2.8.3)

#set(INCLUDE_DIR 
#        ${PROJECT_SOURCE_DIR}/include)
#include_directories(${catkin_INCLUDE_DIRS} ${INCLUDE_DIR})

#add_executable(Single_n
#                test/single_motor.cpp
#                ${BASE_CONTROL}
#)
#target_link_libraries(Single_n  ${catkin_LIBRARIES} crcLib csslLib)
#
#add_executable(Triple
#                test/three_motor.cpp
#                ${BASE_CONTROL}
#)
#target_link_libraries(Triple  ${catkin_LIBRARIES} crcLib csslLib)

add_executable(TX_Test
                tx_test.cpp
)
target_link_libraries(TX_Test  ${catkin_LIBRARIES} csslLib)

add_executable(RX_Test
                rx_test.cpp
)
target_link_libraries(RX_Test  ${catkin_LIBRARIES} csslLib)
