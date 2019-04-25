cmake_minimum_required(VERSION 2.8.3)
project(test)


add_executable(simulate_strategy_node simulate_strategy.cpp)
add_executable(path_plan_node path_plan.cpp)
add_executable(test_type_node test_type.cpp)

target_link_libraries(simulate_strategy_node ${catkin_LIBRARIES})
target_link_libraries(path_plan_node ${catkin_LIBRARIES})

