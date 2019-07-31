cmake_minimum_required(VERSION 2.8.3)


add_executable(fira7
				motion.cpp 
)
target_link_libraries(fira7 ${catkin_LIBRARIES} csslLib crcLib nodeLib baseLib)

