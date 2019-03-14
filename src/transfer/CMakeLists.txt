cmake_minimum_required(VERSION 2.8.3)
project(transfer)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
  message_generation
)
catkin_python_setup()

add_message_files(
  FILES
  PPoint.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
)



install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
