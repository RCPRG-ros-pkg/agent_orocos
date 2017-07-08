cmake_minimum_required(VERSION 2.8.6)

project(common_interfaces_test_msgs)

find_package(catkin REQUIRED
  COMPONENTS common_interfaces message_generation actionlib_msgs)

add_message_files(FILES
  Container.msg
  SubContainer.msg
  SubSubContainer.msg
)
  

generate_messages(
#  DEPENDENCIES
  )


catkin_package(
  CATKIN_DEPENDS message_runtime)
