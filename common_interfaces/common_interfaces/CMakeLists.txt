cmake_minimum_required(VERSION 2.8.3)
project(common_interfaces)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  shm_comm
  rtt_rosclock
  )

catkin_package(
  INCLUDE_DIRS include
  DEPENDS rtt_rosclock
  CATKIN_DEPENDS rtt_ros shm_comm
)

include_directories(${catkin_INCLUDE_DIRS} include)

#orocos_generate_package(
#  INCLUDE_DIRS include
#  DEPENDS shm_comm
#  DEPENDS_TARGETS rtt_ros rtt_rosclock shm_comm
#)

