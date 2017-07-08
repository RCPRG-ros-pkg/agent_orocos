cmake_minimum_required(VERSION 2.8.3)
project(rtt_common_interfaces_test_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm common_interfaces_test_msgs common_interfaces eigen_conversions)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIR})

ros_generate_rtt_typekit(common_interfaces_test_msgs)

orocos_generate_package(
  DEPENDS common_interfaces_test_msgs
  DEPENDS_TARGETS rtt_roscomm

)
