cmake_minimum_required(VERSION 2.8.3)
project(rtt_common_interfaces_test_subsystem_ports)

find_package(catkin REQUIRED COMPONENTS
 rtt_ros
 rtt_subsystem_ports
 common_interfaces
 common_interfaces_test_msgs
)

ros_generate_rtt_subsystem_ports(PACKAGE common_interfaces_test_msgs Messages Container)

orocos_generate_package(
  DEPENDS common_interfaces_test_msgs
  DEPENDS_TARGETS rtt_subsystem_ports
)
