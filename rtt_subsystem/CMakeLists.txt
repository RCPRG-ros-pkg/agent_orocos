cmake_minimum_required(VERSION 2.8.3)
project(rtt_subsystem)

find_package(catkin REQUIRED COMPONENTS rtt_ros roscpp)

catkin_package(
  CATKIN_DEPENDS roscpp
  CFG_EXTRAS GenerateRTTSubsystemPortsPackage.cmake
)

include_directories(
  ${catkin_INCLUDE_DIRS})

# Install template files to both install and develspace
file(COPY src/templates DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/src")
install(DIRECTORY src/templates DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src)

# Install cmake macros
file(COPY cmake/create_master_cpp.py DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake")
install(PROGRAMS cmake/create_master_cpp.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

file(COPY cmake/create_master_h.py DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake")
install(PROGRAMS cmake/create_master_h.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

file(COPY cmake/parse_subsystem_xml.py DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake")
install(PROGRAMS cmake/parse_subsystem_xml.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

file(COPY cmake/create_subsystem_input_buffers.py DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake")
install(PROGRAMS cmake/create_subsystem_input_buffers.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

file(COPY cmake/create_subsystem_output_buffers.py DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake")
install(PROGRAMS cmake/create_subsystem_output_buffers.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

# This one will only be used by create_rtt_msgs with `rospack find rtt_subsystem` => no devel-space installation
#install(DIRECTORY rtt_subsystem_pkg_template DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
