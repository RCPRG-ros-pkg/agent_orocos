cmake_minimum_required(VERSION 2.8.3)
project(rtt_subsystem_ports)

find_package(catkin REQUIRED COMPONENTS rtt_ros roscpp)

catkin_package(
  CATKIN_DEPENDS roscpp
  CFG_EXTRAS GenerateRTTSubsystemPortsPackage.cmake
)

include_directories(
  ${catkin_INCLUDE_DIRS})

# Install cmake macros
file(COPY cmake/create_subsystem_ports_header.py DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake")
install(PROGRAMS cmake/create_subsystem_ports_header.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

file(COPY cmake/create_subsystem_ports_source.py DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake")
install(PROGRAMS cmake/create_subsystem_ports_source.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

file(COPY cmake/create_components_source.py DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake")
install(PROGRAMS cmake/create_components_source.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

file(COPY cmake/parse_subsystem_xml.py DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake")
install(PROGRAMS cmake/parse_subsystem_xml.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

file(COPY cmake/create_msgs_from_ec_config.py DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake")
install(PROGRAMS cmake/create_msgs_from_ec_config.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

file(COPY cmake/create_converter.py DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake")
install(PROGRAMS cmake/create_converter.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

# Install template files to both install and develspace
file(COPY src/templates DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/src")
install(DIRECTORY src/templates DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src)

# This one will only be used by create_rtt_msgs with `rospack find rtt_subsystem_ports` => no devel-space installation
#install(DIRECTORY rtt_subsystem_ports_pkg_template DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
