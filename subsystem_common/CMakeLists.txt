cmake_minimum_required(VERSION 2.8.3)
project(subsystem_common)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  rtt_rosclock
  cmake_modules
  diagnostic_msgs
  )

#find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
#include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file
orocos_use_package( rtt-typekit )

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)

orocos_component(master_component
    src/master_component.cpp
    src/diagnostic_component.cpp
)
target_link_libraries(master_component
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES}
)

### Orocos Package Exports and Install Targets ###

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS_TARGETS rtt_ros rtt_rosclock
)

### Orocos Package Exports and Install Targets ###

install(TARGETS master_component
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

orocos_install_headers(
  DIRECTORY include/${PROJECT_NAME}/
)

catkin_python_setup()

