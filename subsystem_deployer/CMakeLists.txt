cmake_minimum_required(VERSION 2.8.3)
project(subsystem_deployer)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_roscomm
  kdl_conversions
  cmake_modules
  subsystem_common
  subsystem_msgs
  )

find_package(TinyXML REQUIRED)

#find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
#include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(orocos-ocl REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file
orocos_use_package( rtt-typekit )

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)

orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )

link_directories(${orocos-ocl_LIBRARY_DIRS})
include_directories(${orocos-ocl_INCLUDE_DIRS})

add_library(subsystem_deployer src/subsystem_deployer.cpp)
#set_target_properties(subsystem_deployer PROPERTIES COMPILE_FLAGS -DRTT_STATIC)
target_link_libraries( subsystem_deployer ${OROCOS-RTT_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${TinyXML_LIBRARIES})

add_executable( deployer src/deployer.cpp src/subsystem_deployer.cpp )
target_link_libraries( deployer ${OROCOS-RTT_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${TinyXML_LIBRARIES})

### Orocos Package Exports and Install Targets ###

# Export package information (replaces catkin_package() macro) 
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES subsystem_deployer
  CATKIN_DEPENDS subsystem_common subsystem_msgs
)

### Orocos Package Exports and Install Targets ###

install(TARGETS subsystem_deployer deployer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

