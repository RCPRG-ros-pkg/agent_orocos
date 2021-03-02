cmake_minimum_required(VERSION 2.8.3)
project(fabric_logger)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  #geometry_msgs
  #visualization_msgs
  #urdf
  cmake_modules
)

#find_package(Eigen3 REQUIRED)
#find_package(orocos_kdl REQUIRED)
#find_package(kdl_parser REQUIRED)
#find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fabric_logger
  #DEPENDS EIGEN3 orocos_kdl
)


#link_directories(${orocos_kdl_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)
ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)
### Orocos Targets ###

add_library(fabric_logger src/fabric_logger.cpp)
target_link_libraries(fabric_logger ${catkin_LIBRARIES})
# ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable( fabric_logger_test src/fabric_logger_test.cpp )
target_link_libraries( fabric_logger_test fabric_logger pthread ${catkin_LIBRARIES})


### Orocos Package Exports and Install Targets ###
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

