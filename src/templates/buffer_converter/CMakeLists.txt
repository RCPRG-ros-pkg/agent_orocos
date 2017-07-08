find_package(OROCOS-RTT 2.0.0 COMPONENTS rtt-scripting rtt-marshalling)
if (NOT OROCOS-RTT_FOUND)
  message(FATAL_ERROR "\n   RTT not found. Is the version correct? Use the CMAKE_PREFIX_PATH cmake or environment variable to point to the installation directory of RTT.")
else()
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
  #add_definitions( -DRTT_COMPONENT )
endif()

include(AddFileDependencies)


# Configure source and destination paths of generated files
msgs_from_ec_config_destinations()

# Check if we're generating code for messages in this package
if("${_package}" STREQUAL "${PROJECT_NAME}")
  set(${_package}_FOUND True)
else()
  find_package(${_package} QUIET)
endif()

# Set the subsystem ports header generation script path
set(CREATE_CONVERTER_EXE_PATH ${rtt_subsystem_ports_DIR}/create_converter.py)

include_directories(
  ${USE_OROCOS_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${${_package}_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/buffer_converter.cpp
    COMMAND ${CREATE_CONVERTER_EXE_PATH} ${_package} ${CMAKE_CURRENT_BINARY_DIR}/buffer_converter.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS ${${_package}_EXPORTED_TARGETS} ${CREATE_CONVERTER_EXE_PATH}
    VERBATIM)

orocos_library(rtt-${_package}-buffer_converter ${CMAKE_CURRENT_BINARY_DIR}/buffer_converter.cpp)
target_link_libraries(rtt-${_package}-buffer_converter ${catkin_LIBRARIES} ${${_package}_LIBRARIES} ${USE_OROCOS_LIBRARIES})
list(APPEND OROCOS_DEFINED_TYPES " -lrtt-${_package}-buffer_converter")

# Export variables to the PARENT_SCOPE
set(OROCOS_DEFINED_TYPES ${OROCOS_DEFINED_TYPES} PARENT_SCOPE)
set(${PROJECT_NAME}_EXPORTED_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} PARENT_SCOPE)
set(${PROJECT_NAME}_EXPORTED_INCLUDE_DIRS ${${PROJECT_NAME}_EXPORTED_INCLUDE_DIRS} PARENT_SCOPE)
