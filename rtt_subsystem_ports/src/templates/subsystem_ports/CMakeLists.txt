find_package(OROCOS-RTT 2.0.0 COMPONENTS rtt-scripting rtt-marshalling)
if (NOT OROCOS-RTT_FOUND)
  message(FATAL_ERROR "\n   RTT not found. Is the version correct? Use the CMAKE_PREFIX_PATH cmake or environment variable to point to the installation directory of RTT.")
else()
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
  #add_definitions( -DRTT_COMPONENT )
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" PARENT_SCOPE)

include(AddFileDependencies)

# Configure source and destination paths of generated files
rtt_subsystem_ports_destinations()

# Check if we're generating code for messages in this package
if("${_package}" STREQUAL "${PROJECT_NAME}")
  set(${_package}_FOUND True)
else()
  find_package(${_package} QUIET)
endif()

find_package(genmsg REQUIRED)

find_package(common_interfaces REQUIRED)
orocos_use_package(rtt_rosclock)

# Get all .msg files
if(${_package}_FOUND AND NOT ROSBUILD_init_called)
  # Use catkin-based genmsg to find msg files
  if(genmsg_VERSION VERSION_GREATER 0.4.19)
    set(MSG_FILES)
    # TODO: genmsg API is unstable at this level
    foreach(FILE ${${_package}_MESSAGE_FILES})
      if(IS_ABSOLUTE "${FILE}")
        list(APPEND MSG_FILES ${FILE})
      else()
        list(APPEND MSG_FILES ${${_package}_DIR}/../${FILE})
      endif()
    endforeach()
  else()
    message(SEND_ERROR "genmsg version must be 0.4.19 or greater to generate RTT typekits for ROS messages")
  endif()
elseif(ROSBUILD_init_called)
  # try to find rosbuild-style message package
  rosbuild_find_ros_package(${_package})
  if(DEFINED ${_package}_PACKAGE_PATH)
    set(${_package}_FOUND TRUE)
    set(${_package}_INCLUDE_DIRS "${${_package}_PACKAGE_PATH}/include")
    file(GLOB MSG_FILES "${${_package}_PACKAGE_PATH}/msg/*.msg")
    set(${_package}_EXPORTED_TARGETS)
  endif()
endif()

# message package not found
if(NOT ${_package}_FOUND)
  message(SEND_ERROR "Package ${_package} not found. Will not generate RTT typekit.")
  return()
endif()

# Return if nothing to do
if( "${MSG_FILES}" STREQUAL "" )
  message(STATUS "ros_generate_rtt_subsystem_ports: Could not find any .msg files in the ${_package} package.")
  return()
endif()

# Set the subsystem ports header generation script path
set(CREATE_SUBSYSTEM_PORTS_HEADER_EXE_PATH ${rtt_subsystem_ports_DIR}/create_subsystem_ports_header.py)

# Set the subsystem ports sources generation script path
set(CREATE_SUBSYSTEM_PORTS_SOURCE_EXE_PATH ${rtt_subsystem_ports_DIR}/create_subsystem_ports_source.py)

# Set the subsystem ports components' sources generation script path
set(CREATE_COMPONENTS_SOURCE_EXE_PATH ${rtt_subsystem_ports_DIR}/create_components_source.py)

# Store the ros package name
set(ROSPACKAGE ${_package})

# Generate code for each message type
foreach( FILE ${MSG_FILES} )
  # Get just the message name
  string(REGEX REPLACE ".+/\(.+\).msg" "\\1" ROSMSGNAME ${FILE})

  # Define the typenames for this message
  set(ROSMSGTYPE         "${_package}::${ROSMSGNAME}")
  set(ROSMSGTYPENAME     "/${_package}/${ROSMSGNAME}")
  set(ROSMSGCTYPENAME    "/${_package}/c${ROSMSGNAME}")

  # msg_Types.hpp.in, ros_msg_typekit_plugin.cpp.in, ros_msg_typekit_package.cpp.in
  set(ROSMSGSUBSYSTEMPORTSHEADER  "${_package}/subsystem_ports/${ROSMSGNAME}.h")
  # ros_msg_typekit_plugin.cpp.in, ros_msg_typekit_package.cpp.in
  set(ROSMSGSUBSYSTEMPORTSHEADERS "${ROSMSGSUBSYSTEMPORTSHEADERS}#include <orocos/${ROSMSGSUBSYSTEMPORTSHEADER}>\n")
  # ros_msg_typekit_plugin.cpp.in
  set(ROSMSGTYPELINE "
      void rtt_ros_addType_${_package}_${ROSMSGNAME}() {
           // Only the .msg type is sent over ports. The msg[] (variable size) and  cmsg[] (fixed size) exist only as members of larger messages
           RTT::types::Types()->addType( new types::StructTypeInfo<${ROSMSGTYPE}>(\"${ROSMSGTYPENAME}\") );
           RTT::types::Types()->addType( new types::PrimitiveSequenceTypeInfo<std::vector<${ROSMSGTYPE}> >(\"${ROSMSGTYPENAME}[]\") );
           RTT::types::Types()->addType( new types::CArrayTypeInfo<RTT::types::carray<${ROSMSGTYPE}> >(\"${ROSMSGCTYPENAME}[]\") );
      }\n")
  # Types.hpp.in
#  set(ROSMSGTYPESHEADERS "${ROSMSGTYPESHEADERS}#include \"${ROSMSGNAME}.h\"\n")

  # Necessary for create_subsystem_ports_header.py command below
  set(_ROSMSG_GENERATED_SUBSYSTEM_PORTS_HEADER  "${rtt_subsystem_ports_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos/${ROSMSGSUBSYSTEMPORTSHEADER}")
  list(APPEND ROSMSGS_GENERATED_SUBSYSTEM_PORTS_HEADERS ${_ROSMSG_GENERATED_SUBSYSTEM_PORTS_HEADER})

  add_custom_command(
    OUTPUT ${_ROSMSG_GENERATED_SUBSYSTEM_PORTS_HEADER}
    COMMAND ${CREATE_SUBSYSTEM_PORTS_HEADER_EXE_PATH} ${_package} "${_package}/${ROSMSGNAME}" ${FILE} ${_ROSMSG_GENERATED_SUBSYSTEM_PORTS_HEADER}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS ${FILE} ${${_package}_EXPORTED_TARGETS} ${CREATE_SUBSYSTEM_PORTS_HEADER_EXE_PATH}
    VERBATIM)

  #set_source_files_properties(${ROSMSGS_GENERATED_SUBSYSTEM_PORTS_HEADERS} PROPERTIES GENERATED TRUE)

  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ros_${ROSMSGNAME}_subsystem_ports.cpp
    COMMAND ${CREATE_SUBSYSTEM_PORTS_SOURCE_EXE_PATH} ${_package} "${_package}/${ROSMSGNAME}" ${FILE} ${CMAKE_CURRENT_BINARY_DIR}/ros_${ROSMSGNAME}_subsystem_ports.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS ${FILE} ${${_package}_EXPORTED_TARGETS} ${CREATE_SUBSYSTEM_PORTS_SOURCE_EXE_PATH}
    VERBATIM)
#  list(APPEND rtt-${_package}-components_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/ros_${ROSMSGNAME}_subsystem_ports.cpp )
  list(APPEND rtt-${_package}-library_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/ros_${ROSMSGNAME}_subsystem_ports.cpp )
  add_file_dependencies(${CMAKE_CURRENT_BINARY_DIR}/ros_${ROSMSGNAME}_subsystem_ports.cpp ${CMAKE_CURRENT_LIST_FILE} ${_ROSMSG_GENERATED_SUBSYSTEM_PORTS_HEADER})


#  get_filename_component(MSG_FILENAME <FileName> <COMP> [CACHE])
  list (FIND _messages "${ROSMSGNAME}" _index)
  if (${_index} GREATER -1)
      add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_package}_${ROSMSGNAME}_components.cpp
        COMMAND ${CREATE_COMPONENTS_SOURCE_EXE_PATH} ${_package} "${_package}/${ROSMSGNAME}" ${FILE} ${CMAKE_CURRENT_BINARY_DIR}/${_package}_${ROSMSGNAME}_components.cpp
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        DEPENDS ${FILE} ${${_package}_EXPORTED_TARGETS} ${CREATE_COMPONENTS_SOURCE_EXE_PATH}
        VERBATIM)
      list(APPEND rtt-${_package}-components_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/${_package}_${ROSMSGNAME}_components.cpp )
      add_file_dependencies(${CMAKE_CURRENT_BINARY_DIR}/${_package}_${ROSMSGNAME}_components.cpp ${CMAKE_CURRENT_LIST_FILE} ${_ROSMSG_GENERATED_SUBSYSTEM_PORTS_HEADER})
  endif()

endforeach( FILE ${MSG_FILES} )

configure_file(
  component.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/${_package}_component.cpp @ONLY )
list(APPEND rtt-${_package}-components_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/${_package}_component.cpp )

include_directories(
  ${rtt_subsystem_ports_GENERATED_HEADERS_OUTPUT_DIRECTORY}
  ${rtt_subsystem_ports_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos
  ${rtt_subsystem_ports_GENERATED_HEADERS_INSTALL_DESTINATION}/orocos
  ${USE_OROCOS_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${${_package}_INCLUDE_DIRS})

# Targets
if(NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_BUILD_TYPE MinSizeRel)
endif()

add_library(rtt-${_package}-subsystem_ports ${rtt-${_package}-library_SOURCES})
target_link_libraries(rtt-${_package}-subsystem_ports ${catkin_LIBRARIES} ${${_package}_LIBRARIES} ${USE_OROCOS_LIBRARIES})
list(APPEND OROCOS_DEFINED_TYPES " -lrtt-${_package}-subsystem_ports")

if(DEFINED _enable_converter)
  set(CREATE_CONVERTER_EXE_PATH ${rtt_subsystem_ports_DIR}/create_converter.py)

  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/buffer_converter.cpp
    COMMAND ${CREATE_CONVERTER_EXE_PATH} ${_package} ${CMAKE_CURRENT_BINARY_DIR}/buffer_converter.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS ${${_package}_EXPORTED_TARGETS} ${CREATE_CONVERTER_EXE_PATH}
    VERBATIM)

  orocos_component(rtt-${_package}-subsystem_ports-components ${rtt-${_package}-components_SOURCES} ${CMAKE_CURRENT_BINARY_DIR}/buffer_converter.cpp)
else()
  orocos_component(rtt-${_package}-subsystem_ports-components ${rtt-${_package}-components_SOURCES})
endif()

target_link_libraries(rtt-${_package}-subsystem_ports-components ${catkin_LIBRARIES} ${${_package}_LIBRARIES} ${USE_OROCOS_LIBRARIES} rtt-${_package}-subsystem_ports)


# Add an explicit dependency between the typekits and message files
# TODO: Add deps for all msg dependencies
if(${_package}_EXPORTED_TARGETS)
  if(NOT ${_package} STREQUAL ${PROJECT_NAME})
    add_dependencies(rtt-${_package}-subsystem_ports ${${_package}_EXPORTED_TARGETS})
    add_dependencies(rtt-${_package}-subsystem_ports-components ${${_package}_EXPORTED_TARGETS})
  endif()
endif()

# Add the typekit libraries to the dependecies exported by this project
#    LIST(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS "rtt-${_package}-subsystem_ports")  # <-- This is already done in orocos_library().
#    LIST(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS "rtt-${_package}-subsystem_ports-components")  # <-- This is already done in orocos_component().
LIST(APPEND ${PROJECT_NAME}_EXPORTED_INCLUDE_DIRS "${rtt_subsystem_ports_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos" ${${_package}_INCLUDE_DIRS})

#add_file_dependencies(${CMAKE_CURRENT_BINARY_DIR}/ros_${_package}_typekit.cpp ${CMAKE_CURRENT_LIST_FILE} ${ROSMSGS_GENERATED_SUBSYSTEM_PORTS_HEADERS})

get_directory_property(_additional_make_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
list(APPEND _additional_make_clean_files "${rtt-${_package}-components_SOURCES};${rtt_subsystem_ports_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos/${_package}")
set_directory_properties(PROPERTIES
  ADDITIONAL_MAKE_CLEAN_FILES "${_additional_make_clean_files}")

# Install generated header files (dependent packages might need them)
if(DEFINED rtt_subsystem_ports_GENERATED_HEADERS_INSTALL_DESTINATION)
  install(
    DIRECTORY "${rtt_subsystem_ports_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos/${_package}"
    DESTINATION "${rtt_subsystem_ports_GENERATED_HEADERS_INSTALL_DESTINATION}/orocos")
endif()

# Export variables to the PARENT_SCOPE
set(OROCOS_DEFINED_TYPES ${OROCOS_DEFINED_TYPES} PARENT_SCOPE)
set(${PROJECT_NAME}_EXPORTED_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} PARENT_SCOPE)
set(${PROJECT_NAME}_EXPORTED_INCLUDE_DIRS ${${PROJECT_NAME}_EXPORTED_INCLUDE_DIRS} PARENT_SCOPE)
