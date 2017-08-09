find_package(OROCOS-RTT 2.0.0 COMPONENTS rtt-scripting rtt-marshalling)
if (NOT OROCOS-RTT_FOUND)
  message(FATAL_ERROR "\n   RTT not found. Is the version correct? Use the CMAKE_PREFIX_PATH cmake or environment variable to point to the installation directory of RTT.")
else()
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
  #add_definitions( -DRTT_COMPONENT )
endif()

include(AddFileDependencies)

# Configure source and destination paths of generated files
rtt_master_destinations()

#set(_package ${PROJECT_NAME})

#find_package(genmsg REQUIRED)

set(PORT_DEF_FILE ${PROJECT_SOURCE_DIR}/spec/subsystem.xml)

# Set the subsystem ports header generation script path
set(CREATE_MASTER_CPP_EXE_PATH ${rtt_subsystem_DIR}/create_master_cpp.py)
set(CREATE_MASTER_H_EXE_PATH ${rtt_subsystem_DIR}/create_master_h.py)
set(CREATE_SUBSYSTEM_OUTPUT_BUFFERS_EXE_PATH ${rtt_subsystem_DIR}/create_subsystem_output_buffers.py)

set(MASTER_H_OUTPUT ${rtt_master_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos/${_package}/master.h)
set(MASTER_CPP_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/master.cpp)
#set(MASTER_LIBRARY_CPP_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/master_library.cpp)
set(SUBSYSTEM_OUTPUT_BUFFERS_CPP_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/output_buffers.cpp)

#get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#message(WARNING "${dirs}")

add_custom_command(
  OUTPUT ${MASTER_H_OUTPUT}
  COMMAND ${CREATE_MASTER_H_EXE_PATH} ${_package} ${PORT_DEF_FILE} ${MASTER_H_OUTPUT}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  DEPENDS ${PORT_DEF_FILE} ${${_package}_EXPORTED_TARGETS} ${CREATE_INPUT_DATA_EXE_PATH}
  VERBATIM)

include_directories(${rtt_master_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos)
set(PROPAGATED_UP_INCLUDE_DIRECTORIES ${rtt_master_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos PARENT_SCOPE)

add_custom_command(
  OUTPUT ${MASTER_CPP_OUTPUT}
  COMMAND ${CREATE_MASTER_CPP_EXE_PATH} ${_package} ${PORT_DEF_FILE} ${MASTER_CPP_OUTPUT}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  DEPENDS ${PORT_DEF_FILE} ${${_package}_EXPORTED_TARGETS} ${CREATE_MASTER_CPP_EXE_PATH}
  VERBATIM)
list(APPEND rtt-${_package}-library_SOURCES ${MASTER_CPP_OUTPUT} )
add_file_dependencies(${MASTER_CPP_OUTPUT} ${CMAKE_CURRENT_LIST_FILE} ${MASTER_H_OUTPUT})

orocos_service(rtt-${_package}-master ${rtt-${_package}-library_SOURCES})
target_link_libraries(rtt-${_package}-master ${catkin_LIBRARIES} ${${_package}_LIBRARIES} ${USE_OROCOS_LIBRARIES})

ADD_CUSTOM_TARGET(${PROJECT_NAME}_MASTER_HEADERS DEPENDS ${MASTER_H_OUTPUT})


add_custom_command(
  OUTPUT ${SUBSYSTEM_OUTPUT_BUFFERS_CPP_OUTPUT}
  COMMAND ${CREATE_SUBSYSTEM_OUTPUT_BUFFERS_EXE_PATH} ${_package} ${PORT_DEF_FILE} ${SUBSYSTEM_OUTPUT_BUFFERS_CPP_OUTPUT}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  DEPENDS ${PORT_DEF_FILE} ${${_package}_EXPORTED_TARGETS} ${CREATE_SUBSYSTEM_OUTPUT_BUFFERS_EXE_PATH}
  VERBATIM)
list(APPEND subsystem_buffers-${_package}-library_SOURCES ${SUBSYSTEM_OUTPUT_BUFFERS_CPP_OUTPUT} )
add_file_dependencies(${SUBSYSTEM_OUTPUT_BUFFERS_CPP_OUTPUT} ${CMAKE_CURRENT_LIST_FILE} ${MASTER_H_OUTPUT})

configure_file(
  component.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/${_package}_component.cpp @ONLY )
list(APPEND subsystem_buffers-${_package}-library_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/${_package}_component.cpp )

orocos_component(subsystem_buffers-${_package} ${subsystem_buffers-${_package}-library_SOURCES})
target_link_libraries(subsystem_buffers-${_package} ${catkin_LIBRARIES} ${${_package}_LIBRARIES} ${USE_OROCOS_LIBRARIES})




#add_library(rtt-${_package}-master-library

#message(WARNING "${PROJECT_NAME}_INCLUDE_DIRS: ${${PROJECT_NAME}_INCLUDE_DIRS}")
#LIST(APPEND ${PROJECT_NAME}_INCLUDE_DIRS "${rtt_master_GENERATED_HEADERS_OUTPUT_DIRECTORY}")

#  set(ROSMSGSUBSYSTEMPORTSHEADER  "${_package}/subsystem_ports/${ROSMSGNAME}.h")
#  set(_ROSMSG_GENERATED_SUBSYSTEM_PORTS_HEADER  "${rtt_subsystem_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos/${ROSMSGSUBSYSTEMPORTSHEADER}")

#LIST(APPEND ${PROJECT_NAME}_EXPORTED_INCLUDE_DIRS "${rtt_master_GENERATED_HEADERS_OUTPUT_DIRECTORY}" ${${_package}_INCLUDE_DIRS})
#set(${PROJECT_NAME}_EXPORTED_INCLUDE_DIRS ${${PROJECT_NAME}_EXPORTED_INCLUDE_DIRS} PARENT_SCOPE)
#set(${PROJECT_NAME}_INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRS} PARENT_SCOPE)


## Set the subsystem ports header generation script path
#set(CREATE_SUBSYSTEM_PORTS_HEADER_EXE_PATH ${rtt_subsystem_DIR}/create_subsystem_ports_header.py)

## Set the subsystem ports sources generation script path
#set(CREATE_SUBSYSTEM_PORTS_SOURCE_EXE_PATH ${rtt_subsystem_DIR}/create_subsystem_ports_source.py)

## Set the subsystem ports components' sources generation script path
#set(CREATE_COMPONENTS_SOURCE_EXE_PATH ${rtt_subsystem_DIR}/create_components_source.py)

## Store the ros package name
#set(ROSPACKAGE ${_package})

set(${PROJECT_NAME}_MASTER_HEADER_FILES)
list(APPEND ${PROJECT_NAME}_MASTER_HEADER_FILES ${MASTER_H_OUTPUT})

set(${PROJECT_NAME}_MASTER_HEADER_FILES ${${PROJECT_NAME}_MASTER_HEADER_FILES} PARENT_SCOPE)

# Install generated header files (dependent packages might need them)
if(DEFINED rtt_master_GENERATED_HEADERS_INSTALL_DESTINATION)
  install(
    DIRECTORY "${rtt_master_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos/${_package}"
    DESTINATION "${rtt_master_GENERATED_HEADERS_INSTALL_DESTINATION}/orocos")
endif()

# Export variables to the PARENT_SCOPE
set(OROCOS_DEFINED_TYPES ${OROCOS_DEFINED_TYPES} PARENT_SCOPE)
set(${PROJECT_NAME}_EXPORTED_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} PARENT_SCOPE)
set(${PROJECT_NAME}_EXPORTED_INCLUDE_DIRS ${${PROJECT_NAME}_EXPORTED_INCLUDE_DIRS} PARENT_SCOPE)

