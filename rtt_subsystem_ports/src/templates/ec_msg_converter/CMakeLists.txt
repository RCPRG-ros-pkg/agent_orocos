find_package(OROCOS-RTT 2.0.0 COMPONENTS rtt-scripting rtt-marshalling)
if (NOT OROCOS-RTT_FOUND)
  message(FATAL_ERROR "\n   RTT not found. Is the version correct? Use the CMAKE_PREFIX_PATH cmake or environment variable to point to the installation directory of RTT.")
else()
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
  #add_definitions( -DRTT_COMPONENT )
endif()

include(AddFileDependencies)

#find_package(catkin REQUIRED message_generation)

# Configure source and destination paths of generated files
msgs_from_ec_config_destinations()

# Check if we're generating code for messages in this package
if("${_package}" STREQUAL "${PROJECT_NAME}")
  set(${_package}_FOUND True)
else()
  find_package(${_package} QUIET)
endif()

message(ERROR "EC_MSG_CONVERTER_FILES: ${EC_MSG_CONVERTER_FILES}")
#find_package(genmsg REQUIRED)

message(ERROR "${_package}_INCLUDE_DIRS: ${${_package}_INCLUDE_DIRS}")

message(ERROR "ALL_GEN_OUTPUT_FILES_cpp: ${ALL_GEN_OUTPUT_FILES_cpp}")

message(ERROR "adding include directory: ${PKG_MSG_INCLUDE_DIRS}")
include_directories(${PKG_MSG_INCLUDE_DIRS})

SET_SOURCE_FILES_PROPERTIES( ${EC_MSG_CONVERTER_FILES} 
      PROPERTIES OBJECT_DEPENDS "$${ALL_GEN_OUTPUT_FILES_cpp}" ) 

#add_library(${_package}-ec_msg_converter ${EC_MSG_CONVERTER_FILES})
#target_link_libraries(${_package}-ec_msg_converter ${catkin_LIBRARIES} ${${_package}_LIBRARIES})

#add_file_dependencies(${EC_MSG_CONVERTER_FILES} ${CMAKE_CURRENT_LIST_FILE} "/home/dseredyn/ws_full/top/devel/.private/test_ec_msgs_gen/include/test_ec_msgs_gen/EcInput.h")
#add_file_dependencies(${EC_MSG_CONVERTER_FILES} ${CMAKE_CURRENT_LIST_FILE} ${ALL_GEN_OUTPUT_FILES_cpp})

# Set the subsystem ports header generation script path
#set(CREATE_MSGS_FROM_EC_CONFIG_EXE_PATH ${rtt_subsystem_ports_DIR}/create_msgs_from_ec_config.py)

# Set the subsystem ports sources generation script path
#set(CREATE_SUBSYSTEM_PORTS_SOURCE_EXE_PATH ${rtt_subsystem_ports_DIR}/create_subsystem_ports_source.py)

# Set the subsystem ports components' sources generation script path
#set(CREATE_COMPONENTS_SOURCE_EXE_PATH ${rtt_subsystem_ports_DIR}/create_components_source.py)

# Store the ros package name
#set(ROSPACKAGE ${_package})
#set(ECCONFIGFILE ${_config_file})

#set(ECCONFIG_DIR "${PROJECT_SOURCE_DIR}/ec_config")

#safe_execute_process(COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${CREATE_MSGS_FROM_EC_CONFIG_EXE_PATH} ${_package} ${ECCONFIG_DIR}/${ECCONFIGFILE} ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/msg)

#set(MESSAGE_DIR ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/msg)

#file(GLOB ARG_FILES RELATIVE "${MESSAGE_DIR}" "${MESSAGE_DIR}/*.msg")
#list(SORT ARG_FILES)

#set(OUTPUT_FILES ${ARG_FILES})

