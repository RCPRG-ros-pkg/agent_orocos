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
