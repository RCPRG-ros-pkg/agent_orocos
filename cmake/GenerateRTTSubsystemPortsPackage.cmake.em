# 
# Generate RTT subsystem ports from ROS .msg messages
#

cmake_minimum_required(VERSION 2.8.3)

macro(rtt_subsystem_ports_destinations)
  if(ORO_USE_ROSBUILD)
    #message(STATUS "[ros_generate_rtt_subsystem_ports] Generating ROS typekit for ${PROJECT_NAME} with ROSBuild destinations.")
    set(rtt_subsystem_ports_GENERATED_HEADERS_OUTPUT_DIRECTORY    "${PROJECT_SOURCE_DIR}/include")
    set(rtt_subsystem_ports_GENERATED_HEADERS_INSTALL_DESTINATION)
  elseif(ORO_USE_CATKIN)
    #message(STATUS "[ros_generate_rtt_subsystem_ports] Generating ROS typekit for ${PROJECT_NAME} with Catkin destinations.")
    catkin_destinations()
    set(rtt_subsystem_ports_GENERATED_HEADERS_OUTPUT_DIRECTORY    "${CATKIN_DEVEL_PREFIX}/include")
    set(rtt_subsystem_ports_GENERATED_HEADERS_INSTALL_DESTINATION "${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  else()
    #message(STATUS "[ros_generate_rtt_subsystem_ports] Generating ROS typekit for ${PROJECT_NAME} with normal CMake destinations.")
    set(rtt_subsystem_ports_GENERATED_HEADERS_OUTPUT_DIRECTORY    "${PROJECT_BINARY_DIR}/include")
    set(rtt_subsystem_ports_GENERATED_HEADERS_INSTALL_DESTINATION "${CMAKE_INSTALL_PREFIX}/include")
  endif()

  if(DEFINED ENV{VERBOSE_CONFIG})
    message(STATUS "[ros_generate_rtt_subsystem_ports]   Generating headers in: ${rtt_subsystem_ports_GENERATED_HEADERS_OUTPUT_DIRECTORY}")
    message(STATUS "[ros_generate_rtt_subsystem_ports]   Installing headers to: ${rtt_subsystem_ports_GENERATED_HEADERS_INSTALL_DESTINATION}")
  endif()
endmacro()


#macro(rtt_master_destinations)
#  if(ORO_USE_ROSBUILD)
#    #message(STATUS "[ros_generate_rtt_master] Generating ROS typekit for ${PROJECT_NAME} with ROSBuild destinations.")
#    set(rtt_master_GENERATED_HEADERS_OUTPUT_DIRECTORY    "${PROJECT_SOURCE_DIR}/include")
#    set(rtt_master_GENERATED_HEADERS_INSTALL_DESTINATION)
#  elseif(ORO_USE_CATKIN)
#    #message(STATUS "[ros_generate_rtt_master] Generating ROS typekit for ${PROJECT_NAME} with Catkin destinations.")
#    catkin_destinations()
#    set(rtt_master_GENERATED_HEADERS_OUTPUT_DIRECTORY    "${CATKIN_DEVEL_PREFIX}/include")
#    set(rtt_master_GENERATED_HEADERS_INSTALL_DESTINATION "${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
#  else()
#    #message(STATUS "[ros_generate_rtt_master] Generating ROS typekit for ${PROJECT_NAME} with normal CMake destinations.")
#    set(rtt_master_GENERATED_HEADERS_OUTPUT_DIRECTORY    "${PROJECT_BINARY_DIR}/include")
#    set(rtt_master_GENERATED_HEADERS_INSTALL_DESTINATION "${CMAKE_INSTALL_PREFIX}/include")
#  endif()
#  if(DEFINED ENV{VERBOSE_CONFIG})
#    message(STATUS "[ros_generate_rtt_master]   Generating headers in: ${rtt_master_GENERATED_HEADERS_OUTPUT_DIRECTORY}")
#    message(STATUS "[ros_generate_rtt_master]   Installing headers to: ${rtt_master_GENERATED_HEADERS_INSTALL_DESTINATION}")
#  endif()
#endmacro()

macro(msgs_from_ec_config_destinations)
  if(ORO_USE_ROSBUILD)
    #message(STATUS "[ros_generate_msgs_from_ec_config] Generating ROS typekit for ${PROJECT_NAME} with ROSBuild destinations.")
#    set(msgs_from_ec_config_GENERATED_MSGS_OUTPUT_DIRECTORY    "${PROJECT_SOURCE_DIR}/msg")
#    set(msgs_from_ec_config_GENERATED_MSGS_INSTALL_DESTINATION)

    set(msgs_from_ec_config_GENERATED_HEADERS_OUTPUT_DIRECTORY    "${PROJECT_SOURCE_DIR}/include")
    set(msgs_from_ec_config_GENERATED_HEADERS_INSTALL_DESTINATION)
  elseif(ORO_USE_CATKIN)
    #message(STATUS "[ros_generate_msgs_from_ec_config] Generating ROS typekit for ${PROJECT_NAME} with Catkin destinations.")
    catkin_destinations()
#    set(msgs_from_ec_config_GENERATED_MSGS_OUTPUT_DIRECTORY    "${CATKIN_DEVEL_PREFIX}/msg")
#    set(msgs_from_ec_config_GENERATED_MSGS_INSTALL_DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/msg")

    set(msgs_from_ec_config_GENERATED_HEADERS_OUTPUT_DIRECTORY    "${CATKIN_DEVEL_PREFIX}/include")
    set(msgs_from_ec_config_GENERATED_HEADERS_INSTALL_DESTINATION "${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  else()
    #message(STATUS "[ros_generate_msgs_from_ec_config] Generating ROS typekit for ${PROJECT_NAME} with normal CMake destinations.")
#    set(msgs_from_ec_config_GENERATED_MSGS_OUTPUT_DIRECTORY    "${PROJECT_BINARY_DIR}/msg")
#    set(msgs_from_ec_config_GENERATED_MSGS_INSTALL_DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/msg")

    set(msgs_from_ec_config_GENERATED_HEADERS_OUTPUT_DIRECTORY    "${PROJECT_BINARY_DIR}/include")
    set(msgs_from_ec_config_GENERATED_HEADERS_INSTALL_DESTINATION "${CMAKE_INSTALL_PREFIX}/include")
  endif()

#  if(DEFINED ENV{VERBOSE_CONFIG})
#    message(STATUS "[ros_generate_msgs_from_ec_config]   Generating headers in: ${msgs_from_ec_config_GENERATED_HEADERS_OUTPUT_DIRECTORY}")
#    message(STATUS "[ros_generate_msgs_from_ec_config]   Installing headers to: ${msgs_from_ec_config_GENERATED_HEADERS_INSTALL_DESTINATION}")
#  endif()
endmacro()

macro(rtt_subsystem_ports_debug)
  if(DEFINED ENV{VERBOSE_CONFIG})
    message(STATUS "[ros_generate_rtt_subsystem_ports]     catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
  endif()
endmacro()

macro(ros_generate_rtt_subsystem_ports)
  set( _OPTIONS_ARGS ENABLE_CONVERTER)
  set( _ONE_VALUE_ARGS PACKAGE )
  set( _MULTI_VALUE_ARGS MESSAGES )

  cmake_parse_arguments( _ROS_GENERATE_RTT_SUBSYSTEM_PORTS "${_OPTIONS_ARGS}" "${_ONE_VALUE_ARGS}" "${_MULTI_VALUE_ARGS}" ${ARGN} )

  # Mandatory
  if( _ROS_GENERATE_RTT_SUBSYSTEM_PORTS_PACKAGE )
#    message( ERROR "inside PACKAGE=${_ROS_GENERATE_RTT_SUBSYSTEM_PORTS_PACKAGE}" )
  else()
    message( FATAL_ERROR "ros_generate_rtt_subsystem_ports: 'PACKAGE' argument required." )
  endif()

  set(_package "${_ROS_GENERATE_RTT_SUBSYSTEM_PORTS_PACKAGE}")
  set(_messages "${_ROS_GENERATE_RTT_SUBSYSTEM_PORTS_MESSAGES}")

  if( _ROS_GENERATE_RTT_SUBSYSTEM_PORTS_ENABLE_CONVERTER )
    set(_enable_converter "aaa")
#    add_subdirectory(${rtt_subsystem_ports_DIR}/../src/templates/buffer_converter ${_package}_buffer_converter)
  endif()

  add_subdirectory(${rtt_subsystem_ports_DIR}/../src/templates/subsystem_ports ${_package}_subsystem_ports)
endmacro(ros_generate_rtt_subsystem_ports)

macro(generate_msgs_from_ec_config config_file)
  set(_package ${PROJECT_NAME})
  set(_config_file ${config_file})
  add_subdirectory(${rtt_subsystem_ports_DIR}/../src/templates/msgs_from_ec_config ${PROJECT_NAME}_msgs_from_ec_config)
endmacro(generate_msgs_from_ec_config)

macro(build_ec_msg_converter)
  set(_package ${PROJECT_NAME})
  add_subdirectory(${rtt_subsystem_ports_DIR}/../src/templates/ec_msg_converter ${PROJECT_NAME}_ec_msg_converter)
endmacro(build_ec_msg_converter)

