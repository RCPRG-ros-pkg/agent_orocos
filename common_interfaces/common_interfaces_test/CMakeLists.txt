cmake_minimum_required(VERSION 2.8.3)
project(common_interfaces_test)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  shm_comm
  common_interfaces
  common_interfaces_test_msgs
  rtt_common_interfaces_test_subsystem_ports
  )

include_directories(${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

    find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
    include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

    include_directories(${USE_OROCOS_INCLUDE_DIRS} include)

    find_package(orocos-ocl REQUIRED)

    orocos_use_package( ocl-deployment )

    add_executable(deployer_tx tests/deployer_tx.cpp tests/test_deployer.cpp tests/container_utils.cpp)
    target_link_libraries(deployer_tx
        ${catkin_LIBRARIES}
        ${USE_OROCOS_LIBRARIES})

    add_executable(deployer_rx tests/deployer_rx.cpp tests/test_deployer.cpp tests/container_utils.cpp)
    target_link_libraries(deployer_rx
        ${catkin_LIBRARIES}
        ${USE_OROCOS_LIBRARIES})

    catkin_add_gtest(msg_unittest tests/gtest_main.cc tests/msg_concate_unittest.cpp tests/msg_split_unittest.cpp tests/msg_rx_tx.cpp tests/test_deployer.cpp)
#    catkin_add_gtest(msg_unittest tests/gtest_main.cc tests/msg_rx_tx.cpp tests/test_deployer.cpp)
    target_link_libraries(msg_unittest
        ${catkin_LIBRARIES}
        ${USE_OROCOS_LIBRARIES})


endif(CATKIN_ENABLE_TESTING)

catkin_package(
  CATKIN_DEPENDS rtt_ros shm_comm
)


