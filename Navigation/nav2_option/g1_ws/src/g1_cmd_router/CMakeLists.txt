cmake_minimum_required(VERSION 3.16)
project(g1_cmd_router)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Threads REQUIRED)

# g1_sdk_bridge_cpp は独立CMakeプロジェクト(D-08)。ROS 2に依存しないロジック
# (protocol/ipc_transport/safety_manager)だけをここで直接コンパイルして使う。
# sdk_bridge_process.cpp(SDK側専用、ROS非依存プロセスにのみ使う)はここには含めない。
set(G1_SDK_BRIDGE_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../g1_sdk_bridge_cpp)
if(NOT EXISTS ${G1_SDK_BRIDGE_CPP_DIR}/include/g1_sdk_bridge/safety_manager.hpp)
  message(FATAL_ERROR "g1_sdk_bridge_cpp が見つからない: ${G1_SDK_BRIDGE_CPP_DIR}")
endif()

add_executable(g1_cmd_router_node
  src/main.cpp
  src/cmd_router_node.cpp
  ${G1_SDK_BRIDGE_CPP_DIR}/src/safety_manager.cpp
  ${G1_SDK_BRIDGE_CPP_DIR}/src/protocol.cpp
  ${G1_SDK_BRIDGE_CPP_DIR}/src/ipc_transport.cpp
)
target_include_directories(g1_cmd_router_node PRIVATE
  include
  ${G1_SDK_BRIDGE_CPP_DIR}/include
)
target_link_libraries(g1_cmd_router_node Threads::Threads)
ament_target_dependencies(g1_cmd_router_node
  rclcpp geometry_msgs std_srvs diagnostic_msgs
)

install(TARGETS g1_cmd_router_node DESTINATION lib/${PROJECT_NAME})

ament_package()
