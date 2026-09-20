cmake_minimum_required(VERSION 3.16)
project(g1_state_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

# g1_sdk_bridge_cpp は独立CMakeプロジェクト(D-08)。protocol/ipc_transportだけを直接コンパイルする。
set(G1_SDK_BRIDGE_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../g1_sdk_bridge_cpp)
if(NOT EXISTS ${G1_SDK_BRIDGE_CPP_DIR}/include/g1_sdk_bridge/ipc_transport.hpp)
  message(FATAL_ERROR "g1_sdk_bridge_cpp が見つからない: ${G1_SDK_BRIDGE_CPP_DIR}")
endif()

add_executable(g1_state_bridge_node
  src/main.cpp
  src/state_bridge_node.cpp
  ${G1_SDK_BRIDGE_CPP_DIR}/src/protocol.cpp
  ${G1_SDK_BRIDGE_CPP_DIR}/src/ipc_transport.cpp
)
target_include_directories(g1_state_bridge_node PRIVATE
  include
  ${G1_SDK_BRIDGE_CPP_DIR}/include
)
target_link_libraries(g1_state_bridge_node Threads::Threads)
ament_target_dependencies(g1_state_bridge_node
  rclcpp nav_msgs diagnostic_msgs geometry_msgs tf2_ros
)

install(TARGETS g1_state_bridge_node DESTINATION lib/${PROJECT_NAME})

ament_package()
