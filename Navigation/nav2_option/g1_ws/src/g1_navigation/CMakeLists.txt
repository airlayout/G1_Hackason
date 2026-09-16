cmake_minimum_required(VERSION 3.16)
project(g1_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps scripts DESTINATION share/${PROJECT_NAME})

# ⚠️ `g1_slam_odom_tf.py` は `nav2_option/tools/` に**実体を1つだけ**置き、
# ここからインストールする。コピーを作ると必ず片方だけ直して食い違う。
# (`g1_cmd_router` が `g1_sdk_bridge_cpp` を相対パスで参照しているのと同じ方針)
set(G1_TOOLS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../tools)
if(NOT EXISTS ${G1_TOOLS_DIR}/g1_slam_odom_tf.py)
  message(FATAL_ERROR "tools/g1_slam_odom_tf.py が見つからない: ${G1_TOOLS_DIR}")
endif()

install(PROGRAMS
  scripts/fake_sensor_publisher.py
  scripts/patrol_node.py
  ${G1_TOOLS_DIR}/g1_slam_odom_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
