cmake_minimum_required(VERSION 3.16)
project(g1_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch scripts DESTINATION share/${PROJECT_NAME})

# ⚠️ **地図の実体は `nav2_stable/maps/` に集約してある**(2026-09-24)。
# 占有格子・点群・軌跡が4か所に散っていたのをまとめたため、ここからは相対参照する。
# (`tools/g1_slam_odom_tf.py` を実体1つにしているのと同じ方針)
set(G1_MAPS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../maps/grids)
if(NOT EXISTS ${G1_MAPS_DIR}/room_a_map_20260911.yaml)
  message(FATAL_ERROR "maps/grids が見つからない: ${G1_MAPS_DIR}")
endif()
install(DIRECTORY ${G1_MAPS_DIR}/ DESTINATION share/${PROJECT_NAME}/maps)

# ⚠️ `g1_slam_odom_tf.py` は `nav2_stable/tools/` に**実体を1つだけ**置き、
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
