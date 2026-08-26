cmake_minimum_required(VERSION 3.8)
project(g1_sensor_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pointcloud_to_livox src/pointcloud_to_livox.cpp)
ament_target_dependencies(pointcloud_to_livox
  livox_ros_driver2
  rclcpp
  sensor_msgs
)
target_compile_options(pointcloud_to_livox PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS pointcloud_to_livox DESTINATION lib/${PROJECT_NAME})

ament_package()
