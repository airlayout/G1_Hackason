cmake_minimum_required(VERSION 3.16)
project(g1_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps scripts DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/fake_sensor_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
