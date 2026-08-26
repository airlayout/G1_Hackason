cmake_minimum_required(VERSION 3.16)
project(g1_onboard_lio LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(unitree_sdk2 REQUIRED)

add_executable(g1_onboard_lio src/g1_onboard_lio.cpp)
target_compile_options(g1_onboard_lio PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(g1_onboard_lio PRIVATE unitree_sdk2)

install(TARGETS g1_onboard_lio RUNTIME DESTINATION bin)
