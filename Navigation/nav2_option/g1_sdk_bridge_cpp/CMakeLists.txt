cmake_minimum_required(VERSION 3.16)
project(g1_sdk_bridge_cpp CXX)

# D-08: SDK側プロセスは colcon workspace 外の独立CMakeプロジェクトとしてビルドする。
# ROS 2(ament_cmake等)を一切要求しない。ROS環境がsourceされた状態でもされていない状態でも
# 同じようにビルドできることが、DDSライブラリ混入を構造的に防ぐための前提(D-06)。

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(Threads REQUIRED)

add_library(g1_sdk_bridge
    src/protocol.cpp
    src/ipc_transport.cpp
    src/safety_manager.cpp
    src/sdk_bridge_process.cpp
)
target_include_directories(g1_sdk_bridge PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(g1_sdk_bridge PUBLIC Threads::Threads)
target_compile_options(g1_sdk_bridge PRIVATE -Wall -Wextra -Wpedantic)

# 受入基準(D-08): このライブラリ・実行ファイルにROS/DDS関連のシンボルが混入していないこと。
# `ldd`/`nm`によるチェックはPhase 0で実機バイナリに対して実施する(Planning.md参照)。

# 開発・動作確認用スタンドアロンサーバ(MockMoveBackend使用)。本番では使わない。
add_executable(g1_sdk_bridge_mock_server src/mock_server_main.cpp)
target_link_libraries(g1_sdk_bridge_mock_server PRIVATE g1_sdk_bridge)

option(G1_SDK_BRIDGE_BUILD_TESTS "単体テストをビルドする" ON)
if(G1_SDK_BRIDGE_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

    add_executable(g1_sdk_bridge_tests
        tests/test_protocol.cpp
        tests/test_ipc_transport.cpp
        tests/test_safety_manager.cpp
        tests/test_sdk_bridge.cpp
    )
    target_link_libraries(g1_sdk_bridge_tests PRIVATE g1_sdk_bridge GTest::gtest GTest::gtest_main Threads::Threads)
    target_compile_options(g1_sdk_bridge_tests PRIVATE -Wall -Wextra)

    include(GoogleTest)
    gtest_discover_tests(g1_sdk_bridge_tests)
endif()
