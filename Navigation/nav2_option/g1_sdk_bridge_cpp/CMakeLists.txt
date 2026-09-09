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

# --- 本番実行ファイル(unitree_sdk2 が在る環境でのみビルドする) -----------------
#
# SDK が無い開発機(このリポジトリを読むだけの環境、CI 等)でも従来どおりビルドできるよう、
# 見つからなければ警告を出してスキップする。実機接続機(PC2)では
# /usr/local に install 済みのヘッダ・ライブラリが見つかる。
#
# 探索先を明示したい場合: cmake -DUNITREE_SDK2_ROOT=/path/to/unitree_sdk2
set(UNITREE_SDK2_ROOT "" CACHE PATH "unitree_sdk2 のルート(ソースツリー or install先)")

# find_path/find_library を個別に使うと、ヘッダとライブラリが**別の場所から**
# 拾われることがある(実測: PC2 では /usr/local/include に G1 の loco ヘッダが
# 入っておらず、ヘッダはソースツリー・ライブラリは /usr/local という食い違いが起きた)。
# そこで「1つの prefix の中で全部揃っているか」を順に確かめる方式にする。
set(_g1_sdk_prefixes "")
if(UNITREE_SDK2_ROOT)
    list(APPEND _g1_sdk_prefixes "${UNITREE_SDK2_ROOT}")
endif()
list(APPEND _g1_sdk_prefixes /usr/local /usr)

set(UNITREE_SDK2_PREFIX "")
foreach(_prefix IN LISTS _g1_sdk_prefixes)
    if(UNITREE_SDK2_PREFIX)
        continue()
    endif()
    # G1 の高レベルAPI(LocoClient)のヘッダが在ることを必須条件にする。
    # ⚠️ install 済みの unitree(/usr/local)には G1 の loco ヘッダが無い場合がある(PC2 で実測)
    if(NOT EXISTS "${_prefix}/include/unitree/robot/g1/loco/g1_loco_client.hpp")
        continue()
    endif()
    # 静的ライブラリ: ソースツリーは lib/<arch>/、install 先は lib/
    set(_lib "")
    foreach(_cand "${_prefix}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_sdk2.a" "${_prefix}/lib/libunitree_sdk2.a")
        if(NOT _lib AND EXISTS "${_cand}")
            set(_lib "${_cand}")
        endif()
    endforeach()
    if(NOT _lib)
        continue()
    endif()
    # CycloneDDS のヘッダ。**C++ 版の <dds/dds.hpp> は `ddscxx/` 配下**にあり、
    # `thirdparty/include/dds/` は C 版(dds.h)なので取り違えないこと(PC2 で実測)。
    # C 版のヘッダも C++ 版から間接的に必要になるため、両方の include ディレクトリを渡す。
    set(_dds_inc "")
    foreach(_cand "${_prefix}/thirdparty/include" "${_prefix}/include")
        if(NOT _dds_inc AND EXISTS "${_cand}/ddscxx/dds/dds.hpp")
            set(_dds_inc "${_cand}")
        endif()
    endforeach()
    if(NOT _dds_inc)
        continue()
    endif()
    # CycloneDDS の共有ライブラリ
    set(_dds_libdir "")
    foreach(_cand "${_prefix}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}" "${_prefix}/lib")
        if(NOT _dds_libdir AND EXISTS "${_cand}/libddscxx.so")
            set(_dds_libdir "${_cand}")
        endif()
    endforeach()
    if(NOT _dds_libdir)
        continue()
    endif()

    set(UNITREE_SDK2_PREFIX "${_prefix}")
    set(UNITREE_SDK2_INCLUDE_DIR "${_prefix}/include")
    set(UNITREE_SDK2_LIBRARY "${_lib}")
    set(UNITREE_DDS_INCLUDE_DIR "${_dds_inc}" "${_dds_inc}/ddscxx")
    set(UNITREE_DDSC_LIBRARY "${_dds_libdir}/libddsc.so")
    set(UNITREE_DDSCXX_LIBRARY "${_dds_libdir}/libddscxx.so")
endforeach()

# ⚠️ CycloneDDS をリンクするのは正常。D-08 が禁じているのは **ROS 側の**
# rmw/rclcpp/ament の混入で、受入チェックは
# `ldd <bin> | grep -E 'rmw|rclcpp|ament'` が空であること(ddsc は出てよい)。

if(UNITREE_SDK2_PREFIX)
    message(STATUS "unitree_sdk2 が見つかった: ${UNITREE_SDK2_PREFIX} (lib=${UNITREE_SDK2_LIBRARY}) → 本番実行ファイルをビルドする")

    add_library(g1_sdk_bridge_real src/real_move_backend.cpp)
    target_include_directories(g1_sdk_bridge_real
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE ${UNITREE_SDK2_INCLUDE_DIR} ${UNITREE_DDS_INCLUDE_DIR}
    )
    target_link_libraries(g1_sdk_bridge_real
        PUBLIC g1_sdk_bridge
        PRIVATE ${UNITREE_SDK2_LIBRARY} ${UNITREE_DDSCXX_LIBRARY} ${UNITREE_DDSC_LIBRARY}
    )
    target_compile_options(g1_sdk_bridge_real PRIVATE -Wall -Wextra)

    add_executable(g1_sdk_bridge_real_server src/real_server_main.cpp)
    target_link_libraries(g1_sdk_bridge_real_server PRIVATE g1_sdk_bridge_real)
    target_compile_options(g1_sdk_bridge_real_server PRIVATE -Wall -Wextra)
else()
    message(WARNING
        "unitree_sdk2 が見つからないため本番実行ファイル(g1_sdk_bridge_real_server)を"
        "スキップする。モック側(g1_sdk_bridge_mock_server)とテストはビルドされる。"
        " 実機接続機で使う場合は -DUNITREE_SDK2_ROOT=... を指定すること。")
endif()

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
