# mola_relocalization の試作。コンテナの中で建てる（Mac には MRPT が無い）:
#   docker exec -u ubuntu rviz bash -c 'source /opt/ros/humble/setup.bash &&
#     cmake -S /work/.../quickstart/reloc -B /tmp/reloc_build && cmake --build /tmp/reloc_build'
cmake_minimum_required(VERSION 3.16)
project(reloc_probe CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(mola_relocalization REQUIRED)
# ⚠️ map.mm の層は mola::HashedVoxelPointCloud。**これをリンクしないと
# load_from_file が「class not registered」で落ちる**（2026-09-11 に踏んだ）。
# シンボルを 1 つも参照しないので --no-as-needed で落とされないようにする
find_package(mola_metric_maps REQUIRED)
find_package(mp2p_icp_map REQUIRED)
find_package(mrpt-maps REQUIRED)
find_package(mrpt-obs REQUIRED)
find_package(mrpt-poses REQUIRED)

add_executable(reloc_probe reloc_probe.cpp)
target_link_libraries(reloc_probe
  mola::mola_relocalization
  mola::mp2p_icp_map   # ⚠️ 名前空間つき。素の mp2p_icp_map は -l に落ちて link で落ちる
  mrpt::maps
  mrpt::obs
  mrpt::poses
  # 静的登録だけが目的。参照が無いので as-needed に落とされる
  mola::mola_metric_maps)   # 型を直接使うので as-needed でも落ちない
