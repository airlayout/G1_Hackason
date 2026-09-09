FROM osrf/ros:jazzy-desktop

# ---------------------------------------------------------------------------
# FAST_LIO_LOCALIZATION_HUMANOID (vendored, patched) -- ROS 2 Jazzy build
#
# 上流: https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID (humbleブランチ)
# このDockerfileは、このディレクトリ(vendor/fast_lio_localization_humanoid/)を
# ビルドコンテキストとして、vendoring済み・パッチ適用済みのソース(FAST_LIO/, open3d_loc/)を
# ビルドする。上流からの再取得はしない(vendoring方針、Planning.md Q7)。
#
# 適用済みパッチ(詳細はVENDOR_NOTES.mdを参照):
#   1. open3d_loc: Open3D >= 0.16でRegistrationRANSACBasedOnFeatureMatching()の
#      seed引数が削除された差分を吸収(open3d_registration.cpp)
#   2. open3d_loc: Open3D_DIRをCACHE変数化(CMakeLists.txt)
#   3. open3d_loc: initialposeパラメータのサイズ検証を追加し、
#      SIGSEGVではなく明確なエラーで停止するようにした(global_localization.cpp)
#
# ビルド方法:
#   cd nav2_option/vendor/fast_lio_localization_humanoid
#   docker build -t g1-fastlio-jazzy .
# ---------------------------------------------------------------------------

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

SHELL ["/bin/bash", "-c"]

# ---------------------------------------------------------------------------
# 1. apt依存
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl unzip xz-utils \
    python3-pip python3-dev \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    libyaml-cpp-dev pkg-config \
    libboost-filesystem-dev libboost-system-dev \
    libc++-dev libc++abi-dev \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-sensor-msgs \
    ros-jazzy-urdf \
    ros-jazzy-common-interfaces \
    ros-jazzy-eigen3-cmake-module \
    ros-jazzy-rcutils \
    ros-jazzy-ament-cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update || true

# ---------------------------------------------------------------------------
# 2. Livox-SDK2 (ROS非依存の素のCMakeライブラリ。livox_ros_driver2とfast_lioが必要とする)
# ---------------------------------------------------------------------------
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /opt/Livox-SDK2 \
    && cd /opt/Livox-SDK2 && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig

# ---------------------------------------------------------------------------
# 3. Open3D 0.18.0 prebuilt devel package (cxx11 ABI, Jazzy/GCC13のnew ABIと一致)
#    README指定の0.14.1はBaidu Netdisk限定配布で入手困難なため代替する。
# ---------------------------------------------------------------------------
RUN wget -q https://github.com/isl-org/Open3D/releases/download/v0.18.0/open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz -O /tmp/open3d-devel.tar.xz \
    && mkdir -p /opt/open3d \
    && tar -xf /tmp/open3d-devel.tar.xz -C /opt/open3d --strip-components=1 \
    && rm /tmp/open3d-devel.tar.xz

# ---------------------------------------------------------------------------
# 4. ワークスペース構成
# ---------------------------------------------------------------------------
RUN mkdir -p /root/ws_loc/src

# 4.1 livox_ros_driver2 -- 上流がJazzyを正式サポートしているため、vendoringせず
#     upstreamからそのまま取得する(build.shと同じ処理を手動再現)
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git /root/ws_loc/src/livox_ros_driver2 \
    && cd /root/ws_loc/src/livox_ros_driver2 \
    && cp -f package_ROS2.xml package.xml \
    && cp -rf launch_ROS2/. launch/

# 4.2 FAST_LIO / open3d_loc -- このリポジトリにvendoring済み・パッチ適用済みのソースを使う
#     (上流からの再取得はしない)
COPY FAST_LIO /root/ws_loc/src/FAST_LIO
COPY open3d_loc /root/ws_loc/src/open3d_loc
COPY data /root/ws_loc/src/data

# ---------------------------------------------------------------------------
# 5. rosdepで残りの依存を解決(--ignore-srcでワークスペース内のソースはスキップ)
# ---------------------------------------------------------------------------
RUN source /opt/ros/jazzy/setup.bash \
    && cd /root/ws_loc \
    && rosdep install --from-paths src --ignore-src -r -y || true

# ---------------------------------------------------------------------------
# 6. ビルド
# ---------------------------------------------------------------------------
WORKDIR /root/ws_loc
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy 2>&1 | tee /root/ws_loc/build.log

CMD ["/bin/bash"]
