FROM osrf/ros:jazzy-desktop

# ---------------------------------------------------------------------------
# FAST_LIO_LOCALIZATION_HUMANOID -- ROS 2 Jazzy buildability check
#
# Upstream: https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID
# The ROS2 code lives on the "humble" branch (the default "main" branch is a
# ROS1/catkin package despite the repo name). This Dockerfile builds the
# "humble" branch on ROS 2 Jazzy / Ubuntu 24.04 to check portability.
# ---------------------------------------------------------------------------

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

SHELL ["/bin/bash", "-c"]

# ---------------------------------------------------------------------------
# 1. apt dependencies
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

# rosdep is already initialized in the osrf/ros base image; ignore failures.
RUN rosdep init || true && rosdep update || true

# ---------------------------------------------------------------------------
# 2. Livox-SDK2 (plain CMake C/C++ library, no ROS dependency; required by
#    both livox_ros_driver2 and, transitively, fast_lio)
# ---------------------------------------------------------------------------
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /opt/Livox-SDK2 \
    && cd /opt/Livox-SDK2 && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig

# ---------------------------------------------------------------------------
# 3. Open3D 0.18.0 prebuilt "devel" package (cxx11 ABI, matches Jazzy's GCC
#    13 / new libstdc++ ABI).
#
#    The upstream open3d_loc/CMakeLists.txt expects a hand-built or
#    Baidu-Netdisk-hosted Open3D 0.14.1 (China-only download, not usable
#    here). We substitute the official prebuilt Open3D C++ SDK instead of
#    doing an hours-long from-source Open3D build. This requires ONE source
#    patch (step 5b) because Open3D >= 0.16 changed a registration function
#    signature relative to 0.14.1.
# ---------------------------------------------------------------------------
RUN wget -q https://github.com/isl-org/Open3D/releases/download/v0.18.0/open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz -O /tmp/open3d-devel.tar.xz \
    && mkdir -p /opt/open3d \
    && tar -xf /tmp/open3d-devel.tar.xz -C /opt/open3d --strip-components=1 \
    && rm /tmp/open3d-devel.tar.xz

# ---------------------------------------------------------------------------
# 4. Workspace layout
# ---------------------------------------------------------------------------
RUN mkdir -p /root/ws_loc/src

# 4.1 livox_ros_driver2 -- official repo explicitly supports "jazzy" as a
#     distro argument in build.sh, so we just replicate what build.sh does
#     (swap in the ROS2 package.xml/launch dir) and let colcon build it
#     together with the rest of the workspace.
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git /root/ws_loc/src/livox_ros_driver2 \
    && cd /root/ws_loc/src/livox_ros_driver2 \
    && cp -f package_ROS2.xml package.xml \
    && cp -rf launch_ROS2/. launch/

# 4.2 FAST_LIO_LOCALIZATION_HUMANOID, "humble" branch (the actual ROS 2 port;
#     the default "main" branch is ROS1/catkin)
RUN git clone -b humble https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID.git /tmp/fll_src \
    && cp -r /tmp/fll_src/FAST_LIO /root/ws_loc/src/FAST_LIO \
    && cp -r /tmp/fll_src/open3d_loc /root/ws_loc/src/open3d_loc \
    && mkdir -p /root/ws_loc/src/data \
    && cp -r /tmp/fll_src/data/. /root/ws_loc/src/data/ \
    && rm -rf /tmp/fll_src

# 5a. Point open3d_loc at the Open3D devel package installed in step 3,
#     instead of the hard-coded developer path in the original
#     CMakeLists.txt (`/home/sax/open3d141/lib/cmake/Open3D`). This is
#     expected local configuration per the project's own README, not a
#     Jazzy-specific fix.
RUN sed -i 's#set(Open3D_DIR "/home/sax/open3d141/lib/cmake/Open3D")#set(Open3D_DIR "/opt/open3d/lib/cmake/Open3D")#' \
    /root/ws_loc/src/open3d_loc/CMakeLists.txt

# 5b. THE ONE REAL PORTABILITY FIX.
#     open3d_loc/src/open3d_registration/open3d_registration.cpp calls
#     open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(...)
#     with a trailing `seed` argument. That parameter existed in Open3D
#     0.14.1 (what this project was written against) but was REMOVED from
#     the function signature in later Open3D releases (confirmed removed by
#     0.18.0, the version we use here). This is an Open3D-version API drift,
#     NOT a ROS 2 Jazzy / rclcpp / tf2 issue -- the same fix would be needed
#     on Humble too if paired with a modern Open3D. Fix: drop the trailing
#     argument from the call (the wrapper's own `seed_` parameter is kept
#     for source compatibility but no longer forwarded).
RUN sed -i \
    's#RANSACConvergenceCriteria(1000000, 0.999), seed_);#RANSACConvergenceCriteria(1000000, 0.999));#' \
    /root/ws_loc/src/open3d_loc/src/open3d_registration/open3d_registration.cpp

# ---------------------------------------------------------------------------
# 6. rosdep for anything still missing (skips packages already present as
#    workspace sources via --ignore-src)
# ---------------------------------------------------------------------------
RUN source /opt/ros/jazzy/setup.bash \
    && cd /root/ws_loc \
    && rosdep install --from-paths src --ignore-src -r -y || true

# ---------------------------------------------------------------------------
# 7. Build
# ---------------------------------------------------------------------------
WORKDIR /root/ws_loc
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy 2>&1 | tee /root/ws_loc/build.log

CMD ["/bin/bash"]
