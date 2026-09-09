# syntax=docker/dockerfile:1.7

FROM ros:humble-ros-base-jammy AS base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    git \
    iproute2 \
    openssh-client \
    python3-zmq \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosbag2 \
    ros-humble-sensor-msgs-py \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

COPY ros2_ws/src/g1_mapping_tools /opt/common_ws/src/g1_mapping_tools
RUN source /opt/ros/humble/setup.bash \
    && cd /opt/common_ws \
    && colcon build --merge-install --packages-select g1_mapping_tools

COPY docker/ros-entrypoint.sh docker/record-topics.sh docker/trajectory-writer.sh /opt/g1/bin/
RUN chmod +x /opt/g1/bin/*.sh

ENTRYPOINT ["/opt/g1/bin/ros-entrypoint.sh"]
CMD ["bash"]


FROM base AS onboard

ARG UNITREE_SDK2_COMMIT=9754cd153af3da471b0fe5f3aa535e426fb11db3
RUN apt-get update && apt-get install -y --no-install-recommends \
    libboost-all-dev \
    libeigen3-dev \
    libfmt-dev \
    libyaml-cpp-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git /opt/unitree_sdk2 \
    && git -C /opt/unitree_sdk2 checkout "${UNITREE_SDK2_COMMIT}" \
    && cmake -S /opt/unitree_sdk2 -B /opt/unitree_sdk2/build \
        -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /opt/unitree_sdk2/build --parallel \
    && cmake --install /opt/unitree_sdk2/build \
    && ldconfig

COPY backends/onboard_unitree /opt/g1/onboard_unitree
RUN cmake -S /opt/g1/onboard_unitree -B /opt/g1/onboard_unitree/build \
        -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /opt/g1/onboard_unitree/build --parallel \
    && cmake --install /opt/g1/onboard_unitree/build

COPY docker/onboard-entrypoint.sh /opt/g1/bin/onboard-entrypoint.sh
RUN chmod +x /opt/g1/bin/onboard-entrypoint.sh


FROM base AS raw

ARG LIVOX_SDK2_COMMIT=08f523c930b2f0ba1e98a6afaa8d7476bf479908
ARG LIVOX_ROS_DRIVER2_COMMIT=4a1def929e5b59c7a8122d19fce6efba581ce9f7
ARG FAST_LIO_COMMIT=a4743b095409588842a5b30ddfa27e29d2f99164

RUN apt-get update && apt-get install -y --no-install-recommends \
    libapr1-dev \
    libboost-all-dev \
    libeigen3-dev \
    libpcl-dev \
    python3-dev \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-tf2-ros \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /opt/Livox-SDK2 \
    && git -C /opt/Livox-SDK2 checkout "${LIVOX_SDK2_COMMIT}" \
    && cmake -S /opt/Livox-SDK2 -B /opt/Livox-SDK2/build \
        -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /opt/Livox-SDK2/build --parallel \
    && cmake --install /opt/Livox-SDK2/build \
    && ldconfig

RUN mkdir -p /opt/g1_ws/src \
    && git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
        /opt/g1_ws/src/livox_ros_driver2 \
    && git -C /opt/g1_ws/src/livox_ros_driver2 checkout \
        "${LIVOX_ROS_DRIVER2_COMMIT}" \
    && cp /opt/g1_ws/src/livox_ros_driver2/package_ROS2.xml \
        /opt/g1_ws/src/livox_ros_driver2/package.xml \
    && cp -r /opt/g1_ws/src/livox_ros_driver2/launch_ROS2 \
        /opt/g1_ws/src/livox_ros_driver2/launch \
    && git clone --recursive --branch ROS2 \
        https://github.com/hku-mars/FAST_LIO.git /opt/g1_ws/src/fast_lio \
    && git -C /opt/g1_ws/src/fast_lio checkout "${FAST_LIO_COMMIT}" \
    && git -C /opt/g1_ws/src/fast_lio submodule update --init --recursive

COPY ros2_ws/src/g1_sensor_adapter /opt/g1_ws/src/g1_sensor_adapter
COPY ros2_ws/src/g1_mapping_bringup /opt/g1_ws/src/g1_mapping_bringup

RUN source /opt/ros/humble/setup.bash \
    && source /opt/common_ws/install/setup.bash \
    && cd /opt/g1_ws \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro humble -r -y \
    && colcon build --merge-install --cmake-args \
        -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
    && rm -rf /var/lib/apt/lists/*


FROM raw AS visualization

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-image-transport-plugins \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

COPY ros2_ws/src/g1_mapping_visualization \
    /opt/g1_ws/src/g1_mapping_visualization
RUN source /opt/ros/humble/setup.bash \
    && source /opt/common_ws/install/setup.bash \
    && source /opt/g1_ws/install/setup.bash \
    && cd /opt/g1_ws \
    && colcon build --merge-install \
        --packages-select g1_mapping_visualization \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && mkdir -p /tmp/g1-mapping-rviz/runtime \
    && chmod 1777 /tmp/g1-mapping-rviz \
    && chmod 700 /tmp/g1-mapping-rviz/runtime

ENV HOME=/tmp/g1-mapping-rviz
ENV XDG_RUNTIME_DIR=/tmp/g1-mapping-rviz/runtime
