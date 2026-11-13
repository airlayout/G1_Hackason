#!/usr/bin/env bash
# Ubuntu 24.04 + existing ROS 2 Jazzy. Everything added lives in _local/mapping.
set -euo pipefail
NATIVE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REAL_DIR="$(cd "$NATIVE_DIR/.." && pwd)"
REPO_DIR="$(cd "$REAL_DIR/../.." && pwd)"
export MAPPING_NATIVE_ROOT="${MAPPING_NATIVE_ROOT:-$REPO_DIR/_local/mapping}"
ROOT="$MAPPING_NATIVE_ROOT"
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 && -f /opt/ros/jazzy/setup.bash && "$(uname -m)" == x86_64 ]] || {
    echo '[ERROR] Ubuntu 24.04 x86_64と/opt/ros/jazzyが必要です' >&2; exit 1;
}
pkg-config --exists pcl_common eigen3 octomap
mkdir -p "$ROOT/debs" "$ROOT/deps" "$ROOT/vendor" "$ROOT/src/livox_ros_driver2/msg"
mapfile -t PACKAGES < "$NATIVE_DIR/dependencies.txt"
(cd "$ROOT/debs" && apt-get download "${PACKAGES[@]}")
for package in "$ROOT"/debs/*.deb; do
    dpkg-deb --extract "$package" "$ROOT/deps"
done

checkout() {
    local name="$1" url="$2" revision="$3"
    if [[ ! -d "$ROOT/vendor/$name/.git" ]]; then
        git clone --filter=blob:none --no-checkout "$url" "$ROOT/vendor/$name"
        git -C "$ROOT/vendor/$name" checkout --detach "$revision"
    fi
    [[ "$(git -C "$ROOT/vendor/$name" rev-parse HEAD)" == "$revision" ]] || {
        echo "[ERROR] $nameは固定commitと一致しません" >&2; exit 1;
    }
}
checkout FAST_LIO https://github.com/hku-mars/FAST_LIO.git a4743b095409588842a5b30ddfa27e29d2f99164
checkout livox_ros_driver2 https://github.com/Livox-SDK/livox_ros_driver2.git 4a1def929e5b59c7a8122d19fce6efba581ce9f7
git -C "$ROOT/vendor/FAST_LIO" submodule update --init --recursive
for patch in "$REAL_DIR/docker/fast_lio_sensor_pose.patch" "$NATIVE_DIR/fast_lio_jazzy.patch"; do
    if ! git -C "$ROOT/vendor/FAST_LIO" apply --reverse --check "$patch" 2>/dev/null; then
        git -C "$ROOT/vendor/FAST_LIO" apply --check "$patch"
        git -C "$ROOT/vendor/FAST_LIO" apply "$patch"
    fi
done
# Only the unchanged wire messages are needed: the G1 already runs its LiDAR driver.
cp "$ROOT/vendor/livox_ros_driver2/msg/CustomPoint.msg" "$ROOT/vendor/livox_ros_driver2/msg/CustomMsg.msg" "$ROOT/src/livox_ros_driver2/msg/"
cp "$ROOT/vendor/livox_ros_driver2/LICENSE.txt" "$ROOT/src/livox_ros_driver2/"
cp "$NATIVE_DIR/livox_interfaces/CMakeLists.txt" "$NATIVE_DIR/livox_interfaces/package.xml" "$ROOT/src/livox_ros_driver2/"
ln -sfn "$ROOT/vendor/FAST_LIO" "$ROOT/src/fast_lio"
for package in g1_sensor_adapter g1_mapping_bringup g1_mapping_octomap g1_mapping_tools g1_mapping_visualization; do
    ln -sfn "$REAL_DIR/ros2_ws/src/$package" "$ROOT/src/$package"
done
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-1}"
export MAKEFLAGS="-j$CMAKE_BUILD_PARALLEL_LEVEL"
cd "$ROOT"
bash "$NATIVE_DIR/exec.sh" colcon build --merge-install --symlink-install \
    --base-paths "$ROOT/src" --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.12.so \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.12
echo '[OK] native FAST-LIO2 + OctoMap ready'
