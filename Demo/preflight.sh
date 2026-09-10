#!/usr/bin/env bash
# デモ 3 本に共通する前提を 1 本で確認する。**何も起動しないし、何も動かさない。**
#
#   bash Demo/preflight.sh
#
# ロボットや Isaac Sim が無くても**最後まで走り切る**。
# 「使えるもの / 使えないもの」を列挙して終わるので、当日はまずこれを読む。
#
# 終了コード:
#   0 = 4 つとも今すぐ見せられる
#   1 = 見せられないものがある（実機が居ないだけ、という場合も 1 になる）
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

FAIL=0
CAN_LIDAR_LIVE=1     # 項目2 ライブ
CAN_LIDAR_REPLAY=1   # 項目2 再生
CAN_ISAAC=1          # 項目3
CAN_TELEOP=1         # 項目4

note_ng() { _ng "$1"; FAIL=1; }

printf '\033[1mデモ事前確認 — 読み取りだけ。ロボットも Isaac Sim も起動しません\033[0m\n'
_info "設定の出どころ: ${DEMO_CONFIG_SOURCE}"

# ---------------------------------------------------------------------------
_head "1. ネットワーク"
STATE="$(cat "/sys/class/net/${DEMO_WIRED_IFACE}/operstate" 2>/dev/null || echo missing)"
case "${STATE}" in
    missing)
        note_ng "有線インターフェース ${DEMO_WIRED_IFACE} がありません。'ip -br addr' で名前を確認してください"
        CAN_LIDAR_LIVE=0; CAN_TELEOP=0 ;;
    up)
        ADDR="$(ip -4 -br addr show "${DEMO_WIRED_IFACE}" 2>/dev/null || true)"
        if [ "$(printf '%s' "${ADDR}" | grep -c '192\.168\.123\.')" -gt 0 ]; then
            _ok "${DEMO_WIRED_IFACE} : up / ${ADDR}"
        else
            note_ng "${DEMO_WIRED_IFACE} に 192.168.123.x が付いていません（現在: ${ADDR:-なし}）"
            CAN_LIDAR_LIVE=0; CAN_TELEOP=0
        fi ;;
    *)
        note_ng "${DEMO_WIRED_IFACE} のリンクが上がっていません（state=${STATE}）。LAN ケーブルを確認してください"
        CAN_LIDAR_LIVE=0; CAN_TELEOP=0 ;;
esac

if [ -n "${DEMO_WIFI_IFACE}" ] && [ -d "/sys/class/net/${DEMO_WIFI_IFACE}" ]; then
    _ok "${DEMO_WIFI_IFACE} : $(cat "/sys/class/net/${DEMO_WIFI_IFACE}/operstate" 2>/dev/null || echo 不明)（Quest 用）"
else
    _warn "WiFi インターフェース ${DEMO_WIFI_IFACE} が見つかりません（Quest が繋がらない可能性）"
fi

# ---------------------------------------------------------------------------
_head "2. G1 実機の到達性"
# ping は使わない。「コマンドが無い」と「届かない」を取り違える。
if robot_reachable; then
    _ok "G1 PC2 ${DEMO_G1_PC2}:22 に TCP が通りました"
    if tcp_probe "${DEMO_G1_LIDAR}" 80 2; then
        _ok "LiDAR ${DEMO_G1_LIDAR} も応答します"
    else
        _warn "LiDAR ${DEMO_G1_LIDAR} は応答しません（ライブ点群は経路(A) を試すこと）"
    fi
else
    _warn "G1 が見つかりません（${DEMO_G1_PC2}:22 に届かない）。電源とケーブルを確認してください"
    printf '       → \033[1m項目2 のライブ点群と項目4 は使えません。\033[0m 項目2 は記録の再生に落ちます\n'
    CAN_LIDAR_LIVE=0; CAN_TELEOP=0
fi

# ---------------------------------------------------------------------------
_head "3. ROS 2"
if [ -f "${DEMO_ROS_SETUP}" ]; then
    _ok "ROS 2: ${DEMO_ROS_SETUP}"
    # 部分シェルで source する（この後 conda を触るので本体は汚さない）
    RMW_OK="$(bash -c "source '${DEMO_ROS_SETUP}' >/dev/null 2>&1; ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1 && echo yes || echo no")"
    if [ "${RMW_OK}" = "yes" ]; then
        _ok "rmw_cyclonedds_cpp あり（G1 の DDS を購読するのに要る）"
    else
        _warn "rmw_cyclonedds_cpp がありません → bash Demo/setup/install.sh で入ります"
        CAN_LIDAR_LIVE=0
    fi
    NAV2_OK="$(bash -c "source '${DEMO_ROS_SETUP}' >/dev/null 2>&1; ros2 pkg prefix nav2_bringup >/dev/null 2>&1 && echo yes || echo no")"
    if [ "${NAV2_OK}" = "yes" ]; then
        _ok "nav2_bringup あり"
    else
        note_ng "nav2_bringup がありません（項目3 が動きません）"
        CAN_ISAAC=0
    fi
    _info "ROS_DOMAIN_ID=${DEMO_ROS_DOMAIN_ID}（ロボットの Unitree DDS は 0。必ず 0 以外にすること）"
else
    note_ng "ROS 2 がありません: ${DEMO_ROS_SETUP}"
    CAN_ISAAC=0; CAN_LIDAR_LIVE=0; CAN_LIDAR_REPLAY=0
fi

# ---------------------------------------------------------------------------
_head "4. Isaac Sim（項目3）"
if [ -d "${DEMO_ISAAC_ENV}" ]; then
    _ok "Isaac Sim の環境: ${DEMO_ISAAC_ENV}"
else
    note_ng "Isaac Sim の環境がありません: ${DEMO_ISAAC_ENV}"
    CAN_ISAAC=0
fi
for f in "${REPO_DIR}/IsaacSim_Env/run_nav2.sh" \
         "${REPO_DIR}/IsaacSim_Env/stop_nav2.sh" \
         "${REPO_DIR}/IsaacSim_Env/assets/uis_room_v3_sim_obstacles.usd" \
         "${REPO_DIR}/IsaacSim_Env/maps/uis_room_v3_clean.yaml"; do
    if [ -f "${f}" ]; then
        _ok "$(basename "${f}")"
    else
        note_ng "ありません: ${f}"
        CAN_ISAAC=0
    fi
done

# ---------------------------------------------------------------------------
_head "5. テレオペ（項目4）"
if [ -f "${DEMO_CONDA_HOME}/etc/profile.d/conda.sh" ]; then
    TV_OK="$(bash -c "source '${DEMO_CONDA_HOME}/etc/profile.d/conda.sh' >/dev/null 2>&1; conda activate tv >/dev/null 2>&1 && python3 -V 2>&1 || echo NG")"
    if [ "${TV_OK}" = "NG" ]; then
        note_ng "conda 環境 tv がありません"
        CAN_TELEOP=0
    else
        _ok "conda 環境 tv: ${TV_OK}"
    fi
else
    note_ng "conda がありません: ${DEMO_CONDA_HOME}"
    CAN_TELEOP=0
fi
if [ -d "${DEMO_XR_TELEOP}/teleop" ]; then
    _ok "xr_teleoperate: ${DEMO_XR_TELEOP}"
else
    note_ng "xr_teleoperate がありません: ${DEMO_XR_TELEOP}"
    CAN_TELEOP=0
fi
if [ -f "${REPO_DIR}/Teleop/vendor/g1-starter-kit/config/g1.env" ]; then
    _ok "キットの設定が置かれています"
else
    _warn "キットの設定がありません → Teleop/config/README.md の手順で置いてください"
    CAN_TELEOP=0
fi

# ---------------------------------------------------------------------------
_head "6. LiDAR の記録（項目2 の再生）"
BAG_DIR="${REPO_DIR}/Mapping/real/runs/20260906T135940_UiS_room_v3/raw/rosbag2"
if [ -f "${BAG_DIR}/metadata.yaml" ]; then
    _ok "記録あり: ${BAG_DIR}"
else
    _warn "記録がありません → bash Mapping/real/ubuntu/fetch_bag.sh で Mac から持ってきます（3.0GB）"
    CAN_LIDAR_REPLAY=0
fi

# ---------------------------------------------------------------------------
_head "まとめ"
show() { if [ "$1" = "1" ]; then _ok "$2"; else _ng "$2"; fi; }
show "${CAN_ISAAC}"        "項目3  Isaac Sim + Nav2      … bash Demo/02_isaac_nav2.sh"
if [ "${CAN_LIDAR_LIVE}" = "1" ]; then
    _ok  "項目2  LiDAR ライブ           … bash Demo/01_lidar.sh live"
else
    _ng  "項目2  LiDAR ライブ           … 使えません"
fi
show "${CAN_LIDAR_REPLAY}" "項目2  LiDAR 記録の再生      … bash Demo/01_lidar.sh replay"
show "${CAN_TELEOP}"       "項目4  Quest でエピソード     … bash Demo/03_teleop.sh"

# ---------------------------------------------------------------------------
# 何をすれば直るかを書く。「使えない」で終わらせない。
_head "次にやること"
TODO=0
if [ "${CAN_LIDAR_REPLAY}" = "0" ] && [ -f "${DEMO_ROS_SETUP}" ]; then
    printf '  記録を持ってくる     : bash Mapping/real/ubuntu/fetch_bag.sh\n'
    TODO=1
fi
if [ "${RMW_OK:-no}" = "no" ] || [ "${FAIL}" = "1" ]; then
    printf '  足りない apt を入れる: bash Demo/setup/install.sh\n'
    TODO=1
fi
if ! robot_reachable; then
    printf '  G1 の電源を入れる    : 項目2 ライブと項目4 はこれが要ります\n'
    TODO=1
fi
if [ "${CAN_TELEOP}" = "0" ] && [ ! -f "${REPO_DIR}/Teleop/vendor/g1-starter-kit/config/g1.env" ]; then
    printf '  キットの設定を置く   : Teleop/config/README.md の手順\n'
    TODO=1
fi
[ "${TODO}" = "0" ] && printf '  ありません\n'

# ---------------------------------------------------------------------------
READY=$((CAN_ISAAC + CAN_LIDAR_LIVE + CAN_LIDAR_REPLAY + CAN_TELEOP))
printf '\n'
if [ "${READY}" = "4" ]; then
    printf '\033[32m4 つとも今すぐ見せられます。\033[0m\n'
    exit 0
fi
printf '今すぐ見せられるのは \033[1m%d / 4\033[0m です。\n' "${READY}"
if [ "${FAIL}" = "1" ]; then
    printf '上の \033[31m[NG]\033[0m のうち、導入で直るものは bash Demo/setup/install.sh です。\n'
fi
exit 1
