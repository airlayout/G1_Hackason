#!/usr/bin/env bash
# デモ 3 本（01_lidar / 02_isaac_nav2 / 03_teleop）が共有する処理。単体では実行しない。
#
#   source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
#
# ここに置くものは 2 つだけ。
#   1. 設定の**唯一の定義**（有線 iface / IP / ROS_DOMAIN_ID / conda・ROS・Isaac の場所）
#   2. **止め方と待ち方**。過去に事故った作法を 1 箇所に閉じ込める
#
# ## 設定を 1 箇所に置く理由
#
# Mapping/real/quickstart/_common.sh で「同じ URI が 6 ファイル 8 箇所に散り、
# 直したつもりが直っていない」をやった。同じことを繰り返さない。
#
# ## 踏んだ罠（消さないこと）
#
# - **止める signal は送る先で逆になる。**
#   run_nav2.sh のような外側スクリプトには SIGTERM（SIGINT は SIG_IGN で黙って無視される）、
#   内側の `ros2 launch` には SIGINT（SIGTERM だと子を孤児化する）。
#   Isaac Sim + Nav2 については IsaacSim_Env/stop_nav2.sh が既に正しく実装しているので、
#   **こちらで再実装せず必ずそれを呼ぶ**。
# - **`pgrep -af` は自分自身と `bash -c` のラッパにマッチする。**
#   除外し損ねて ssh セッションごと kill した（2026-09-09、exit 255 で気づいた）。
#   だから **pgrep のポーリングで待たない。ログを grep して待つ**（wait_for_log）。
# - **`set -o pipefail` の下で `grep -q` を使うとパイプ元が SIGPIPE で死ぬ。**
#   見つかった側（＝合格の経路）だけが失敗するので気づきにくい。
#   このファイルでは `grep -q` を使わず `grep -c` と比較で書く。
# - **`$VAR` の直後に全角文字を置かない。** `$DEMO_HOST_IP に` は変数名が
#   `DEMO_HOST_IPに` と解釈されうる。必ず `${VAR}` で囲むか半角空白を空ける。

set +u   # ROS の setup.bash は未定義変数を参照するので set -u と併用できない

DEMO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${DEMO_DIR}/.." && pwd)"
export DEMO_DIR REPO_DIR

# ---------------------------------------------------------------------------
# 表示
# ---------------------------------------------------------------------------
_ok()   { printf '\033[32m[OK]\033[0m   %s\n' "$*"; }
_ng()   { printf '\033[31m[NG]\033[0m   %s\n' "$*"; }
_warn() { printf '\033[33m[注意]\033[0m %s\n' "$*"; }
_info() { printf '[INFO] %s\n' "$*"; }
_die()  { printf '\n\033[31m[中断]\033[0m %s\n' "$*" >&2; exit 1; }
_head() { printf '\n\033[1m=== %s ===\033[0m\n' "$*"; }

# ---------------------------------------------------------------------------
# 設定 — ここが唯一の定義
# ---------------------------------------------------------------------------
# 上書きしたいときは Demo/demo.env に書く（.gitignore 済み。デモ機ごとの値）。
# Teleop 側と同じ値が要るものは Teleop/config/g1.env.omen から読む。二重に持たない。
load_demo_config() {
    local teleop_env="${REPO_DIR}/Teleop/config/g1.env.omen"
    if [ -f "${teleop_env}" ]; then
        set -a
        # shellcheck disable=SC1090
        source <(grep -vE '^\s*(#|$)' "${teleop_env}")
        set +a
        DEMO_CONFIG_SOURCE="${teleop_env}"
    else
        DEMO_CONFIG_SOURCE="（既定値。Teleop/config/g1.env.omen が無い）"
    fi

    # Teleop 側に無ければ OMEN の既定値を使う
    DEMO_WIRED_IFACE="${G1_WIRED_IFACE:-enp129s0}"
    DEMO_HOST_IP="${G1_HOST_IP:-192.168.123.200}"
    DEMO_WIFI_IFACE="${G1_WIFI_IFACE:-wlp128s20f3}"
    DEMO_ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

    # 場所
    DEMO_ROS_SETUP="${DEMO_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
    DEMO_CONDA_HOME="${DEMO_CONDA_HOME:-${HOME}/miniforge3}"
    DEMO_ISAAC_ENV="${DEMO_ISAAC_ENV:-${HOME}/NVIDIA/env_isaaclab}"
    DEMO_XR_TELEOP="${DEMO_XR_TELEOP:-${HOME}/xr_teleoperate}"

    # G1 の機器（到達性の確認に使う）
    DEMO_G1_PC1="${DEMO_G1_PC1:-192.168.123.161}"
    DEMO_G1_PC2="${DEMO_G1_PC2:-192.168.123.164}"
    DEMO_G1_LIDAR="${DEMO_G1_LIDAR:-192.168.123.120}"

    # デモ機ごとの上書き（最後に読むので最優先）
    if [ -f "${DEMO_DIR}/demo.env" ]; then
        set -a
        # shellcheck disable=SC1090
        source "${DEMO_DIR}/demo.env"
        set +a
        DEMO_CONFIG_SOURCE="${DEMO_CONFIG_SOURCE} ＋ Demo/demo.env"
    fi

    export DEMO_WIRED_IFACE DEMO_HOST_IP DEMO_WIFI_IFACE DEMO_ROS_DOMAIN_ID
    export DEMO_ROS_SETUP DEMO_CONDA_HOME DEMO_ISAAC_ENV DEMO_XR_TELEOP
    export DEMO_G1_PC1 DEMO_G1_PC2 DEMO_G1_LIDAR DEMO_CONFIG_SOURCE
}

# ---------------------------------------------------------------------------
# スタックの有効化
# ---------------------------------------------------------------------------
# ROS 2 とテレオペ用 conda は **同時に有効にしない**。
# xr_teleoperate は conda 環境 tv の python、ROS 2 はシステムの python で動く。
# 両方入れると python とライブラリが衝突する（キットの lib.sh と同じ理由）。
use_ros() {
    [ -f "${DEMO_ROS_SETUP}" ] || _die "ROS 2 が見つかりません: ${DEMO_ROS_SETUP}"
    # shellcheck disable=SC1090
    source "${DEMO_ROS_SETUP}"
    export ROS_DOMAIN_ID="${DEMO_ROS_DOMAIN_ID}"
}

use_tv() {
    local hook="${DEMO_CONDA_HOME}/etc/profile.d/conda.sh"
    [ -f "${hook}" ] || _die "conda が見つかりません: ${hook}"
    # shellcheck disable=SC1090
    source "${hook}"
    conda activate tv 2>/dev/null \
        || _die "conda 環境 tv がありません。Teleop/setup.sh を実行してください。"
    export CYCLONEDDS_HOME="${HOME}/cyclonedds/install"
}

# ---------------------------------------------------------------------------
# 到達性 — ping は使わない
# ---------------------------------------------------------------------------
# ping を使うと「コマンドが無い」と「届かない」が同じ NG になる。
# 過去に colima の VM とコンテナで実際に取り違えた。TCP を直接叩く。
tcp_probe() {
    local host="$1" port="$2" timeout="${3:-2}"
    timeout "${timeout}" bash -c "exec 3<>/dev/tcp/${host}/${port}" 2>/dev/null
}

# G1 が居るか。**居なくても落とさない**（呼び出し側が判断する）
robot_reachable() {
    # PC2 の SSH（22）で見る。PC1 はポートが全閉なので判定に使えない。
    tcp_probe "${DEMO_G1_PC2}" 22 2
}

# ---------------------------------------------------------------------------
# 待ち方 — pgrep のポーリングをしない
# ---------------------------------------------------------------------------
# until 文で pgrep -af を回すと、自分のコマンドラインにマッチして無限ループになる。
# ログに出る文言を待つほうが安全で、失敗したときに理由もそのまま読める。
#
#   wait_for_log <ログファイル> <待つ文言> <秒> [説明]
wait_for_log() {
    local logfile="$1" pattern="$2" limit="$3" what="${4:-$2}"
    local waited=0
    _info "待っています: ${what}（最大 ${limit} 秒）"
    while [ "${waited}" -lt "${limit}" ]; do
        # grep -q は使わない（pipefail 下でパイプ元を SIGPIPE で殺すため）
        if [ -f "${logfile}" ] && [ "$(grep -c -- "${pattern}" "${logfile}" 2>/dev/null || echo 0)" -gt 0 ]; then
            _ok "出ました: ${what}（${waited} 秒）"
            return 0
        fi
        sleep 2
        waited=$((waited + 2))
    done
    _ng "${limit} 秒待っても出ませんでした: ${what}"
    if [ -f "${logfile}" ]; then
        printf '\n--- %s の末尾 30 行 ---\n' "${logfile}"
        tail -30 "${logfile}"
    fi
    return 1
}

# ---------------------------------------------------------------------------
# 止め方
# ---------------------------------------------------------------------------
# Isaac Sim + Nav2 は IsaacSim_Env/stop_nav2.sh が正しい順序と signal を知っている。
# **ここで再実装しない。**
stop_isaac_nav2() {
    local stopper="${REPO_DIR}/IsaacSim_Env/stop_nav2.sh"
    if [ -f "${stopper}" ]; then
        _info "Isaac Sim + Nav2 を止めます（stop_nav2.sh）"
        bash "${stopper}"
    else
        _warn "stop_nav2.sh がありません: ${stopper}"
    fi
}

# 自分が起動した子だけを止める。pgrep でパターン検索しない。
# 使い方: PIDS+=($!) で溜めておいて stop_tracked_pids "${PIDS[@]}"
stop_tracked_pids() {
    local pid
    for pid in "$@"; do
        [ -n "${pid}" ] || continue
        kill -0 "${pid}" 2>/dev/null || continue
        _info "止めます pid=${pid}"
        kill -INT "${pid}" 2>/dev/null || true
    done
    sleep 3
    for pid in "$@"; do
        [ -n "${pid}" ] || continue
        if kill -0 "${pid}" 2>/dev/null; then
            _warn "SIGINT で止まらないので強制します pid=${pid}"
            kill -9 "${pid}" 2>/dev/null || true
        fi
    done
}

load_demo_config
