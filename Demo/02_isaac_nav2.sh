#!/usr/bin/env bash
# 実演2 — Isaac Sim の UiS_room_v3 で、Nav2 に指定した場所まで G1 を歩かせる。
#
#   bash Demo/02_isaac_nav2.sh            # 起動して RViz2 まで開く
#   bash Demo/02_isaac_nav2.sh --no-rviz      # RViz2 を開かない（別画面で開きたいとき）
#   bash Demo/02_isaac_nav2.sh --follow-cam   # Isaac Sim のカメラを G1 に追従させる
#                                             # （長い距離を歩かせるとき。既定は固定）
#
# 実機は要らない。Isaac Sim の中だけで完結する。
#
# ## 当日の見せ方
#
#   RViz2 のツールバーいちばん右の「Nav2 Goal」でドラッグ → G1 がそこまで歩く。
#   （nav2_rviz_plugins/GoalTool。古い案内にある「2D Goal Pose」と同じもの）
#   ⚠️ **「2D Pose Estimate」は使わないこと。** この地図は原点がずれているので
#      クリックすると自己位置が大きく飛ぶ（実測で位置 31m / 向き 179度）。
#      初期姿勢はこのスクリプトが Isaac Sim の真値から自動で入れる。
#
# ## ここに焼き込んである値を変えないこと
#
#   SPAWN_X / SPAWN_Y は「立たせたい場所」ではなく **「漂流の着地点」** で選んである。
#   Isaac Sim の G1 は指令ゼロでも -x 方向へ 3.4cm/s 漂い、起動待ちの間に 1.7〜2.3m 動く。
#   見た目の良い場所に置くと、Nav2 が立ち上がる頃には壁に埋まっている。
#
#   G1_PERFECT_LOC=1 は map -> odom を恒等変換にする（Isaac Sim の /odom は真値）。
#   これを外すと AMCL の測位ずれで着かなくなる（実測 0/3 → 3/3 の差はここだけ）。
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

OPEN_RVIZ=1
FOLLOW_CAM=0
for a in "$@"; do
    case "${a}" in
        --no-rviz)     OPEN_RVIZ=0 ;;
        --follow-cam)  FOLLOW_CAM=1 ;;
        *) _die "知らない引数です: ${a}" ;;
    esac
done

ISAAC_DIR="${REPO_DIR}/IsaacSim_Env"
LOG_DIR="${ISAAC_DIR}/logs"
CONSOLE_LOG="${LOG_DIR}/run_nav2_console.log"
mkdir -p "${LOG_DIR}"

for f in "${ISAAC_DIR}/run_nav2.sh" \
         "${ISAAC_DIR}/assets/uis_room_v3_sim_obstacles.usd" \
         "${ISAAC_DIR}/maps/uis_room_v3_clean.yaml"; do
    [ -f "${f}" ] || _die "ありません: ${f}
     bash Demo/preflight.sh で足りないものを確認してください。"
done

RVIZ_PID=""
cleanup() {
    printf '\n'
    _info "後片付けをします"
    [ -n "${RVIZ_PID}" ] && stop_tracked_pids "${RVIZ_PID}"
    # Isaac Sim + Nav2 は signal の向きが送る先で逆になる。stop_nav2.sh に任せる
    stop_isaac_nav2
}
trap cleanup EXIT INT TERM

_head "実演2 — Isaac Sim + Nav2"
_info "シーン : assets/uis_room_v3_sim_obstacles.usd"
_info "地図   : maps/uis_room_v3_clean.yaml"
_info "初期位置: (6.55, -0.78)  ※漂流の着地点で選んである。変えないこと"
_info "測位   : G1_PERFECT_LOC=1（map -> odom を恒等に）"
_info "ログ   : ${CONSOLE_LOG}"
printf '\n起動に 3〜6 分かかります。Isaac Sim のウィンドウが開いても、まだ触らないでください。\n\n'

: > "${CONSOLE_LOG}"
(
    cd "${ISAAC_DIR}" || exit 1
    export SCENE_USD="${ISAAC_DIR}/assets/uis_room_v3_sim_obstacles.usd"
    export SPAWN_X=6.55
    export SPAWN_Y=-0.78
    export G1_PERFECT_LOC=1
    # 長い距離を歩かせるときだけカメラを追従させる。起動時の固定カメラは
    # スポーン地点を向いたままなので、遠くへ行くと画角から出る。
    export G1_FOLLOW_CAM="${FOLLOW_CAM}"
    exec bash run_nav2.sh maps/uis_room_v3_clean.yaml
) >> "${CONSOLE_LOG}" 2>&1 &
NAV_WRAPPER_PID=$!

# pgrep のポーリングでは待たない（自己マッチで無限ループになる）。ログを待つ。
if ! wait_for_log "${CONSOLE_LOG}" "Isaac Sim が起動しました" 480 "Isaac Sim の起動"; then
    _die "Isaac Sim が起動しませんでした。上のログを見てください。"
fi
if ! wait_for_log "${CONSOLE_LOG}" "Nav2 が起動しました" 240 "Nav2 の起動"; then
    _die "Nav2 が起動しませんでした。上のログを見てください。"
fi

# 初期姿勢の設定結果を出す（run_nav2.sh が真値から入れている）
grep -E "初期姿勢|initial pose|設定しました" "${CONSOLE_LOG}" | tail -3 || true

if [ "${OPEN_RVIZ}" = "1" ]; then
    # IsaacSim_Env/run_rviz.sh は nav2 の既定設定を使う。あれは右のドック
    # （Views / Docking / Selector）が開いたままで、**地図が細い帯になって
    # ツールバーの文字が読めない**。Demo は右のドックを畳んだ設定を持つ。
    # シミュレータで歩く姿を見せたいときは Alt+Tab で切り替える
    # （並べて置くと、どちらも狭くなって両方読めなくなる）。
    _info "RViz2 を開きます（Isaac Sim を見るときは Alt+Tab）"
    (
        cd "${ISAAC_DIR}" || exit 1
        # shellcheck disable=SC1091
        source ./env.sh
        exec ros2 run rviz2 rviz2 -d "${DEMO_DIR}/nav2_g1.rviz" --ros-args -p use_sim_time:=true
    ) > "${LOG_DIR}/rviz.log" 2>&1 &
    RVIZ_PID=$!
    sleep 5
    if kill -0 "${RVIZ_PID}" 2>/dev/null; then
        _ok "RViz2 を開きました"
    else
        _warn "RViz2 が落ちました。${LOG_DIR}/rviz.log を見てください"
        RVIZ_PID=""
    fi
fi

cat <<'GUIDE'

==============================================================
 準備できました。

   RViz2 の上のツールバーでいちばん右の「Nav2 Goal」を選び、
   地図の上の行かせたい場所をドラッグしてください。G1 が歩きます。

   ⚠️ 「2D Pose Estimate」は押さないこと。自己位置が大きく飛びます。

 終了: このターミナルで Ctrl-C（Isaac Sim も RViz2 もまとめて止まります）
==============================================================

GUIDE

wait "${NAV_WRAPPER_PID}"
