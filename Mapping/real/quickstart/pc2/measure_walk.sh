#!/usr/bin/env bash
# **歩かせながら測る。**候補（AMCL / FAST_LIO / GLIM / MOLA）を同じ条件で比べるため。
#
#   bash measure_walk.sh <ラベル> --from "0.70 12.97" --goal "2.70 12.97 auto" --goal "0.70 12.97 auto"
#   bash measure_walk.sh <ラベル> --goal "2.70 12.97 -57.5" --timeout 90
#
# ## なぜ静止では足りないのか（2026-09-15 実測）
#
# 静止だけだと **AMCL に自動満点が出る**。`update_min_d = 0.20 m` なので観測更新が
# 一度も走らず、測っているのは実質**脚 odom の静止安定性**である（滑り 0.000 m）。
# 順位づけには機体が動く必要がある。
#
# ## なぜ `/goal_pose` に投げるのか
#
# **RViz2 の「2D Goal Pose」と同じ経路**だから。アクションに直投げすると
# `stray_guard` が発火しない（2026-09-10 に判明）。目的は「クリックで歩かせる」なので、
# 測るときもクリックと同じ道を通す。
#
# ⚠️ **ゴールの yaw を固定値で投げると Spin 90° が走る。**
# 1.79 m のゴールで累積 5.59 m・174° 回った実測がある（2026-09-10）。
# だから `auto` を用意した —— **進行方向を向く** yaw を `--from`（または直前のゴール）から計算する。
#
# ⚠️ **候補は 1 つずつ動かすこと。** 2 つが同時に `map->base_link` を出すと `/tf` が壊れる。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PC2="${G1_PC2_HOST:-10.42.0.76}"
KEY="${G1_PC2_KEY:-$HOME/.ssh/id_ed25519_g1}"
REPO="$(cd "$HERE/../../../.." && pwd)"
OUT_DIR="${G1_MEAS_DIR:-$REPO/G1_Hackason/Mapping/real/runs/_meas}"
TOPICS_FILE="${G1_TOPICS_FILE:-$HERE/../record_topics.txt}"
TIMEOUT="${G1_WALK_TIMEOUT:-120}"
REMOTE_TOPICS="${G1_PC2_TOPICS:-/home/unitree/g1_cfg/record_topics.txt}"

LABEL=""; FROM=""; TIMEOUT_SET=""
GOALS=""            # 改行区切りの "X Y YAW"
while [ $# -gt 0 ]; do
    case "$1" in
        --from)    FROM="$2"; shift 2 ;;
        --goal)    GOALS="$GOALS$2"$'\n'; shift 2 ;;
        --timeout) TIMEOUT="$2"; TIMEOUT_SET=1; shift 2 ;;
        -h|--help) sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *)         [ -z "$LABEL" ] && LABEL="$1" || { echo "余分な引数: $1" >&2; exit 2; }; shift ;;
    esac
done
[ -n "$LABEL" ] || { echo "使い方: measure_walk.sh <ラベル> --goal \"X Y YAW\" [...]" >&2; exit 2; }
[ -n "$GOALS" ] || { echo "⛔ --goal が 1 つも無い" >&2; exit 2; }

say() { echo "[walk] $*"; }
ssh_pc2() { ssh -o BatchMode=yes -o ConnectTimeout=10 -i "$KEY" -o IdentitiesOnly=yes "unitree@$PC2" "$@"; }

# ── 前提の確認（走り終えてから「測れません」では遅い）────────────────
say "PC2 の測位を確認"
RUNNING="$(ssh_pc2 'pgrep -cf "nav2_amc[l]|mola-cl[i]|global_localization_nod[e]|glim_ro[s]" 2>/dev/null || echo 0')"
[ "${RUNNING:-0}" -ge 1 ] || { echo "⛔ PC2 に測位が動いていない" >&2; exit 3; }
say "  測位らしきプロセス: $RUNNING 本"
NAV2="$(ssh_pc2 'pgrep -cf "nav2_bt_navigato[r]|bt_navigato[r]" 2>/dev/null || echo 0')"
[ "${NAV2:-0}" -ge 1 ] || { echo "⛔ Nav2 が動いていない（bt_navigator が無い）" >&2; exit 3; }

# 記録トピックは正典から読む。**ここで直書きしない**（record_topics.txt の冒頭参照）
scp -q -o BatchMode=yes -i "$KEY" -o IdentitiesOnly=yes "$TOPICS_FILE" "unitree@$PC2:$REMOTE_TOPICS" \
    || { echo "⛔ record_topics.txt を配れない" >&2; exit 2; }
REC_TOPICS="$(awk '/^[[:space:]]*#/{next} $2=="sensor"||$2=="ros"{printf "%s ", $1}' "$TOPICS_FILE" | sed 's/ *$//')"
[ -n "$REC_TOPICS" ] || { echo "⛔ $TOPICS_FILE から記録トピックを読めない" >&2; exit 2; }
say "録るトピック $(echo "$REC_TOPICS" | wc -w | tr -d ' ') 件"

# ディスク（1 本 5 分で約 1.6 GB になる）
FREE_MB="$(ssh_pc2 "df -Pm /tmp | awk 'NR==2{print \$4}'")"
say "PC2 /tmp の空き ${FREE_MB} MB"
[ "${FREE_MB:-0}" -ge 3000 ] || echo "⚠️ 空きが少ない。1 本 5 分で約 1600 MB 使う" >&2

# ── 記録を開始 ──────────────────────────────────────────────────────
BAG="/tmp/walk_$LABEL"
say "記録を開始 -> $BAG"
ssh_pc2 "rm -rf $BAG; . \$HOME/jammy_ros/env.sh
export ROS_DOMAIN_ID=\${G1_PC2_DOMAIN:-0}
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"eth0\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>'
setsid nohup jros2 bag record -o $BAG $REC_TOPICS > $BAG.log 2>&1 < /dev/null &
sleep 3" || { echo "⛔ 記録を開始できない" >&2; exit 4; }

# ── ゴールを順に投げる ──────────────────────────────────────────────
PREV="$FROM"
N=0
# ⚠️ **パイプで while に渡さない。** サブシェルになり、中の `exit` が効かず
#    `PREV` の更新も外へ伝わらない。合格側の経路でしか出ないので気づけない（bash 3.2）
while IFS= read -r g; do
    [ -n "$g" ] || continue
    N=$((N + 1))
    set -- $g
    GX="$1"; GY="$2"; GYAW="${3:-auto}"
    if [ "$GYAW" = auto ]; then
        [ -n "$PREV" ] || { echo "⛔ yaw=auto には --from か直前のゴールが要る" >&2; exit 2; }
        set -- $PREV
        GYAW="$(awk -v gx="$GX" -v gy="$GY" -v px="$1" -v py="$2" \
                'BEGIN{printf "%.2f", atan2(gy-py, gx-px)*180/3.141592653589793}')"
        say "  ゴール $N: yaw=auto -> ${GYAW} deg（進行方向を向く）"
    fi
    QZ="$(awk -v y="$GYAW" 'BEGIN{printf "%.9f", sin(y*3.141592653589793/360)}')"
    QW="$(awk -v y="$GYAW" 'BEGIN{printf "%.9f", cos(y*3.141592653589793/360)}')"
    say "ゴール $N を投げる: ($GX, $GY, ${GYAW} deg)  上限 ${TIMEOUT} 秒"
    ssh_pc2 ". \$HOME/jammy_ros/env.sh
export ROS_DOMAIN_ID=\${G1_PC2_DOMAIN:-0}
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"eth0\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>'
jros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: $GX, y: $GY, z: 0.0}, orientation: {z: $QZ, w: $QW}}}' \
  > /tmp/walk_goal_$N.log 2>&1" || echo "⚠️ ゴール $N の publish が失敗した" >&2
    # 到達/中止を待つ。⚠️ 満了で打ち切ったら、その根拠を報告に書くこと
    ssh_pc2 "for i in \$(seq 1 $TIMEOUT); do
        grep -qE 'Goal succeeded|Goal was (aborted|canceled)' /tmp/pc2_nav2.log 2>/dev/null && break
        sleep 1
      done" >/dev/null 2>&1
    PREV="$GX $GY"
done <<EOF
$GOALS
EOF

# ── 止めて回収 ──────────────────────────────────────────────────────
say "記録を止める"
ssh_pc2 'pkill -INT -f "bin/ros2 bag recor[d]"; sleep 3'
mkdir -p "$OUT_DIR"
rm -rf "${OUT_DIR:?}/walk_$LABEL"
scp -q -r -o BatchMode=yes -i "$KEY" -o IdentitiesOnly=yes "unitree@$PC2:$BAG" "$OUT_DIR/walk_$LABEL" \
    || { echo "⛔ 回収できない" >&2; exit 5; }
say "取得: $OUT_DIR/walk_$LABEL ($(du -sh "$OUT_DIR/walk_$LABEL" | cut -f1))"

# ── 採点 ────────────────────────────────────────────────────────────
REF="${G1_OVERLAY_REF_MAP:-$REPO/G1_Hackason/Mapping/real/runs/20260906T135940_UiS_room_v3/map/nav_map_ref.yaml}"
VENV="$REPO/G1_Hackason/Navigation/.venv/bin/python"
say "採点（M1 幻の並進 / M2 回転の追従。純回転の窓でしか出ない）"
"$VENV" "$HERE/../eval_traj.py" "$OUT_DIR/walk_$LABEL" 2>&1 | tail -20
echo
say "重畳と見かけの速さ"
"$VENV" "$HERE/still_report.py" "$OUT_DIR/walk_$LABEL" --ref "$REF" --label "walk_$LABEL" 2>&1 | tail -12
