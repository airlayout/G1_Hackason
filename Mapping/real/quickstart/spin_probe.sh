#!/usr/bin/env bash
# 純回転させながら「頭の IMU / 胴体の IMU / MOLA の姿勢」を同時に録る。
#
# 失敗した回とまったく同じ入力（vx=0, vy=0, vyaw=0.5）を /cmd_vel に直接流す。
# Nav2 は関与しないので入力が完全に制御される。
#
# ⚠️ **止める道は 3 重にしてある**: (1) 明示的にゼロを投げる (2) trap で必ず通る
#    (3) loco_driver 自身が 0.5 秒指令が来なければ止める
set -uo pipefail
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
. quickstart/_common.sh
DDS="$(g1_dds_uri live)"
NAME=rviz
OUT="/work/G1_Hackason/Mapping/real/runs/spin_$(date +%Y%m%dT%H%M%S)"
SPIN_S="${SPIN_S:-10}"
VYAW="${VYAW:-0.5}"
# RETURN=1 で **逆回しを足して向きを戻す**。
# ⚠️ これが無いと 1 回ごとに機体が約 200° 向きを変えるので、掃引の各点が
# 「違う壁を見ている」状態になり比較にならない（2026-09-12 に踏んだ）。
# 戻すことで次の点が同じ姿勢から始まり、しかも 1 回で往路・復路の 2 標本が取れる。
RETURN="${RETURN:-0}"

ros()  { docker exec -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
             -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 "$NAME" \
             bash -c "source /opt/ros/humble/setup.bash && $*"; }
rosd() { docker exec -d -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
             -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 "$NAME" \
             bash -c "source /opt/ros/humble/setup.bash && $*"; }

stop_all() {
    echo "[spin] 止める"
    docker exec "$NAME" bash -c 'pkill -f "topic pu[b]"' 2>/dev/null || true
    # 明示的にゼロ。loco_driver のウォッチドッグ（0.5s）にも任せるが待たない
    ros "ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
         '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'" >/dev/null 2>&1 || true
    docker exec "$NAME" bash -c 'pkill -INT -f "ros2 bag recor[d]"' 2>/dev/null || true
}
trap stop_all EXIT INT TERM

REC="$(awk '/^[[:space:]]*#/{next} $2=="sensor"||$2=="ros"{printf "%s ", $1}' quickstart/record_topics.txt)"

PROBE_S=$((SPIN_S + 8)); [ "$RETURN" = "1" ] && PROBE_S=$((SPIN_S * 2 + 12))
echo "[spin] PC2 のプローブを起こす（両 IMU、${PROBE_S} 秒）"
ssh -o ConnectTimeout=10 g1 "nohup python3 ~/mapping_tools/probe_neck_imu.py \
    --seconds ${PROBE_S} --out /tmp/neck_spin.json > /tmp/neck_spin.log 2>&1 &" || exit 1

echo "[spin] bag を録る -> $OUT"
ros "mkdir -p $OUT"
rosd "cd $OUT && ros2 bag record -o bag $REC > $OUT/bagrecord.log 2>&1"
sleep 2

spin() {
    echo "[spin] **純回転を ${SPIN_S} 秒**（vx=0 / vy=0 / vyaw=$1）"
    ros "timeout ${SPIN_S} ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
         '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $1}}'" \
         >/dev/null 2>&1 || true
}
spin "$VYAW"
if [ "$RETURN" = "1" ]; then
    ros "ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
         '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'" >/dev/null 2>&1 || true
    sleep 2                                  # 足を止めてから逆へ（往路と復路を分けて読めるように）
    spin "-${VYAW}"
fi

stop_all
trap - EXIT INT TERM
sleep 3
echo "[spin] プローブの終わりを待つ"
until ssh -o ConnectTimeout=8 g1 'test -f /tmp/neck_spin.json && ! pgrep -f "[p]robe_neck_imu" >/dev/null'; do
    sleep 2
done
LOCAL_OUT="${OUT/#\/work/$G1_REPO_ROOT}"
scp -o ConnectTimeout=10 g1:/tmp/neck_spin.json "$LOCAL_OUT/neck.json"
echo "[spin] 完了 -> $LOCAL_OUT （bag / neck.json）"
