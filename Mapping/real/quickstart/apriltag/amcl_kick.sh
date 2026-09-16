# AMCL に粒子を散らした初期姿勢を投げ、1 回だけ観測更新を走らせる。
# ⚠️ 共分散 0 のまま（＝全粒子が同一点）だと、静止中は永久に更新されない。
. "$HOME/jammy_ros/env.sh"
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'

X="${1:-3.0}"; Y="${2:-0.0}"; YAW_DEG="${3:--15.0}"
QZ=$(awk -v d="$YAW_DEG" 'BEGIN{printf "%.9f", sin(d*3.141592653589793/360)}')
QW=$(awk -v d="$YAW_DEG" 'BEGIN{printf "%.9f", cos(d*3.141592653589793/360)}')
COV="[0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]"

MSG="{header: {frame_id: map}, pose: {pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}, covariance: $COV}}"
echo "投げる: x=$X y=$Y yaw=${YAW_DEG}deg （散らし 0.5 m / 15 deg）"
jros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$MSG" > /tmp/pub.log 2>&1 &
PID=$!
for _ in $(seq 24); do kill -0 "$PID" 2>/dev/null || break; sleep 0.5; done
kill -9 "$PID" 2>/dev/null || true
tail -2 /tmp/pub.log
