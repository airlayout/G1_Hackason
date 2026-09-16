# ⚠️ `timeout jros2 ...` は使えない（jros2 はシェル関数）。背景に置いて時間で切る。
. "$HOME/jammy_ros/env.sh"
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'

run_with_limit() {  # $1 秒で打ち切る
    local limit="$1"; shift
    "$@" > /tmp/jros_out.txt 2>&1 &
    local pid=$!
    local waited=0
    while kill -0 "$pid" 2>/dev/null; do
        sleep 0.5; waited=$((waited+1))
        [ "$waited" -ge $((limit*2)) ] && { kill -9 "$pid" 2>/dev/null; break; }
    done
    cat /tmp/jros_out.txt
}

echo "=== /amcl_pose ==="
run_with_limit 12 jros2 topic echo /amcl_pose --once | head -40
