. "$HOME/jammy_ros/env.sh"
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
run_with_limit() { local limit="$1"; shift; "$@" > "$2x" 2>&1 & local pid=$!
  local n=0; while kill -0 "$pid" 2>/dev/null; do sleep 0.5; n=$((n+1)); [ "$n" -ge $((limit*2)) ] && { kill -9 "$pid" 2>/dev/null; break; }; done; }
jros2 topic echo /scan --once --full-length > /tmp/scan.yaml 2>&1 &
P1=$!; n=0; while kill -0 $P1 2>/dev/null; do sleep 0.5; n=$((n+1)); [ $n -ge 30 ] && { kill -9 $P1 2>/dev/null; break; }; done
jros2 topic echo /amcl_pose --once > /tmp/pose.yaml 2>&1 &
P2=$!; n=0; while kill -0 $P2 2>/dev/null; do sleep 0.5; n=$((n+1)); [ $n -ge 24 ] && { kill -9 $P2 2>/dev/null; break; }; done
echo "scan: $(wc -l < /tmp/scan.yaml) 行 / pose: $(wc -l < /tmp/pose.yaml) 行"
