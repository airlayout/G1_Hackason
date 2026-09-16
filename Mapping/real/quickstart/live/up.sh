#!/usr/bin/env bash
# **無線のまま RViz2 で Nav2 を動かすところまで、一気に起こす。**
#
#   bash quickstart/live/preflight.sh          # 先にこれ
#   bash quickstart/live/up.sh --init "X Y YAW_DEG"
#   bash quickstart/live/up.sh                 # 初期姿勢をタグから自動で取る
#
# ⚠️ **足は繋がない。** 歩かせるのは `live/legs.sh` で別に行う（事故の幅を狭くする）。
#
# ## 何が立つか
#
#   PC2 : FAST_LIO(+open3d_loc) -> cloud_to_scan -> Nav2 -> foxglove_bridge -> 姿勢ロガー
#   Mac : 中継(in) -> map_server -> 中継(back) -> RViz2
#
#   機体 --> 橋 --(WS/TCP・AP 越え)--> 中継(in) --> コンテナの DDS --> RViz2
#   RViz2 --> コンテナの DDS --> 中継(back) --(WS)--> 橋 --> 機体の /goal_pose
#
# **なぜ DDS を直接 AP に流さないのか**（2026-09-16 に全部実測した）:
#
# | PC2 の DDS | 同一ホスト内 | PC1 の LiDAR | コンテナから見える |
# |---|---|---|---|
# | eth0 単独 | OK | OK 130 枚 | ⛔ |
# | wlan0 単独 | OK | **⛔ 0 枚** | OK |
# | 2 NIC（peer=コンテナのみ）| **⛔ 2 件** | ⛔ | — |
# | 2 NIC（+ localhost）| OK 25 件 | **⛔ 0 枚** | — |
#
# 2 NIC にすると CycloneDDS が `selected interface "wlan0"` となり、eth0 側の PC1 が
# 見えなくなる。priority / ExternalNetworkAddress / PC1 を peer に追加、どれも効かない。
# ⇒ **DDS は eth0 に固定し、外へは WebSocket で出す。**
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/_common.sh"
QS="$(dirname "$HERE")"

INIT=""
while [ $# -gt 0 ]; do
    case "$1" in
        --init) INIT="$2"; shift 2 ;;
        *) die "知らない引数: $1" ;;
    esac
done

# ── 0. 初期姿勢（タグから取る）──────────────────────────────────────
if [ -z "$INIT" ]; then
    say "初期姿勢をタグから取る（大域探索も種も要らない）"
    OUT="$(pc2 'cd "$HOME/g1_cfg/apriltag" && timeout 150 python3 localize_from_tags.py \
                 --live --seconds 5 \
                 --registry "$HOME/g1_cfg/apriltag/tag_registry.json" \
                 --extrinsics "$HOME/g1_cfg/apriltag/extrinsics.json" 2>&1' || true)"
    printf '%s\n' "$OUT" | grep -E '^\*\*|\*\*\(x|食い違い|z .*roll|ばらつき' | sed 's/^/     /' >&2
    INIT="$(printf '%s\n' "$OUT" | sed -n 's/.*\*\*(x *\([-0-9.]*\), *y *\([-0-9.]*\), *yaw *\([-0-9.]*\) deg)\*\*.*/\1 \2 \3/p' | head -1)"
    [ -n "$INIT" ] || die "タグから姿勢が取れなかった。--init \"X Y YAW\" で渡すこと"
    ok "タグ由来の初期姿勢: $INIT"
    # ⚠️ 自己検査が落ちていたら採用しない（誤った種は FAST_LIO では復帰しない）
    printf '%s\n' "$OUT" | grep -q '⛔' && die "タグの自己検査が落ちている。貼り直すか近づく"
fi

# ── 1. 測位 ────────────────────────────────────────────────────────
say "1. FAST_LIO を起こす（初期姿勢 $INIT）"
pc2 "rm -f /tmp/pc2_c5_run.log
     nohup setsid bash \$HOME/g1_cfg/run_fastlio_loc_live.sh --init '$INIT' \
       > /tmp/pc2_c5_run.log 2>&1 < /dev/null &
     sleep 30
     grep -E '初期姿勢|動作中|エラー' /tmp/pc2_c5_run.log | sed 's/^/     /'"

say "   ⚠️ **最初に roll/pitch を見る**（立位なら 0 付近。±3 度を超えたら取付を疑う）"
TF="$(pc2_ros "\$HOME/g1_cfg/../nav_tools/../:; true" 2>/dev/null; \
      pc2 "cat > /tmp/_tf.sh <<'EOS'
. \"\$HOME/jammy_ros/env.sh\" >/dev/null 2>&1
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='$G1_DDS_ETH0'
setsid bash -c \". \\\$HOME/jammy_ros/env.sh >/dev/null 2>&1
  export ROS_DOMAIN_ID=0 CYCLONEDDS_URI='$G1_DDS_ETH0'
  jros2 run tf2_ros tf2_echo map base_link\" > /tmp/_tf.txt 2>&1 &
p=\$!; g=\$(ps -o pgid= -p \$p | tr -d ' '); sleep 12; kill -9 -\$g 2>/dev/null
grep -E 'Translation|RPY \(degree\)' /tmp/_tf.txt | tail -2
EOS
      bash /tmp/_tf.sh")"
printf '%s\n' "$TF" | sed 's/^/     /' >&2
printf '%s\n' "$TF" | grep -q Translation || die "map->base_link が出ていない。/tmp/pc2_o3dloc.log を見る"

# ── 2. /scan（2D 化）────────────────────────────────────────────────
# ⚠️ CYCLONEDDS_URI を渡さないと wlan0 を掴んで Nav2 から見えなくなる。
say "2. cloud_to_scan を起こす（帯 1.30〜1.82 m）"
pc2 "P=\"\$HOME/jammy_ros/rootfs\"
     . \"\$HOME/jammy_ros/env.sh\" >/dev/null 2>&1
     export ROS_DOMAIN_ID=0
     export CYCLONEDDS_URI='$G1_DDS_ETH0'
     nohup setsid env \$JAMMY_ENV \"\$LOADER\" --library-path \"\$JAMMY_LIBS\" \
       \"\$P/usr/bin/python3.10\" \"\$HOME/g1_cfg/cloud_to_scan.py\" \
       --min-height 1.30 --max-height 1.82 > /tmp/pc2_scan.log 2>&1 < /dev/null &
     sleep 12; tail -1 /tmp/pc2_scan.log | cut -c1-110 | sed 's/^/     /'"

# ── 3. Nav2 ────────────────────────────────────────────────────────
say "3. Nav2 を起こす"
pc2 "rm -f /tmp/pc2_nav2_run.log
     nohup setsid bash \$HOME/g1_cfg/run_nav2_live.sh > /tmp/pc2_nav2_run.log 2>&1 < /dev/null &
     sleep 40
     grep -E '完了|ログ' /tmp/pc2_nav2_run.log | sed 's/^/     /'"

# ── 4. 橋 ──────────────────────────────────────────────────────────
# ⚠️ Nav2 より**後**に起こす（先に起こすと Nav2 のトピックを拾い損ねる）。
say "4. foxglove_bridge を起こす（⚠️ Nav2 の後）"
pc2 "rm -f /tmp/fox.log
     nohup bash \"\$HOME/mapping_tools/start_fox\"\"glove_bridge.sh\" > /tmp/fox.log 2>&1 < /dev/null &
     sleep 30
     ss -ltn 2>/dev/null | grep -q 8765 && echo '     8765 待受中' || echo '     ⛔ 橋が上がらない'"

# ── 5. 姿勢ロガー（⚠️ /tmp に置かない）──────────────────────────────
say "5. 姿勢ロガー（記録は \$HOME/g1_runs。/tmp だと電源断で消える）"
STAMP="$(date +%Y%m%dT%H%M%S)"
pc2 "mkdir -p \$HOME/g1_runs
     P=\"\$HOME/jammy_ros/rootfs\"
     . \"\$HOME/jammy_ros/env.sh\" >/dev/null 2>&1
     export ROS_DOMAIN_ID=0
     export CYCLONEDDS_URI='$G1_DDS_ETH0'
     nohup setsid env \$JAMMY_ENV \"\$LOADER\" --library-path \"\$JAMMY_LIBS\" \
       \"\$P/usr/bin/python3.10\" \"\$HOME/g1_cfg/apriltag/log_tf_pose.py\" \
       --target map --source base_link --out \$HOME/g1_runs/pose_$STAMP.txt \
       > \$HOME/g1_runs/pose_$STAMP.log 2>&1 < /dev/null &
     sleep 12
     echo \"     \$(wc -l < \$HOME/g1_runs/pose_$STAMP.txt) 行 -> ~/g1_runs/pose_$STAMP.txt\""
echo "$STAMP" > "$HERE/.last_run"

# ── 6. Mac 側 ──────────────────────────────────────────────────────
say "6. コンテナの中継・地図・RViz2"
MAP=/work/G1_Hackason/Mapping/real/runs/20260906T135940_UiS_room_v3/map/nav_map_run.yaml
ctr_kill "foxglove_to_r" "os.py"; ctr_kill "ros_to_foxglo" "ve.py"
ctr_kill "map_ser" "ver"; ctr_kill "rvi" "z2"

ctr_bg "source /opt/ros/humble/setup.bash >/dev/null 2>&1
        cd /work/G1_Hackason/Mapping/real/quickstart
        exec python3 -u foxglove_to_ros.py --host $G1_PC2_HOST > /tmp/relay.log 2>&1"
ctr_bg "source /opt/ros/humble/setup.bash >/dev/null 2>&1
        exec ros2 run nav2_map_server map_server --ros-args \
             -p yaml_filename:=$MAP -p use_sim_time:=false > /tmp/mapsrv.log 2>&1"
sleep 18
ctr "source /opt/ros/humble/setup.bash >/dev/null 2>&1
     ros2 lifecycle set /map_server configure >/dev/null 2>&1
     ros2 lifecycle set /map_server activate  >/dev/null 2>&1
     echo '     map_server 活性化'"
ctr_bg "source /opt/ros/humble/setup.bash >/dev/null 2>&1
        cd /work/G1_Hackason/Mapping/real/quickstart
        exec python3 -u ros_to_foxglove.py --host $G1_PC2_HOST > /tmp/back.log 2>&1"
sleep 12

docker cp "$QS/rviz/g1_nav_nocloud.rviz" "$G1_CONTAINER:/home/ubuntu/g1_nav_nocloud.rviz" >/dev/null
docker exec -d -u ubuntu -e HOME=/home/ubuntu -e DISPLAY=:1 \
    -e XAUTHORITY=/home/ubuntu/.Xauthority \
    -e ROS_DOMAIN_ID=0 -e CYCLONEDDS_URI="$G1_DDS_COL0" "$G1_CONTAINER" \
    bash -c 'source /opt/ros/humble/setup.bash; rviz2 -d /home/ubuntu/g1_nav_nocloud.rviz > /home/ubuntu/rviz2.log 2>&1'
sleep 20

# ── 7. 通しの確認 ──────────────────────────────────────────────────
say "7. 通しの確認"
ctr "source /opt/ros/humble/setup.bash >/dev/null 2>&1
     echo '  -- コンテナ側で map->base_link --'
     timeout 18 ros2 run tf2_ros tf2_echo map base_link 2>&1 \
       | grep -E 'Translation|RPY \(degree\)' | head -2 | sed 's/^/     /'" || true
ctr "tail -1 /tmp/relay.log | sed 's/^/     中継(in)  /'; tail -1 /tmp/back.log | sed 's/^/     中継(back) /'"

printf '\n'
say "RViz2:            http://$G1_VM_IP/"
say "軽量ビュー:        bash $QS/run_nav_live_view.sh $G1_PC2_HOST  → http://<Mac>:8080/"
say "記録:             ~/g1_runs/pose_$STAMP.txt（PC2）"
printf '\n'
say "⚠️ **足はまだ繋がっていない**（/cmd_vel の購読者 0 ＝ 機体は動かない）"
say "   歩かせる: bash quickstart/live/legs.sh --arm"
