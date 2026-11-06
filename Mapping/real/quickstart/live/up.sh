#!/usr/bin/env bash
# **無線のまま RViz2 で Nav2 を動かすところまで、一気に起こす。**
#
#   bash quickstart/live/preflight.sh          # 先にこれ
#   bash quickstart/live/up.sh --init "X Y YAW_DEG"
#   bash quickstart/live/up.sh                 # 初期姿勢を自動で取る（タグ → 回る → 指紋）
#
# ⚠️ **足は繋がない。** 歩かせるのは `live/legs.sh` で別に行う（事故の幅を狭くする）。
#
# ## 何が立つか
#
#   PC2 : FAST_LIO(+open3d_loc) -> cloud_to_scan -> octomap_server -> Nav2
#         -> foxglove_bridge -> 姿勢ロガー
#   Mac : 中継(in) -> 中継(back) -> RViz2
#
# ⚠️ **2026-09-16 に静的レイヤを `octomap_server` の `/projected_map` に移した**
# （机が動くたびに地図を作り直す手間をなくす。段 1〜5 は機体なしで測って合格）。
#   - 種は `quickstart/octomap_seed_from_nav_map.py` が `nav_map_run` から作る
#   - `octomap_server` は `run_nav2_live.sh` が起こす（値の根拠はあちらの注記）
#   - **コンテナの map_server は既定 off**（`G1_SHOW_SEED_MAP=1` で比較用に出せる）
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

# ⚠️ **有線なら橋も中継も要らない（2026-09-17 に実測）。**
# PC2 を 192.168.123.0/24 に有線で繋ぐと、コンテナ（col0 = 192.168.123.201）から
# 機体の DDS が全部見え、RViz2 が直接 Nav2 と喋れる。中継は AP 越え専用の仕掛けである。
#   G1_LINK=wired … 橋と中継を起こさない（既定。PC2 の有線 IP に届けばこちら）
#   G1_LINK=ap    … 従来どおり橋＋中継を起こす
LINK="${G1_LINK:-auto}"
if [ "$LINK" = "auto" ]; then
    if tcp_ok "$G1_PC2_WIRED" 22 2>/dev/null; then LINK=wired; else LINK=ap; fi
fi
say "リンク構成: ${LINK}"

INIT=""
while [ $# -gt 0 ]; do
    case "$1" in
        --init) INIT="$2"; shift 2 ;;
        *) die "知らない引数: $1" ;;
    esac
done

# ── 0. 初期姿勢（タグ → 回って見直す → LiDAR の指紋）────────────────
# ⚠️ 2026-09-26: 段 0 は initial_pose.sh に切り出した（タグが見えないときの手段を足したため）。
# タグの自己検査の案内（水平だけ落ちたら機体を直す、等）もあちらに移してある。
if [ -z "$INIT" ]; then
    INIT="$(bash "$HERE/initial_pose.sh")" || die "初期姿勢が決まらなかった（上の行）"
    [ -n "$INIT" ] || die "initial_pose.sh が姿勢を返さなかった"
    ok "初期姿勢: $INIT"
fi

# ── 1. 測位 ────────────────────────────────────────────────────────
say "1. FAST_LIO を起こす（初期姿勢 ${INIT}）"
pc2 "rm -f /tmp/pc2_c5_run.log
     nohup setsid bash \$HOME/g1_cfg/run_fastlio_loc_live.sh --init '$INIT' \
       > /tmp/pc2_c5_run.log 2>&1 < /dev/null &"
# ⚠️ 旧 sleep 30。run_fastlio_loc_live.sh は最後に「動作中。ログ:」を出すので、それを待つ
pc2_wait "FAST_LIO の起動" 60 'grep -qE "動作中|エラー" /tmp/pc2_c5_run.log'
pc2 "grep -E '初期姿勢|動作中|エラー' /tmp/pc2_c5_run.log | sed 's/^/     /'"

say "   ⚠️ **最初に roll/pitch を見る**（立位なら 0 付近。±3 度を超えたら取付を疑う）"
# ⚠️ **待つ。**FAST_LIO を起こした直後は `map` フレームがまだ無く、
# tf2_echo が `Invalid frame ID "map" ... frame does not exist` を返す。
# open3d_loc は 2.5 Hz（1 周 2.3 秒スリープ）なので収束まで数十秒かかる。
# 2026-09-17 に 12 秒だけ見て die し、**測位は正常なのに「出ていない」と誤診した**
# （そのとき ICP の fitness は 0.974 で良好だった）。
TF=""
for attempt in 1 2 3 4 5 6 7 8 9 10; do
    TF="$(pc2_ros2 12 run tf2_ros tf2_echo map base_link 2>/dev/null \
          | grep -E 'Translation|RPY \(degree\)' | tail -2)"
    [ -n "$TF" ] && break
    say "   map->base_link を待つ（${attempt}/10・open3d_loc の収束待ち）"
done
printf '%s\n' "$TF" | sed 's/^/     /' >&2
printf '%s\n' "$TF" | grep -q Translation \
    || die "map->base_link が 2 分待っても出ない。/tmp/pc2_o3dloc.log と /tmp/pc2_fastlio.log を見る"

# ── 2. /scan（2D 化）────────────────────────────────────────────────
# ⚠️ CYCLONEDDS_URI を渡さないと wlan0 を掴んで Nav2 から見えなくなる。
say "2. cloud_to_scan を起こす（帯 1.30〜1.82 m）"
pc2 "P=\"\$HOME/jammy_ros/rootfs\"
     . \"\$HOME/jammy_ros/env.sh\" >/dev/null 2>&1
     export ROS_DOMAIN_ID=0
     export CYCLONEDDS_URI='$G1_DDS_ETH0'
     nohup setsid env \$JAMMY_ENV \"\$LOADER\" --library-path \"\$JAMMY_LIBS\" \
       \"\$P/usr/bin/python3.10\" \"\$HOME/g1_cfg/cloud_to_scan.py\" \
       --min-height 1.30 --max-height 1.82 > /tmp/pc2_scan.log 2>&1 < /dev/null &"
# ⚠️ 旧 sleep 12。「N 枚目」が出れば**実際に点が流れている**証拠になる（時間より強い）
pc2_wait "cloud_to_scan の出力" 40 'grep -q "枚目" /tmp/pc2_scan.log'
pc2 "tail -1 /tmp/pc2_scan.log | cut -c1-110 | sed 's/^/     /'"

# ── 2.5 種から立ち位置を空ける ──────────────────────────────────────
# ⚠️ 2026-09-28: 種に**記録のあとで動かされた家具**が残り、それが立ち位置の 0.10 m に
# あったため `Starting point in lethal space!` で経路が作れず、その場で回って止まった。
# 頭の LiDAR は機体から水平 0.90 m より内側の床を見られないので、octomap_server は
# 足元の古い占有を**消せない**。機体の体がある円柱（0.35 m ＝ filter_self_returns.py と同じ）
# に障害物は存在できないので、そこだけを空きにした種を起動のたびに作る。
# 根拠と半径の決め方は quickstart/carve_seed_start.py の冒頭。G1_CARVE_START=0 で飛ばす。
SEED_ENV=""
if [ "${G1_CARVE_START:-1}" = "1" ]; then
    say "2.5 種から立ち位置を空ける（半径 0.35 m）"
    CARVE_PY="$QS/../../../.venv/bin/python"
    CARVE_DIR="$(mktemp -d)"
    if [ ! -x "$CARVE_PY" ]; then
        warn "Python が無い（${CARVE_PY}）。元の種のまま起こす"
    elif ! scp -q -i "$G1_KEY" "$G1_PC2_USER@$G1_PC2_HOST:g1_cfg/map/seed.bt" "$CARVE_DIR/seed.bt"; then
        warn "PC2 の ~/g1_cfg/map/seed.bt を取れない。元の種のまま起こす"
    elif "$CARVE_PY" "$QS/carve_seed_start.py" --seed "$CARVE_DIR/seed.bt" \
            --out "$CARVE_DIR/seed_start.bt" --pose "$INIT" 2>&1 | sed 's/^/     /' >&2 \
         && [ -s "$CARVE_DIR/seed_start.bt" ] \
         && scp -q -i "$G1_KEY" "$CARVE_DIR/seed_start.bt" \
                "$G1_PC2_USER@$G1_PC2_HOST:g1_cfg/map/seed_start.bt"; then
        SEED_ENV='G1_OCTOMAP_SEED=$HOME/g1_cfg/map/seed_start.bt'   # $HOME は PC2 で展開させる
        ok "立ち位置を空けた種 → ~/g1_cfg/map/seed_start.bt"
    else
        warn "立ち位置を空けられなかった（上の行）。元の種のまま起こす"
    fi
    rm -rf "$CARVE_DIR"
fi

# ── 3. Nav2 ────────────────────────────────────────────────────────
say "3. Nav2 を起こす"
pc2 "rm -f /tmp/pc2_nav2_run.log /tmp/pc2_nav2.log   # ⚠️ 前回のログが残ると下の待ちが即合格する（2026-09-28）
     nohup setsid env $SEED_ENV bash \$HOME/g1_cfg/run_nav2_live.sh > /tmp/pc2_nav2_run.log 2>&1 < /dev/null &"
# ⚠️ 旧 sleep 40。run_nav2_live.sh の「ログ:」は**起こした直後**に出るので合図にならない。
# nav2_bringup は lifecycle_manager を autostart するので、その完了を待つ。
pc2_wait "Nav2 の活性化" 120 'grep -q "Managed nodes are active" /tmp/pc2_nav2.log'
pc2 "grep -E '完了|ログ' /tmp/pc2_nav2_run.log | sed 's/^/     /'"

# ── 4. 橋 ──────────────────────────────────────────────────────────
# ⚠️ Nav2 より**後**に起こす（先に起こすと Nav2 のトピックを拾い損ねる）。
if [ "$LINK" = "wired" ]; then
    say "4. foxglove_bridge は飛ばす（有線なので DDS が直接見える）"
else
say "4. foxglove_bridge を起こす（⚠️ Nav2 の後）"
pc2 "rm -f /tmp/fox.log
     nohup bash \"\$HOME/mapping_tools/start_fox\"\"glove_bridge.sh\" > /tmp/fox.log 2>&1 < /dev/null &"
# ⚠️ 旧 sleep 30。待受ポートが立つことだけが合図になる
pc2_wait "foxglove_bridge の待受 8765" 60 'ss -ltn 2>/dev/null | grep -q 8765' \
    || pc2 "tail -3 /tmp/fox.log | sed 's/^/     /'"
fi

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
       > \$HOME/g1_runs/pose_$STAMP.log 2>&1 < /dev/null &"
# ⚠️ 旧 sleep 12。**行が書けている**ことが合図（10 Hz なので 10 行 = 約 1 秒ぶん）。
# ⚠️ `wc` の前に `-s` を見る。ファイルが無いと `wc -l <` は何も印字せず、
#    `[ "" -ge 10 ]` が構文エラーになる（`|| echo 0` で繕うと "00" 型の再発）。
pc2_wait "姿勢ロガーの書き込み" 40 \
    "[ -s \$HOME/g1_runs/pose_$STAMP.txt ] && [ \"\$(wc -l < \$HOME/g1_runs/pose_$STAMP.txt)\" -ge 10 ]"
pc2 "echo \"     \$(wc -l < \$HOME/g1_runs/pose_$STAMP.txt) 行 -> ~/g1_runs/pose_$STAMP.txt\""
echo "$STAMP" > "$HERE/.last_run"

# ── 6. Mac 側 ──────────────────────────────────────────────────────
say "6. コンテナの中継・地図・RViz2"
# ⚠️ 比較表示（G1_SHOW_SEED_MAP=1）用の地図。走る静的レイヤは PC2 の ~/g1_cfg/map/seed.bt
# なので、**ここを変えても Nav2 の地図は変わらない**。差し替えは PC2 へ配る側で行う
MAP_SESSION="${G1_MAP_SESSION:-20260906T135940_UiS_room_v3}"
MAP=/work/G1_Hackason/Mapping/real/runs/$MAP_SESSION/map/nav_map_run.yaml
ctr_kill "foxglove_to_r" "os.py"; ctr_kill "ros_to_foxglo" "ve.py"
ctr_kill "map_ser" "ver"; ctr_kill "rvi" "z2"

if [ "$LINK" = "wired" ]; then
    say "   中継(in) は飛ばす（DDS が直接見える）"
else
    ctr_bg "source /opt/ros/humble/setup.bash >/dev/null 2>&1
            cd /work/G1_Hackason/Mapping/real/quickstart
            exec python3 -u foxglove_to_ros.py --host $G1_PC2_HOST > /tmp/relay.log 2>&1"
    # ⚠️ 旧 sleep 18。**ログの有無で見ない。**「橋に繋ぐ ws://…」は接続する*前*に
    # 出るので、繋がらなくても即合格になる。広告を受け取った行だけが証拠になる。
    ctr_wait "中継(in)" 40 'grep -q "橋が広告するトピック" /tmp/relay.log' \
        || ctr "tail -3 /tmp/relay.log | sed 's/^/     /'"
fi

# ── コンテナ側の固定地図（既定 off）────────────────────────────────
# 2026-09-16 に既定を off にした。静的レイヤは `octomap_server` の `/projected_map`
# に移り、**中継がそれを運ぶ**（`foxglove_to_ros.py` の DEFAULT_TOPICS。0.5 Hz に間引く）。
# ⚠️ ここで map_server を立てると RViz2 に**育たない固定地図**が `/map` として並び、
# 「机を動かしても画面が変わらない」と誤診する元になる（計画 §5-2 の警告）。
# 種と育った地図を見比べたいときだけ G1_SHOW_SEED_MAP=1 で出す。
if [ "${G1_SHOW_SEED_MAP:-0}" = "1" ]; then
    say "   固定地図（種の元）を /map に出す ＝ 比較用"
    ctr_bg "source /opt/ros/humble/setup.bash >/dev/null 2>&1
            exec ros2 run nav2_map_server map_server --ros-args \
                 -p yaml_filename:=$MAP -p use_sim_time:=false > /tmp/mapsrv.log 2>&1"
    ctr_wait "map_server の起動" 40 '[ -s /tmp/mapsrv.log ]'
    ctr "source /opt/ros/humble/setup.bash >/dev/null 2>&1
         ros2 lifecycle set /map_server configure >/dev/null 2>&1
         ros2 lifecycle set /map_server activate  >/dev/null 2>&1
         echo '     map_server 活性化'"
fi
if [ "$LINK" = "wired" ]; then
    say "   中継(back) は飛ばす（RViz2 のクリックは DDS で直接届く）"
else
    ctr_bg "source /opt/ros/humble/setup.bash >/dev/null 2>&1
            cd /work/G1_Hackason/Mapping/real/quickstart
            exec python3 -u ros_to_foxglove.py --host $G1_PC2_HOST > /tmp/back.log 2>&1"
    # ⚠️ 旧 sleep 12。ここも「繋ぐ」ではなく**設定が済んだ**行で見る
    ctr_wait "中継(back)" 40 'grep -q "待機中" /tmp/back.log' \
        || ctr "tail -3 /tmp/back.log | sed 's/^/     /'"
fi

docker cp "$QS/rviz/g1_nav_nocloud.rviz" "$G1_CONTAINER:/home/ubuntu/g1_nav_nocloud.rviz" >/dev/null
docker exec -d -u ubuntu -e HOME=/home/ubuntu -e DISPLAY=:1 \
    -e XAUTHORITY=/home/ubuntu/.Xauthority \
    -e ROS_DOMAIN_ID=0 -e CYCLONEDDS_URI="$G1_DDS_COL0" "$G1_CONTAINER" \
    bash -c 'source /opt/ros/humble/setup.bash; rviz2 -d /home/ubuntu/g1_nav_nocloud.rviz > /home/ubuntu/rviz2.log 2>&1'
# ⚠️ 旧 sleep 20。**`ps | grep` で見ない。**判定文のコマンド行に名前が載って
# 自己マッチし、起動していなくても「居る」と出る（2026-09-18 にこの形で実際に騙された）。
# ログが書けたかだけを見る。遅れても後段は困らないので上限で先へ進んでよい。
ctr_wait "RViz2" 40 '[ -s /home/ubuntu/rviz2.log ]'

# ── 7. 通しの確認 ──────────────────────────────────────────────────
say "7. 通しの確認"
ctr "source /opt/ros/humble/setup.bash >/dev/null 2>&1
     echo '  -- コンテナ側で map->base_link --'
     timeout 18 ros2 run tf2_ros tf2_echo map base_link 2>&1 \
       | grep -E 'Translation|RPY \(degree\)' | head -2 | sed 's/^/     /'" || true
if [ "$LINK" = "wired" ]; then
    ctr "source /opt/ros/humble/setup.bash >/dev/null 2>&1
         echo '  -- 静的レイヤ（/projected_map）--'
         timeout 15 ros2 topic echo --once --field info /projected_map 2>&1 | head -6 | sed 's/^/     /'" || true
else
    ctr "tail -1 /tmp/relay.log | sed 's/^/     中継(in)  /'; tail -1 /tmp/back.log | sed 's/^/     中継(back) /'"
fi

# ── 立ち位置の cost（クリックする前に分かるように）────────────────────
# ⚠️ lethal だと、クリックしても「その場で 90° 回って 5 秒待つ」を繰り返して止まるだけで、
# 原因は PC2 の /tmp/pc2_nav2.log にしか出ない（2026-09-28）。PC2 で見るので AP 構成でも効く
echo '  -- 立ち位置の cost（global costmap）--' >&2
scp -q -i "$G1_KEY" "$QS/check_start_cost.py" "$G1_PC2_USER@$G1_PC2_HOST:g1_cfg/" \
    || warn "check_start_cost.py を送れない"
START_OUT="$(pc2 "P=\"\$HOME/jammy_ros/rootfs\"
     . \"\$HOME/jammy_ros/env.sh\" >/dev/null 2>&1
     export ROS_DOMAIN_ID=0
     export CYCLONEDDS_URI='$G1_DDS_ETH0'
     timeout 40 env \$JAMMY_ENV \"\$LOADER\" --library-path \"\$JAMMY_LIBS\" \
       \"\$P/usr/bin/python3.10\" \"\$HOME/g1_cfg/check_start_cost.py\"; echo \"rc=\$?\"" 2>&1)"
printf '%s\n' "$START_OUT" | grep '^\[start\]' | sed 's/^/     /' >&2
case "$START_OUT" in
    *rc=0*) ok "立ち位置から経路を作れる" ;;
    *rc=1*) bad "立ち位置が lethal。**クリックしても回って止まる**"
            echo "     → リモコンで空いた床へ 1 m ずらしてからクリックする" >&2
            echo "       （G1_CARVE_START=0 で起こしていないか。初期姿勢がずれていないか も疑う）" >&2 ;;
    *)      warn "立ち位置の cost を測れなかった（上の行）" ;;
esac

printf '\n'
say "RViz2:            http://$G1_VM_IP/"
say "軽量ビュー:        bash $QS/run_nav_live_view.sh $G1_PC2_HOST  → http://<Mac>:8080/"
say "記録:             ~/g1_runs/pose_$STAMP.txt（PC2）"
say "頭カメラの映像:    bash quickstart/live/camera.sh start → RViz2 の「Head camera (IR)」にチェック"
printf '\n'
say "⚠️ **足はまだ繋がっていない**（/cmd_vel の購読者 0 ＝ 機体は動かない）"
say "   歩かせる: bash quickstart/live/legs.sh --arm"
