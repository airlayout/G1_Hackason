#!/usr/bin/env bash
# octomap_server に種（.bt）を読ませて、育っても種が残るかを**機体なしで**測る。
# 計画 docs/plan/2026-09-16-growing-map-octomap.md の段 2。
#
# ## やること
#   1. 孤児の octomap_server を始末してから、種つきで起こす
#   2. /projected_map を PGM に落とす（before ＝種を読んだ直後）
#   3. 09-06 の姿勢つきスキャンを配信して木を育てる
#   4. 配信中にもう一度落とす（during）
#   5. 配信が止まってから落とす（after ＝ latched かどうかが出る）
#   6. 3 段を report_octomap_stages.py が並べて合否を出す
#
# ## ⚠️ ノードのバイナリを直接起こす理由
# `ros2 run octomap_server octomap_server_node` を kill すると、**ラッパだけ死んで
# ノードが孤児になる**（2026-09-16 に 5 個溜めた。複数パブリッシャになって
# /projected_map の数字が黙って混ざり、「1 セルも動いていない」と誤診した）。
# バイナリを直に起こせば $! が本体の PID になる。
#
# ## ⚠️ 姿勢源に 09-13 の実機記録を使わない理由
# あの /tf は事前地図との重畳が 65〜70 % しかない（MOLA が偽の極大に入った疑い）。
# 「種が壊れた」のか「姿勢がずれていた」のか切り分けられない。
# 09-06 の benchmark_s5 は**その地図を作った当のスキャンと姿勢**なので定義上ずれない。
#
# ## ⚠️ during を撮る理由
# `latch:=false` のパブリッシャは VOLATILE ＝履歴を持たない。点群が止まってから
# 購読しても 1 通も来ない（2026-09-16 に「来なかった」と出して QoS 不一致と誤診した）。
#
# ## 使い方（Mac から。コンテナ `rviz` の中で走る）
#   bash quickstart/verify_octomap_seed.sh runs/20260906T135940_UiS_room_v3
#   LATCH=false bash quickstart/verify_octomap_seed.sh runs/<id>
#   BAND_MIN=1.30 BAND_MAX=1.82 bash quickstart/verify_octomap_seed.sh runs/<id>
set -eo pipefail

SESSION_REL="${1:?使い方: verify_octomap_seed.sh runs/<session>}"
CONTAINER="${CONTAINER:-rviz}"
SEED="${SEED:-seed.bt}"
BAND_MIN="${BAND_MIN:-0.23}"
BAND_MAX="${BAND_MAX:-1.80}"
RESOLUTION="${RESOLUTION:-0.1}"
LATCH="${LATCH:-true}"
STRIDE="${STRIDE:-5}"
RATE="${RATE:-10}"
SCANS="${SCANS:-benchmark_s5}"
# 配備済みの *_floor0.pcd が元の SLAM 出力から z にずれている量。
# octomap_seed_from_scans.py の FLOOR0_Z_OFFSET と同じ値にすること
Z_OFFSET="${Z_OFFSET:-1.247803}"
DOMAIN="${DOMAIN:-77}"
STAGE_DIR="${STAGE_DIR:-stage2}"
# ⚠️ octomap_server の既定は **-1.0 ＝無制限**。無制限のレイは静止構造の voxel を
# grazing 角で舐めて「空」に投票するので、2026-09-06 の地図作成では天井の 56.7 % と
# 遠方構造の 31.6 % が消えた。実機では必ず絞る（過去の掃引の最適は 3〜4 m）
MAX_RANGE="${MAX_RANGE:--1.0}"
# 既定は octomap_server の既定値そのまま（種を作ったときと同じ値でもある）
HIT="${HIT:-0.7}"
MISS="${MISS:-0.4}"

# latch=false のパブリッシャは VOLATILE。TRANSIENT_LOCAL で購読すると噛み合わない
DUMP_QOS=""
if [ "$LATCH" != "true" ]; then DUMP_QOS="--volatile"; fi

MAC_REAL=/Users/inouereo/git_research/physical_ai/G1_Hackason/Mapping/real
REAL=/work/G1_Hackason/Mapping/real
SESSION="$REAL/$SESSION_REL"
OUT="$SESSION/map/$STAGE_DIR"

docker exec -u ubuntu "$CONTAINER" bash -lc "
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=$DOMAIN RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# lo だけに閉じる。機体や他のセッションと混ざらせない
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\" /></Interfaces></General></Domain></CycloneDDS>'

# ⚠️ **プロセス名で数える（pgrep -x）。**'octomap_server_node' という文字列で探すと、
# この docker exec の bash -lc 自身の引数に同じ文字列が入っているので**自分に一致する**
# （2026-09-16 に「残った 2 個」と誤報した。pkill -f の自己マッチと同じ型）。
# Linux の comm は 15 文字で切れるので名前は octomap_server_ になる
for pid in \$(pgrep -x octomap_server_ 2>/dev/null); do
  echo \"[掃除] 孤児の octomap_server \$pid を殺す\"
  kill -KILL \"\$pid\" 2>/dev/null || true
done

rm -rf $OUT/before $OUT/during $OUT/after
mkdir -p $OUT/before $OUT/during $OUT/after
/opt/ros/humble/lib/octomap_server/octomap_server_node --ros-args \
  -r cloud_in:=/replay/cloud \
  -p octomap_path:=$SESSION/map/$SEED \
  -p frame_id:=map -p resolution:=$RESOLUTION \
  -p occupancy_min_z:=$BAND_MIN -p occupancy_max_z:=$BAND_MAX \
  -p sensor_model.max_range:=$MAX_RANGE \
  -p sensor_model.hit:=$HIT -p sensor_model.miss:=$MISS \
  -p latch:=$LATCH > /tmp/octomap_server.log 2>&1 &
SERVER=\$!
trap 'kill -TERM \$SERVER 2>/dev/null || true' EXIT INT TERM

for _ in \$(seq 1 40); do
  if grep -q 'loaded (' /tmp/octomap_server.log; then break; fi
  sleep 1
done
if ! grep -q 'loaded (' /tmp/octomap_server.log; then
  echo '[NG] 種を読めていない。ログ:' >&2; cat /tmp/octomap_server.log >&2; exit 1
fi
grep 'loaded (' /tmp/octomap_server.log
sleep 3

echo
echo '--- before（種を読んだ直後）'
python3 $REAL/quickstart/dump_grids.py --layer projected=/projected_map \
  --out $OUT/before --timeout 15 --settle 2 $DUMP_QOS 2>&1 | grep -E '^OK|^--' || true

echo
echo \"--- 姿勢つきスキャンを配信（$SCANS / stride $STRIDE / $RATE Hz / 帯 ${BAND_MIN} - ${BAND_MAX} m）\"
python3 $REAL/quickstart/publish_posed_pcd.py \
  $SESSION/$SCANS/pcd --stride $STRIDE --rate $RATE --z-offset $Z_OFFSET \
  > /tmp/publish_posed.log 2>&1 &
PUBLISHER=\$!
sleep 6
echo -n '  配信中の octomap_server: '
ps -o %cpu=,rss= -p \$SERVER | tail -1 | awk '{printf \"CPU %s%% / RSS %.0f MB\\n\", \$1, \$2/1024}'

echo
echo '--- during（点群が流れている間）'
python3 $REAL/quickstart/dump_grids.py --layer projected=/projected_map \
  --out $OUT/during --timeout 15 --settle 3 $DUMP_QOS 2>&1 | grep -E '^OK|^--' || true

wait \$PUBLISHER
tail -1 /tmp/publish_posed.log
sleep 3

echo
echo '--- after（配信が止まってから購読）'
python3 $REAL/quickstart/dump_grids.py --layer projected=/projected_map \
  --out $OUT/after --timeout 12 --settle 2 $DUMP_QOS 2>&1 | grep -E '^OK|^--' || true

kill -TERM \$SERVER 2>/dev/null || true
for _ in \$(seq 1 10); do
  if ! kill -0 \$SERVER 2>/dev/null; then break; fi
  sleep 1
done
kill -KILL \$SERVER 2>/dev/null || true
sleep 1
echo
echo \"後片付け: 残った octomap_server \$(pgrep -xc octomap_server_ 2>/dev/null || true) 個\"
" 2>&1 | grep -v "not multicast-capable"

echo
echo "設定: latch=$LATCH / 帯 ${BAND_MIN} - ${BAND_MAX} m / max_range=$MAX_RANGE / hit=$HIT miss=$MISS / 点群 ${RATE} Hz"
"$MAC_REAL/../../.venv/bin/python" "$MAC_REAL/quickstart/report_octomap_stages.py" \
  "$MAC_REAL/$SESSION_REL/map/$STAGE_DIR" --latch "$LATCH" \
  --seed-map "$MAC_REAL/$SESSION_REL/map/seed_projected.yaml"
