#!/usr/bin/env bash
# 11-11 手順 L7（計画書 docs/plan/2026-11-09-g1-door-staged-rgbd.md 9.4・9.16）: 段に分けた走らせ役（door_run_stock.py --staged）を
# 純正の脚で動かす。door_start.sh で起こす。先に ./fastlio_start.sh（FAST-LIO と、その姿勢を扉の座標で送る口）を起こしておく。
# ここでは深度の中継（depth_relay.py）と目の網 c2（door_vision_node.py・特徴を UDP で 10 Hz）を起こし、特徴が届き始めてから
# 走らせ役に替わる（exec。pid は door_start.sh の logs/door.pid のまま）。
# 機体は吊って綱を緩め、リモコンで通常運転（500/501）に入れ、扉の正面 0.8〜1.3 m・横 ±0.3 m・向き ±10° に立たせておく。
# 指示: ./stock_cmd.sh a（腕を借りる。--staged では腕は下ろしたまま）→ d（段 0 から）→ stop（立ったまま止まる）→ r（腕を返す）→ q
#   段 0 網で 2 回測る（間にまっすぐ 0.2 m）→ 段 1 構えの位置へ（先に −45° 回る → 斜めに寄る）→ 段 2 腕を構えて s2f → 3° で f3t3
#
#   DOOR=<幅,高さ,レバーの高さ,バックセット> ./door_start.sh run_staged.sh
#     例: DOOR=0.94,2.05,0.99,0.064 PELVIS_Z=0.75 STOP_DEG=3 ./door_start.sh run_staged.sh   （段 2 の 3° で止める日）
#         DOOR=0.94,2.05,0.99,0.064 NOSEND=1 ./door_start.sh run_staged.sh                     （送らずに段 0 の値だけ見る）
#   S2F       腕だけの方策の npz（既定 door_policy_s2f_it79500.npz）
#   F3T3      引き継ぐ教師の npz（既定 door_policy_f3t3.npz）
#   NET       段 0 の目の網の npz（既定 door_vision_c2.npz。隣に _encoder.onnx と _meta.json）
#   PELVIS_Z  立った pelvis の床からの高さ [m]（既定 0.75 は置いた値。巻き尺で測って渡す。レバーを下げたかの判定の高さに効く）
#   SECS      DOOR の上限 [s]（既定 180）
#   STOP_DEG  扉（手のひらから出した角度）がこの角度 [°] まで開いたら立ったまま止まる（無ければ止めない）
#   NET_DX    段 0 の網の前後の値に足す補正 [m]（11-11: 網は巻き尺より扉を 4〜5 cm 遠くに出した → 0.045）
#   PRESS_R   段 2 の引き金の水平の線 [m]（既定 0.16＝腕の力を使わない時）
#   STAGE2_S  段 2 の腕だけの方策をこの秒で止める（既定 15。11-11 は扉が 2.2 秒で開いたので 4.5 で試す）
#   NO_RELEASE=1  段 2 の間は脚を放さない（11-11 の切り分け）
#   TAPE      段 0 を巻き尺で: 左つま先→板,右つま先→板,左つま先←把手の側の縁,右つま先←縁 [m]（例 0.65,0.64,0.55,0.75）
#   STOP_STAGE この段に入ったら立ったまま止まる（1＝段 0 が終わった所・2＝構えの位置に入った所・3＝f3t3 へ引き継ぐ所）
#   NOSEND=1  腕も歩きも送らない（--no-send）。d を打つと段 0 の網の値・FAST-LIO の口の値が記録と表示に出る（機体は動かない）
#   PITCH     カメラの俯角 [°]。既定 auto＝起こす前にカメラの加速度計で測る（cam_tilt.py。止まって立っていること）
#   MAX_VX・ARM_TEMP・ARM_TEMP_HOLD・RPC_TIMEOUT・LOCO・ODOM・VPY・VTHREADS は run_vision.sh と同じ
cd ~/door_real || exit 1
: "${DOOR:?扉の寸法を DOOR=幅,高さ,レバーの高さ,バックセット [m] で渡す（巻き尺）}"
S2F="${S2F:-door_policy_s2f_it79500.npz}"
F3T3="${F3T3:-door_policy_f3t3.npz}"
NET="${NET:-door_vision_c2.npz}"
ENC="${NET%.npz}_encoder.onnx"
for f in "$S2F" "$F3T3" "$NET" "$ENC" g1_29dof_rev_1_0.urdf; do
  [ -f "$f" ] || { echo "[run] 無い: $f（Mac から deploy_to_pc2.sh で配る）"; exit 1; }
done
# FAST-LIO の口が動いていること（REF の口 127.0.0.1:47322 を持っている）
pid=$(cat logs/fastlio_node.pid 2>/dev/null)
if [ -z "$pid" ] || ! tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -q door_fastlio_node; then
  echo "[run] FAST-LIO の口が動いていない（先に ./fastlio_start.sh）"; exit 1
fi
ts=$(date +%Y%m%d_%H%M%S)
PITCH="${PITCH:-auto}"
CAM_TILT=""
if [ "$PITCH" = auto ]; then
  TILT_OUT=$(python3 cam_tilt.py 5 2>&1)
  echo "$TILT_OUT"
  CAM_TILT=$(echo "$TILT_OUT" | sed -n 's/.*（床から） \([0-9.]*\)°.*/\1/p' | head -1)
  if [ -z "$CAM_TILT" ] || echo "$TILT_OUT" | grep -q "⚠️"; then
    echo "[run] 頭の俯角を測れない（上の cam_tilt を見る）。止まって立たせて起こし直すか、PITCH=<数字> で渡す"
    exit 1
  fi
fi
VPY="${VPY:-$HOME/miniforge3/envs/lerobot/bin/python}"
[ -x "$VPY" ] || { echo "[run] ⚠️ $VPY が無い: python3 で起こす（網が 10 Hz に届かない恐れ）"; VPY=python3; }
# 深度の中継（機体の python3＝3.8 の pyrealsense2 が /dev/shm に置く。3.12 には入らない）
DLOG="logs/depth_$ts.log"
setsid nohup python3 -u depth_relay.py --parent-pid $$ > "$DLOG" 2>&1 < /dev/null &
DPID=$!
for _ in $(seq 30); do
  grep -q "最初の深度を置いた" "$DLOG" 2>/dev/null && break
  kill -0 "$DPID" 2>/dev/null || break
  sleep 0.5
done
cat "$DLOG"
grep -q "最初の深度を置いた" "$DLOG" || { echo "[run] 深度の中継から枚が来ない（$DLOG を見る）"; kill "$DPID" 2>/dev/null; exit 1; }
# 目の網 c2（カラー＋深度 → 特徴 128）
VLOG="logs/vision_$ts.log"
OPENBLAS_NUM_THREADS="${VTHREADS:-3}" OMP_NUM_THREADS="${VTHREADS:-3}" setsid nohup "$VPY" -u door_vision_node.py \
  --rgbd-onnx "$ENC" --depth shm --threads "${VTHREADS:-3}" --iface eth0 \
  --parent-pid $$ --log-dir logs > "$VLOG" 2>&1 < /dev/null &
VPID=$!
ln -sf "vision_$ts.log" logs/vision_latest.log
for _ in $(seq 60); do
  grep -q "最初の特徴を送った" "$VLOG" 2>/dev/null && break
  kill -0 "$VPID" 2>/dev/null || break
  sleep 0.5
done
cat "$VLOG"
if ! grep -q "最初の特徴を送った" "$VLOG"; then
  echo "[run] 網から特徴が来ない（$VLOG を見る）。走らせ役は起こさない"
  kill "$VPID" "$DPID" 2>/dev/null
  exit 1
fi
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1  # 10-16: BLAS の糸が多いと周期が乱れる
ARGS=(--staged --policy "$S2F" --policy-f3t3 "$F3T3" --net "$NET" --iface eth0 --door-source lidar --lidar-mode track
      --door "$DOOR" --run-seconds "${SECS:-180}" --odom "${ODOM:-relay}" --max-vx "${MAX_VX:-0.5}" --arm-temp-stop "${ARM_TEMP:-75}"
      --rpc-timeout "${RPC_TIMEOUT:-1.0}" --arm-temp-hold "${ARM_TEMP_HOLD:-1.0}" --loco "${LOCO:-relay}"
      --cmd-fifo logs/stock_cmd.fifo)
if [ -n "$CAM_TILT" ]; then ARGS+=(--cam-tilt-deg "$CAM_TILT"); else ARGS+=(--head-pitch-deg "$PITCH"); fi
[ -n "${PELVIS_Z:-}" ] && ARGS+=(--pelvis-z "$PELVIS_Z")
[ -n "${STOP_DEG:-}" ] && ARGS+=(--stop-door-deg "$STOP_DEG")
[ -n "${STOP_STAGE:-}" ] && ARGS+=(--staged-stop-stage "$STOP_STAGE")
[ -n "${NET_DX:-}" ] && ARGS+=(--net-dx "$NET_DX")
[ -n "${PRESS_R:-}" ] && ARGS+=(--press-radius "$PRESS_R")
[ -n "${STAGE2_S:-}" ] && ARGS+=(--stage2-max-s "$STAGE2_S")
[ "${NO_RELEASE:-0}" = 1 ] && ARGS+=(--no-leg-release)
[ -n "${TAPE:-}" ] && ARGS+=(--stage0-tape "$TAPE")
[ "${NOSEND:-0}" = 1 ] && ARGS+=(--no-send)
exec python3 -u door_run_stock.py "${ARGS[@]}"
