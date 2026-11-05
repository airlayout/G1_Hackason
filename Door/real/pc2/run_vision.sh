#!/usr/bin/env bash
# 11-05: 頭のカラーの画像だけの生徒を純正の脚で動かす（door_run_stock.py --door-source vision）。door_start.sh で起こす。
# 先にカメラ側（door_vision_node.py: 純正の VideoClient → 画像の網 → 特徴を UDP で 10 Hz）を起こし、特徴が届き始めてから
# 走らせ役に替わる（exec。pid は door_start.sh の logs/door.pid のまま）。カメラ側は --parent-pid で走らせ役が終わると自分も終わり、
# 特徴と縮めた画像を logs/vision_<日時>.npz に、2 秒ごとの元の JPEG を logs/vision_<日時>_jpeg/ に書く。
# 機体は吊って綱を緩め、リモコンで通常運転（500/501）に入れておく。指示は run_stock.sh と同じ ./stock_cmd.sh a → d → stop → r → q
#
#   VPOLICY=<名前>.npz DOOR=<幅,高さ,レバーの高さ,バックセット> ./door_start.sh run_vision.sh
#     例: VPOLICY=door_vision_d4.npz DOOR=0.94,2.05,0.99,0.064 SECS=8 ./door_start.sh run_vision.sh
#   VPOLICY   export_vision_student.py の npz（隣に <名前>_encoder.npz と <名前>_encoder.onnx）
#   PITCH     カメラの俯角 [°]（torso から見て下向き）。既定 auto＝起こす前に立っている間の頭カメラ D435i の加速度計を
#             5 秒読み（cam_tilt.py）、その床からの俯角に合うように走らせ役が起動時の姿勢から決める（--cam-tilt-deg）。
#             11-05 夜に手で合わせた値は 53.4°（カメラの加速度計 53.25°・READY の表示との差から）。数字を渡せばその値
#   NOSEND=1  腕も歩きも送らない（--no-send）。READY で画像から当てた扉の位置を表示するだけ（巻き尺と比べる）
#   SECS      扉の方策を動かす秒（既定 10）
#   BACKEND   画像の網: auto（onnxruntime があればそれ・既定）／numpy／ort。VTHREADS 網の糸（既定 3）
#   VPY       カメラ側の Python（既定 lerobot の 3.12＝onnxruntime がある。無ければ python3）。
#             11-05 実測: python3（3.8・numpy）は網 136〜300 ms・送る 3〜6 回/秒で線に届かず、
#             lerobot の onnxruntime（糸 3）は網 25〜28 ms・10.0 回/秒・受ける 42〜53 枚/秒・撮ってから 39〜56 ms
#   LEGACT・MAX_VX・ARM_TEMP・ARM_TEMP_HOLD・RPC_TIMEOUT・LOCO・ODOM は run_stock.sh と同じ
cd ~/door_real || exit 1
: "${VPOLICY:?画像の生徒の npz を VPOLICY=<名前>.npz で渡す}"
: "${DOOR:?扉の寸法を DOOR=幅,高さ,レバーの高さ,バックセット [m] で渡す（画像の生徒は使わないが走らせ役が要る）}"
ENC="${VPOLICY%.npz}_encoder"
[ -f "$VPOLICY" ] && [ -f "$ENC.npz" ] || { echo "[run] 無い: $VPOLICY か $ENC.npz（deploy_to_pc2.sh で配る）"; exit 1; }
ts=$(date +%Y%m%d_%H%M%S)
VLOG="logs/vision_$ts.log"
PITCH="${PITCH:-auto}"
CAM_TILT=""
if [ "$PITCH" = auto ]; then  # カメラの加速度計で頭の床からの俯角を測る（機体は止まって立っていること）
  TILT_OUT=$(python3 cam_tilt.py 5 2>&1)
  echo "$TILT_OUT"
  CAM_TILT=$(echo "$TILT_OUT" | sed -n 's/.*（床から） \([0-9.]*\)°.*/\1/p' | head -1)
  if [ -z "$CAM_TILT" ] || echo "$TILT_OUT" | grep -q "⚠️"; then
    echo "[run] 頭の俯角を測れない（上の cam_tilt を見る）。止まって立たせて起こし直すか、PITCH=<数字> で渡す"
    exit 1
  fi
fi
VPY="${VPY:-$HOME/miniforge3/envs/lerobot/bin/python}"
[ -x "$VPY" ] || { echo "[run] ⚠️ $VPY が無い: python3（numpy の網は 10 Hz に届かない）で起こす"; VPY=python3; }
OPENBLAS_NUM_THREADS="${VTHREADS:-3}" OMP_NUM_THREADS="${VTHREADS:-3}" setsid nohup "$VPY" -u door_vision_node.py \
  --encoder "$ENC.npz" --onnx "$ENC.onnx" --backend "${BACKEND:-auto}" --threads "${VTHREADS:-3}" --iface eth0 \
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
  echo "[run] カメラ側から特徴が来ない（$VLOG を見る）。走らせ役は起こさない"
  kill "$VPID" 2>/dev/null
  exit 1
fi
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1  # 10-16: BLAS の糸が多いと周期が乱れる
ARGS=(--policy "$VPOLICY" --iface eth0 --door-source vision --door "$DOOR" --run-seconds "${SECS:-10}" --odom "${ODOM:-relay}"
      --leg-actions "${LEGACT:-shadow}" --max-vx "${MAX_VX:-0.5}" --arm-temp-stop "${ARM_TEMP:-75}"
      --rpc-timeout "${RPC_TIMEOUT:-1.0}" --arm-temp-hold "${ARM_TEMP_HOLD:-1.0}" --loco "${LOCO:-relay}"
      --cmd-fifo logs/stock_cmd.fifo)
if [ -n "$CAM_TILT" ]; then ARGS+=(--cam-tilt-deg "$CAM_TILT"); else ARGS+=(--head-pitch-deg "$PITCH"); fi
[ -n "${VEL_GAIN:-}" ] && ARGS+=(--vel-gain "$VEL_GAIN")  # 11-06: 純正の鈍さの補い（例 VEL_GAIN=1.5,2,1 VX_FLOOR=0.15）
[ -n "${VX_FLOOR:-}" ] && ARGS+=(--vx-floor "$VX_FLOOR")
[ -n "${VACTOR:-}" ] && ARGS+=(--vision-actor "$VACTOR")  # 11-06: VACTOR=teacher で扉の値は画像・行動は教師
[ "${NOSEND:-0}" = 1 ] && ARGS+=(--no-send)
exec python3 -u door_run_stock.py "${ARGS[@]}"
