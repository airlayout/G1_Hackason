#!/usr/bin/env bash
# 11-10 手順 L1 (b)（計画書 docs/plan/2026-11-09-g1-door-staged-rgbd.md 9.4 節）: 学習に使っていない 11-09 の 7 回で、
# 段 0 にあたる止まった区間（READY・扉の正面 0.8〜1.0 m）の目の網 c2 の外れを出す。
#   1. R0 と同じ正解付け（real_truth_1110.py・取付は URDF の計算だけ・扉と地図は READY の止まった回で作る）
#   2. 実物の束（real_shard_1110.py）
#   3. c2 で当てる（real_pred_pt_1110.py・CPU。s2f の学習は止めない）。比べに今の実機の e2b も（記録の画像から作り直した特徴）
#   4. 正解と比べる（real_eval_1110.py --pred）
# 巻き尺: 生徒の 4 回（06:42〜07:18）は足先 76 cm・右 12 cm、教師の 3 回（07:43〜07:55）は足先 68 cm・右 17.5 cm
#   （開口の中心が pelvis の左に 0.12・0.175 m）。READY は走らせ役の段 2（monotonic の範囲は door_stock の t から）
# 使い方（OMEN）: bash ~/work_inoue/doorman6/real_ana_1106/l1b_1109.sh   [RUNS="064236 ..."]
set -uo pipefail
PY="$HOME/miniforge3/envs/doorman6/bin/python"
A="$HOME/work_inoue/doorman6/real_ana_1106"
V="$HOME/work_inoue/doorman6/VIRAL6_vis"
D="$HOME/work_inoue/real_data/20261109"
O="$HOME/work_inoue/l1_1110/b1109"
T="$HOME/work_inoue/truth_1110"
RAM_MIN_MB="${RAM_MIN_MB:-2500}"
mkdir -p "$O"
log() { echo "=== [$(date '+%m-%d %T')] l1b: $*"; }

# 記録の名前（ライダー）・画像・走らせ役・READY の範囲（monotonic）・足先・左
runs() {
  cat <<'EOF'
064236 vision_20261109_064243 door_stock_20261109_064247 1155.0:1174.0 0.76 0.12
064954 vision_20261109_065001 door_stock_20261109_065006 1519.6:1526.1 0.76 0.12
071005 vision_20261109_071012 door_stock_20261109_071016 2730.3:2736.8 0.76 0.12
071812 vision_20261109_071819 door_stock_20261109_071823 3217.4:3224.0 0.76 0.12
074320 vision_20261109_074327 door_stock_20261109_074331 4725.2:4731.8 0.68 0.175
074552 vision_20261109_074559 door_stock_20261109_074604 4877.6:4884.1 0.68 0.175
075521 vision_20261109_075528 door_stock_20261109_075532 5446.3:5452.8 0.68 0.175
EOF
}

ram_ok() {
  local a; a=$(free -m | awk '/^Mem:/{print $7}')
  [ "${a:-0}" -ge "$RAM_MIN_MB" ] || { log "RAM の空き ${a} MB < ${RAM_MIN_MB} MB。ここで止める（s2f の学習を守る）"; return 1; }
}

cd "$A" || exit 1
SEL="${RUNS:-}"
shards=()
while read -r name vis stock ready toe left; do
  [ -n "$SEL" ] && ! grep -qw "$name" <<< "$SEL" && continue
  li=$(ls -d "$D"/mapping_runs/*_"$name"_li 2>/dev/null | head -1)
  od=$(ls -d "$D"/mapping_runs/*_"$name"_odom 2>/dev/null | head -1)
  [ -d "$li" ] && [ -d "$od" ] || { log "$name: ライダーか odom の記録が無い"; continue; }
  ram_ok || exit 1
  t0=${ready%%:*}; t1=${ready##*:}
  log "$name: 正解付け（READY $ready・巻き尺 足先 $toe 左 $left）"
  nice -n 15 env OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 "$PY" -u real_truth_1110.py --policy door_vision_e2b.npz \
    --door 0.94,2.05,0.99,0.064 --thickness 0.04 --fit-spot -1 --center fk \
    --vision "$D/door_real/logs/$vis.npz" --stock "$D/door_real/logs/$stock.npz" --lidar "$li" --odom "$od" \
    --spot "$t0:$t1:$toe:$left" --ref-spot 0 --out "$O/truth_$name.npz" < /dev/null > "$O/truth_$name.log" 2>&1
  rc=$?
  grep -E "時計の差|扉と地図|取付|正解が付いた|地点 0" "$O/truth_$name.log" | sed 's/^/    /'
  [ "$rc" -ne 0 ] && { log "$name: 正解付けが失敗（rc=$rc・$O/truth_$name.log）"; tail -3 "$O/truth_$name.log"; continue; }
  nice -n 15 "$PY" -u real_shard_1110.py --truth "$O/truth_$name.npz" --vision "$D/door_real/logs/$vis.npz" \
    --stock "$D/door_real/logs/$stock.npz" --policy door_vision_e2b.npz --out "$O/real_$name.npz" < /dev/null > "$O/shard_$name.log" 2>&1 \
    || { log "$name: 束が作れない（$O/shard_$name.log）"; tail -3 "$O/shard_$name.log"; continue; }
  shards+=("$O/real_$name.npz")
done < <(runs)

[ "${#shards[@]}" -eq 0 ] && { log "束が 1 つも無い"; exit 1; }
ram_ok || exit 1
log "c2 で当てる（${#shards[@]} 本・CPU）"
( cd "$V" && PYTHONPATH=. OMP_NUM_THREADS=4 nice -n 15 "$PY" -u real_pred_pt_1110.py --init "$T/r2b_c2.pt" \
    --real_val "${shards[@]}" --out "$O/c2_realval.npz" 2>&1 | grep -v Warning ) || { log "c2 の当てが失敗"; exit 1; }

while read -r name vis stock ready toe left; do
  [ -f "$O/real_$name.npz" ] || continue
  for net in c2 e2b; do
    if [ "$net" = c2 ]; then
      extra=(--pred "$O/c2_realval.npz" --pred-key "real_$name.npz")
    else
      extra=(--onnx door_vision_e2b_encoder.onnx)
    fi
    nice -n 15 "$PY" -u real_eval_1110.py --truth "$O/truth_$name.npz" --vision "$D/door_real/logs/$vis.npz" \
      --stock "$D/door_real/logs/$stock.npz" --policy door_vision_e2b.npz "${extra[@]}" --out "$O/eval_${net}_$name.npz" \
      < /dev/null > "$O/eval_${net}_$name.log" 2>&1 || log "$name $net: 比べが失敗（$O/eval_${net}_$name.log）"
  done
done < <(runs)
log "終わり（$O）"
