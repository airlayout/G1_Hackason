#!/usr/bin/env bash
# 11-07（案 A・計画 docs/plan/2026-11-06-g1-door-depth-rl.md の 7.6）: シミュレータの脚（層＋HOMIE）の応え方を測り、
# 実機の純正の応え方（11-05〜06 の 8 回）に合わせる層（パッチ 0095 の env.config.loco_resp）の値を決めて、答え合わせをする。
#   A: 層を切って（0085 の遅れ 0・倍率 1）、掃引と実機の指令の列を流す → HOMIE だけの応え方
#   fit: A と実機の記録から層の値を決める（fit_loco_resp_1107.py → loco_resp_1107.json）
#   B: その層を入れて同じ列を流す → 実機の列で、シミュレータと実機の応え方・動いた量を比べる
#   C: 参考に、昨夜の f3t2 の脚（倍率 0.3〜0.8・遅れ 5〜28 歩）で同じ列を流す（鈍くしすぎていたかを見る）
# 腕は教師 f3t1 が動かす（歩きの指令だけ列で置き換える）。256 台・回は 17 秒・段階の持ち時間で打ち切らない。
# 使い方（OMEN の ~/work_inoue/doorman）: setsid nohup bash run_loco_resp_1107.sh >> logs_loco_resp_1107.log 2>&1 < /dev/null &
#   段を選ぶ: STEPS="A fit B C"（既定）。答え合わせの後で倍率を一律に掛ける: FIT_SCALE=前後,横,旋回・B の名前: B_RUN=B2
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
log() { echo "=== [$(date "+%m-%d %T")] loco_resp: $*"; }
ANA="$HOME/work_inoue/doorman6/real_ana_1106"
PY="$HOME/miniforge3/envs/doorman6/bin/python"
CK="$HOME/work_inoue/doorman6/VIRAL6_fix/logs_rl/door_f3t1/wbmanip/door_open_homie_29dof_state_ppo-lstm_lstm-20261106_101525/last.pt"
REPLAY="$HOME/work_inoue/doorman/loco_replay_1107.npz"
RESP="$HOME/work_inoue/doorman/loco_resp_1107.json"
REAL=$(ls "$HOME"/work_inoue/real_data/20261106/door_real/logs/door_stock_*.npz)
TASK=door_en1154_g13_shaped_left_weight
BASE_ARGS="++env.config.task.name=$TASK ++env.config.reset_from_dataset.enabled=False \
 ++env.config.max_stage_time=[5000,5000,5000,5000,5000,5000] ++env.config.max_episode_length_s=17 \
 ++env.config.shoulder_roll_press_max_nm=6.0 ++algo.config.homie_switch_threshold=0.1 ++env.config.walk_in_hold_threshold=0.1 \
 ++env.config.loco_cmd_replay.path=$REPLAY"
OFF="++env.config.loco_cmd_delay_steps_range=[0,0] ++env.config.loco_cmd_gain_range=[1.0,1.0]"
T2LEGS="++env.config.loco_cmd_delay_steps_range=[5,28] ++env.config.loco_cmd_gain_range=[0.3,0.8]"
grep -qF "_init_loco_resp" "$HOME/work_inoue/doorman6/VIRAL6_fix/gr00t/rl/envs/base_task/homie_base.py" ||
  { log "パッチ 0095 が VIRAL6_fix に当たっていない"; exit 1; }
[ -f "$REPLAY" ] && [ -f "$CK" ] || { log "列か重みが無い"; exit 1; }
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_F3"
# 夜通しの台本（生徒 e3 → 評価 → 書き出し）が終わるまで待つ。e3 の学習と評価の間に GPU が一瞬空くので、そこで始めると重なって溢れる
if [ "${WAIT_NIGHT:-1}" = 1 ]; then
  log "night_1106.sh の終わりを待つ"
  while pgrep -f "[n]ight_1106.sh" > /dev/null; do sleep 30; done
  log "night_1106.sh が終わった: $(tail -n 1 "$HOME/work_inoue/doorman6/vis_1104/night_1106.log" | cut -c1-200)"
fi
# 引数: 名前・脚の引数。記録の場所を REC_<名前> に返す
run() {
  local name="$1" legs="$2" lg rec
  lg="$HOME/work_inoue/doorman/logs_loco_resp_1107_$name.log"
  log "$name を流す legs=$legs"
  EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion through" MAX_WAIT_MIN="${MAX_WAIT_MIN:-120}" MEM_MAX=9G \
    EXTRA_ARGS="$BASE_ARGS $legs" ./eval_from_start.sh "$CK" 256 > "$lg" 2>&1
  rec=$(grep -o 'out=[^ ]*' "$lg" | tail -1 | cut -d= -f2)
  [ -n "$rec" ] && ls "$rec"/states_*.npz > /dev/null 2>&1 || { log "$name の記録が無い（$lg）"; return 1; }
  printf -v "REC_$name" '%s' "$rec"
  log "$name の記録: $rec"
  (cd "$ANA" && "$PY" ana_sim_loco_resp_1107.py "$rec" "$REPLAY") > "$HOME/work_inoue/doorman/ana_loco_resp_1107_$name.txt" 2>&1
  log "$name の解析: ana_loco_resp_1107_$name.txt"
}
for step in ${STEPS:-A fit B C}; do
  case "$step" in
    A) run A "$OFF" || exit 1 ;;
    fit)
      [ -n "${REC_A:-}" ] || REC_A=$(cat "$HOME/work_inoue/doorman/loco_resp_1107_A.rec" 2>/dev/null)
      (cd "$ANA" && "$PY" fit_loco_resp_1107.py --sim "$REC_A" --replay "$REPLAY" --out "$RESP" --scale "${FIT_SCALE:-1,1,1}" $REAL) \
        > "$HOME/work_inoue/doorman/fit_loco_resp_1107.txt" 2>&1 || { log "fit が落ちた"; exit 1; }
      log "fit: $(grep -A1 '評価の引数' "$HOME/work_inoue/doorman/fit_loco_resp_1107.txt" | tail -1 | cut -c1-300)" ;;
    B)
      ARGS=$(grep -A1 '評価の引数' "$HOME/work_inoue/doorman/fit_loco_resp_1107.txt" | tail -1)
      run "${B_RUN:-B}" "$ARGS" || exit 1 ;;
    C) run C "$T2LEGS" || exit 1 ;;
  esac
  [ "$step" = A ] && echo "$REC_A" > "$HOME/work_inoue/doorman/loco_resp_1107_A.rec"
done
log "終わり"
