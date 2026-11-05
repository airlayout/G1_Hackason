#!/usr/bin/env bash
# 11-04（画像だけの生徒 V3・V4）: 生徒の重みを「最初の区間から」評価して、通り抜けの成功を数える（f3sc の 95.7% と同じ数え方）。
# 扉は学習に使わない種（既定 SEED=999）で作る＝学習に無い扉。生徒の重みの隣の config.yaml（DAgger の exp）を読む。
#
# 使い方（OMEN の ~/work_inoue/doorman6）: bash eval_vis_1104.sh <生徒の last.pt> <出力先> [台数 128]
#   TEACHER_DRIVE=1: 教師に運転させる（配管の検算。f3sc 単独の率に戻るか。[[doorman-dagger-three-silent-traps]]）
#   SEED: 扉の種（既定 999）。PITCH_DEG: 俯角を固める（例 51。既定は学習と同じ 30〜70° の乱択）
set -uo pipefail
CK="$1"; OUT="$2"; N="${3:-128}"; shift $(( $# < 3 ? $# : 3 ))
B="$HOME/work_inoue/doorman6"
BASE="$HOME/work_inoue/doorman"
PY="$HOME/miniforge3/envs/doorman6/bin/python"
EXTRA=(++seed="${SEED:-999}" ++env.config.reset_from_dataset.enabled=False
       "++env.config.max_stage_time=[500,200,200,200,600,400]" ++env.config.max_episode_length_s=60)
if [ "${TEACHER_DRIVE:-0}" = 1 ]; then
  EXTRA+=(++algo.config.enforce_teacher_rollout=True ++algo.config.ratio_teacher_rollout=1.0)
else
  EXTRA+=(++algo.config.enforce_teacher_rollout=False)
fi
[ -n "${PITCH_DEG:-}" ] && EXTRA+=(++domain_rand.camera_pitch_eval_deg="$PITCH_DEG")
mkdir -p "$OUT"
cd "$BASE" || exit 1
WORK="$B" REPO="${REPO:-VIRAL6_vis}" ENV_NAME=doorman6 IGNORE_PAUSE=1 IDLE_GATE=0 MAX_WAIT_MIN="${MAX_WAIT_MIN:-60}" \
  MEM_MAX="${MEM_MAX:-12G}" NEED_MARGIN_MB="${NEED_MARGIN_MB:-256}" \
  ./run_eval_record.sh "$CK" "$OUT" "$N" "$N" "${MINUTES:-25}" \
  ++env.config.enable_staged_reset=False ++algo.config.eval.eval_num_envs_episodes=True "${EXTRA[@]}" "$@"
"$PY" "$BASE/count_success.py" "$OUT" --criterion through --json "$OUT/summary.json" | tee "$OUT/summary.txt"
