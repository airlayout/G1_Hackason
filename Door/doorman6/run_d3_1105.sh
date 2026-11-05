#!/usr/bin/env bash
# 11-05 午後: 追う段（記憶つきで扉の値を当て直す段。vision_door_student.py の door_tracker_hidden）を足した生徒の DAgger（d3）を
# d2 の重みから回し、止まったら評価する。扉の頭は固め、追う段は本当の扉の値（補助の答えの損失）だけで学ぶ。
# 生徒の隠れ状態を溜める（save_actor_hidden_states）ので、学習の窓（32 歩）より長く覚えることを学べる。
# 18:00 の止め札は stop_at_1800_1105.sh が置く（区切りの境目。pipeline.log の「区切り N seed=」を読むので、出力は pipeline.log へ）。
# 止まったら d3 の最後の重みを評価する（生徒だけ・頷き 4°。d2 は 83.6%・82.4%）＝学習は止めない。
#
# 使い方（OMEN の ~/work_inoue/doorman6）:
#   setsid nohup bash run_d3_1105.sh >> vis_1104/pipeline.log 2>&1 < /dev/null &
set -uo pipefail
B="$HOME/work_inoue/doorman6"
V="$B/vis_1104"
D2="$B/VIRAL6_vis/logs_rl/vis_d2/wbmanip/door_open_homie_29dof_vision_f3_dagger-lstm_lstm-20261105_084344/last.pt"
ENC="$V/pretrain_color_1105_0807.pt"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
[ -f "$D2" ] && [ -f "$ENC" ] || { log "d3: d2 の重みか ENC が無い"; exit 1; }
NOD=++env.config.vision_feat.nod_deg=4.0
TRK="++algo.config.actor.door_tracker_hidden=128 ++algo.config.actor.init_student_from=$D2"
TRK="$TRK ++algo.config.actor.freeze_door_head=True ++algo.config.save_actor_hidden_states=True"
log "d3: 追う段つきの DAgger を始める（d2 から・追う段 128・扉の頭は固める・隠れ状態を溜める）"
ENC="$ENC" NUM_ENVS=256 ROUNDS="${ROUNDS:-12}" ROUND_MIN=30 SEED0=3100 EXTRA_ARGS="$NOD $TRK" bash "$B/exp_vis_1104.sh" d3
CK=$(ls -t "$B"/VIRAL6_vis/logs_rl/vis_d3/*/*/last.pt 2>/dev/null | head -1)
[ -n "$CK" ] || { log "d3: 重みが無い"; exit 1; }
TAG=$(date +%m%d_%H%M)
log "d3: 評価 ck=$CK"
bash "$B/eval_vis_1104.sh" "$CK" "$V/eval_${TAG}_d3" 256 > "$V/eval_${TAG}_d3.log" 2>&1
log "d3: 生徒だけ $(tail -n 1 "$V/eval_${TAG}_d3.log")"
bash "$B/eval_vis_1104.sh" "$CK" "$V/eval_${TAG}_d3_nod4" 256 "$NOD" > "$V/eval_${TAG}_d3_nod4.log" 2>&1
log "d3: 頷き 4° $(tail -n 1 "$V/eval_${TAG}_d3_nod4.log")"
log "d3: 終わり"
