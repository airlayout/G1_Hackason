#!/usr/bin/env bash
# 2026-10-08 23:15: 段 2-3 の延長その 2。等級 1 は 6 回学習して 89/128 で頭打ち（chain/1008_1625・1008_1951）。
# 反復 33,950 を上限 2.5〜54 N·m の扉で評価すると、上限 約 20 N·m まで成功 7〜8 割で平ら、20〜25 N·m で 4%、25 N·m 以上で 0%（records/start_1008_2305）。
# 崖（レバーの位置で約 22〜27 N）が押す力の報酬の頭打ち 20 N のすぐ上にあるので、頭打ちを上げた等級 3 を学習して崖が動くかを見る。
# 等級 3 に合格すれば等級 4・5 へ。終わったら最後の ckpt を全範囲（door_en1154_upto_g5・256 台）で評価する
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK="$HOME/work_inoue/doorman/ckpt/start_1008_2301/last.pt"
PY="$HOME/miniforge3/envs/doorman6/bin/python"
CAP3="++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12"
CAP4="++env.config.push_door_force_max=75,++env.config.push_unlatched_force_max=75,++rewards.reward_scales.push_door_force=0.08"
CAP5="++env.config.push_door_force_max=100,++env.config.push_unlatched_force_max=100,++rewards.reward_scales.push_door_force=0.06"
EVAL_FIRST=1 MEM_MAX=9G NEED_MARGIN_MB=1024 PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B" \
  ./stage_chain.sh "$CK" \
  "g3:door_en1154_g3:3:60:door30:$CAP3" \
  "g4:door_en1154_g4:3:60:door30:$CAP4" \
  "g5:door_en1154_g5:3:60:door30:$CAP5"
L=$(ls -td chain/*/ | head -1)chain.log
LAST=$(grep -o "chain_done .*last_ckpt=[^ ]*" "$L" | tail -1 | grep -o "last_ckpt=[^ ]*" | cut -d= -f2)
[ -f "$LAST" ] || exit 0
echo "=== [$(date '+%m-%d %T')] 全範囲の評価 ckpt=$LAST"
PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" COUNT_ARGS="--criterion door30" EXTRA_ARGS=++env.config.task.name=door_en1154_upto_g5 \
  ./eval_from_start.sh "$LAST" 256 > "${L%chain.log}eval_full_range.log" 2>&1
R=$(grep -o "out=[^ ]*" "${L%chain.log}eval_full_range.log" | tail -1 | cut -d= -f2)
"$PY" door_force_curve.py "$R" --bins 2.5,5,7.5,10,12.5,15,17.5,20,25,30,40,54 --json "$R/force_curve.json"
