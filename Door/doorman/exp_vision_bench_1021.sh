#!/usr/bin/env bash
# 段 6 (b)（10-21）: 映像の生徒（DoorMan のカメラ付き生徒、29 関節・指なし、door_open_homie_29dof_dagger-lstm）が
# OMEN（RTX 5060 Ti 16 GB・RAM 15 GB）で 16・32・64 台のときに、どれだけの速さで回り、メモリはどこまで使うかを測る。
#
# 10-06 は Isaac Sim 5.1 で描画の初期化で落ちた。10-07 に 6.0.1 で 16 台が 10 分回った（RAM の山 10.6 GB）。32・64 台は未測。
# DoorMan の README は映像の生徒に 64 台を推奨している。
# 扉の条件と教師は状態入力の生徒（exp_s25_1015.sh）と同じ。学習の設定は bench_queue.sh（10-06）の映像の生徒と同じ。
#
# 読み方: 1 分あたりの学習の回数（Learning iteration）と RAM・VRAM の山。64 台が RAM 15 GB に入るか、台数を増やすと速さが伸びるか
set -uo pipefail
cd "$HOME/work_inoue/doorman6" || exit 1
TEACHER="$HOME/work_inoue/doorman/ckpt/s24b3_pass_it42200/last.pt"
TASK=door_en1154_g13_shaped_left_weight
COND=(++env.config.task.name=$TASK
      ++env.config.push_door_force_max=50 ++env.config.push_unlatched_force_max=50 ++rewards.reward_scales.push_door_force=0.12
      ++env.config.door_through_angle_deg=70 ++env.config.door_hold_open_angle_deg=90
      ++env.config.mirror_left_doors=True
      ++env.config.push_door_hinge_max_deg=80 ++env.config.door_overopen_start_deg=85 ++rewards.reward_scales.door_overopen=-3.0)
NEED_MB="${NEED_MB:-11000}"
source "$HOME/work_inoue/doorman/gate.sh"
for N in ${NS:-16 32 64}; do
  echo "=== [$(date +%T)] wait machine free for n=$N"
  wait_machine_free || { echo "=== [$(date +%T)] gate_timeout"; exit 1; }
  MON="$HOME/work_inoue/doorman6/tests/vis29_bench_n${N}_mon.log"
  ( while true; do echo "$(date +%T) $(nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader,nounits | tr -d ' ') $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)"; sleep 2; done ) > "$MON" &
  MP=$!
  echo "=== [$(date +%T)] start n=$N"
  # ⚠️ 10-21 の 32 台は Isaac Sim の起動（headless.rendering.kit を読んだ直後）で固まり、smoke6.sh の SIGINT も効かず 57 分止まった。
  #    STARTUP_MAX 秒たっても学習が 1 回も進まなければ強制的に止めて、1 回だけやり直す
  for attempt in 1 2; do
  STUDENT=1 ./smoke6.sh "vis29_n$N" "${MINUTES:-8}" "${MEM_MAX:-13G}" \
    +exp=wbmanip/door_open_homie_29dof_dagger-lstm ++num_envs="$N" ++teacher_actor_path="$TEACHER" "${COND[@]}" \
    ++algo.config.num_steps_per_env=32 ++algo.config.actor.backbone.vision_module.module_config_dict.layer_config.trainable=True \
    ++algo.config.obj_pred_loss_coef=1.0 ++algo.config.actor.running_mean_std=True ++algo.config.num_learning_epochs=1 \
    ++algo.config.num_mini_batches="$N" ++algo.config.teacher_rollout_ratio=0.3 &
  SP=$!
  sleep 20
  L=$(ls -td "$HOME"/work_inoue/doorman6/tests/vis29_n${N}_* | head -1)/train.log
  started=0
  for _ in $(seq 1 $(( ${STARTUP_MAX:-600} / 10 ))); do
    grep -a -q "Learning iteration" "$L" 2>/dev/null && { started=1; break; }
    kill -0 "$SP" 2>/dev/null || break
    sleep 10
  done
  if [ "$started" = 1 ]; then wait "$SP"; break; fi
  echo "=== [$(date +%T)] n=$N 試行 $attempt: ${STARTUP_MAX:-600} 秒たっても学習が始まらない → 強制的に止める"
  pkill -9 -f "[t]rain_agent_trl.py .*num_envs=$N " ; sleep 5; wait "$SP" 2>/dev/null
  done
  kill "$MP" 2>/dev/null
  echo "=== [$(date +%T)] n=$N done  VRAM の山 $(awk '{split($2,a,","); if (a[1]>m) m=a[1]} END {print m}' "$MON") MiB・空き RAM の底 $(awk 'NR==1{m=$3} {if ($3<m) m=$3} END {print m}' "$MON") MB"
  echo "  学習の回数: $(grep -a -c "Learning iteration" "$L")・最後: $(grep -a "Learning iteration" "$L" | tail -1 | tr -s ' ' | cut -c1-120)"
  grep -a -E "Iteration time|Collection time|steps per second|fps|Total timesteps" "$L" | tail -4 | sed 's/^/  /'
done
echo "=== [$(date +%T)] exp_vision_bench_1021 done"
