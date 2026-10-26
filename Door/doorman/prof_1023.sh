#!/usr/bin/env bash
# 10-23 項目 2（ユーザー指示）: 映像の生徒の学習（5 周目と同じ設定・16 台）の時間が、どこに使われているかを py-spy で測る。
# OMEN は ptrace_scope=1 で後から取り付けられないので、py-spy から学習を起こす。呼び出しの履歴を 1 秒に PROF_RATE 回取り、
# 学習の 1 回（集める段・学習の段）の中で、描画・物理・方策の計算などの割合を prof_summary.py で数える。
# 描いた画（確かめの記録）も残す（項目 3 の比べに使う）。試し用の写し（REPO、既定 VIRAL6_speed）で回すので、
# 止めている学習の重み（VIRAL6/logs_rl）とは混ざらない。
#
# 使い方（OMEN の ~/work_inoue/doorman6）: TAG=<名前> bash prof_1023.sh [足す hydra の引数...]
#   例: TAG=base bash prof_1023.sh
#       TAG=ri4up bash prof_1023.sh ++simulator.config.sim.render_interval=4 ++simulator.config.cameras.camera_update_period=0.02
# 環境変数: NUM_ENVS（既定 16）・SEC（py-spy で測る秒、起動を含む。既定 480）・TOTAL（学習を回す秒。既定 540）・PROF_RATE（既定 25）
#           CK（出発の重み。台数が 16 でないときは、台ごとの数を作り直した写し tmp/ck_s6va_r4_n<台数>.pt を渡す）
set -uo pipefail
B="$HOME/work_inoue/doorman6"
REPO="${REPO:-VIRAL6_speed}"
N="${NUM_ENVS:-16}"
OUT="$B/tests/prof_${TAG:-x}_$(date +%m%d_%H%M)"
mkdir -p "$OUT" "$B/tmp"
export PATH="$HOME/miniforge3/envs/doorman6/bin:$PATH" TMPDIR="$B/tmp" OMNI_KIT_ACCEPT_EULA=YES HYDRA_FULL_ERROR=1 \
  PYTHONUNBUFFERED=1 XDG_RUNTIME_DIR="/run/user/$(id -u)" PYTHONPATH="$B/$REPO"
source "$HOME/work_inoue/doorman/gate.sh"
export IGNORE_PAUSE=1  # 学習を札で止めている間に測るので、空き待ちで札を無視する（10-23 に付け忘れて 12 分待った）
kill_tree() { local c; for c in $(pgrep -P "$1"); do kill_tree "$c"; done; kill -9 "$1" 2>/dev/null; }
log() { echo "=== [$(date +%T)] $*" | tee -a "$OUT/prof.log"; }
cd "$B/$REPO" || exit 1
TEACHER="$HOME/work_inoue/doorman/ckpt/s24b3_pass_it42200/last.pt"
CK="${CK:-$HOME/work_inoue/doorman/ckpt/s6va_r4/last.pt}"
ARGS=(+exp=wbmanip/door_open_homie_29dof_dagger-lstm ++num_envs="$N" ++teacher_actor_path="$TEACHER"
      ++env.config.task.name=door_en1154_g13_shaped_left_weight
      ++env.config.push_door_force_max=50 ++env.config.push_unlatched_force_max=50 ++rewards.reward_scales.push_door_force=0.12
      ++env.config.door_through_angle_deg=70 ++env.config.door_hold_open_angle_deg=90 ++env.config.mirror_left_doors=True
      ++env.config.push_door_hinge_max_deg=80 ++env.config.door_overopen_start_deg=85 ++rewards.reward_scales.door_overopen=-3.0
      ++simulator.config.cameras.camera_horizontal_aperture=3.567 ++simulator.config.cameras.camera_vertical_aperture=2.006
      ++domain_rand.randomize_camera_pitch=True "++domain_rand.camera_pitch_range_deg=[35.0,70.0]" ++domain_rand.image_grayscale=True
      ++algo.config.num_steps_per_env=32 ++algo.config.actor.backbone.vision_module.module_config_dict.layer_config.trainable=True
      ++algo.config.obj_pred_loss_coef=1.0 ++algo.config.actor.running_mean_std=True ++algo.config.num_learning_epochs=1
      ++algo.config.num_mini_batches="$N" ++env.config.enable_staged_reset=True ++checkpoint="$CK"
      ++domain_rand.camera_pitch_debug_dir="$OUT/cam" ++domain_rand.camera_pitch_debug_every=40
      ++env.config.delta_action_scale=0.3 ++use_wandb=False ++headless=True ++project_name=port6
      ++env.config.reset_from_dataset.motion_file_dir="$HOME/work_inoue/doorman/LAFAN-G1" "$@")
NEED_MB=11000 wait_machine_free || { log "空かない"; exit 1; }
MON="$OUT/gpu.csv"
# 学習の段の山は 1 秒より短い間に立つので 0.25 秒ごと（nvidia-smi を 1 本常駐。行は「時刻, 使用量 MiB, 使用率 %」）
nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader,nounits -lms 250 > "$MON" &
MP=$!
# 学習のログの「Computation」「Iteration time」は、1 回の中の gc.collect 3 か所のうち 2 か所の時間を含まない。
# 速さは実際の経過時間で比べるので、2 秒ごとに「時刻（秒）,学習の回数」を書く（compare_prof.py が傾きを取る）
( while sleep 2; do echo "$(date +%s.%N),$(grep -ac "Learning iteration" "$OUT/train.log" 2>/dev/null)"; done ) > "$OUT/iters.csv" &
IP=$!
trap 'kill "$MP" "$IP" 2>/dev/null' EXIT
log "start repo=$REPO n=$N sec=${SEC:-480} extra=$*"
setsid systemd-run --user --scope --quiet -p MemoryMax=13G -p MemorySwapMax=0 -p CPUWeight=20 \
  "${SIGINT_DEFAULT_EXEC[@]}" py-spy record -r "${PROF_RATE:-25}" --nonblocking --threads -f raw -d "${SEC:-480}" -o "$OUT/prof.txt" -- \
  python "$B/stackdump_run.py" gr00t/rl/train_agent_trl.py "${ARGS[@]}" > "$OUT/train.log" 2>&1 < /dev/null &
SP=$!
sleep "${TOTAL:-540}"
log "止める（$(grep -ac "Learning iteration" "$OUT/train.log") 回学習した・py-spy の記録 $(ls -la "$OUT/prof.txt" 2>/dev/null | awk '{print $5}') バイト）"
kill_tree "$SP"
sleep 5
pgrep -f "^python .*train_agent_trl.*$OUT/cam" > /dev/null && { log "⚠️ 学習が残っている → 止める"; pkill -9 -f "^python .*train_agent_trl.*$OUT/cam"; }
log "done out=$OUT"
