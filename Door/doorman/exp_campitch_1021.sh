#!/usr/bin/env bash
# 段 6 (b)（10-21）: 映像の生徒を、実機の頭カメラに合わせた画像で教師から DAgger で学ぶ（OMEN、~/work_inoue/doorman6/）。
#   - 俯角を回ごと・台ごとに 35〜70° で引く（パッチ 0061。実機の俯角は日によって 32〜68° 動いた）
#   - 赤外線の画角 87°×56°（V2 と同じ aperture 3.567/2.006。元の設定はカラーの 69°×43°）・灰色（パッチ 0062）
#   - 解像度は元の 216×384 のまま（PC2 で推論の速さを測る形と同じ）
# 扉の条件・教師・学習の設定は exp_vision_bench_1021.sh（16 台が上限）と同じ。
#
#   bash exp_campitch_1021.sh smoke    # 4 台・8 分。俯角の記録（画像と CSV）を書く
#   bash exp_campitch_1021.sh dagger   # 16 台（~/work_inoue/doorman6/s6va_num_envs.txt があればその台数）・MINUTES 分（既定 720）。ckpt は 500 回ごと
#   （10-23: 32 台。台数の違う重みから続けるときは、台ごとの数 2 つだけ 0 から数え直す＝パッチ 0065）
#   RESUME_CKPT=<last.pt> で、その重みから続きを学習する（chain_s6va_1021.sh が使う）
#   TRAIN_EXTRA="<hydra の引数 ...>" で学習の引数を足す（空白区切り。10-22: 途中の区間から始める仕組み ++env.config.enable_staged_reset=True）
#   ~/work_inoue/doorman6/s6va_extra_args.txt があれば、その中身（空白か改行区切りの hydra の引数）も起こすたびに足す
#   （動いている鎖の設定を変えずに、次の起動から効かせるため。10-23 の速さの測定で採用: 詰め物を通さない・学習の段の
#    ResNet を bf16・学習前の gc.freeze＝パッチ 0063・0064。32 台と合わせて 88→173 歩/秒。描画の間引きは効かず不採用）
#
# smoke の記録は tests/campitch_smoke_cam_<時刻>/ に書く。読み方: camera_pitch.csv の pitch_measured_deg が pitch_sampled_deg と ±0.5° で一致し、
#   off_x/y/z が camera_pos_offset（0.0576, 0.0525, 0.4299）のまま歩いても変わらなければ、カメラは胴に付いたまま向きだけ変わっている
set -uo pipefail
MODE="${1:?smoke か dagger}"
cd "$HOME/work_inoue/doorman6" || exit 1
TEACHER="$HOME/work_inoue/doorman/ckpt/s24b3_pass_it42200/last.pt"
TASK=door_en1154_g13_shaped_left_weight
COND=(++env.config.task.name=$TASK
      ++env.config.push_door_force_max=50 ++env.config.push_unlatched_force_max=50 ++rewards.reward_scales.push_door_force=0.12
      ++env.config.door_through_angle_deg=70 ++env.config.door_hold_open_angle_deg=90
      ++env.config.mirror_left_doors=True
      ++env.config.push_door_hinge_max_deg=80 ++env.config.door_overopen_start_deg=85 ++rewards.reward_scales.door_overopen=-3.0)
CAM=(++simulator.config.cameras.camera_horizontal_aperture=3.567 ++simulator.config.cameras.camera_vertical_aperture=2.006
     ++domain_rand.randomize_camera_pitch=True "++domain_rand.camera_pitch_range_deg=[35.0,70.0]"
     ++domain_rand.image_grayscale=True)
case "$MODE" in
  smoke) N=4; MIN="${MINUTES:-8}"; NAME=campitch_smoke
         EXTRA=("++domain_rand.camera_pitch_debug_dir=__OUT__/campitch" ++domain_rand.camera_pitch_debug_every=25) ;;
  # 台数は s6va_num_envs.txt があればその数（10-23: 32。無ければ 16）。名前は見張りが探すので台数を替えても campitch_dagger16 のまま
  dagger) N="$(tr -d '[:space:]' < "$HOME/work_inoue/doorman6/s6va_num_envs.txt" 2>/dev/null)"; [[ "$N" =~ ^[0-9]+$ ]] || N=16
          MIN="${MINUTES:-720}"; NAME=campitch_dagger16; EXTRA=(++callbacks.model_save.save_frequency=500) ;;
  *) echo "smoke か dagger"; exit 1 ;;
esac
echo "=== [$(date +%T)] 台数 ${N}（ミニバッチも ${N} 個＝1 個に 1 台分）"
NEED_MB="${NEED_MB:-11000}"
source "$HOME/work_inoue/doorman/gate.sh"
echo "=== [$(date +%T)] wait machine free ($MODE)"
wait_machine_free || { echo "=== [$(date +%T)] gate_timeout"; exit 1; }
TESTS="$HOME/work_inoue/doorman6/tests"
MON="$TESTS/${NAME}_mon.log"
( while true; do echo "$(date +%T) $(nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader,nounits | tr -d ' ') $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)"; sleep 10; done ) > "$MON" &
MP=$!
trap 'kill "$MP" 2>/dev/null' EXIT
# 自分が起こしたプロセスの木だけを止める（OMEN は共用機。名前で pkill すると他の人の train_agent_trl.py も巻き込む）
kill_tree() { local c; for c in $(pgrep -P "$1"); do kill_tree "$c"; done; kill -9 "$1" 2>/dev/null; }
# smoke6.sh の出力先は <名前>_<月日>_<時分>/。⚠️ ${NAME}_* だと俯角の記録（_cam_）と見張りのログ（_mon.log）にも当たり、
# 新しい方を拾って「学習が始まらない」と誤って止めた（10-21）。数字の形で絞り、フォルダだけを見る
run_dirs() { ls -d "$TESTS"/${NAME}_[0-9][0-9][0-9][0-9]_[0-9][0-9][0-9][0-9]/ 2>/dev/null | sort; }
# ⚠️ Isaac Sim は起動（headless.rendering.kit を読んだ直後）で固まることがあり、smoke6.sh の SIGINT も効かない（10-21）。
#    STARTUP_MAX 秒たっても学習が 1 回も進まず、まだ生きていれば固まりとみて止め、1 回だけやり直す。
#    学習が始まる前に落ちたときは固まりではない（設定やコードの誤り）のでやり直さない
started=0
L=""
FILE_EXTRA=""
if [ -f "$HOME/work_inoue/doorman6/s6va_extra_args.txt" ]; then
  FILE_EXTRA="$(tr '\n' ' ' < "$HOME/work_inoue/doorman6/s6va_extra_args.txt")"
  echo "=== [$(date +%T)] s6va_extra_args.txt の引数を足す: $FILE_EXTRA"
fi
for attempt in 1 2; do
  OUTDIR="$TESTS/${NAME}_cam_$(date +%m%d_%H%M%S)"
  ARGS=("${EXTRA[@]/__OUT__/$OUTDIR}")
  before="$(run_dirs)"
  echo "=== [$(date +%T)] start $MODE n=$N ${MIN} 分（試行 ${attempt}）"
  STUDENT=1 ./smoke6.sh "$NAME" "$MIN" "${MEM_MAX:-13G}" \
    +exp=wbmanip/door_open_homie_29dof_dagger-lstm ++num_envs="$N" ++teacher_actor_path="$TEACHER" "${COND[@]}" "${CAM[@]}" \
    ++algo.config.num_steps_per_env=32 ++algo.config.actor.backbone.vision_module.module_config_dict.layer_config.trainable=True \
    ++algo.config.obj_pred_loss_coef=1.0 ++algo.config.actor.running_mean_std=True ++algo.config.num_learning_epochs=1 \
    ++algo.config.num_mini_batches="$N" ++algo.config.teacher_rollout_ratio=0.3 "${ARGS[@]}" \
    ${RESUME_CKPT:+"++checkpoint=$RESUME_CKPT"} ${TRAIN_EXTRA:-} ${FILE_EXTRA} &
  SP=$!
  # 起動の前に無かったフォルダ＝今回の出力先（前の回の train.log を拾わない）
  L=""
  for _ in $(seq 1 60); do
    new="$(comm -13 <(echo "$before") <(run_dirs) | head -1)"
    if [ -n "$new" ]; then L="${new}train.log"; break; fi
    kill -0 "$SP" 2>/dev/null || break
    sleep 2
  done
  if [ -z "$L" ]; then
    echo "=== [$(date +%T)] 試行 ${attempt}: 今回の出力先が 2 分たっても見つからない → 止める"; kill_tree "$SP"; wait "$SP" 2>/dev/null; exit 1
  fi
  for _ in $(seq 1 $(( ${STARTUP_MAX:-600} / 10 ))); do
    if grep -a -q "Learning iteration" "$L" 2>/dev/null; then started=1; break; fi
    kill -0 "$SP" 2>/dev/null || break
    sleep 10
  done
  if [ "$started" = 1 ]; then wait "$SP"; break; fi
  if ! kill -0 "$SP" 2>/dev/null; then
    echo "=== [$(date +%T)] 試行 ${attempt}: 学習が始まる前に落ちた（固まりではないのでやり直さない）"
    grep -aE "Traceback|Error" "$L" | tail -5
    exit 1
  fi
  echo "=== [$(date +%T)] 試行 ${attempt}: ${STARTUP_MAX:-600} 秒たっても学習が始まらない → 自分のプロセスの木だけ強制的に止める"
  kill_tree "$SP"; wait "$SP" 2>/dev/null
done
echo "=== [$(date +%T)] $MODE done  学習の回数: $(grep -a -c "Learning iteration" "$L")  log: $L"
grep -aE "Traceback|Error" "$L" | tail -3
[ "$started" = 1 ] || exit 1
