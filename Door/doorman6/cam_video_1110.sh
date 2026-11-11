#!/usr/bin/env bash
# 11-10（ユーザー「腕を乗せる際に、RGB・Depth がどうやって撮影しているかを含めて動画を見たい」）:
#   腕だけの段の方策（s2f）を、頭のカメラ（画像の生徒と同じ取付: torso_link・俯角 51°・D435 のカラーの画角 69°×42°・192×108・
#   カラーと深度）を有効にして 8 台だけ回し、外から見た動画（render_results）と、網に入れる枚（10 Hz）の記録（vision_dump:
#   カラー 192×108・深度 54×96・扉の頭への手掛かり・扉の値の正解・段）を書く。学習はしない。GPU を使うので学習を止めてから回す。
#   そのあと cam_netpass_1110.py が、記録の枚を RGBD の網（pretrain_rgbd_1106_1117.pt の encoder＋head）に通して扉の値の当てを書く
# 使い方（OMEN の ~/work_inoue/doorman）: ARM_CK=<s2f の last.pt> setsid nohup bash cam_video_1110.sh >> logs_cam_video_1110.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
PY="$HOME/miniforge3/envs/doorman6/bin/python"
log() { echo "=== [$(date "+%m-%d %T")] cam_video: $*"; }
ARM_CK="${ARM_CK:?ARM_CK=<腕の方策の last.pt>}"
SEED="${SEED:-999}"
N="${N:-8}"
DUMP="$HOME/work_inoue/doorman/cam_dump_1110_$(date +%H%M)"
BASE="++env.config.task.name=door_en1154_g13_shaped_left_weight ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++seed=$SEED"
CAM="++simulator.config.cameras.enable_cameras=True ++simulator.config.cameras.camera_types=[{rgb:true},{depth:true}]"
CAM="$CAM ++simulator.config.cameras.camera_resolutions=[108,192] ++simulator.config.cameras.camera_attached_link=torso_link"
CAM="$CAM ++simulator.config.cameras.camera_pos_offset=[0.0576,0.0525,0.4299] ++simulator.config.cameras.camera_rot_offset=[0.90244,0.0,0.43081,0.0]"
VDUMP="++env.config.vision_dump.dir=$DUMP ++env.config.vision_dump.depth=True ++env.config.vision_dump.keep_frac=1.0 ++env.config.vision_dump.shard_size=8"  # 8 枚ごとに書く（4,096 だと短い評価では一度も書かれない・11-10 に踏んだ）
[ -f "$ARM_CK" ] || { log "重みが無い: $ARM_CK"; exit 1; }
log "始め arm=$ARM_CK seed=$SEED n=$N dump=$DUMP"
VIDEO=1 RENDER_W=960 RENDER_H=720 EVAL_REPO=VIRAL6_vis COUNT_ARGS="--criterion arm3" MAX_WAIT_MIN=60 \
  EXTRA_ARGS="$BASE $CAM $VDUMP" ./eval_from_start.sh "$ARM_CK" "$N" > logs_cam_video_1110_run.log 2>&1
REC=$(grep -o 'out=[^ ]*' logs_cam_video_1110_run.log | tail -1 | cut -d= -f2)
log "撮った record=$REC videos=$(ls "$REC"/video 2>/dev/null | tr '\n' ' ') dump=$(ls "$DUMP" 2>/dev/null | tr '\n' ' ')"
cd "$HOME/work_inoue/doorman6/VIRAL6_vis" && PYTHONPATH=. "$PY" "$HOME/work_inoue/doorman/cam_netpass_1110.py" "$DUMP" \
  "$HOME/work_inoue/doorman6/vis_1104/pretrain_rgbd_1106_1117.pt" && log "網に通した: $DUMP/netpass.npz"
log "終わり REC=$REC DUMP=$DUMP"
