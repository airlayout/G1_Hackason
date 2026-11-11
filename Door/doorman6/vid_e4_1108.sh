#!/usr/bin/env bash
# 11-08（ユーザー「学習を中断して、残る失敗の型をそれぞれ二つずつ動画にして」）: 生徒 e4 の最終（区切り 10・07:00 の評価 70.3%）を
# 同じ重み・同じ扉（SEED 999・256 台・引数は night_1107.sh の e4_resp と同じ）で評価し直し、選んだ台だけ Isaac Sim の画で動画に書く
# （パッチ 0089 の render_env_ids。カメラは全部の台に置いて描き、書く台だけ絞る）。同じ重み・同じ台数なら評価は決まった結果になる
# （exp_video_1103.sh）ので、07:00 の評価の失敗の台がそのまま写る見込み。照合は per_env.json の段階と終わった理由で。
# 使い方（OMEN の ~/work_inoue/doorman6）: IDS="[0,15,...]" setsid nohup bash vid_e4_1108.sh >> vis_1104/vid_e4_1108.log 2>&1 < /dev/null &
set -uo pipefail
B="$HOME/work_inoue/doorman6"; V="$B/vis_1104"; D="$HOME/work_inoue/doorman"
PY="$HOME/miniforge3/envs/doorman6/bin/python"
CK="${CK:-$B/VIRAL6_vis/logs_rl/vis_e4/wbmanip/door_open_homie_29dof_vision_f3_dagger-lstm_lstm-20261108_063331/last.pt}"
IDS="${IDS:?動画に書く台（例 [0,15]）}"
OUT="${OUT:-$V/vid_1108_e4}"
log() { echo "=== [$(date "+%m-%d %T")] vid_e4: $*"; }
RESP_EVAL=$(grep -A1 '評価の引数' "$D/fit_loco_resp_1107.txt" | tail -1)
case "$RESP_EVAL" in ++env.config.loco_resp.enabled=true*) ;; *) log "応え方の層の値が無い"; exit 1 ;; esac
read -r -a RESP <<< "$RESP_EVAL"
SW=(++algo.config.homie_switch_threshold=0.1 ++env.config.walk_in_hold_threshold=0.1)
VDELAY="++env.config.vision_feat.delay_steps=[2,4]"; NOD0=++env.config.vision_feat.nod_deg=0.0
COFF0="++env.config.vision_feat.color_off_prob=0.0"
N="${N:-256}"  # 台数。256 台に 640×480 の撮影用カメラは RTX の作業領域（AOV）が GPU に載らなかった（11-08 07:26）
VID=(++simulator.config.render_results=True ++simulator.config.render_width="${RENDER_W:-480}" ++simulator.config.render_height="${RENDER_H:-360}"
     "++simulator.config.render_eye=[-2.6,-1.4,1.7]" "++simulator.config.render_target=[-0.2,-0.1,0.9]"
     "++simulator.config.render_env_ids=$IDS" ++env.config.save_rendering_dir="$OUT/video")
[ -f "$CK" ] || { log "重みが無い: $CK"; exit 1; }
log "評価し直す ck=$CK 台数=$N 動画の台=$IDS 大きさ=${RENDER_W:-480}x${RENDER_H:-360} out=$OUT"
MINUTES="${MINUTES:-40}" bash "$B/eval_vis_1104.sh" "$CK" "$OUT" "$N" "$VDELAY" "$NOD0" "$COFF0" "${SW[@]}" "${RESP[@]}" "${VID[@]}" > "$OUT.log" 2>&1
log "評価: $(tail -n 1 "$OUT.log")"
[ -f "$OUT/summary.json" ] || { log "summary.json が無い"; exit 1; }
"$PY" - "$OUT" <<'PYEOF'
import glob, os, re, sys
out = sys.argv[1]
for f in glob.glob(f"{out}/video/*_viewer_*.mp4"):
    i = int(re.search(r"_viewer_(\d+)\.mp4$", f).group(1))
    os.rename(f, f"{out}/video/env{i:03d}.mp4")
PYEOF
"$PY" "$D/term_analysis.py" "$OUT" | tail -n 4
nice -n 15 "$PY" "$D/annotate_0089.py" "$OUT" "${VOUT:-$V/video_1108_e4}" e4
log "終わり"
