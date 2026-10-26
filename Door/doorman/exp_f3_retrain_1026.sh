#!/usr/bin/env bash
# F3（10-26）: シミュレータと実機の食い違いを直した条件で、状態入力の生徒（45,150）を PPO で仕上げ直す。
# 計画書 docs/plan/2026-10-26-g1-door-stand-foot-load-check.md の「学習し直し」。作り（報酬・扉・学習の引数）は
# exp_s25rl_1015.sh（10-15 に 950 の生徒から仕上げて 45,150 を得た学習。押し切りの減点 −6.0 の版）と同じで、
# 直し（Door/doorman6/simfix_1026_args.txt の全部: A 当たりの形・B 腕の積み上げ〔δ 関節ごと〕・2・1・5・6・7・4・押す・3・C）を足す。
# 作業ツリーは OMEN の ~/work_inoue/doorman6/VIRAL6_fix（VIRAL6 ＋パッチ 0069〜）。映像の生徒が使う VIRAL6 には触れない。
# 30 分の学習 → 256 回の評価（立った姿勢から・時間 2 倍・評価では押さない）を最大 ROUNDS 回。合格の線は 80%（計画の F1）。
# 10-26 の V12: 45,150 をこの条件で評価すると 11.3%（古い条件 87.1%）。
#
# 札: 自分の札は PAUSE_DOOR_F3（置くと次の区切りで止まって譲る）・止め札 STOP_DOOR_F3。
#     映像の生徒の学習の札（PAUSE_DOORMAN_TRAINING、10-24 にユーザーが置いた）には触れない。
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./exp_f3_retrain_1026.sh > logs_f3_retrain_1026.log 2>&1 < /dev/null &
#   続きから: CK=<last.pt> ROUNDS=<回数> を足す
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK="${CK:-$HOME/work_inoue/doorman/ckpt/s25rl_best_it45150_ovr6/last.pt}"
TASK=door_en1154_g13_shaped_left_weight
CAP3="++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12"
RULES="++env.config.door_through_angle_deg=70,++env.config.door_hold_open_angle_deg=90"
MIRROR="++env.config.mirror_left_doors=True"
WINDOW="++env.config.push_door_hinge_max_deg=80,++env.config.door_overopen_start_deg=85,++rewards.reward_scales.door_overopen=${OVR:--6.0}"
# 直しの上書き。stage_chain.sh の段の書き方では「:」が区切り・「,」が引数の区切り・リストの中のカンマは「|」なので、
# 重心の範囲（{x:[..],..}）は鍵ごとの 3 つに分け、値の中のカンマは | にする
FIX=$("$HOME/work_inoue/doorman6/simfix_args.sh" | tr ' ' '\n' | python3 -c '
import sys
out = []
for a in sys.stdin.read().split():
    if a.startswith("++domain_rand.base_com_range="):
        for k in ("x", "y", "z"):
            out.append(f"++domain_rand.base_com_range.{k}=[-0.03|0.03]")
        continue
    k, v = a.split("=", 1)
    out.append(k + "=" + v.replace(",", "|"))
print(",".join(out))')
case "$FIX" in *:*) echo "直しの上書きに「:」が残っている: $FIX"; exit 1 ;; esac
TRAIN="$CAP3,$RULES,$MIRROR,$WINDOW,$FIX"
export EXP=wbmanip/door_open_homie_29dof_state_ppo-lstm
export EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
export MINUTES=30 MEM_MAX="${MEM_MAX:-9G}" NEED_MARGIN_MB=1024 PASS_RATE="${PASS_RATE:-0.80}" EVAL_N=256 EVAL_FIRST=0
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_F3" STOP_FILE="$HOME/work_inoue/STOP_DOOR_F3" CHAIN_REPO=VIRAL6_fix
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
log "exp_f3 start ck=$CK rounds=${ROUNDS:-96} pass_rate=$PASS_RATE overopen=${OVR:--6.0} repo=$CHAIN_REPO"
log "fix=$FIX"
./stage_chain.sh "$CK" "${NAME:-f3}:$TASK:${ROUNDS:-96}:30:through:$TRAIN"
L=$(ls -td chain/*/ | head -1)
log "chain done chain=$L"
cut -f1-6 "$L/results.tsv" | sed "s/^/  [chain] /"
log "exp_f3_done"
