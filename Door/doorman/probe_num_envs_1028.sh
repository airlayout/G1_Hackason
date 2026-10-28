#!/usr/bin/env bash
# 10-28: 台数を増やすと学習が速くなるかを短く測る（ユーザー「1,024 で進めて」「GPU メモリをめいいっぱい使ってよい」）。
# 段 C（f3d）と同じ引数・同じ出発点で、台数ごとに PROBE_MIN 分（既定 12）だけ学習し、次を 1 行にまとめる:
#   反復の数・1 秒あたりの歩数（中央値）・1 反復の秒（うち集める秒）・GPU のメモリの山・GPU の使用率・RAM の空きの底・
#   PhysX の器の溢れ（overflow）の行数（溢れると接触が黙って落ちるので、0 でない台数は使わない）
# 重みは logs_rl/door_probe1028 に分けて置く（段 C〜E の重みと混ぜない）。止め札は STOP_DOOR_PROBE。
# 譲る札は段 C〜E と同じ PAUSE_DOOR_F3（gate.sh の既定は映像の生徒の札 PAUSE_DOORMAN_TRAINING で、10-24 からユーザーが置いたまま。
# 渡し忘れると始まらずに待ち続ける。10-28 に一度そうなった）
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./probe_num_envs_1028.sh 1024 2048 >> logs_probe_num_envs_1028.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
BEST="${BEST:-$HOME/work_inoue/doorman/ckpt/f3_best_it53750/last.pt}"
CHAIN="${CHAIN:-chain/1028_0903}"
TASK=door_en1154_g13_shaped_left_weight
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
# 段 C の学習の引数（stage_chain.sh が chain.log に書いた「extra=」。空白区切りに戻した後の形）
EXTRA=$(grep -o ' extra=.*' "$CHAIN/chain.log" | tail -1 | cut -d= -f2-)
[ -n "$EXTRA" ] || { log "段 C の学習の引数が ${CHAIN}/chain.log から取れない"; exit 1; }
[ -f "$BEST" ] || { log "出発点の重みが無い: $BEST"; exit 1; }
log "probe start n=$* ck=$BEST min=${PROBE_MIN:-12} mem_max=${MEM_MAX:-12G}"

summarize() {
  python3 - "$1" "$2" <<'EOF'
import re, statistics, sys
d, n = sys.argv[1], sys.argv[2]
log = open(f"{d}/train.log", errors="replace").read()
comp = [(int(a), float(b)) for a, b in re.findall(r"Computation: (\d+) steps/s \(Collection: ([\d.]+)s", log)]
its = [float(x) for x in re.findall(r"Iteration time: ([\d.]+)s", log)]
comp, its = comp[5:], its[5:]  # 立ち上がりの数回は除く
gpu = [l.split(",") for l in open(f"{d}/gpu.csv") if l.count(",") == 2]
mem_gpu = max((int(m.split()[0]) for _, m, _ in gpu), default=-1)
util = [int(u.split()[0]) for _, _, u in gpu if u.strip()]
avail = [int(l.split(",")[1]) for l in open(f"{d}/mem.csv") if "," in l]
over = sum(1 for l in log.splitlines() if "overflow" in l.lower())
if not comp:
    print(f"n={n} 学習の反復が出ていない（落ちたか起動が間に合わない）: {d} gpu_max={mem_gpu}MiB ram_avail_min={min(avail, default=-1)}MB overflow={over}")
    sys.exit(0)
sps = statistics.median(a for a, _ in comp)
col = statistics.median(b for _, b in comp)
print(f"n={n} iters={len(comp) + 5} steps_per_s={sps:.0f} iter_s={statistics.median(its):.2f} collect_s={col:.2f} "
      f"gpu_max={mem_gpu}MiB gpu_util_mean={statistics.mean(util) if util else -1:.0f}% ram_avail_min={min(avail, default=-1)}MB overflow={over} dir={d}")
EOF
}

for N in "$@"; do
  UNTIL=$(date -d "+${PROBE_MIN:-12} min" +%H:%M)
  log "probe n=$N until=$UNTIL"
  WORK="$HOME/work_inoue/doorman6" REPO=VIRAL6_fix ENV_NAME=doorman6 IDLE_GATE=1 PROJECT=door_probe1028 KIND=probe_601 \
    MEM_MAX="${MEM_MAX:-12G}" NEED_MARGIN_MB=0 EXP=wbmanip/door_open_homie_29dof_state_ppo-lstm \
    PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_F3" STOP_FILE="$HOME/work_inoue/STOP_DOOR_PROBE" RESUME_CKPT="$BEST" \
    ./run_train_segments.sh "$N" "$UNTIL" ++env.config.task.name="$TASK" $EXTRA
  D=$(ls -td bench/probe_601_n"${N}"_* 2>/dev/null | head -1)
  if [ -z "$D" ]; then log "n=$N の出力が無い（学習が始まらなかった）"; continue; fi
  log "result $(summarize "$D" "$N")"
done
log "probe_done"
