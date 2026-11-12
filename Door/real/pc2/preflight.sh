#!/usr/bin/env bash
# 11-12 夜（作業ログ 2026-11-12 の 6 章 ㋓・7 章の 3）: 走らせ役を起こす直前の一括の確かめ（読むだけ・機体には何も送らない）。
# 11-12 の機体では、頭の俯角が朝の後に 49°→23° と動いていた・巻き尺の打ち間違い・右足首の揺れで、起動の前後に 8 分止まった。
#
#   ./preflight.sh                                    頭の俯角・動いているもの・FAST-LIO・空き・機体の状態
#   TAPE=0.73,0.72,0.17,0.38 DOOR=0.94,2.05,0.99,0.064 SCRIPT=script_params.json PELVIS_Z=0.73 ./preflight.sh
#                                                     ↑ 巻き尺の値の形・台本の定数の門番（<定数>.ok）も見る
# 終わりに「[preflight] すべて OK」か「[preflight] ⚠️ n 件」
cd ~/door_real || exit 1
n=0
warn() { echo "[preflight] ⚠️ $*"; n=$((n + 1)); }
alive_as() { local pid; pid=$(cat "$1" 2>/dev/null) || return 1; [ -n "$pid" ] && tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -q "$2"; }
# 1. 動いているもの（走らせ役が残っていると、起こす台本が「もう動いている」で止まる）
if alive_as logs/door.pid door_run_stock; then warn "走らせ役がもう動いている（pid $(cat logs/door.pid)。./stock_cmd.sh q で終わらせる）"; fi
if alive_as logs/fastlio_node.pid door_fastlio_node; then echo "[preflight] FAST-LIO の口: 動いている"; else warn "FAST-LIO の口が動いていない（./fastlio_start.sh）"; fi
# 2. 空き
free_gb=$(df -BG --output=avail ~/mapping_runs 2>/dev/null | tail -1 | tr -dc 0-9)
echo "[preflight] 空き ${free_gb:-?} GB（~/mapping_runs）"; [ -n "$free_gb" ] && [ "$free_gb" -lt 20 ] && warn "空きが 20 GB 未満"
# 3. 頭の俯角（網 c2 の学習の範囲 30〜70°）
TILT=$(python3 cam_tilt.py 3 2>&1 | tail -1); echo "$TILT"
deg=$(echo "$TILT" | sed -n 's/.*（床から） \([0-9.]*\)°.*/\1/p')
if [ -z "$deg" ]; then warn "頭の俯角を測れない"; else
  python3 -c "import sys; d=float('$deg'); sys.exit(0 if 30<=d<=70 else 1)" || warn "頭の俯角 ${deg}° が 30〜70° の外（頭を手で戻す）"
fi
# 4. 機体の状態（腕の温度・札・脚の揺れ・傾き）
python3 preflight.py --secs 2 2>&1 | tail -2 | tee /tmp/preflight_robot.txt
grep -q "機体 OK" /tmp/preflight_robot.txt || warn "機体の状態（上の行）"
# 5. 巻き尺の値の形
if [ -n "${TAPE:-}" ]; then
  why=$(python3 -c "import door_staged as d; print(d.tape_check(*[float(v) for v in '$TAPE'.split(',')]))")
  if [ -n "$why" ]; then warn "巻き尺 $TAPE: $why"; else echo "[preflight] 巻き尺 $TAPE: 形は OK"; fi
fi
# 6. 台本の定数の門番
if [ -n "${SCRIPT:-}" ]; then
  : "${DOOR:?SCRIPT を見るには DOOR も渡す}"
  python3 script_gate.py "$SCRIPT" --door "$DOOR" --pelvis-z "${PELVIS_Z:-0.75}" --check | tail -1 | grep -q "合う" \
    && echo "[preflight] 台本の定数 $SCRIPT: 門番を通っている" || warn "台本の定数 $SCRIPT が門番を通っていない（Mac で push_params.sh）"
fi
[ "$n" = 0 ] && echo "[preflight] すべて OK" || echo "[preflight] ⚠️ $n 件"
exit "$n"
