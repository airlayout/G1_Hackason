#!/usr/bin/env bash
# 段 2-4（2026-10-13）: 把手が左の扉（実物の UIS Room 4 の向き）で、観測と行動の鏡写し（パッチ 0038）が効くかを確かめる。
# 方策は反復 34,950（把手が右の扉だけで学習）。学習はしない。変えるのは鏡写しの有無だけ。
# 見込み: 鏡写しなしは 0%（10-13 の door_room4_g13 で +1 の扉は 0/70）。鏡写しが正しければ、把手が右の同じ等級の扉
#   （door30: 等級 1 111/127・等級 3 114/128・等級 4 104/128）に近い数字になる。大きく下回るなら入れ替え表のどこかが違う。
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./exp_mirror_1013.sh > exp_mirror_1013.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK=ckpt/g4_pass_it34950/last.pt
export IDLE_GATE="${IDLE_GATE:-0}"   # 10-13 にユーザーが「今すぐ回す」と判断（OMEN の前に人がいても回す）
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" COUNT_ARGS="--criterion door30"

run() {  # 名前 扉 足す引数
  local name="$1" task="$2" extra="${3:-}"
  echo "=== [$(date '+%m-%d %T')] $name task=$task extra=$extra"
  EXTRA_ARGS="++env.config.task.name=$task $extra" ./eval_from_start.sh "$CK" 128 > "eval_${name}.log" 2>&1
  grep -a "eval_from_start out=\|区間 s 以上\|end eval_record" "eval_${name}.log" | sed "s/^/  [$name] /"
  sleep 61  # 記録のディレクトリは分単位の名前
}

run mirror_off_g1 door_en1154_g1_left
run mirror_on_g1 door_en1154_g1_left "++env.config.mirror_left_doors=True"
run mirror_on_g3 door_en1154_g3_left "++env.config.mirror_left_doors=True"
run mirror_on_g4 door_en1154_g4_left "++env.config.mirror_left_doors=True"
echo "=== [$(date '+%m-%d %T')] exp_mirror_done"
