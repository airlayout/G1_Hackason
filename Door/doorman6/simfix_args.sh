#!/usr/bin/env bash
# simfix_1026_args.txt から、選んだ直しの Hydra の上書きを 1 行に並べる（重複は最初の 1 つだけ残す）。
# 使い方: simfix_args.sh            → 全部
#         simfix_args.sh 1 5 7      → [1]・[5]・[7] だけ
#         simfix_args.sh -x A C     → [A]・[C] 以外の全部
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FILE="${SIMFIX_ARGS_FILE:-$HERE/simfix_1026_args.txt}"
EXCLUDE=0
if [ "${1:-}" = "-x" ]; then EXCLUDE=1; shift; fi
awk -v sel="$*" -v exclude="$EXCLUDE" '
  BEGIN { n = split(sel, s, " "); for (i = 1; i <= n; i++) want[s[i]] = 1 }
  /^[[:space:]]*#/ || /^[[:space:]]*$/ { next }
  {
    if (match($0, /^\[[^]]*\]/) == 0) { print "記号の無い行: " $0 > "/dev/stderr"; exit 1 }
    tag = substr($0, 2, RLENGTH - 2); rest = substr($0, RLENGTH + 1); sub(/^[[:space:]]+/, "", rest)
    keep = (n == 0) ? 1 : ((tag in want) ? !exclude : exclude)
    if (keep && !(rest in seen)) { seen[rest] = 1; out = out (out == "" ? "" : " ") rest }
  }
  END { print out }
' "$FILE"
