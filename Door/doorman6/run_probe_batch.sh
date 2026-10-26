#!/usr/bin/env bash
# run_probe.sh を続けて回す。最初に 1 回だけ空きを待ち、以後は SKIP_GATE=1（他の人が GPU を使い始めたら止める）。
# 使い方: run_probe_batch.sh <一覧のファイル>   一覧は 1 行 1 回「名前 probe 台数 [上書き...]」（# と空行は飛ばす）。
#         上書きの中の $(...) は使えないので、直しは「@1,4,5」のように書く（simfix_args.sh の記号。@-x,A,C は除く形）
set -uo pipefail
cd "$(dirname "$0")"
LIST="$1"
source "$HOME/work_inoue/doorman/gate.sh"
MEM="${MEM_MAX:-8G}"; NEED_MB=$(( ${MEM%G} * 1024 + 1536 ))
echo "=== [$(date +%T)] batch: wait machine free once"
IGNORE_PAUSE=1 wait_machine_free || { echo "=== batch: gate_timeout"; exit 1; }
while read -r name probe n rest; do
  [ -z "${name:-}" ] && continue
  case "$name" in \#*) continue ;; esac
  args=()
  for a in $rest; do
    if [ "${a:0:1}" = "@" ]; then
      sel="${a:1}"; sel="${sel//,/ }"
      # shellcheck disable=SC2086
      for o in $(./simfix_args.sh $sel); do args+=("$o"); done
    else
      args+=("$a")
    fi
  done
  SKIP_GATE=1 ./run_probe.sh "$name" "$probe" "$n" "${args[@]}"
  rc=$?
  [ "$rc" = 2 ] && { echo "=== batch: 機械が使われ始めたので止める"; exit 2; }
done < "$LIST"
echo "=== [$(date +%T)] batch done"
