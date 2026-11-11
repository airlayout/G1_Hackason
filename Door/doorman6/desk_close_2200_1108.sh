#!/usr/bin/env bash
# 11-08（ユーザー了承「22時には必ず帰るので、それまでに帰らなかったり、プロセスが残っていれば、それらをキルして良い」）:
#   22:00 に OMEN の画面のアプリ（Claude のデスクトップ・Chrome・VS Code）が残っていれば閉じ、
#   関門の評価（rl_r1_1108.sh の R4・空き RAM 12.5 GB 待ち）が走れるようにする。
#   team-waza の遠隔操作（実機を動かす）と起動係の waza-quest-launcher.service には触れない（残っていれば記録だけ）
# 使い方（OMEN の ~/work_inoue/doorman6）: ( setsid nohup bash desk_close_2200_1108.sh < /dev/null > /dev/null 2>&1 & )
set -uo pipefail
L="$HOME/work_inoue/doorman6/vis_1104/desk_close_2200_1108.log"
AT="${AT:-22:00}"
log() { echo "=== [$(date "+%m-%d %T")] desk: $*" >> "$L"; }
avail() { awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo; }

T=$(date -d "today $AT" +%s)
log "$AT まで待つ（空き $(avail) MB）"
while [ "$(date +%s)" -lt "$T" ]; do sleep 30; done
log "$AT になった: 空き $(avail) MB・画面のロック $(loginctl show-session 349 -p LockedHint --value 2>/dev/null)"
ps -eo pid,rss,etime,comm --sort=-rss | head -12 >> "$L"
pgrep -f "[w]aza_teleop.py" > /dev/null && log "遠隔操作が残っている（触れない）"

for u in $(systemctl --user list-units --type=scope --no-legend --plain | awk '{print $1}' \
           | grep -E '^app-(gnome-)?(com\.anthropic\.Claude|com\.google\.Chrome|code)'); do
  log "閉じる: $u"
  systemctl --user stop "$u"
done
sleep 10
log "閉じた後の空き $(avail) MB"
log "終わり"
