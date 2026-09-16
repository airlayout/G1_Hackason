#!/usr/bin/env bash
# **止める。** 足 → Nav2 → 測位 → 中継 の順（危ない順）に落とす。
#
#   bash quickstart/live/down.sh            # 全部止めて記録を Mac へ回収する
#   bash quickstart/live/down.sh --keep     # 記録は回収するが PC2 側は消さない
#
# ⚠️ **止める順番に意味がある。** 測位を先に落とすと、足が繋がったまま Nav2 が
#    おかしな `/cmd_vel` を出しうる。足を最初に外す。
#
# ⚠️ **Nav2 に `--stop` は無い。** `run_nav2_live.sh --stop` と打つと
#    **2 つ目の Nav2 が起動する**（2026-09-15 に踏んだ）。起動役の
#    プロセスグループに SIGINT を送る。SIGTERM だと内側の ros2 launch が
#    子を孤児にする。
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/_common.sh"

KEEP=0
[ "${1:-}" = "--keep" ] && KEEP=1

# ── 0. 記録を先に回収する（落とす前に）──────────────────────────────
say "0. 記録を Mac へ回収"
DST="$(dirname "$(dirname "$HERE")")/runs/_live/$(date +%Y%m%dT%H%M%S)"
mkdir -p "$DST"
if pc2 'ls $HOME/g1_runs/*.txt >/dev/null 2>&1'; then
    scp -q -i "$G1_KEY" -o IdentitiesOnly=yes \
        "$G1_PC2_USER@$G1_PC2_HOST:\$HOME/g1_runs/*" "$DST/" 2>/dev/null \
        && ok "回収した -> $DST" || warn "回収に失敗した"
    ls -1 "$DST" 2>/dev/null | sed 's/^/     /' >&2
else
    warn "PC2 に記録が無い"
fi

# ── 1. 足（いちばん危ない）──────────────────────────────────────────
say "1. 足を外す"
bash "$HERE/legs.sh" --stop >/dev/null 2>&1 || true
ok "外した"

# ── 2. Nav2 ────────────────────────────────────────────────────────
say "2. Nav2（起動役の PGID に SIGINT。⚠️ --stop は無い）"
pc2 "P=\$(ps -eo pid,args | grep -F \"\$(printf '%s%s' 'run_nav2_liv' 'e.sh')\" \
          | grep -v ' grep ' | awk '{print \$1}' | head -1)
     if [ -n \"\$P\" ]; then
       G=\$(ps -o pgid= -p \$P | tr -d ' ')
       kill -INT -\$G 2>/dev/null
       for i in \$(seq 30); do ps -eo args | grep -q '[b]t_navigator' || break; sleep 1; done
     fi
     echo \"     残り \$(ps -eo args | grep -c '[b]t_navigator') 個\""

# ── 3. 測位 ────────────────────────────────────────────────────────
say "3. 測位（run_fastlio_loc_live.sh --stop は有る）"
pc2 "timeout 60 bash \$HOME/g1_cfg/run_fastlio_loc_live.sh --stop 2>&1 | tail -2 | sed 's/^/     /'"

# ── 4. その他 PC2 ───────────────────────────────────────────────────
say "4. /scan・橋・姿勢ロガー"
pc2_kill "cloud_to_sca" "n.py" KILL   >/dev/null
pc2_kill "log_tf_pos"  "e.py"  TERM   >/dev/null
pc2_kill "foxglove_bridge --ros" "-args" TERM >/dev/null
ok "止めた"

# ── 5. Mac 側 ──────────────────────────────────────────────────────
say "5. コンテナの中継・地図・RViz2"
ctr_kill "rvi" "z2"
ctr_kill "ros_to_foxglo" "ve.py"
ctr_kill "foxglove_to_r" "os.py"
ctr_kill "map_ser" "ver"
ok "止めた"
pkill -f 'ssh -N -L 8765' 2>/dev/null && ok "Foxglove のトンネルも止めた" || true

# ── 6. ロック ──────────────────────────────────────────────────────
if [ "$KEEP" -eq 0 ]; then
    pc2 'rmdir /tmp/pc2_lock 2>/dev/null' && ok "/tmp/pc2_lock を解放した" || true
fi

printf '\n'
say "止めおわり。記録: $DST"
