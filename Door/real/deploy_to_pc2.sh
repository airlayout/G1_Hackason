#!/usr/bin/env bash
# **Mac の上で**動かす。段 5 の実機用のコード（door_run_real.py ほか）と重みを PC2 の ~/door_real/ へ配り、md5 で確かめる。
#
#   bash Door/real/deploy_to_pc2.sh <door_policy_*.npz>    # 配って確かめ、PC2 の Python を調べる
#   bash Door/real/deploy_to_pc2.sh check                  # 配らずに PC2 の Python だけ調べる
#
# 接続先は Mapping/real/quickstart/live/_common.sh と同じ（既定は有線 192.168.123.164。無線は G1_PC2_HOST=192.168.0.82）
#
# ⚠️ 配るだけで機体は動かさない。door_run_real.py は人が吊りとリモコンを用意してから自分の手で起こす（README.md）
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$HERE/../../Mapping/real/quickstart/live/_common.sh"
DEST="door_real"  # $HOME 相対
say() { echo "[deploy] $*"; }

if ! pc2 true 2>/dev/null; then
  say "PC2（$G1_PC2_USER@$G1_PC2_HOST）に鍵で入れない。機体の電源・有線／無線・G1_PC2_HOST を確かめる"
  exit 1
fi
say "PC2 OK: $(pc2 hostname)（$G1_PC2_HOST）"

if [ "${1:-}" != check ]; then
  POLICY="${1:?door_policy_*.npz を渡す}"
  [ -f "$POLICY" ] || { say "無い: $POLICY"; exit 1; }
  FILES=("$HERE/door_core.py" "$HERE/door_frame.py" "$HERE/door_run_real.py" "$POLICY")
  pc2 "mkdir -p ~/$DEST/logs" || exit 1
  scp -q -i "$G1_KEY" -o BatchMode=yes -o IdentitiesOnly=yes "${FILES[@]}" "$G1_PC2_USER@$G1_PC2_HOST:$DEST/" || { say "scp 失敗"; exit 1; }
  bad=0
  for f in "${FILES[@]}"; do
    want=$(md5 -q "$f")
    got=$(pc2 "md5sum ~/$DEST/$(basename "$f")" | cut -d' ' -f1)
    if [ "$want" = "$got" ]; then say "md5 一致: $(basename "$f")"; else say "⚠️ md5 不一致: $(basename "$f")"; bad=1; fi
  done
  [ "$bad" = 0 ] || exit 1
fi

# PC2 の Python で、SDK・numpy・脚の odom の型がそろうかを見る（読むだけ）
say "PC2 の Python を調べる"
pc2 'for py in /usr/bin/python3 "$HOME/miniforge3/envs/lerobot/bin/python"; do
  [ -x "$py" ] || { echo "  $py: 無い"; continue; }
  "$py" - <<EOF 2>&1 | sed "s|^|  $py: |"
import sys
sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
try:
    import numpy, unitree_sdk2py
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    print("OK python %s numpy %s sdk %s" % (sys.version.split()[0], numpy.__version__, unitree_sdk2py.__file__))
except Exception as e:
    print("NG %s: %s" % (type(e).__name__, e))
EOF
done'
