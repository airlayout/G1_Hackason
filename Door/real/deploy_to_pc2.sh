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
# 10-23: Mac から機体（無線 192.168.0.82）へ直接つながらない日は、OMEN を踏み台にする（10-16・10-21 と同じ。直接だと止まる）
#   G1_PC2_HOST=192.168.0.82 G1_JUMP=ubuntu@100.99.102.70 bash Door/real/deploy_to_pc2.sh <door_policy_*.npz>
JUMP=()
if [ -n "${G1_JUMP:-}" ]; then
  JUMP=(-J "$G1_JUMP")
  pc2() { ssh "${JUMP[@]}" -i "$G1_KEY" -o BatchMode=yes -o ConnectTimeout=15 -o IdentitiesOnly=yes "$G1_PC2_USER@$G1_PC2_HOST" "$@"; }
fi

if ! pc2 true 2>/dev/null; then
  say "PC2（${G1_PC2_USER}@${G1_PC2_HOST}）に鍵で入れない。機体の電源・有線／無線・G1_PC2_HOST を確かめる"
  exit 1
fi
say "PC2 OK: $(pc2 hostname)（${G1_PC2_HOST}）"

if [ "${1:-}" != check ]; then
  POLICY="${1:?door_policy_*.npz を渡す}"
  [ -f "$POLICY" ] || { say "無い: $POLICY"; exit 1; }
  FILES=("$HERE/door_core.py" "$HERE/door_frame.py" "$HERE/door_run_real.py" "$POLICY"
         "$HERE/target_guard.py"  # 10-26: 腕の積み上げ・送る目標・立つ HOMIE の指令の切り方（学習と同じファイル）
         "$HERE/door_foot_load.py" "$HERE/leg_model_g1_29dof_rev_1_0.npz"  # 10-26: HOLD で両足に体重が乗っているかの確かめ
         # 10-23: ライダーだけの本線（L1）。推定器と、制御の側の受け取り・電文・点群の当て方
         "$HERE/door_lidar_msg.py" "$HERE/door_lidar_source.py" "$HERE/door_lidar_node.py" "$HERE/lidar_door_est.py"
         "$HERE/door_lidar_angle.py" "$HERE/door_lidar_frame.py" "$HERE/door_lidar_geometry.py" "$HERE/lidar_offset_from_tape.py"
         "$HERE/door_stand_guard.py" "$HERE/pc2/release_native.py"  # 10-23 の再発防止（STAND の見張り・純正の制御を SDK で外す）
         # PC2 で起こす・止める・録るスクリプト
         "$HERE/pc2/door_start.sh" "$HERE/pc2/door_stop.sh" "$HERE/pc2/lidar_start.sh" "$HERE/pc2/lidar_stop.sh"
         "$HERE/pc2/run_door_lidar.sh" "$HERE/pc2/run_arms_dry.sh" "$HERE/pc2/rec_take.sh" "$HERE/pc2/rec_stop.sh" "$HERE/pc2/watch_remote.py")
  pc2 "mkdir -p ~/$DEST/logs" || exit 1
  scp -q "${JUMP[@]}" -i "$G1_KEY" -o BatchMode=yes -o IdentitiesOnly=yes "${FILES[@]}" "$G1_PC2_USER@$G1_PC2_HOST:$DEST/" || { say "scp 失敗"; exit 1; }
  bad=0
  for f in "${FILES[@]}"; do
    want=$(md5 -q "$f")
    got=$(pc2 "md5sum ~/$DEST/$(basename "$f")" | cut -d' ' -f1)
    if [ "$want" = "$got" ]; then say "md5 一致: $(basename "$f")"; else say "⚠️ md5 不一致: $(basename "$f")"; bad=1; fi
  done
  [ "$bad" = 0 ] || exit 1
  pc2 "chmod +x ~/$DEST/*.sh" || exit 1
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
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_  # ライダーの推定器（door_lidar_node.py）
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    print("OK python %s numpy %s sdk %s" % (sys.version.split()[0], numpy.__version__, unitree_sdk2py.__file__))
except Exception as e:
    print("NG %s: %s" % (type(e).__name__, e))
EOF
done'
