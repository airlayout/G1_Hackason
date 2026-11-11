#!/usr/bin/env bash
# **Mac の上で**動かす。段 5 の実機用のコード（door_run_real.py ほか）と重みを PC2 の ~/door_real/ へ配り、md5 で確かめる。
#
#   bash Door/real/deploy_to_pc2.sh <door_policy_*.npz>    # 配って確かめ、PC2 の Python を調べる
#   bash Door/real/deploy_to_pc2.sh check                  # 配らずに PC2 の Python だけ調べる
#   bash Door/real/deploy_to_pc2.sh <door_vision_*.npz>    # 11-05: 画像の生徒。隣の <名前>_encoder.npz・_encoder.onnx も配る
#   bash Door/real/deploy_to_pc2.sh <s2f.npz> <f3t3.npz> <door_vision_c2.npz>   # 11-11: 段の走らせ役（pc2/run_staged.sh）。npz はいくつでも
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
  [ $# -ge 1 ] || { say "door_policy_*.npz を渡す（11-11: いくつでも。--staged は s2f・f3t3・c2 の 3 つ）"; exit 1; }
  for POLICY in "$@"; do [ -f "$POLICY" ] || { say "無い: $POLICY"; exit 1; }; done
  FILES=("$HERE/door_core.py" "$HERE/door_frame.py" "$HERE/door_run_real.py" "$@"
         "$HERE/target_guard.py"  # 10-26: 腕の積み上げ・送る目標・立つ HOMIE の指令の切り方（学習と同じファイル）
         "$HERE/door_foot_load.py" "$HERE/leg_model_g1_29dof_rev_1_0.npz"  # 10-26: HOLD で両足に体重が乗っているかの確かめ
         # 10-23: ライダーだけの本線（L1）。推定器と、制御の側の受け取り・電文・点群の当て方
         "$HERE/door_lidar_msg.py" "$HERE/door_lidar_source.py" "$HERE/door_lidar_node.py" "$HERE/lidar_door_est.py"
         "$HERE/door_lidar_angle.py" "$HERE/door_lidar_frame.py" "$HERE/door_lidar_geometry.py" "$HERE/lidar_offset_from_tape.py"
         "$HERE/door_stand_guard.py" "$HERE/pc2/release_native.py"  # 10-23 の再発防止（STAND の見張り・純正の制御を SDK で外す）
         "$HERE/door_run_stock.py" "$HERE/door_stock_io.py" "$HERE/odom_relay.py" "$HERE/loco_relay.py" "$HERE/turn_probe.py" "$HERE/pc2/run_stock.sh" "$HERE/pc2/stock_cmd.sh"  # 11-02: 純正の脚の口（odom_relay は 11-03・loco_relay は 11-04）
         # PC2 で起こす・止める・録るスクリプト
         "$HERE/pc2/door_start.sh" "$HERE/pc2/door_stop.sh" "$HERE/pc2/lidar_start.sh" "$HERE/pc2/lidar_stop.sh"
         "$HERE/pc2/run_door_lidar.sh" "$HERE/pc2/run_arms_dry.sh" "$HERE/pc2/rec_take.sh" "$HERE/pc2/rec_stop.sh" "$HERE/pc2/watch_remote.py"
         "$HERE/pc2/rec_follow.sh"  # 11-03: 方策の回の生の点群（door_start.sh が run_stock.sh と一緒に起こす）
         # 11-05: 頭のカラーの画像だけの生徒（カメラ側のプロセス・特徴の受け取り・numpy の計算・起こす台本）
         "$HERE/door_vision_core.py" "$HERE/door_vision_encoder.py" "$HERE/door_vision_source.py" "$HERE/door_vision_node.py"
         "$HERE/pc2/run_vision.sh" "$HERE/pc2/run_staged.sh" "$HERE/pc2/cam_tilt.py"  # cam_tilt: 頭カメラの俯角を D435i の加速度計で（床の AprilTag の代わり）
         "$HERE/pc2/depth_probe.py"  # 11-06: 頭カメラの深度が PC2 で取れるか（計画 D1）
         "$HERE/door_depth_shm.py" "$HERE/pc2/depth_relay.py"  # 11-06: 深度は機体の python3（3.8）の pyrealsense2 の中継で受ける
         # 11-10 手順 L2: FAST-LIO（ボクセル 0.25）の姿勢を扉の座標にして送る口と、起こす・止める台本
         "$HERE/door_fastlio_node.py" "$HERE/fastlio_cfg_1110/fastlio_g1_vox025.yaml" "$HERE/pc2/fastlio_start.sh" "$HERE/pc2/fastlio_stop.sh"
         # 11-10 手順 L4: 段の走らせ役（door_run_stock.py --staged）の部品・手のひらの扉の角度（L2b）・左腕の順運動学と URDF
         "$HERE/door_staged.py" "$HERE/door_run_staged.py" "$HERE/hand_door_angle.py" "$HERE/g1_arm_fk.py"
         "$HERE/../doorman6/usd_fix/urdf_src/g1_29dof_rev_1_0.urdf")
  for ext in _encoder.npz _encoder.onnx _meta.json; do  # 画像の生徒の書き出しなら、画像の網の重みも配る（meta で rgbd かを見る）
    for POLICY in "$@"; do [ -f "${POLICY%.npz}$ext" ] && FILES+=("${POLICY%.npz}$ext"); done
  done
  pc2 "mkdir -p ~/$DEST/logs" || exit 1
  scp -q ${JUMP[@]+"${JUMP[@]}"} -i "$G1_KEY" -o BatchMode=yes -o IdentitiesOnly=yes "${FILES[@]}" "$G1_PC2_USER@$G1_PC2_HOST:$DEST/" || { say "scp 失敗"; exit 1; }
  bad=0
  for f in "${FILES[@]}"; do
    want=$(md5 -q "$f")
    got=$(pc2 "md5sum ~/$DEST/$(basename "$f")" | cut -d' ' -f1)
    if [ "$want" = "$got" ]; then say "md5 一致: $(basename "$f")"; else say "⚠️ md5 不一致: $(basename "$f")"; bad=1; fi
  done
  [ "$bad" = 0 ] || exit 1
  pc2 "chmod +x ~/$DEST/*.sh" || exit 1
fi

# 11-03: 方策の回の生の点群を録れるか（記録の道具・空き容量。足りないと door_start.sh は run_stock.sh を起こさない）
say "生の点群を録れるか: $(pc2 "~/$DEST/rec_follow.sh --check" 2>&1 | tr '\n' ' ')"

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
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient  # 11-02: 純正の脚の口（door_run_stock.py）
    assert hasattr(LocoClient, "GetFsmId") and hasattr(LocoClient, "SetVelocity"), "LocoClient に GetFsmId・SetVelocity が無い"
    print("OK python %s numpy %s sdk %s" % (sys.version.split()[0], numpy.__version__, unitree_sdk2py.__file__))
except Exception as e:
    print("NG %s: %s" % (type(e).__name__, e))
# 11-05: 画像の生徒（door_vision_node.py）: カラーは純正の VideoClient・JPEG の復号は cv2・網は onnxruntime（無ければ numpy）
for name, stmt in (("VideoClient", "from unitree_sdk2py.go2.video.video_client import VideoClient"),
                   ("cv2", "import cv2; v = cv2.__version__"), ("onnxruntime", "import onnxruntime; v = onnxruntime.__version__")):
    try:
        v = ""
        exec(stmt)
        print("  画像: %s OK %s" % (name, v))
    except Exception as e:
        print("  画像: %s 無い（%s: %s）" % (name, type(e).__name__, e))
# 11-06: 深度の中継（depth_relay.py）は /usr/bin/python3 の pyrealsense2 を使う（lerobot の 3.12 には入らない＝glibc 2.38）
try:
    import pyrealsense2 as rs
    print("  深度: pyrealsense2 OK（機器 %d 台）" % len(rs.context().devices))
except Exception as e:
    print("  深度: pyrealsense2 無い（%s: %s）" % (type(e).__name__, e))
EOF
done'
