"""FK の確認（タスク7、段階3）。lowstate と頭カメラを読むだけで、何も送信しない。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/fk_check.py
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/fk_check.py --overlay

- 左右の指先の位置（FK、pelvis 座標）を 1 秒ごとに表示する。定規で測った実際の位置と比べる
- --overlay: FK で求めた指先の位置を頭カメラの画像に投影して丸を描き、_local/button_press/fk_check/ に保存する。
  **丸が画像の中の実際の中指の先に重なれば、FK（URDF と関節の対応、手先の点）とカメラの取り付け位置の両方が合っている。**
  ずれていれば、その向きと大きさを記録する（一定のずれなら較正で補正できる）
- --ee-offset X Y Z: 指先の点（configs/robot.yaml の end_effector）を、その場で別の値にして試す。
  丸が中指の先に重なる値が見つかったら、robot.yaml に書き写す
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.camera_geometry import HeadCameraTransform  # noqa: E402
from common.config import REPO_ROOT, load_config  # noqa: E402
from common.dds import LowStateReader  # noqa: E402
from common.realday import FingertipFK, project_to_image  # noqa: E402
from common.robot_model import WAIST_IDX  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--overlay", action="store_true", help="指先を頭カメラの画像に投影して保存する")
    p.add_argument("--seconds", type=float, default=30.0, help="続ける秒数（Ctrl+C でも止まる）")
    p.add_argument("--network-interface", help="G1 につないでいる NIC（既定は configs/arm.yaml）")
    p.add_argument("--camera-config", default="camera.yaml", help="カメラの接続先の設定（模擬ロボットは camera_sim.yaml）")
    p.add_argument("--ee-offset", type=float, nargs=3, metavar=("X", "Y", "Z"),
                   help="指先の点を、この値（右手の wrist_yaw_link 基準 [m]。左手は y の符号を逆にする）で試す。"
                        "合っていたら configs/robot.yaml の end_effector に書き写す")
    p.add_argument("--out-dir", type=Path, help="画像などの保存先（既定は _local/button_press/ の下）")
    args = p.parse_args()

    robot_cfg = load_config("robot.yaml")
    if args.ee_offset is not None:
        x, y, z = args.ee_offset
        robot_cfg["end_effector"]["right"]["offset"] = [x, y, z]
        robot_cfg["end_effector"]["left"]["offset"] = [x, -y, z]
    for side in ("right", "left"):
        print(f"[fk_check] 指先の点（{side}）: {robot_cfg['end_effector'][side]['link']} から "
              f"{robot_cfg['end_effector'][side]['offset']} m（中指の先。configs/robot.yaml の end_effector）")
    arm_cfg = load_config("arm.yaml")
    reader = LowStateReader(int(arm_cfg.get("domain_id", 0)), args.network_interface or arm_cfg["network_interface"])
    reader.open()
    reader.wait(10.0)
    fk = FingertipFK(robot_cfg, load_config("press.yaml")["ik"])
    transform = HeadCameraTransform(robot_cfg, load_config("localize.yaml")["calibration"]["offset_pelvis_m"])
    src = None
    out = args.out_dir or REPO_ROOT / "_local" / "button_press" / "fk_check"
    if args.overlay:
        from common.camera_rgbd import RgbdZmqSource

        cam = load_config(args.camera_config)["rgbd"]
        src = RgbdZmqSource(cam["server_address"], int(cam["port"]), int(cam["timeout_ms"]))
        src.open()
        out.mkdir(parents=True, exist_ok=True)
    t_end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < t_end:
            st = reader.latest()
            assert st is not None
            tips = fk.positions(st.q)
            qw = st.q[list(WAIST_IDX)]
            print("[fk_check] " + "  ".join(f"{s}: {np.round(v, 3)}" for s, v in tips.items()) + " m")
            if src is not None:
                f = src.read_rgbd()
                if f is None:
                    print("[fk_check] ⚠️ 深度付きストリームが届かない")
                else:
                    img = f.color_bgr.copy()
                    for side, pos in tips.items():
                        uv = project_to_image(pos, qw, transform, f.intrinsics)
                        if uv is not None:
                            c = (0, 255, 255) if side == "right" else (255, 255, 0)
                            cv2.circle(img, (int(uv[0]), int(uv[1])), 8, c, 2)
                            cv2.putText(img, side, (int(uv[0]) + 10, int(uv[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1)
                    path = out / f"{time.strftime('%Y%m%d_%H%M%S')}.png"
                    cv2.imwrite(str(path), img)
                    print(f"[fk_check] 保存: {path}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        if src is not None:
            src.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
