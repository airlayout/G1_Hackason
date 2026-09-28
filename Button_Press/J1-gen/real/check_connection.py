"""実機日の段階0: 接続の確認（タスク7）。lowstate を読むだけで、何も送信しない。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_connection.py
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_connection.py --rgbd --seconds 5

表示するもの:
- lowstate の受信の頻度
- mode_machine（5 = g1_29dof_rev_1_0 であること）
- 腕・腰のモータの mode（0 = ゼロトルク。この状態では送信しても動かない。リモコンでダンピングに入れる）
- 腰の角度、左右の指先の位置（FK、pelvis 座標）
- --rgbd を付けると、深度付きストリームも確かめる（probe_rgbd.py と同じ）
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import load_config  # noqa: E402
from common.dds import LowStateReader  # noqa: E402
from common.realday import FingertipFK  # noqa: E402
from common.robot_model import JOINT_NAMES, LEFT_ARM_IDX, RIGHT_ARM_IDX, WAIST_IDX  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--network-interface", help="G1 につないでいる NIC（既定は configs/arm.yaml）")
    p.add_argument("--seconds", type=float, default=3.0, help="lowstate を受信する秒数")
    p.add_argument("--rgbd", action="store_true", help="深度付きストリームも確かめる")
    p.add_argument("--camera-config", default="camera.yaml", help="カメラの接続先の設定（模擬ロボットは camera_sim.yaml）")
    args = p.parse_args()

    arm_cfg = load_config("arm.yaml")
    iface = args.network_interface or arm_cfg["network_interface"]
    from common.dds import peer_label

    print(f"[check] ネットワークの口: {iface or '（未設定）'}、想定する相手: {peer_label(iface) if iface else '?'}")
    reader = LowStateReader(int(arm_cfg.get("domain_id", 0)), iface)
    ok = True
    try:
        reader.open()
        reader.wait(10.0)
    except (TimeoutError, ValueError) as e:
        print(f"[check] ❌ lowstate: {e}")
        return 1
    n0 = reader.count
    time.sleep(args.seconds)
    rate = (reader.count - n0) / args.seconds
    st = reader.latest()
    assert st is not None
    print(f"[check] lowstate: {rate:.0f} Hz")

    expected = int(arm_cfg["expected_mode_machine"])
    mark = "OK" if st.mode_machine == expected else f"❌ 期待値は {expected}"
    ok &= st.mode_machine == expected
    print(f"[check] mode_machine = {st.mode_machine}（{mark}）")

    watch = list(WAIST_IDX) + list(LEFT_ARM_IDX) + list(RIGHT_ARM_IDX)
    off = [JOINT_NAMES[i] for i in watch if st.motor_mode[i] != 1]
    if off:
        ok = False
        print(f"[check] ❌ mode が 1 でないモータ: {', '.join(off)}")
        print("[check]    ゼロトルク（FSM 0）のままだと送信しても動かない。リモコンでダンピング（FSM 1）に入れる")
    else:
        print("[check] 腰・腕のモータの mode: すべて 1（OK）")

    waist = ", ".join(f"{v:+.2f}" for v in np.degrees(st.q[list(WAIST_IDX)]))
    print(f"[check] 腰 yaw / roll / pitch = {waist}°")
    fk = FingertipFK(load_config("robot.yaml"), load_config("press.yaml")["ik"])
    for side, pos in fk.positions(st.q).items():
        print(f"[check] 指先（FK、pelvis 座標）{side}: {np.round(pos, 3)} m")

    if args.rgbd:
        from common.camera_rgbd import RgbdZmqSource

        cam = load_config(args.camera_config)["rgbd"]
        with RgbdZmqSource(cam["server_address"], int(cam["port"]), int(cam["timeout_ms"])) as src:
            f = src.read_rgbd()
        if f is None:
            ok = False
            print("[check] ❌ 深度付きストリームが届かない（configs/camera.yaml の接続先とポート、PC2 のサーバ）")
        else:
            zero = float(np.mean(f.depth == 0)) * 100
            print(f"[check] 深度付きストリーム: {f.color_bgr.shape[1]}x{f.color_bgr.shape[0]}、"
                  f"fx={f.intrinsics.fx:.1f}、深度が 0 の画素 {zero:.0f}%（OK）")
    print(f"[check] {'すべて OK' if ok else '❌ 問題あり（上を見る）'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
