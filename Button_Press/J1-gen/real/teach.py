"""ティーチング: 人が手で動かした腕の姿勢を記録する（タスク7）。lowstate を読むだけで、何も送信しない。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/teach.py --name press_bottle --arm right

⚠️ 腕を手で動かすには、リモコンでダンピング（L2+B）状態にする。ダンピング中は全身が脱力するので、
   必ず座った状態か、吊り下げた状態で行う（開始時に確認して Enter を待つ）。

1. 人が腕を持って、押し込み姿勢（指先が対象に触れる手前）に動かす
2. Enter を押すと、そのときの関節角・腰の角度・FK の指先の位置を configs/taught_poses.yaml に保存する
   （同じ名前は上書き）。q を入力すると終わる

記録した押し込み姿勢は、全体をつなぐスクリプト（タスク6）で IK の初期値に使う。
表示される指先の位置（FK）を、定規で測った実際の位置と比べて FK を確かめる（段階3）。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import load_config  # noqa: E402
from common.dds import LowStateReader  # noqa: E402
from common.realday import TAUGHT_POSES, FingertipFK, arm_q, confirm_support, save_taught_pose  # noqa: E402
from common.robot_model import WAIST_IDX  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--name", required=True, help="姿勢の名前（例: press_bottle）")
    p.add_argument("--arm", choices=["left", "right"], default="right")
    p.add_argument("--note", default="", help="メモ（置き場所など）")
    p.add_argument("--network-interface", help="G1 につないでいる NIC（既定は configs/arm.yaml）")
    args = p.parse_args()

    if not confirm_support():
        print("[teach] 中止した")
        return 1
    arm_cfg = load_config("arm.yaml")
    reader = LowStateReader(int(arm_cfg.get("domain_id", 0)), args.network_interface or arm_cfg["network_interface"])
    reader.open()
    reader.wait(10.0)
    fk = FingertipFK(load_config("robot.yaml"), load_config("press.yaml")["ik"])
    print(f"[teach] 腕を押し込み姿勢に動かして Enter（保存先 {TAUGHT_POSES}、名前 {args.name}）。q で終わる")
    saved = 0
    while True:
        ans = input("[teach] Enter で記録 / q で終わる: ").strip().lower()
        if ans == "q":
            break
        st = reader.latest()
        assert st is not None
        tip = fk.positions(st.q)[args.arm]
        print(f"[teach] 腕 {args.arm} [deg]: {np.round(np.degrees(arm_q(st.q, args.arm)), 1)}")
        print(f"[teach] 腰 [deg]: {np.round(np.degrees(st.q[list(WAIST_IDX)]), 2)}")
        print(f"[teach] 指先（FK、pelvis 座標）: {np.round(tip, 3)} m")
        save_taught_pose(args.name, args.arm, st.q, tip, args.note)
        saved += 1
        print(f"[teach] 保存した（{args.name}。もう一度 Enter で上書き）")
    print(f"[teach] 終了（{saved} 回保存）。実機日が終わったら configs/taught_poses.yaml をコミットして残す")
    return 0 if saved else 1


if __name__ == "__main__":
    sys.exit(main())
