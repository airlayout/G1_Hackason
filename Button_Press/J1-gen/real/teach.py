"""ティーチング: 人が手で動かした腕の姿勢を記録する（タスク7）。lowstate を読むだけで、何も送信しない。

    # 姿勢を記録する（押し込み姿勢、机の縁より手前で手を上げた経由の姿勢、ボトルに触れた姿勢など）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/teach.py --name press_bottle --arm right
    # 障害物（机）の箱を作る: 中指の先で机の天板の角を何か所か触って記録する
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/teach.py --obstacle table --arm right

⚠️ 腕を手で動かすには、リモコンでダンピング（L2+B）状態にする。ダンピング中は全身が脱力するので、
   必ず座った状態か、吊り下げた状態で行う（開始時に確認して Enter を待つ）。

姿勢（--name）:
1. 人が腕を持って、押し込み姿勢（指先が対象に触れる手前）などに動かす
2. Enter を押すと、そのときの関節角・腰の角度・FK の指先の位置を configs/taught_poses.yaml に保存する
   （同じ名前は上書き）。q を入力すると終わる
記録した押し込み姿勢は、全体をつなぐスクリプト（タスク6）で IK の初期値に使う。
表示される指先の位置（FK）を、定規で測った実際の位置と比べて FK を確かめる（段階3）。

障害物（--obstacle）:
1. 中指の先で、机の天板の角（少なくとも手前の左右の 2 か所。奥の角も届けば触る）を 1 か所ずつ触り、Enter
2. q で終わると、触った点から箱を作り、configs/obstacles.yaml に保存する（同じ名前は置き換える）
   - 手前の縁から奥へ --depth-m（既定 0.6 m）、天板の上面から下へ --below-m（既定 0.8 m）、
     まわりに --margin-m（既定 0.03 m）の余裕を足す
全体をつなぐスクリプトは、configs/press.yaml の obstacles に加えて、この箱も衝突の確認に使う
（腕が箱に近づく・入る経路は、送る前に拒否する）。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import load_config  # noqa: E402
from common.dds import LowStateReader  # noqa: E402
from common.realday import (  # noqa: E402
    OBSTACLES,
    TAUGHT_POSES,
    FingertipFK,
    arm_q,
    box_from_touch_points,
    confirm_support,
    save_obstacle,
    save_taught_pose,
)
from common.robot_model import WAIST_IDX  # noqa: E402


def fmt(v: np.ndarray) -> str:
    return "[" + ", ".join(f"{x:+.3f}" for x in v) + "]"


def teach_pose(args: argparse.Namespace, reader: LowStateReader, fk: FingertipFK) -> int:
    print(f"[teach] 腕を動かして Enter（保存先 {args.poses_file}、名前 {args.name}）。q で終わる")
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
        print(f"[teach] 指先（FK、pelvis 座標）: {fmt(tip)} m")
        save_taught_pose(args.name, args.arm, st.q, tip, args.note, path=args.poses_file)
        saved += 1
        print(f"[teach] 保存した（{args.name}。もう一度 Enter で上書き）")
    print(f"[teach] 終了（{saved} 回保存）。実機日が終わったら {args.poses_file.name} をコミットして残す")
    return 0 if saved else 1


def teach_obstacle(args: argparse.Namespace, reader: LowStateReader, fk: FingertipFK) -> int:
    print(f"[teach] 障害物 {args.obstacle}: 中指の先で机の天板の角を 1 か所ずつ触って Enter"
          "（少なくとも手前の左右の 2 か所）。q で終わる")
    points: list[np.ndarray] = []
    while True:
        ans = input(f"[teach] {len(points) + 1} 点目: 触って Enter / q で終わる: ").strip().lower()
        if ans == "q":
            break
        st = reader.latest()
        assert st is not None
        tip = fk.positions(st.q)[args.arm]
        points.append(tip)
        print(f"[teach] {len(points)} 点目（FK、pelvis 座標）: {fmt(tip)} m")
    if len(points) < 2:
        print("[teach] ❌ 点が 2 つ未満なので、箱を作らない")
        return 1
    box = box_from_touch_points(points, args.margin_m, args.depth_m, args.below_m)
    save_obstacle(args.obstacle, box, points, args.note, path=args.obstacles_file)
    lo = np.array(box["center"]) - np.array(box["half_size"])
    hi = np.array(box["center"]) + np.array(box["half_size"])
    print(f"[teach] 箱 {args.obstacle}: x {lo[0]:+.3f}〜{hi[0]:+.3f}、y {lo[1]:+.3f}〜{hi[1]:+.3f}、"
          f"z {lo[2]:+.3f}〜{hi[2]:+.3f} m（余裕 {args.margin_m * 100:.0f} cm 込み）")
    print(f"[teach] 保存した: {args.obstacles_file}。実機日が終わったらコミットして残す")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    what = p.add_mutually_exclusive_group(required=True)
    what.add_argument("--name", help="記録する姿勢の名前（例: press_bottle）")
    what.add_argument("--obstacle", help="作る障害物の箱の名前（例: table）")
    p.add_argument("--arm", choices=["left", "right"], default="right")
    p.add_argument("--note", default="", help="メモ（置き場所など）")
    p.add_argument("--network-interface", help="G1 につないでいる NIC（既定は configs/arm.yaml）")
    p.add_argument("--poses-file", type=Path, default=TAUGHT_POSES, help="姿勢の保存先（既定は configs/taught_poses.yaml）")
    p.add_argument("--obstacles-file", type=Path, default=OBSTACLES, help="箱の保存先（既定は configs/obstacles.yaml）")
    p.add_argument("--margin-m", type=float, default=0.03, help="箱のまわりの余裕 [m]")
    p.add_argument("--depth-m", type=float, default=0.6, help="手前の縁から奥への長さ [m]")
    p.add_argument("--below-m", type=float, default=0.8, help="天板の上面から下への長さ [m]")
    args = p.parse_args()

    if not confirm_support():
        print("[teach] 中止した")
        return 1
    arm_cfg = load_config("arm.yaml")
    reader = LowStateReader(int(arm_cfg.get("domain_id", 0)), args.network_interface or arm_cfg["network_interface"])
    reader.open()
    reader.wait(10.0)
    fk = FingertipFK(load_config("robot.yaml"), load_config("press.yaml")["ik"])
    return teach_obstacle(args, reader, fk) if args.obstacle else teach_pose(args, reader, fk)


if __name__ == "__main__":
    sys.exit(main())
