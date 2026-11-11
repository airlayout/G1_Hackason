"""人がリモコンで柱の手前（約 0.6〜1.5 m）に置いたあと、頭カメラだけで押せる位置まで寄る（common/approach.py）。腕は動かさない。

    # 送らずに、観測と「次にどう動くか」だけを見る（既定。1 回観測して終わる）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/approach.py --target up --button-set wheelchair --detector yolo
    # 実機で寄る（G1 は立って歩ける状態・腕は下ろす・周りに人がいない・リモコンを持った人がそばにいること）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/approach.py --target up --button-set wheelchair --detector yolo --execute
    # 左腕も候補にする（計画が立った腕を出す）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/approach.py --target up --button-set wheelchair --arms right left --execute

押せる位置に着いたら（ready）、出てきた腕で real/auto_press.py を実行する（このスクリプトは押さない）。
--execute のときは、歩き出す前に Enter を待つ。歩いている途中の Ctrl+C でも StopMove を送る。
結果（観測・判断・1 回ごとの動き）は _local/button_press/approach/<時刻>/result.json に残す。
設定は configs/approach.yaml（face・loco・approach）と elevator.yaml（押す組・勧める立つ位置・▲▼ の見つけ方）。
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.approach import Approacher, Frame, HeadPerception, PlanFeasibility  # noqa: E402
from common.camera_cfg import add_camera_args, load_camera_config  # noqa: E402
from common.column_face import FaceFinder  # noqa: E402
from common.config import REPO_ROOT, load_config  # noqa: E402
from common.elevator_buttons import ButtonFinder  # noqa: E402
from common.loco import LegStillness, LocoMover  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--target", choices=["up", "down"], required=True, help="押すボタン（計画を確かめるのに使う）")
    p.add_argument("--arms", nargs="+", choices=["right", "left"], help="計画を試す腕（既定は approach.yaml）")
    p.add_argument("--button-set", choices=["general", "wheelchair"], help="押す組（既定は elevator.yaml）")
    p.add_argument("--detector", choices=["classic", "yolo"], help="▲▼ の見つけ方（既定は elevator.yaml）")
    p.add_argument("--execute", action="store_true", help="歩く（付けなければ 1 回観測して、次の動きを表示するだけ）")
    p.add_argument("--network-interface", help="G1 につないでいる NIC（既定は configs/arm.yaml）")
    p.add_argument("--no-lowstate", action="store_true",
                   help="lowstate を読まない（--execute なしのときだけ。腰 0°・骨盤は水平・腕は下ろしたままと仮定）")
    add_camera_args(p)
    args = p.parse_args()
    if args.execute and args.no_lowstate:
        p.error("--execute には lowstate が要る（脚が止まったかを見る）")

    cfg = load_config("approach.yaml")
    app = copy.deepcopy(cfg["approach"])
    if args.arms:
        app["arms"] = args.arms
    elevator = load_config("elevator.yaml")
    if args.button_set:
        elevator["geometry"]["button_set"] = args.button_set
    if args.detector:
        elevator["detector"] = args.detector
    robot_cfg = load_config("robot.yaml")

    reader = None
    if not args.no_lowstate:
        from common.dds import LowStateReader, init_dds

        iface = args.network_interface or load_config("arm.yaml")["network_interface"]
        init_dds(0, iface)
        reader = LowStateReader(0, iface)
        reader.open()
        reader.wait(10.0)
    else:
        print("[approach] ⚠️ lowstate を読まない: 腰は 0°、骨盤は水平、腕は下ろしたままと仮定する")

    from common.camera_rgbd import RgbdZmqSource

    cam = load_camera_config(args)["rgbd"]
    source = RgbdZmqSource(cam["server_address"], int(cam["port"]), int(cam["timeout_ms"]))

    def get_frame() -> Frame | None:
        f = source.read_rgbd()
        if f is None:
            return None
        st = reader.latest() if reader is not None else None
        i = f.intrinsics
        K = np.array([[i.fx, 0, i.cx], [0, i.fy, i.cy], [0, 0, 1.0]])
        q = st.q.copy() if st is not None else np.zeros(29)
        return Frame(f.color_bgr[:, :, ::-1].copy(), f.depth_m(), K, q, st.imu_quat if st is not None else None)

    perception = HeadPerception(get_frame, ButtonFinder.from_config(robot_cfg, elevator),
                                FaceFinder.from_config(robot_cfg, cfg["face"]), elevator, cfg["face"], app)
    feasibility = PlanFeasibility.from_config(robot_cfg, load_config("press.yaml"), load_config("arm.yaml"), elevator,
                                              list(app["arms"]), args.target)
    backend = stillness = None
    if args.execute:
        from common.loco import SdkLocoBackend

        backend = SdkLocoBackend()
        stillness = LegStillness(reader, cfg["loco"]["still"])
    mover = LocoMover(backend, stillness, cfg["loco"], go=args.execute)
    print(f"[approach] 押す組: {elevator['geometry'].get('button_set', 'general')}、見つけ方: {elevator['detector']}、"
          f"腕の候補: {app['arms']}、{'歩く' if args.execute else '歩かない（1 回観測するだけ）'}")
    if args.execute:
        input("[approach] 腕を下ろした・周りに人がいない・リモコンを持った人がそばにいることを確かめて Enter（やめるなら Ctrl+C）: ")

    with source:
        result = Approacher(perception, feasibility, mover, app).run()

    if result.outcome == "ready":
        print(f"[approach] 次: real/auto_press.py --target {args.target} --arm {result.arm}"
              + (f" --button-set {args.button_set}" if args.button_set else "")
              + (f" --detector {args.detector}" if args.detector else ""))
    out = REPO_ROOT / "_local" / "button_press" / "approach" / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    path = out / "result.json"
    path.write_text(json.dumps({"args": vars(args), "outcome": result.outcome, "reason": result.reason, "arm": result.arm,
                                "steps": result.steps}, indent=2, ensure_ascii=False, default=float))
    print(f"[approach] 保存: {path}")
    return 0 if result.outcome in ("ready", "dry_run") else 1


if __name__ == "__main__":
    sys.exit(main())
