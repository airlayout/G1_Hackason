"""純正の歩行コントローラに「1 回分だけ」歩かせる（common/loco.py）。実機日に、1 回でどれだけ動くか・止まるまで何秒かを測る。

    # 送らずに、送るはずの指令だけを見る（既定）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/loco_pulse.py --axis forward --amount 20
    # 実機に送る（有線のラボ PC。G1 は立って歩ける状態・周りに人がいない・リモコンを持った人がそばにいること）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/loco_pulse.py --axis forward --amount 20 --execute
    # 速さと時間を直接（下限・上限の確認は同じ）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/loco_pulse.py --raw 0.3 0 0 0.6 --execute
    # 前後に頭カメラで柱の面を測り、動いた量（前後・横・向き）も記録する（柱の手前 0.6〜1.2 m に正対して立たせて）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/loco_pulse.py --raw 0.3 0 0 0.6 --execute --measure-face

--amount は cm（回る向きは °）。--execute のときは、送る前に Enter を待つ（Ctrl+C でやめる。歩いている途中の Ctrl+C でも StopMove を送る）。
送ったあと、脚 12 関節の角速度を記録し、止まるまでの時間を出す。_local/button_press/loco_pulse/<時刻>/ に
pulse.json（指令・結果）と legs.csv（時刻・脚の角速度の最大）を残す。動いた距離はメジャーで測って pulse.json の measured に書く。
--measure-face なら、送る前と止まったあとに柱の面（common/column_face.py）を測り、面に対する距離・向き・端の変化から
動いた量を出して measured_by_camera に書く（横は柱の端が見えたときだけ）。まとめは real/loco_fit.py。
⚠️ 腕は下ろしておく（このスクリプトは腕を見ない。auto_press などの腕の指令を止めてから使う）。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.camera_cfg import add_camera_args, load_camera_config  # noqa: E402
from common.config import REPO_ROOT, load_config  # noqa: E402
from common.column_face import motion_between  # noqa: E402
from common.loco import AXES, LEG_IDX, LegStillness, LocoError, LocoMover, Pulse  # noqa: E402
from common.robot_model import WAIST_IDX  # noqa: E402


class RecordingStillness(LegStillness):
    """止まったかを見るたびに、脚の角速度の最大を記録する。"""

    def __init__(self, *a: object, **k: object) -> None:
        super().__init__(*a, **k)  # type: ignore[arg-type]
        self.rows: list[tuple[float, float]] = []

    def is_still(self) -> bool:
        st = self.reader.latest()
        if st is not None:
            self.rows.append((self.now(), float(np.abs(np.asarray(st.dq)[list(LEG_IDX)]).max())))
        return super().is_still()


def measure_face(source: object, reader: object, finder: object, tracker: object, max_frames: int = 20) -> object | None:
    """新しいフレームで続けて柱の面を測り、そろったまとめ（ColumnFace）を返す。そろわなければ None。"""
    from common.column_face import FaceNotFound

    tracker.reset()
    for _ in range(max_frames):
        f = source.read_rgbd()
        st = reader.latest()
        if f is None or st is None:
            continue
        i = f.intrinsics
        K = np.array([[i.fx, 0, i.cx], [0, i.fy, i.cy], [0, 0, 1.0]])
        try:
            merged = tracker.add(finder.find(f.depth_m(), K, st.q[list(WAIST_IDX)], st.imu_quat))
        except FaceNotFound as e:
            print(f"[loco_pulse] 柱の面: {e}")
            tracker.reset()
            continue
        if merged is not None:
            return merged
    return None


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    what = p.add_mutually_exclusive_group(required=True)
    what.add_argument("--axis", choices=list(AXES))
    what.add_argument("--raw", type=float, nargs=4, metavar=("VX", "VY", "WZ", "DURATION"))
    p.add_argument("--amount", type=float, help="--axis で動きたい量（cm、回る向きは °）")
    p.add_argument("--execute", action="store_true", help="実機に送る（付けなければ表示だけ）")
    p.add_argument("--network-interface")
    p.add_argument("--measure-face", action="store_true",
                   help="送る前と止まったあとに、頭カメラで柱の面を測り、動いた量を記録する（--execute のときだけ）")
    add_camera_args(p)
    args = p.parse_args()

    cfg = load_config("approach.yaml")["loco"]
    if args.axis:
        if args.amount is None:
            p.error("--axis には --amount が要る")
        amount = np.radians(args.amount) if args.axis.startswith("turn") else args.amount / 100.0
    reader = stillness = backend = None
    if args.execute:
        from common.dds import LowStateReader, init_dds
        from common.loco import SdkLocoBackend

        iface = args.network_interface or load_config("arm.yaml")["network_interface"]
        init_dds(0, iface)
        reader = LowStateReader(0, iface)
        reader.open()
        reader.wait(10.0)
        backend = SdkLocoBackend()
        stillness = RecordingStillness(reader, cfg["still"])
    mover = LocoMover(backend, stillness, cfg, go=args.execute)
    try:
        if args.axis:
            pulse = mover.plan(args.axis, amount)
            if pulse is None:
                step = mover.min_step(args.axis)
                step_s = f"{np.degrees(step):.1f}°" if args.axis.startswith("turn") else f"{step * 100:.1f} cm"
                print(f"[loco_pulse] {args.amount:g} は 1 回で動ける最小（{step_s}）の半分より小さい。送らない")
                return 1
        else:
            pulse = Pulse(*args.raw, axis="raw")
        mover.check(pulse)  # だめなら送る前にここで止まる
    except (ValueError, LocoError) as e:
        print(f"[loco_pulse] ❌ 送らない: {e}")
        return 1
    print(f"[loco_pulse] {pulse.summary()}")
    source = finder = tracker = before = None
    if args.execute and args.measure_face:
        from common.camera_rgbd import RgbdZmqSource
        from common.column_face import FaceFinder, FaceTracker

        face_cfg = load_config("approach.yaml")["face"]
        finder = FaceFinder.from_config(load_config("robot.yaml"), face_cfg)
        tracker = FaceTracker(face_cfg["tracking"])
        cam = load_camera_config(args)["rgbd"]
        source = RgbdZmqSource(cam["server_address"], int(cam["port"]), int(cam["timeout_ms"]))
        source.open()
        before = measure_face(source, reader, finder, tracker)
        print(f"[loco_pulse] 動く前: {before.summary() if before is not None else '柱の面がそろわない（動いた量は測れない）'}")
    if args.execute:
        input("[loco_pulse] 周りに人がいない・リモコンを持った人がそばにいることを確かめて Enter（やめるなら Ctrl+C）: ")
    with mover:
        result = mover.pulse(pulse)
    if not args.execute:
        print("[loco_pulse] 送っていない（実機に送るなら --execute）")
        return 0
    print(f"[loco_pulse] {'止まった' if result.stopped else '❌ 止まらなかった'}（送ってから {result.settle_s:.2f} 秒）"
          f"{'。' + result.note if result.note else ''}")
    by_camera = None
    if source is not None:
        after = measure_face(source, reader, finder, tracker) if result.stopped else None
        source.close()
        if before is not None and after is not None:
            by_camera = motion_between(before, after)
            side = "?" if by_camera["left_cm"] is None else f"{by_camera['left_cm']:+.1f}"
            print(f"[loco_pulse] 頭カメラで測った動き: 前 {by_camera['forward_cm']:+.1f} cm、左 {side} cm、"
                  f"左回り {by_camera['turn_deg']:+.1f}°（柱の端が見えないと横は分からない）")
        else:
            print("[loco_pulse] 頭カメラでは動いた量を測れなかった（柱の面がそろわない）")
    out = REPO_ROOT / "_local" / "button_press" / "loco_pulse" / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    t0 = stillness.rows[0][0] if stillness.rows else 0.0
    (out / "legs.csv").write_text("t_s,max_leg_dq_rad_s\n" + "".join(f"{t - t0:.3f},{v:.4f}\n" for t, v in stillness.rows))
    (out / "pulse.json").write_text(json.dumps({
        "pulse": vars(pulse), "stopped": result.stopped, "settle_s": result.settle_s, "note": result.note,
        "measured": {"forward_cm": None, "left_cm": None, "turn_deg": None},  # メジャーで測って書き足す
        "measured_by_camera": by_camera,
    }, indent=2, ensure_ascii=False))
    print(f"[loco_pulse] 保存: {out}")
    return 0 if result.stopped else 1


if __name__ == "__main__":
    sys.exit(main())
