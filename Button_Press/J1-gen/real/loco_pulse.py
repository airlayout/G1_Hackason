"""純正の歩行コントローラに「1 回分だけ」歩かせる（common/loco.py）。実機日に、1 回でどれだけ動くか・止まるまで何秒かを測る。

    # 送らずに、送るはずの指令だけを見る（既定）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/loco_pulse.py --axis forward --amount 20
    # 実機に送る（有線のラボ PC。G1 は立って歩ける状態・周りに人がいない・リモコンを持った人がそばにいること）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/loco_pulse.py --axis forward --amount 20 --execute
    # 速さと時間を直接（下限・上限の確認は同じ）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/loco_pulse.py --raw 0.3 0 0 0.6 --execute

--amount は cm（回る向きは °）。--execute のときは、送る前に Enter を待つ（Ctrl+C でやめる。歩いている途中の Ctrl+C でも StopMove を送る）。
送ったあと、脚 12 関節の角速度を記録し、止まるまでの時間を出す。_local/button_press/loco_pulse/<時刻>/ に
pulse.json（指令・結果）と legs.csv（時刻・脚の角速度の最大）を残す。動いた距離はメジャーか頭カメラ（real/check_face.py）で測る。
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

from common.config import REPO_ROOT, load_config  # noqa: E402
from common.loco import AXES, LEG_IDX, LegStillness, LocoError, LocoMover, Pulse  # noqa: E402


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


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    what = p.add_mutually_exclusive_group(required=True)
    what.add_argument("--axis", choices=list(AXES))
    what.add_argument("--raw", type=float, nargs=4, metavar=("VX", "VY", "WZ", "DURATION"))
    p.add_argument("--amount", type=float, help="--axis で動きたい量（cm、回る向きは °）")
    p.add_argument("--execute", action="store_true", help="実機に送る（付けなければ表示だけ）")
    p.add_argument("--network-interface")
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
    if args.execute:
        input("[loco_pulse] 周りに人がいない・リモコンを持った人がそばにいることを確かめて Enter（やめるなら Ctrl+C）: ")
    with mover:
        result = mover.pulse(pulse)
    if not args.execute:
        print("[loco_pulse] 送っていない（実機に送るなら --execute）")
        return 0
    print(f"[loco_pulse] {'止まった' if result.stopped else '❌ 止まらなかった'}（送ってから {result.settle_s:.2f} 秒）"
          f"{'。' + result.note if result.note else ''}")
    out = REPO_ROOT / "_local" / "button_press" / "loco_pulse" / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    t0 = stillness.rows[0][0] if stillness.rows else 0.0
    (out / "legs.csv").write_text("t_s,max_leg_dq_rad_s\n" + "".join(f"{t - t0:.3f},{v:.4f}\n" for t, v in stillness.rows))
    (out / "pulse.json").write_text(json.dumps({
        "pulse": vars(pulse), "stopped": result.stopped, "settle_s": result.settle_s, "note": result.note,
        "measured": {"forward_cm": None, "left_cm": None, "turn_deg": None},  # メジャー・頭カメラで測って書き足す
    }, indent=2, ensure_ascii=False))
    print(f"[loco_pulse] 保存: {out}")
    return 0 if result.stopped else 1


if __name__ == "__main__":
    sys.exit(main())
