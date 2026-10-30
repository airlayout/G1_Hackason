"""ボトルを倒す条件の掃引（MuJoCo）。押し込みの深さ・押す高さ・ボトルの水の量・摩擦を振って、倒れるかを表にする。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/knock_sweep.py --depth-mm 60 80 100 120 --raise-mm 20 40
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/knock_sweep.py --water-ml 0 50 100 200 --friction 0.3 0.5

各軸に値を並べると、その組み合わせをすべて試す（既定は knock.yaml / sim_scene_knock.yaml の値 1 つ）。1 回あたり約 10 秒。
倒れた = 最後のボトルの傾きが 60° 以上（knock_bottle_sim.TIPPED_DEG）。
"""

from __future__ import annotations

import argparse
import contextlib
import io
import itertools
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from common.pipeline_cli import add_common_args  # noqa: E402
from knock_bottle_sim import KnockResult, run_knock_sim  # noqa: E402


def agree(r: KnockResult) -> str:
    """自動判定が、MuJoCo のボトルの傾きの判定と合うか。unk = 判定できない（ボトルが見えなくなった、など）。"""
    if r.auto_state is None:
        return "-"
    if r.auto_state == "unknown":
        return "unk"
    return "ok" if (r.auto_state == "fallen") == r.tipped else "NG"


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--depth-mm", type=float, nargs="+", default=[None], help="押し込みの深さ [mm]")
    p.add_argument("--raise-mm", type=float, nargs="+", default=[None], help="測った面から押す点を上げる量 [mm]")
    p.add_argument("--speed-mm-s", type=float, nargs="+", default=[None], help="押す速さ [mm/s]")
    p.add_argument("--water-ml", type=float, nargs="+", default=[None], help="中の水 [mL]")
    p.add_argument("--shell-g", type=float, nargs="+", default=[None], help="空のボトルの質量 [g]")
    p.add_argument("--friction", type=float, nargs="+", default=[None], help="ボトルと机・ハンドの摩擦係数")
    a = p.parse_args()

    print(f"{'深さmm':>6} {'上げmm':>6} {'速さ':>5} {'水mL':>5} {'殻g':>5} {'摩擦':>5} | {'結果':<6} {'最大°':>6} {'最後°':>6} "
          f"{'動いたcm':>8} {'落下':>4} | {'自動判定':<9} {'下がりcm':>7} {'一致':>4}")
    n = 0
    for depth, rise, speed, water, shell, fric in itertools.product(a.depth_mm, a.raise_mm, a.speed_mm_s, a.water_ml,
                                                                     a.shell_g, a.friction):
        ap = argparse.ArgumentParser()
        add_common_args(ap)
        args = ap.parse_args([])
        args.sim_detector, args.confirm, args.view = "segmentation", False, False
        args.overlay = "knock.yaml"
        n += 1
        args.label = f"sweep{n:03d}"  # 同じ秒に始まる記録のフォルダ名がぶつからないように
        if depth is not None:
            args.depth_mm = depth
        if rise is not None:
            args.target_offset = [0.0, 0.0, rise / 1000.0]
        if speed is not None:
            args.speed_mm_s = speed
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                r, backend = run_knock_sim(args, water, shell, fric)
            backend.close()
        except Exception as e:  # noqa: BLE001 掃引なので、1 回の失敗で止めない
            print(f"{depth!s:>6} {rise!s:>6} {speed!s:>5} {water!s:>5} {shell!s:>5} {fric!s:>5} | 失敗: {e}")
            continue
        verdict = "倒れた" if r.tipped else ("中止" if r.pipeline.code else "倒れず")
        print(f"{depth!s:>6} {rise!s:>6} {speed!s:>5} {water!s:>5} {shell!s:>5} {fric!s:>5} | {verdict:<6} "
              f"{r.max_tilt_deg:6.1f} {r.final_tilt_deg:6.1f} {r.moved_m * 100:8.1f} {'はい' if r.fell_off_table else '-':>4} | "
              f"{r.auto_state!s:<9} {'-' if r.auto_drop_m is None else f'{r.auto_drop_m * 100:7.1f}':>7} "
              f"{agree(r):>4}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
