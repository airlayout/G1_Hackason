"""ボトルを倒す動作（MuJoCo、同じプロセスの中）: 検出 → 目標 → IK → 手前の姿勢 → 前方へ押し込む → 戻る。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/knock_bottle_sim.py
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/knock_bottle_sim.py --water-ml 100 --view

流れは sim/press_bottle_sim.py（common/pipeline.py）と同じ。違うのは、configs/knock.yaml の設定（深く押す・上のほうを押す）と、
ボトルが倒れる自由物体であること（configs/sim_scene_knock.yaml）。倒れたかは、MuJoCo のボトルの傾きから判定する
（実機では、まず人が見る）。ボトルの質量・水の量・摩擦は引数で変えられる（最適な条件を探すのに使う: sim/knock_sweep.py）。
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from common.pipeline import PipelineResult  # noqa: E402
from common.pipeline_cli import add_common_args  # noqa: E402
from common.sim_scene import bottle_position, bottle_tilt_deg  # noqa: E402
from press_bottle_sim import run_sim  # noqa: E402

SCENE = "sim_scene_knock.yaml"
OVERLAY = "knock.yaml"
# 倒れたとみなす最終の傾き [deg]。立っていれば 0、横に寝ていれば 90。机の上で斜めに寄りかかった状態（倒れかけ）は除く
TIPPED_DEG = 60.0
EXIT_NOT_TIPPED = 20


@dataclass
class KnockResult:
    pipeline: PipelineResult
    tipped: bool
    final_tilt_deg: float
    max_tilt_deg: float
    # ボトルが動いた距離（底の中心。水平方向）[m] と、落ちたか（底の高さが天板より 5 cm 以上低い）
    moved_m: float
    fell_off_table: bool
    log_dir: Path
    # カメラの深度での自動判定（common/tip_check.py）。MuJoCo のボトルの傾きの判定（tipped）と合うかを見る
    auto_state: str | None = None
    auto_drop_m: float | None = None
    tilt_history: list[tuple[float, float]] = field(default_factory=list)  # (時刻 [s]、傾き [deg])


def run_knock_sim(args: argparse.Namespace, water_ml: float | None = None, shell_g: float | None = None,
                  friction: float | None = None, arm_overrides: dict[str, Any] | None = None) -> tuple[KnockResult, Any]:
    """ボトルを倒すシミュレーションを 1 回行う。(結果, バックエンド) を返す。"""
    args.overlay = args.overlay if getattr(args, "overlay", None) else OVERLAY
    bottle: dict[str, Any] = {}
    if water_ml is not None:
        bottle["water_ml"] = water_ml
    if shell_g is not None:
        bottle["shell_mass_kg"] = shell_g / 1000.0
    scene: dict[str, Any] = {"bottle": bottle}
    if friction is not None:
        bottle["friction"] = friction
        scene["table"] = {"friction": friction}

    hist: list[tuple[float, float]] = []
    start: dict[str, Any] = {}

    def hook(backend: Any) -> None:
        start["pos"] = bottle_position(backend.model, backend.data)
        orig = backend.tick

        def tick(dt: float) -> None:
            orig(dt)
            hist.append((float(backend.data.time), bottle_tilt_deg(backend.model, backend.data)))

        backend.tick = tick

    res, backend, logger = run_sim(args, arm_overrides=arm_overrides, scene_name=SCENE, scene_overrides=scene,
                                   on_backend=hook, tip_check=True)
    pos = bottle_position(backend.model, backend.data)
    tilt = bottle_tilt_deg(backend.model, backend.data)
    result = KnockResult(
        pipeline=res, tipped=tilt >= TIPPED_DEG, final_tilt_deg=tilt,
        max_tilt_deg=max((h[1] for h in hist), default=tilt),
        moved_m=float(np.linalg.norm((pos - start["pos"])[:2])),
        fell_off_table=bool(pos[2] < start["pos"][2] - 0.05), log_dir=logger.dir, tilt_history=hist,
        auto_state=None if res.tip is None else res.tip.state, auto_drop_m=None if res.tip is None else res.tip.drop_m)
    return result, backend


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p)
    p.add_argument("--sim-detector", choices=["segmentation", "color", "yolo"], default="segmentation")
    p.add_argument("--confirm", action="store_true", help="各段階で Enter を待つ")
    p.add_argument("--view", action="store_true", help="MuJoCo の画面を開いて、動きを実時間で見る")
    p.add_argument("--water-ml", type=float, help="中の水 [mL]（既定は sim_scene_knock.yaml）")
    p.add_argument("--shell-g", type=float, help="空のボトルの質量 [g]")
    p.add_argument("--friction", type=float, help="ボトルと机・ハンドの摩擦係数")
    args = p.parse_args()
    if args.sim_detector in ("color", "yolo"):
        args.detector = args.sim_detector
    r, backend = run_knock_sim(args, args.water_ml, args.shell_g, args.friction)
    print(f"[knock_sim] ボトルの傾き: 最大 {r.max_tilt_deg:.1f}°、最後 {r.final_tilt_deg:.1f}°、"
          f"動いた距離 {r.moved_m * 100:.1f} cm、机から{'落ちた' if r.fell_off_table else '落ちていない'}"
          f" → {'倒れた' if r.tipped else '倒れなかった'}")
    print(f"[knock_sim] 深度での自動判定: {r.auto_state}（いちばん高い点の下がり "
          f"{'—' if r.auto_drop_m is None else f'{r.auto_drop_m * 100:.1f} cm'}）"
          f"{'（傾きの判定と一致）' if (r.auto_state == 'fallen') == r.tipped else '（⚠️ 傾きの判定と食い違う）'}")
    print(f"[knock_sim] 記録: {r.log_dir}")
    if backend.viewer_running():
        print("[knock_sim] 終わった。画面を閉じると終わる（Ctrl+C でも終わる）")
        try:
            while backend.viewer_running():
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
    backend.close()
    if r.pipeline.code != 0:
        return r.pipeline.code
    return 0 if r.tipped else EXIT_NOT_TIPPED


if __name__ == "__main__":
    sys.exit(main())
