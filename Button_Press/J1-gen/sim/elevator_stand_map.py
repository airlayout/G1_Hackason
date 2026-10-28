"""エレベーターの車いす用の ▲▼ を押すときの、立つ位置の許容範囲の地図を作る（計画だけ。MuJoCo は動かさない）。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/elevator_stand_map.py --workers 8
    $P Button_Press/J1-gen/sim/elevator_stand_map.py --xs 0.30 0.34 --ys -0.04 0.0 --tilts 0 --targets down

ラボのエレベーターの車いす用の組（2026-10-22 にメジャーで測定。床から ▲ 約 93.5 cm・▼ 約 87 cm、間隔 約 7 cm、直径 約 3.5 cm。
pelvis は床から約 0.73 m なので z は ▲ +0.21・▼ +0.14 m）を、pelvis の前方 x_mid・左右 y_mid（左が +）・柱の面の向きの傾き tilt（z まわり）で
振って、実際の計画（ElevatorPressController._plan: IK・指の向きと経由点の候補・体との衝突・柱と壁の箱との衝突）が通るかを調べる。
あわせて、頭カメラ（URDF の約 48° 下向き、640×360）に、2 つのボタンと柱の面の範囲が入るかを出す（見つけられない位置では押せない）。
開始の姿勢は、10/21 の実機の立った姿勢（sim/reach_map.py と同じ）。今の勧める範囲（elevator.yaml の press.stand）も重ねて表示する。
記号: # = 計画が通り、カメラにも入る / o = 計画は通るが、カメラに入らない / 英字 = 断られた段階
  W 作業空間の外 / R 手前の姿勢に届かない / A 手前の姿勢で体に当たる / V 経由点で当たる / 1・2 手前の姿勢へ動く途中で体に当たる
  P・Q 押し込みの直線で届かない・当たる / J 関節が急に動く / K 手前の姿勢へ動く途中で柱・壁に当たる
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from common.camera_geometry import HeadCameraTransform  # noqa: E402
from common.config import REPO_ROOT, load_config  # noqa: E402
from common.elevator_buttons import Button3D, CallButtons  # noqa: E402
from common.elevator_press import ElevatorPressController  # noqa: E402
from common.press_planner import UnreachableError  # noqa: E402
from reach_map import CX, CY, FX, FY, H, REAL_STAND_1021, W, classify  # noqa: E402

Z_UP, Z_DOWN, DIAM, SPACING = 0.21, 0.14, 0.035, 0.07
STAGES = ["W", "R", "A", "V", "1", "2", "P", "Q", "J", "K"]
_G: dict[str, Any] = {}


def init_worker() -> None:
    robot, press, arm, elev = (load_config(n) for n in ("robot.yaml", "press.yaml", "arm.yaml", "elevator.yaml"))
    ctl = ElevatorPressController.from_config(robot, press, arm, elev)
    _G.update(ctl=ctl, tf=HeadCameraTransform(robot), elev=elev)


def make_call(x_mid: float, y_mid: float, tilt_deg: float) -> CallButtons:
    n = np.array([np.cos(np.radians(tilt_deg)), np.sin(np.radians(tilt_deg)), 0.0])
    up = Button3D(np.array([x_mid, y_mid, Z_UP]), DIAM, "up", (0, 0, 1, 1), 100, "up")
    down = Button3D(np.array([x_mid, y_mid, Z_DOWN]), DIAM, "down", (0, 0, 1, 1), 100, "down")
    return CallButtons(up, down, n)


def visible(tf: HeadCameraTransform, x_mid: float, y_mid: float, margin_px: float = 8.0, region_m: float = 0.06) -> tuple[bool, bool]:
    """(2 つのボタンの丸が全部画像に入る, 柱の面を当てはめる範囲（中点から左右 region_m・上下 spacing/2 + region_m）の四隅も入る)。"""
    r, p = tf.pelvis_from_optical(np.zeros(3))

    def inside(pt: np.ndarray, m: float) -> bool:
        o = r.T @ (pt - p)
        if o[2] < 0.2:
            return False
        u, v = CX + FX * o[0] / o[2], CY + FY * o[1] / o[2]
        return m <= u <= W - m and m <= v <= H - m

    rad = DIAM / 2
    btn = all(inside(np.array([x_mid, y_mid + dy, z + dz]), margin_px)
              for z in (Z_UP, Z_DOWN) for dy in (-rad, rad) for dz in (-rad, rad))
    zc = (Z_UP + Z_DOWN) / 2
    reg = all(inside(np.array([x_mid, y_mid + dy, zc + dz]), 0.0)
              for dy in (-region_m, region_m) for dz in (-(SPACING / 2 + region_m), SPACING / 2 + region_m))
    return btn, reg


def run_cell(cell: tuple[float, float, float, str]) -> dict[str, Any]:
    x, y, tilt, target = cell
    ctl: ElevatorPressController = _G["ctl"]
    call = make_call(x, y, tilt)
    ctl.reset(target, 0.02, np.full(17, 60.0))
    ok, st, why, reason = 0, "", "", ""
    try:
        ctl._plan(REAL_STAND_1021, call)
        ok = 1
    except UnreachableError as e:
        best = (-1, "R", "")
        for part in str(e).split(" / "):
            s, w = ("K", "wall") if "柱・壁" in part else classify(part)
            if STAGES.index(s) > best[0]:
                best = (STAGES.index(s), s, w)
        st, why = best[1], best[2]
        reason = str(e)[:140]
    vb, vr = visible(_G["tf"], x, y)
    plan = ctl.plan_ if ok else None
    return {"x": x, "y": y, "tilt": tilt, "target": target, "plan_ok": ok, "stage": st, "partner": why,
            "vis_buttons": int(vb), "vis_region": int(vr), "advice": ctl._stand_advice(call),
            "summary": "" if plan is None else plan.summary()[:60], "reason": reason}


def sym(r: dict[str, Any]) -> str:
    if r["plan_ok"]:
        return "#" if r["vis_buttons"] and r["vis_region"] else ("+" if r["vis_buttons"] else "o")
    return r["stage"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--xs", type=float, nargs="+", default=[round(0.20 + 0.02 * i, 2) for i in range(14)])
    ap.add_argument("--ys", type=float, nargs="+", default=[round(-0.20 + 0.04 * i, 2) for i in range(11)])
    ap.add_argument("--tilts", type=float, nargs="+", default=[-10.0, -5.0, 0.0, 5.0, 10.0])
    ap.add_argument("--targets", nargs="+", default=["down", "up"], choices=["up", "down"])
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()

    cells = [(x, y, t, tg) for tg in args.targets for t in args.tilts for x in args.xs for y in args.ys]
    t0 = time.time()
    print(f"[stand] {len(cells)} 通りを計画する（{args.workers} プロセス）")
    with ProcessPoolExecutor(args.workers, initializer=init_worker) as ex:
        rows = list(ex.map(run_cell, cells, chunksize=4))
    out = args.out or REPO_ROOT / "_local" / "button_press" / "stand_map" / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "map.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    idx = {(r["x"], r["y"], r["tilt"], r["target"]): r for r in rows}
    elev = load_config("elevator.yaml")["press"]["stand"]
    lo, hi, side = float(elev["min_dist_m"]), float(elev["max_dist_m"]), float(elev["max_side_m"])
    text = [f"今の勧める範囲（elevator.yaml の press.stand）: 柱まで {lo * 100:.0f}〜{hi * 100:.0f} cm、左右 {side * 100:.0f} cm 以内（[ ] で囲む）",
            "記号: # 計画が通り、ボタンも柱の範囲もカメラに入る / + ボタンは入るが柱の範囲が一部出る / o 計画は通るがカメラに入らない / 英字 断られた段階"]
    for tg in args.targets:
        for t in args.tilts:
            text.append(f"\n■ 押すボタン {tg}、柱の面の傾き {t:+.0f}°   （行 = ボタンの左右 y [cm]、列 = 柱の面までの距離 x [cm]。y > 0 が左）")
            text.append("   y \\ x " + " ".join(f"{x * 100:3.0f}" for x in args.xs))
            for y in args.ys:
                cells_s = []
                for x in args.xs:
                    r = idx[(x, y, t, tg)]
                    d = x * np.cos(np.radians(t)) + y * np.sin(np.radians(t))   # 中点から面までの押す向きの距離
                    inwin = lo <= d <= hi and abs(y) <= side + 1e-9
                    cells_s.append(f"[{sym(r)}]" if inwin else f" {sym(r)} ")
                text.append(f"  {y * 100:+4.0f}   " + " ".join(cells_s))
    ok = sum(r["plan_ok"] for r in rows)
    full = sum(r["plan_ok"] and r["vis_buttons"] and r["vis_region"] for r in rows)
    summ = [f"計画が通る {ok}/{len(rows)}、カメラにも入る {full}/{len(rows)}（{time.time() - t0:.0f} 秒）"]
    cnt: dict[tuple[str, str], int] = {}
    for r in rows:
        if not r["plan_ok"]:
            cnt[(r["stage"], r["partner"])] = cnt.get((r["stage"], r["partner"]), 0) + 1
    summ.append("断られた段階（件数）: " + "、".join(f"{s}{('/' + w) if w else ''} {n}" for (s, w), n in sorted(cnt.items(), key=lambda kv: -kv[1])))
    (out / "map.txt").write_text("\n".join(summ + text) + "\n", encoding="utf-8")
    print("\n".join(summ))
    print(f"[stand] 保存: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
