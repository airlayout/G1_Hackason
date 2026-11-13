"""エレベーターの車いす用の ▲▼ を押すときの、立つ位置の許容範囲の地図を作る（計画だけ。MuJoCo は動かさない）。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/elevator_stand_map.py --workers 8
    $P Button_Press/J1-gen/sim/elevator_stand_map.py --xs 0.30 0.34 --ys -0.04 0.0 --tilts 0 --targets down
    $P Button_Press/J1-gen/sim/elevator_stand_map.py --button-set wheelchair --ys -0.28 -0.24 -0.20 -0.16 -0.12 -0.08 -0.04 0.0 0.04 --tilts 0
    # 腰 yaw を振る（[deg]。+ は上から見て左回り＝上体が左を向く。0 を含める。複数のときは、腰 yaw ごとの地図とまとめの地図を出す）
    $P Button_Press/J1-gen/sim/elevator_stand_map.py --button-set wheelchair --tilts 0 --waist-yaws 0 10 20 30 -10 -20 -30 --workers 8
    # 左腕（ボタンを体の左に置く）
    $P Button_Press/J1-gen/sim/elevator_stand_map.py --arm left --button-set wheelchair --ys -0.04 0.0 0.04 0.08 0.12 0.16 0.20 0.24 0.28 --tilts 0

ラボのエレベーターの車いす用の組（2026-10-22 にメジャーで測定。床から ▲ 約 93.5 cm・▼ 約 87 cm、間隔 約 7 cm、直径 約 3.5 cm。
pelvis は床から約 0.73 m なので z は ▲ +0.21・▼ +0.14 m）を、pelvis の前方 x_mid・左右 y_mid（左が +）・柱の面の向きの傾き tilt（z まわり）で
振って、実際の計画（ElevatorPressController._plan: IK・指の向きと経由点の候補・体との衝突・柱と壁の箱との衝突）が通るかを調べる。
あわせて、頭カメラ（URDF の約 48° 下向き、640×360）に、2 つのボタンと柱の面の範囲が入るかを出す（見つけられない位置では押せない）。
開始の姿勢は、10/21 の実機の立った姿勢（sim/reach_map.py と同じ）。今の勧める範囲（elevator.yaml の press.stand）も重ねて表示する。
記号: # = 計画が通り、カメラにも入る / o = 計画は通るが、カメラに入らない / 英字 = 断られた段階
  W 作業空間の外 / R 手前の姿勢に届かない / A 手前の姿勢で体に当たる / V 経由点で当たる / D 手前の姿勢へ（経由点なしで）動く途中で体に当たる / 1・2 手前の姿勢へ動く途中で体に当たる
  P・Q 押し込みの直線で届かない・当たる / J 関節が急に動く / K 手前の姿勢へ動く途中で柱・壁に当たる
腰 yaw（--waist-yaws）: 開始の姿勢の腰（motor 12）だけを入れ替えて計画する。ボタンの位置・柱の面の向きは pelvis 座標のまま。
カメラに入るかの判定も、その腰の角度で出す（頭カメラは胴体に付いていて、腰と一緒に回る）。
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
WAIST_YAW = 12   # motor 番号順での waist_yaw_joint
STAGES = ["W", "R", "A", "V", "D", "1", "2", "P", "Q", "J", "K"]
_G: dict[str, Any] = {}


def init_worker(button_set: str = "general", arm: str = "right") -> None:
    robot, press, arm_cfg, elev = (load_config(n) for n in ("robot.yaml", "press.yaml", "arm.yaml", "elevator.yaml"))
    # 押す組ごとの設定（press.by_button_set。wheelchair は指をまっすぐ向ける）を使う
    elev["geometry"]["button_set"] = button_set
    elev["press"]["arm"] = arm
    ctl = ElevatorPressController.from_config(robot, press, arm_cfg, elev)
    _G.update(ctl=ctl, tf=HeadCameraTransform(robot), elev=elev)


def make_call(x_mid: float, y_mid: float, tilt_deg: float) -> CallButtons:
    n = np.array([np.cos(np.radians(tilt_deg)), np.sin(np.radians(tilt_deg)), 0.0])
    up = Button3D(np.array([x_mid, y_mid, Z_UP]), DIAM, "up", (0, 0, 1, 1), 100, "up")
    down = Button3D(np.array([x_mid, y_mid, Z_DOWN]), DIAM, "down", (0, 0, 1, 1), 100, "down")
    return CallButtons(up, down, n)


def visible(tf: HeadCameraTransform, x_mid: float, y_mid: float, margin_px: float = 8.0, region_m: float = 0.06,
            waist_yaw_deg: float = 0.0) -> tuple[bool, bool]:
    """(2 つのボタンの丸が全部画像に入る, 柱の面を当てはめる範囲（中点から左右 region_m・上下 spacing/2 + region_m）の四隅も入る)。

    waist_yaw_deg: 腰 yaw [deg]。頭カメラは胴体に付いているので、腰をひねるとカメラも回る（roll・pitch は 0 のまま）。
    """
    r, p = tf.pelvis_from_optical(np.array([np.radians(waist_yaw_deg), 0.0, 0.0]))

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


def run_cell(cell: tuple[float, float, float, str] | tuple[float, float, float, str, float]) -> dict[str, Any]:
    x, y, tilt, target = cell[:4]
    yaw = float(cell[4]) if len(cell) > 4 else 0.0   # 腰 yaw [deg]
    q0 = REAL_STAND_1021.copy()
    q0[WAIST_YAW] = np.radians(yaw)   # 腰は固定の関節として扱われるので、入れ替えるだけでひねった姿勢での計画になる
    ctl: ElevatorPressController = _G["ctl"]
    call = make_call(x, y, tilt)
    ctl.reset(target, 0.02, np.full(17, 60.0))
    ok, st, why, reason = 0, "", "", ""
    try:
        ctl._plan(q0, call)
        ok = 1
    except UnreachableError as e:
        best = (-1, "R", "")
        for part in str(e).split(" / "):
            s, w = ("K", "wall") if "柱・壁" in part else classify(part)
            if STAGES.index(s) > best[0]:
                best = (STAGES.index(s), s, w)
        st, why = best[1], best[2]
        reason = str(e)[:140]
    vb, vr = visible(_G["tf"], x, y, waist_yaw_deg=yaw)
    plan = ctl.plan_ if ok else None
    return {"x": x, "y": y, "tilt": tilt, "target": target, "waist_yaw": yaw, "plan_ok": ok, "stage": st, "partner": why,
            "vis_buttons": int(vb), "vis_region": int(vr), "advice": ctl._stand_advice(call),
            "summary": "" if plan is None else plan.summary()[:60], "reason": reason}


def sym(r: dict[str, Any]) -> str:
    if r["plan_ok"]:
        return "#" if r["vis_buttons"] and r["vis_region"] else ("+" if r["vis_buttons"] else "o")
    return r["stage"]


def full_ok(r: dict[str, Any]) -> bool:
    """計画が通り、ボタンも柱の範囲もカメラに入る。"""
    return bool(r["plan_ok"] and r["vis_buttons"] and r["vis_region"])


def yaw_symbols(yaws: list[float]) -> dict[float, str]:
    """0 以外の腰 yaw の、まとめの地図の 1 文字。正は絶対値の小さい順に 1・2・3…、負は a・b・c…。"""
    pos = sorted(y for y in yaws if y > 0)
    neg = sorted((y for y in yaws if y < 0), key=abs)
    return {**{y: str(i + 1) for i, y in enumerate(pos)}, **{y: "abcdefghij"[i] for i, y in enumerate(neg)}}


def best_yaw_symbol(by_yaw: dict[float, dict[str, Any]], yaws: list[float], syms: dict[float, str]) -> str:
    """まとめの地図の 1 文字。0° で通りカメラにも入る = #、0° でだめでどれかの腰 yaw で通る = その中で絶対値が最小（同じなら正）の記号、
    どれでもだめ = .。"""
    if 0.0 in by_yaw and full_ok(by_yaw[0.0]):
        return "#"
    ok = [y for y in yaws if y != 0.0 and full_ok(by_yaw[y])]
    return syms[min(ok, key=lambda y: (abs(y), -y))] if ok else "."


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--xs", type=float, nargs="+", default=[round(0.20 + 0.02 * i, 2) for i in range(14)])
    ap.add_argument("--ys", type=float, nargs="+", default=[round(-0.20 + 0.04 * i, 2) for i in range(11)])
    ap.add_argument("--tilts", type=float, nargs="+", default=[-10.0, -5.0, 0.0, 5.0, 10.0])
    ap.add_argument("--targets", nargs="+", default=["down", "up"], choices=["up", "down"])
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--button-set", default="general", choices=["general", "wheelchair"],
                    help="押す組（press.by_button_set の設定と勧める範囲を使う。ボタンの高さはどちらもラボの車いす用）")
    ap.add_argument("--arm", default="right", choices=["left", "right"], help="押す腕（elevator.yaml の press.arm）")
    ap.add_argument("--waist-yaws", type=float, nargs="+", default=[0.0],
                    help="腰 yaw の並び [deg]（+ は上から見て左回り）。既定は 0 だけ（今までと同じ出力）。複数のとき、まとめの地図も出す")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()

    yaws = list(dict.fromkeys(args.waist_yaws))
    use_yaw = yaws != [0.0]   # 既定のときは、出力（CSV・地図・数）を今までと同じにする
    if use_yaw and 0.0 not in yaws:
        yaws.insert(0, 0.0)
        print("[stand] 腰 yaw に 0 を足した（まとめの地図の比べる基準）")
    cells = [(x, y, t, tg, w) for tg in args.targets for t in args.tilts for x in args.xs for y in args.ys for w in yaws]
    t0 = time.time()
    print(f"[stand] {len(cells)} 通りを計画する（{args.workers} プロセス）")
    with ProcessPoolExecutor(args.workers, initializer=init_worker, initargs=(args.button_set, args.arm)) as ex:
        rows = list(ex.map(run_cell, cells, chunksize=4))
    out = args.out or REPO_ROOT / "_local" / "button_press" / "stand_map" / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    idx = {(r["x"], r["y"], r["tilt"], r["target"], r["waist_yaw"]): r for r in rows}   # 腰 yaw の列は --waist-yaws のときだけ CSV に出す
    csv_rows = rows if use_yaw else [{k: v for k, v in r.items() if k != "waist_yaw"} for r in rows]
    with open(out / "map.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(csv_rows[0]))
        w.writeheader()
        w.writerows(csv_rows)
    press = load_config("elevator.yaml")["press"]
    elev = {**press["stand"], **((press.get("by_button_set") or {}).get(args.button_set, {}).get("stand", {}))}
    lo, hi, side = float(elev["min_dist_m"]), float(elev["max_dist_m"]), float(elev["max_side_m"])
    center = (1.0 if args.arm == "right" else -1.0) * float(elev.get("center_side_m", 0.0))  # 右腕の値。左腕なら反転
    text = [f"押す腕 {args.arm}、押す組 {args.button_set}。今の勧める範囲（elevator.yaml の press.stand）: 柱まで {lo * 100:.0f}〜{hi * 100:.0f} cm、"
            f"左右 {center * 100:+.0f}±{side * 100:.0f} cm（[ ] で囲む）",
            "記号: # 計画が通り、ボタンも柱の範囲もカメラに入る / + ボタンは入るが柱の範囲が一部出る / o 計画は通るがカメラに入らない / 英字 断られた段階"]
    syms = yaw_symbols(yaws)
    if use_yaw and len(yaws) > 1:
        text.append("まとめの地図の記号: # 腰 0° で計画が通り、カメラにも入る / 数字・英字 0° ではだめだが、その腰 yaw で通り、その腰の角度でカメラにも入る"
                    "（通った腰 yaw のうち絶対値が最小。同じなら正）: "
                    + "、".join(f"{c} = {y:+.0f}°" for y, c in sorted(syms.items(), key=lambda kv: (kv[0] < 0, abs(kv[0]))))
                    + " / . どの腰 yaw でもだめ")

    def grid(tg: str, t: float, cellsym: Any, title: str) -> None:
        text.append(f"\n■ 押すボタン {tg}、柱の面の傾き {t:+.0f}°{title}   （行 = ボタンの左右 y [cm]、列 = 柱の面までの距離 x [cm]。y > 0 が左）")
        text.append("   y \\ x " + " ".join(f"{x * 100:3.0f}" for x in args.xs))
        for y in args.ys:
            cells_s = []
            for x in args.xs:
                d = x * np.cos(np.radians(t)) + y * np.sin(np.radians(t))   # 中点から面までの押す向きの距離
                inwin = lo <= d <= hi and abs(y - center) <= side + 1e-9
                cells_s.append(f"[{cellsym(x, y)}]" if inwin else f" {cellsym(x, y)} ")
            text.append(f"  {y * 100:+4.0f}   " + " ".join(cells_s))

    for tg in args.targets:
        for t in args.tilts:
            for w in yaws:
                grid(tg, t, lambda x, y, w=w: sym(idx[(x, y, t, tg, w)]), f"、腰 yaw {w:+.0f}°" if use_yaw else "")
            if use_yaw and len(yaws) > 1:
                grid(tg, t, lambda x, y: best_yaw_symbol({w: idx[(x, y, t, tg, w)] for w in yaws}, yaws, syms), "、まとめ（腰 yaw を選んだとき）")
    ok = sum(r["plan_ok"] for r in rows)
    full = sum(r["plan_ok"] and r["vis_buttons"] and r["vis_region"] for r in rows)
    summ = [f"計画が通る {ok}/{len(rows)}、カメラにも入る {full}/{len(rows)}（{time.time() - t0:.0f} 秒）"]
    cnt: dict[tuple[str, str], int] = {}
    for r in rows:
        if not r["plan_ok"]:
            cnt[(r["stage"], r["partner"])] = cnt.get((r["stage"], r["partner"]), 0) + 1
    summ.append("断られた段階（件数）: " + "、".join(f"{s}{('/' + w) if w else ''} {n}" for (s, w), n in sorted(cnt.items(), key=lambda kv: -kv[1])))
    if use_yaw and len(yaws) > 1:
        keys = sorted({k[:4] for k in idx})
        n_key = len(keys)
        base_ok = sum(1 for k in keys if idx[(*k, 0.0)]["plan_ok"])
        base_full = sum(1 for k in keys if full_ok(idx[(*k, 0.0)]))
        any_ok = sum(1 for k in keys if any(idx[(*k, w)]["plan_ok"] for w in yaws))
        any_full = sum(1 for k in keys if any(full_ok(idx[(*k, w)]) for w in yaws))
        summ.append(f"配置 {n_key} 通り。腰 0° で計画が通る {base_ok}（カメラにも入る {base_full}）、"
                    f"どれかの腰 yaw で通る {any_ok}（カメラにも入る {any_full}）、増えた数 計画が通る {any_ok - base_ok}・カメラにも入る {any_full - base_full}")
        summ.append("腰 yaw ごとの、計画が通る数（カメラにも入る数）: " + "、".join(
            f"{w:+.0f}° {sum(idx[(*k, w)]['plan_ok'] for k in keys)}（{sum(full_ok(idx[(*k, w)]) for k in keys)}）" for w in yaws))
        for w in yaws:
            c2: dict[tuple[str, str], int] = {}
            for k in keys:
                r = idx[(*k, w)]
                if not r["plan_ok"]:
                    c2[(r["stage"], r["partner"])] = c2.get((r["stage"], r["partner"]), 0) + 1
            summ.append(f"  腰 {w:+.0f}° の断られた段階: " + "、".join(f"{s}{('/' + p) if p else ''} {n}" for (s, p), n in sorted(c2.items(), key=lambda kv: -kv[1])))
    (out / "map.txt").write_text("\n".join(summ + text) + "\n", encoding="utf-8")
    print("\n".join(summ))
    print(f"[stand] 保存: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
