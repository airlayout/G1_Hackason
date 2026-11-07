"""台の高さごとに同じ悪条件で押す流れを繰り返し、どの高さが一番安定するかを比べる。

    G1_HuggingFace/venv/bin/python Trash_Bin/sim/sweep_stand_height.py
    G1_HuggingFace/venv/bin/python Trash_Bin/sim/sweep_stand_height.py --heights 0.08 0.10 0.12 --waists 0 --trials 20

1 回の試行は sim/push_bin_sim.py --locate（深度で測って、計画して、押す）。試行の番号ごとに悪条件を決めるので、
どの高さにも同じ悪条件が当たる（高さの違いだけを比べられる）:
- 立つ位置のずれ: 前後左右 ±2 cm・向き ±5°（近づく動作の許容と同じくらい）
- 上面（鏡面）の深度の欠け: 0〜60%、深度のノイズ: 1 m あたり 1 cm
- 頭カメラの本当の角度: 43〜49°（位置の計算は 47.6° のまま）、較正のずれ: 各軸 ±1 cm

成功の条件: 計画が通って最後まで動き、フラップが min_flap_deg 以上開き、フラップ以外のゴミ箱への接触が max_contact_n 以下。
結果は _local/trash_bin/stand_sweep/<日時>/results.csv と、画面の表。
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import os
import re
import statistics
import subprocess
import sys
import time
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "Trash_Bin" / "sim"))


def conditions(trial: int) -> dict[str, float]:
    """試行の番号ごとの悪条件（高さや腰の傾きによらず同じ）。"""
    rng = np.random.default_rng(20_000 + trial)
    return {"dx": float(rng.uniform(-0.02, 0.02)), "dy": float(rng.uniform(-0.02, 0.02)), "yaw": float(rng.uniform(-5, 5)),
            "dropout": float(rng.uniform(0, 0.6)), "cam_true": float(rng.uniform(43, 49))}


def run_trial(height: float, waist: float, front: float, y: float, pelvis: float | None, trial: int, args: argparse.Namespace) -> dict:
    c = conditions(trial)
    cmd = [sys.executable, str(REPO / "Trash_Bin/sim/push_bin_sim.py"), "--locate", "--lift-m", str(height),
           "--waist-pitch", str(waist), "--bin-front", f"{front + c['dx']:.4f}", "--bin-y", f"{y + c['dy']:.4f}",
           "--bin-yaw", f"{c['yaw']:.2f}", "--seed", str(trial), "--depth-dropout", f"{c['dropout']:.2f}",
           "--depth-noise", "0.01", "--cam-pitch-true", f"{c['cam_true']:.1f}", "--calib-error-m", "0.01",
           "--label", f"sweep_h{height}_w{waist}_f{front}_y{y}_p{pelvis}_t{trial}_{os.getpid()}"]
    if pelvis is not None:
        cmd += ["--pelvis-true", str(pelvis)]
    if args.back_wall_gap is not None:
        cmd += ["--back-wall-gap", str(args.back_wall_gap)]
    res: dict = {"height": height, "waist": waist, "front": front, "y": y, "pelvis": pelvis, "trial": trial,
                 **{k: round(v, 3) for k, v in c.items()}}
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=600, cwd=REPO,
                           env={**os.environ, "MUJOCO_GL": "glfw"})
        m = re.search(r"\[push_bin_sim\] 結果: (\{.*\})", p.stdout)
        r = json.loads(m.group(1)) if m else {"code": -1, "result": (p.stdout + p.stderr)[-150:]}
    except subprocess.TimeoutExpired:
        r = {"code": -2, "result": "時間切れ"}
    contacts = r.get("bin_contacts_N") or {}
    px = r.get("visible_px_at_end") or {}
    max_contact = max(contacts.values(), default=0.0)
    flap = r.get("flap_deg_at_end")
    if r["code"] != 0:
        why = "計画できない・見つからない" if r["code"] == 2 else f"実行エラー {r['code']}"
    elif flap is None or flap < args.min_flap_deg:
        why = "フラップが開かない"
    elif max_contact > args.max_contact_n:
        why = "フラップ以外に当たる"
    else:
        why = ""
    res.update(ok=not why, why=why, code=r["code"], flap=flap, max_contact=max_contact, contacts=json.dumps(contacts, ensure_ascii=False),
               bag_px=px.get("bag"), red_px=px.get("red"), cup_px=px.get("cup"),
               locate_err_cm=(r.get("located") or {}).get("center_err_cm"), result=r["result"][:120])
    return res


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--heights", type=float, nargs="+", default=[0.06, 0.08, 0.10, 0.12, 0.14, 0.16, 0.18], help="台の高さ [m]")
    p.add_argument("--waists", type=float, nargs="+", default=[0], help="腰の傾き [deg]")
    p.add_argument("--fronts", type=float, nargs="+", help="ゴミ箱の正面の x [m]（既定はシーンの設定）")
    p.add_argument("--ys", type=float, nargs="+", help="ゴミ箱の中心の y [m]（既定はシーンの設定）")
    p.add_argument("--pelvis-trues", type=float, nargs="+", help="世界の本当の pelvis の高さ [m]（計画は設定の値のまま。既定は設定と同じ）")
    p.add_argument("--trials", type=int, default=12, help="1 条件あたりの試行の数")
    p.add_argument("--back-wall-gap", type=float, help="ゴミ箱の背面から後ろの壁までの距離 [m]（既定は壁なし）")
    p.add_argument("--first-trial", type=int, default=0, help="試行の番号の始め（別の悪条件で確かめ直すときに変える）")
    p.add_argument("--workers", type=int, default=6)
    p.add_argument("--min-flap-deg", type=float, default=30.0, help="成功とみなすフラップの角度の下限")
    p.add_argument("--max-contact-n", type=float, default=5.0, help="フラップ以外への接触の力の上限 [N]")
    args = p.parse_args()

    from bin_scene import load_scene_config

    scene = load_scene_config()
    fronts = args.fronts or [float(scene["bin"]["front_x"])]
    ys = args.ys or [float(scene["bin"]["center_y"])]
    out_dir = REPO / "_local/trash_bin/stand_sweep" / time.strftime("%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    jobs = list(itertools.product(args.heights, args.waists, fronts, ys, args.pelvis_trues or [None],
                                  range(args.first_trial, args.first_trial + args.trials)))
    print(f"[sweep] {len(jobs)} 回（台 {args.heights}、腰 {args.waists}、正面 {fronts}、左右 {ys}、各 {args.trials} 回）", flush=True)
    rows = []
    with ThreadPoolExecutor(args.workers) as ex:
        for k, res in enumerate(ex.map(lambda j: run_trial(*j, args), jobs), start=1):
            rows.append(res)
            if k % 12 == 0:
                print(f"[sweep] {k}/{len(jobs)} 回", flush=True)
    keys = list(rows[0].keys())
    with open(out_dir / "results.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    groups = defaultdict(list)
    for r in rows:
        groups[(r["height"], r["waist"], r["front"], r["y"], r["pelvis"] or 0.0)].append(r)
    print("[sweep] 台 | 腰 | 正面 | 左右 | 本当の pelvis | 成功 | フラップ（成功の中央値・最小）| 頭カメラの画素 袋・赤・コップ（成功の中央値）| 失敗の内訳")
    for (h, w_, f_, y_, pv), rs in sorted(groups.items()):
        good = [r for r in rs if r["ok"]]
        fl = [r["flap"] for r in good]
        med = {k: statistics.median([r[k] or 0 for r in good]) if good else 0 for k in ("bag_px", "red_px", "cup_px")}
        why = defaultdict(int)
        for r in rs:
            if r["why"]:
                why[r["why"]] += 1
        print(f"[sweep] {h:.2f} m | {w_:.0f}° | {f_:.2f} | {y_:+.2f} | {pv or '設定'} | {len(good)}/{len(rs)} | "
              f"{statistics.median(fl) if fl else float('nan'):.0f}°・{min(fl) if fl else float('nan'):.0f}° | "
              f"{med['bag_px']:.0f}・{med['red_px']:.0f}・{med['cup_px']:.0f} | {dict(why) or 'なし'}")
    print(f"[sweep] 結果: {out_dir / 'results.csv'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
