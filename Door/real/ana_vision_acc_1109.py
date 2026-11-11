"""11-09: 画像の網が当てた扉の位置（pelvis から見た開口の中心の前・左）が、止まっている時と動いている時でどれだけ外れるか。

正解: READY（止まって立つ）は巻き尺。DOOR（歩いている）は巻き尺の出発の位置に脚の odom の動きを足した値。
/dog_odom は距離を前後で 4〜6%・横で 7〜10% 短く出す（10-21 実測）ので ODOM_SCALE で補正する（残る誤差は動いた距離の数 % ≒ 数 cm）。
網の値は 2 つ: 扉の頭（今の 1 枚だけから当てた値 vis_head）と、追う段込み（方策に渡した値 vis_door）。

  Navigation/.venv/bin/python Door/real/ana_vision_acc_1109.py --policy <door_vision_r1c7.npz> \
      --run <door_stock_*.npz>:<前 m>:<左 m>[:<名前>] [--run …]

前・左は READY の pelvis から見た開口の中心（前 = 足先の巻き尺 + 0.123 m、左 = (左の端 − 右の端) / 2）。
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import door_vision_core as dvc  # noqa: E402

READY, DOOR = 2, 3  # door_run_stock.PHASES の番号
ODOM_SCALE = (1.05, 1.085)  # /dog_odom の前後・横の取りこぼしの補正（10-21 の実測の中ほど）
HALF_HFOV_DEG = 34.7  # カラーの画角 69.4° の半分（扉の中心が画面の内か外かの目安）


def real_xy(p: dc.Params, vp: dvc.VisionParams, obs: np.ndarray, d17: np.ndarray) -> np.ndarray:
    _, pos = dvc.door_real(p, vp, dvc.insert_door(vp, obs, d17))
    return pos[:2]


def truth_xy(odom: np.ndarray, o0: np.ndarray, start: np.ndarray) -> np.ndarray:
    """READY の基準 o0 からの odom の動きを足した、今の pelvis から見た開口の中心（前・左）"""
    dx, dy = odom[0] - o0[0], odom[1] - o0[1]
    dyaw = odom[2] - o0[2]
    c0, s0 = math.cos(o0[2]), math.sin(o0[2])
    mx, my = c0 * dx + s0 * dy, -s0 * dx + c0 * dy  # 出発の pelvis の座標での動き
    rx, ry = start[0] - ODOM_SCALE[0] * mx, start[1] - ODOM_SCALE[1] * my
    c, s = math.cos(dyaw), math.sin(dyaw)
    return np.array([c * rx + s * ry, -s * rx + c * ry])


def run_rows(p: dc.Params, vp: dvc.VisionParams, path: str, start: np.ndarray, name: str) -> list[dict]:
    z = np.load(path, allow_pickle=True)
    ph = z["phase"].astype(int)
    t = z["t"]
    ready = np.nonzero(ph == READY)[0]
    door = np.nonzero(ph == DOOR)[0]
    o0 = z["odom"][ready[-1]] if len(door) == 0 else z["odom"][door[0]]
    t0 = t[door[0]] if len(door) else t[ready[-1]]
    gyro = z["gyro"]
    rows = []
    for i in np.concatenate([ready[-150:], door]):
        if not (np.isfinite(z["vis_head"][i]).all() and np.isfinite(z["vis_door"][i]).all()):
            continue
        obs = z["obs"][i].astype(float)
        tr = start if ph[i] == READY else truth_xy(z["odom"][i], o0, start)
        head = real_xy(p, vp, obs, z["vis_head"][i])
        full = real_xy(p, vp, obs, z["vis_door"][i])
        rows.append(dict(run=name, phase="止まる" if ph[i] == READY else "歩く", t=t[i] - t0,
                         dist=float(np.hypot(*tr)), bearing=math.degrees(math.atan2(tr[1], tr[0])),
                         wz=abs(float(gyro[i][2])), age=float(z["vis_age"][i]) * 1000,
                         eh=head - tr, ef=full - tr))
    return rows


def summarize(rows: list[dict], label: str) -> None:
    if not rows:
        print(f"  {label:34s} 0 周期")
        return
    eh = np.array([r["eh"] for r in rows])
    ef = np.array([r["ef"] for r in rows])
    med = lambda v: float(np.median(np.abs(v))) * 100  # noqa: E731
    p90 = lambda v: float(np.percentile(np.abs(v), 90)) * 100  # noqa: E731
    print(f"  {label:34s} {len(rows):5d} 周期  扉の頭 前 {med(eh[:, 0]):4.1f}/{p90(eh[:, 0]):5.1f}・左 {med(eh[:, 1]):4.1f}/{p90(eh[:, 1]):5.1f}"
          f"   追う段込み 前 {med(ef[:, 0]):4.1f}/{p90(ef[:, 0]):5.1f}・左 {med(ef[:, 1]):4.1f}/{p90(ef[:, 1]):5.1f}"
          f"   片寄り（頭）前 {100 * np.median(eh[:, 0]):+5.1f}・左 {100 * np.median(eh[:, 1]):+5.1f}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", required=True)
    ap.add_argument("--run", action="append", required=True)
    a = ap.parse_args(argv)
    p = dc.load_params(a.policy, mirror=True)
    vp = dvc.load_vision(a.policy, p)
    rows = []
    for spec in a.run:
        parts = spec.split(":")
        rows += run_rows(p, vp, parts[0], np.array([float(parts[1]), float(parts[2])]),
                         parts[3] if len(parts) > 3 else Path(parts[0]).stem[-6:])
    print("誤差の大きさ [cm]（中央値/90%）。正解は止まる＝巻き尺、歩く＝巻き尺＋odom（補正 前 ×1.05・横 ×1.085）")
    print("== 回ごと（止まる＝READY の最後の 3 秒・歩く＝DOOR の全部）")
    for name in dict.fromkeys(r["run"] for r in rows):
        for ph in ("止まる", "歩く"):
            summarize([r for r in rows if r["run"] == name and r["phase"] == ph], f"{name} {ph}")
    walk = [r for r in rows if r["phase"] == "歩く"]
    print("== 歩く間を条件で分ける（全回）")
    summarize([r for r in rows if r["phase"] == "止まる"], "止まる（全回）")
    summarize([r for r in walk if r["t"] < 3.0], "歩く: 始めの 3 秒（寄っていく）")
    summarize([r for r in walk if r["t"] >= 3.0], "歩く: 3 秒より後（扉の前）")
    summarize([r for r in walk if abs(r["bearing"]) <= HALF_HFOV_DEG * 0.5], "歩く: 扉の中心が正面 ±17°")
    summarize([r for r in walk if HALF_HFOV_DEG * 0.5 < abs(r["bearing"]) <= HALF_HFOV_DEG], "歩く: 扉の中心が 17〜35°（画面の端）")
    summarize([r for r in walk if abs(r["bearing"]) > HALF_HFOV_DEG], "歩く: 扉の中心が 35° より外（画面の外）")
    summarize([r for r in walk if r["wz"] < 0.2], "歩く: 胴の回る速さ 0.2 rad/s 未満")
    summarize([r for r in walk if r["wz"] >= 0.5], "歩く: 胴の回る速さ 0.5 rad/s 以上")
    summarize([r for r in walk if r["dist"] >= 0.6], "歩く: 扉の中心まで 0.6 m 以上")
    summarize([r for r in walk if r["dist"] < 0.6], "歩く: 扉の中心まで 0.6 m 未満")
    ages = np.array([r["age"] for r in rows if np.isfinite(r["age"])])
    print(f"== 特徴の古さ（撮ってから方策が使うまで）中央値 {np.median(ages):.0f} ms・90% {np.percentile(ages, 90):.0f} ms")
    return 0


if __name__ == "__main__":
    sys.exit(main())
