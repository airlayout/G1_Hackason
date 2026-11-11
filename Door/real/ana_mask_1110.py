"""11-10: 実物の画の一部を隠して画像の網（実機の e2b）に通し直し、その部分が扉の値の外れを生んでいるかを確かめる。

18:59 の立っていた所（0.91 m・+4.4°）で、網は前 −10 cm・向き −8° 外す。シムの同じ立ち位置では外さない（前 −2.6 cm・向き +0.8°）。
深度を見ると、18:59 にだけ画面の左下に 0.5 m より近い物（構えた機体の左手の見込み）が写っている。そこを隠して外れが戻れば手が原因。
隠し方: 深度は 0（測れない）、カラーは同じ画の床の色（右下の中央値）で塗る。カラーだけ・深度だけも分けて試す。
比べとして、手の写っていない 18:33 の地点 0 にも同じ隠し方をする（外れが大きく動かなければ、隠すこと自体の影響は小さい）。

使い方（OMEN の real_ana_1106/）:
  python ana_mask_1110.py --vision <vision_*.npz> --stock <door_stock_*.npz> --truth <truth_*.npz> --win 2740:3160 --label 18:59
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import door_vision_core as dvc  # noqa: E402
import door_vision_encoder as dve  # noqa: E402
from real_eval_1110 import door_yaw_deg, stock_rows_by_seq  # noqa: E402
from real_truth_1110 import mirror_door17  # noqa: E402

# 隠す所（深度 54×96 とカラー 108×192 の行・列）。18:59 の深度で 0.75 m より近い塊を少し広めに囲む
DEPTH_BOX = (slice(36, 54), slice(10, 44))
COLOR_BOX = (slice(72, 108), slice(20, 88))
FLOOR_REF = (slice(72, 108), slice(90, 150))  # 塗る色を取る床（右下）
MODES = ("そのまま", "左下を隠す", "深度だけ隠す", "カラーだけ隠す")


def masked(rgb: np.ndarray, depth: np.ndarray, mode: str):
    rgb, d = rgb.copy(), depth.astype(np.float32).copy()
    if mode in ("左下を隠す", "深度だけ隠す"):
        d[DEPTH_BOX] = 0.0
    if mode in ("左下を隠す", "カラーだけ隠す"):
        rgb[COLOR_BOX] = np.median(rgb[FLOOR_REF].reshape(-1, 3), axis=0).astype(np.uint8)
    return rgb, d


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vision", required=True)
    ap.add_argument("--stock", required=True)
    ap.add_argument("--truth", required=True)
    ap.add_argument("--policy", default="door_vision_e2b.npz")
    ap.add_argument("--onnx", default="door_vision_e2b_encoder.onnx")
    ap.add_argument("--win", required=True, help="t_capture の範囲 t0:t1")
    ap.add_argument("--every", type=int, default=15)
    ap.add_argument("--label", default="")
    a = ap.parse_args(argv)
    p = dc.load_params(a.policy, mirror=True)
    vp = dvc.load_vision(a.policy, p)
    enc = dve.OrtRGBDEncoder(a.onnx, 4)
    zv = np.load(a.vision, allow_pickle=True)
    seq, t = zv["seq"], zv["t_capture"]
    truth = np.load(a.truth)["truth17_real"]
    row_of, s = stock_rows_by_seq(a.stock)
    t0, t1 = (float(x) for x in a.win.split(":"))
    js = [j for j in np.nonzero((t >= t0) & (t <= t1))[0] if int(seq[j]) in row_of and np.isfinite(truth[j]).all()][:: a.every]
    rgb_all, depth_all = zv["rgb"], zv["depth"]  # npz は項目を取るたびに全部を展開し直すので 1 回だけ読む
    for mode in MODES:
        err = []
        for j in js:
            rgb, d = masked(rgb_all[j], depth_all[j], mode)
            f = enc(dve.preprocess_rgb(rgb), dve.depth_input(d), False)
            i = row_of[int(seq[j])]
            pred = mirror_door17(p, vp, dvc.door_values(vp, s["obs"][i], f, s["vis_hg"][i]))
            tr = truth[j]
            err.append([100 * (pred[1] - tr[1]), 100 * (pred[2] - tr[2]), (door_yaw_deg(pred) - door_yaw_deg(tr) + 180) % 360 - 180])
        e = np.median(np.array(err), axis=0)
        print(f"[mask] {a.label:<14} {mode:<8} {len(js)} 枚: 片寄り 前 {e[0]:+.1f} cm 左 {e[1]:+.1f} cm 向き {e[2]:+.1f}°", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
