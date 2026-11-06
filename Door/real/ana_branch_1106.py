"""11-06: 実物の画像で、画像の網のどの枝（カラー・深度）が扉の距離・左右を決めているかを調べる（D4 の答え合わせの続き）。

カメラ側の記録（vision_*.npz: 網に入れたカラー 192×108・深度 54×96・特徴）と走らせ役の記録（door_stock_*.npz: 観測 151・頭の重力・
特徴の番号）を特徴の番号で突き合わせ、同じ 1 枚を入れ方を変えて網 → 扉の頭（追う段は通さない＝今の 1 枚が言う値）に通す。
入れ方: 記録の特徴・カラー＋深度・深度だけ（カラーを落とす）・カラーだけ（深度を 0＝測れない）・深度を k 倍。
深度を縮めて網の「前」も縮めば、網は深度で距離を読んでいる。

使い方（OMEN・doorman6 の Python）:
  python ana_branch_1106.py --vision <vision_*.npz> --stock <door_stock_*.npz> --policy door_vision_e2b.npz \
      --onnx door_vision_e2b_encoder.onnx --win 1150:1235:0.883:0.080 --win 1540:1610:0.333:0.070
  --win は t_capture（PC2 の monotonic 秒）の範囲と巻き尺の 前・左 [m]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import door_vision_core as dvc  # noqa: E402
import door_vision_encoder as dve  # noqa: E402

VARIANTS: Tuple[Tuple[str, bool, float], ...] = (  # (名前, カラーを落とす, 深度の倍率。0 は深度を消す)
    ("カラー＋深度", False, 1.0),
    ("深度だけ", True, 1.0),
    ("カラーだけ", False, 0.0),
    ("カラー＋深度×0.7", False, 0.7),
    ("カラー＋深度×1.4", False, 1.4),
    ("深度だけ×0.7", True, 0.7),
)


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vision", required=True)
    ap.add_argument("--stock", required=True)
    ap.add_argument("--policy", required=True)
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--win", action="append", required=True, help="t0:t1:前:左")
    ap.add_argument("--every", type=int, default=3, help="窓の中の何枚に 1 枚を使うか")
    return ap.parse_args(argv)


def door_xy(p: dc.Params, vp: dvc.VisionParams, obs: np.ndarray, feat: np.ndarray, hg_m: np.ndarray) -> np.ndarray:
    """扉の頭（追う段なし）→ 実の向きの 前・左 [m]"""
    d17 = dvc.door_values(vp, obs, feat, hg_m)
    _, pos = dvc.door_real(p, vp, dvc.insert_door(vp, obs, d17))
    return pos[:2]


def main(argv=None) -> int:
    a = parse_args(argv)
    p = dc.load_params(a.policy, mirror=True)
    vp = dvc.load_vision(a.policy, p)
    enc = dve.OrtRGBDEncoder(a.onnx, 4)
    # npz は項目を取るたびに展開し直すので、使う配列を先に 1 回だけ読む
    zv = np.load(a.vision, allow_pickle=True)
    v = {k: zv[k] for k in ("rgb", "depth", "seq", "t_capture", "feat")}
    zs = np.load(a.stock, allow_pickle=True)
    s = {k: zs[k] for k in ("vis_seq", "vis_hg", "obs")}
    row_of: Dict[int, int] = {}
    for i, k in enumerate(s["vis_seq"]):
        if np.isfinite(k) and int(k) not in row_of and np.isfinite(s["vis_hg"][i]).all():
            row_of[int(k)] = i
    print(f"[ana] 画像 {len(v['seq'])} 枚・走らせ役の行と突き合えた番号 {len(row_of)}", flush=True)
    for w in a.win:
        t0, t1, fwd, left = (float(x) for x in w.split(":"))
        ks = [j for j, t in enumerate(v["t_capture"]) if t0 <= t <= t1 and int(v["seq"][j]) in row_of][:: a.every]
        if not ks:
            print(f"[ana] 窓 {w}: 画像が無い", flush=True)
            continue
        res: Dict[str, List[np.ndarray]] = {"記録の特徴": [], "記録の扉（追う段込み）": []}
        res.update({n: [] for n, _, _ in VARIANTS})
        diff = []
        for j in ks:
            i = row_of[int(v["seq"][j])]
            obs, hg = s["obs"][i], s["vis_hg"][i]
            x_rgb = dve.preprocess_rgb(v["rgb"][j])
            d = v["depth"][j].astype(np.float32)
            res["記録の特徴"].append(door_xy(p, vp, obs, v["feat"][j], hg))
            _, pos = dvc.door_real(p, vp, obs)
            res["記録の扉（追う段込み）"].append(pos[:2])
            for name, off, k in VARIANTS:
                dd = np.clip(d * k, 0.0, dve.DEPTH_MAX) if k > 0 else np.zeros_like(d)
                f = enc(x_rgb, dve.depth_input(dd), off)
                if name == "カラー＋深度":
                    diff.append(float(np.abs(f - v["feat"][j]).max()))
                res[name].append(door_xy(p, vp, obs, f, hg))
        print(f"\n[ana] 窓 t {t0:.0f}〜{t1:.0f}（{len(ks)} 枚）・巻き尺 前 {fwd:.3f} 左 {left:+.3f}・"
              f"深度の中央 {np.median(v['depth'][ks][v['depth'][ks] > 0]):.2f} m・記録の特徴との差の最大 {max(diff):.1e}", flush=True)
        for name, xs in res.items():
            m = np.mean(xs, axis=0)
            sd = np.std(xs, axis=0)
            print(f"  {name:<18} 前 {m[0]:.3f}±{sd[0]:.3f}（差 {100 * (m[0] - fwd):+5.1f} cm）  "
                  f"左 {m[1]:+.3f}±{sd[1]:.3f}（差 {100 * (m[1] - left):+5.1f} cm）", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
