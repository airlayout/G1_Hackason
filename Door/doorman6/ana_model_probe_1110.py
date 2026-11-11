"""11-10: 学び直した画像の網（door_vision_pretrain.py の出力 .pt）の片寄りを、シムと実物の同じような立ち位置で比べる（CPU）。

1. シム: 扉が閉じ、扉の 0.8〜1.0 m・横 ±0.15 m にいる枚を、扉の向き（網の側・鏡写し）の帯ごとに集め、前・左・向きの片寄りを出す
   （e2b の頭はシムでもこの立ち位置で左に 2〜3 cm 片寄っていた。学び直した網に残っているか）
2. 実物: 実物の束（real_shard_1110.py）の時刻の範囲ごとに、そのままと、画面の左下を隠した時（ana_mask_1110.py と同じ隠し方）の片寄り
   （18:59 は左下の近い物＝構えた左手の見込みに引っ張られていた。写り込む物の真似で学んだ網でも引っ張られるか）
値は網の側（鏡写し）。把手が左の扉の「左」は実の向きで符号が逆になる（実の左の片寄り = −網の側の左の片寄り）。

使い方（OMEN・VIRAL6_vis で PYTHONPATH=.）:
  python ana_model_probe_1110.py --pt init_e2b.pt r2b_c2.pt --sim <ds_dep>/s*/frames --real real_185929_S.npz:2740:3160 ...
"""
from __future__ import annotations

import argparse
import glob
import os

import numpy as np
import torch

from gr00t.rl.envs.door.door_vision_feat import DoorHead, DoorRGBDEncoder, depth_noise

DEPTH_BOX = (slice(36, 54), slice(10, 44))  # ana_mask_1110.py と同じ
COLOR_BOX = (slice(72, 108), slice(20, 88))
FLOOR_REF = (slice(72, 108), slice(90, 150))
YAW_BINS = ((-12, -6), (-6, -2), (-2, 2), (2, 6), (6, 12))


def load_model(path: str):
    ck = torch.load(path, map_location="cpu", weights_only=False)
    enc = DoorRGBDEncoder(128, pretrained=False)
    enc.load_state_dict(ck["encoder"])
    head = DoorHead(128)
    head.load_state_dict(ck["head"])
    return enc.eval(), head.eval()


def run(enc, head, rgb, depth, aux, noise_seed=None) -> np.ndarray:
    out = []
    gen = None
    if noise_seed is not None:
        gen = torch.Generator()
        gen.manual_seed(noise_seed)
    with torch.no_grad():
        for i in range(0, len(rgb), 64):
            d = torch.from_numpy(depth[i : i + 64].astype(np.float32))
            if gen is not None:
                d = depth_noise(d, gen)
            f = enc(torch.from_numpy(rgb[i : i + 64]), d, torch.zeros(len(d), dtype=torch.bool))
            out.append(head(f, torch.from_numpy(aux[i : i + 64]).float()).numpy())
    return np.concatenate(out)


def yaw_m(t: np.ndarray) -> np.ndarray:
    return np.degrees(np.arctan2(t[:, 5], t[:, 4]))


def bias(p: np.ndarray, t: np.ndarray) -> str:
    return (f"前 {100 * np.median(p[:, 1] - t[:, 1]):+5.1f} 左(網) {100 * np.median(p[:, 2] - t[:, 2]):+5.1f} cm "
            f"向き(網) {np.median(yaw_m(p) - yaw_m(t)):+5.1f}°")


def mask(rgb: np.ndarray, depth: np.ndarray):
    rgb, depth = rgb.copy(), depth.copy()
    depth[:, DEPTH_BOX[0], DEPTH_BOX[1]] = 0
    for k in range(len(rgb)):
        rgb[k][COLOR_BOX] = np.median(rgb[k][FLOOR_REF].reshape(-1, 3), axis=0).astype(np.uint8)
    return rgb, depth


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pt", nargs="+", required=True)
    ap.add_argument("--sim", nargs="*", default=[])
    ap.add_argument("--real", nargs="*", default=[], help="実物の束:t0:t1")
    ap.add_argument("--every", type=int, default=10, help="実物の何枚に 1 枚を使うか")
    ap.add_argument("--threads", type=int, default=6)
    a = ap.parse_args()
    torch.set_num_threads(a.threads)
    # シムの枚を先に選ぶ（全部の束から、閉じた扉の 0.9 m 付近だけ）
    sim = {k: [] for k in ("rgb", "depth", "aux", "truth")}
    for d in a.sim:
        for f in sorted(glob.glob(os.path.join(d, "shard_*.npz"))):
            with np.load(f) as z:
                t = z["truth"]
                sel = ((np.abs(t[:, 1] - 0.9) < 0.1) & (np.abs(t[:, 2]) < 0.15) & (np.degrees(t[:, 0]) < 1.0)
                       & (np.abs(yaw_m(t)) < 12))
                if sel.any():
                    for k in sim:
                        sim[k].append(z[k][sel])
    sim = {k: np.concatenate(v) for k, v in sim.items()} if sim["truth"] else None
    reals = []
    for spec in a.real:
        path, t0, t1 = spec.rsplit(":", 2)
        with np.load(path) as z:
            sel = np.nonzero((z["t_capture"] >= float(t0)) & (z["t_capture"] <= float(t1)))[0][:: a.every]
            reals.append((f"{os.path.basename(path)} t {t0}〜{t1}", {k: z[k][sel] for k in ("rgb", "depth", "aux", "truth")}))
    for pt in a.pt:
        enc, head = load_model(pt)
        print(f"##### {os.path.basename(pt)}", flush=True)
        if sim is not None:
            p = run(enc, head, sim["rgb"], sim["depth"], sim["aux"], noise_seed=0)
            yw = yaw_m(sim["truth"])
            for lo, hi in YAW_BINS:
                s = (yw >= lo) & (yw < hi)
                if s.sum():
                    print(f"  シム 0.9 m・向き(網) {lo:+d}〜{hi:+d}° {int(s.sum()):4d} 枚: {bias(p[s], sim['truth'][s])}", flush=True)
        for name, r in reals:
            p = run(enc, head, r["rgb"], r["depth"], r["aux"])
            rgb_m, dep_m = mask(r["rgb"], r["depth"])
            pm = run(enc, head, rgb_m, dep_m, r["aux"])
            print(f"  実物 {name} {len(p)} 枚: そのまま {bias(p, r['truth'])}／左下を隠す {bias(pm, r['truth'])}", flush=True)


if __name__ == "__main__":
    main()
