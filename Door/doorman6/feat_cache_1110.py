"""11-10 手順 R2 の続き: 画像の網（DoorRGBDEncoder）を固めたまま、シムと実物の全部の枚の特徴を一度だけ出して貯める。

網を固めて扉の頭だけを学び直す（head_adapt_1110.py）ための下ごしらえ。特徴を貯めれば頭の学習は CPU で数秒になり、
学習中の s2f（GPU）を止めずに回せる。束ごとに読んで書く（RAM は 1 束ぶん・約 0.2 GB）。

- シムの束（ds_dep/s*/frames/shard_*.npz）: 深度には D435 の型の揺れと欠けを、束ごとに決まった種で 1 回だけ引く
  （door_vision_pretrain.py の evaluate と同じ。学習の毎回の引き直しの代わり）。扉の番号は door_vision_pretrain.load_frames と同じ
  （束の並びの番号 × 100000 ＋ 台の番号）ので、確かめの扉の分け方も同じになる
- 実物の束（real_shard_1110.py の npz）: 深度は記録のまま
- 出力（1 束 1 ファイル）: feat (n, 128) float32・aux・truth・stage・door_id

使い方（OMEN・VIRAL6_vis で PYTHONPATH=.）:
  python feat_cache_1110.py --init init_e2b.pt --sim <ds_dep>/s*/frames --real real_183336.npz real_185929.npz --out <置き場>
"""
from __future__ import annotations

import argparse
import glob
import os
import time

import numpy as np
import torch

from gr00t.rl.envs.door.door_vision_feat import DoorRGBDEncoder, depth_noise

BATCH = 64


def encode(enc: DoorRGBDEncoder, rgb: np.ndarray, depth: np.ndarray, noise_seed: int | None) -> np.ndarray:
    """カラー [n, 108, 192, 3]・深度 [n, 54, 96] → 特徴 [n, 128]（noise_seed があれば深度に揺れと欠けを引く）"""
    gen = None
    if noise_seed is not None:
        gen = torch.Generator()
        gen.manual_seed(noise_seed)
    out = np.zeros((len(rgb), enc.feat_dim), np.float32)
    with torch.no_grad():
        for i in range(0, len(rgb), BATCH):
            d = torch.from_numpy(depth[i : i + BATCH].astype(np.float32))
            if gen is not None:
                d = depth_noise(d, gen)
            off = torch.zeros(len(d), dtype=torch.bool)
            out[i : i + BATCH] = enc(torch.from_numpy(rgb[i : i + BATCH]), d, off).numpy()
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--init", required=True, help="網の重み（door_vision_pretrain.py の出力の形）")
    ap.add_argument("--sim", nargs="*", default=[], help="シムの束の frames の置き場（door_vision_pretrain.py の --data と同じ並びで）")
    ap.add_argument("--real", nargs="*", default=[], help="実物の束（real_shard_1110.py の npz）")
    ap.add_argument("--out", required=True)
    ap.add_argument("--threads", type=int, default=6)
    a = ap.parse_args()
    torch.set_num_threads(a.threads)
    os.makedirs(a.out, exist_ok=True)
    ck = torch.load(a.init, map_location="cpu", weights_only=False)
    enc = DoorRGBDEncoder(128, pretrained=False)
    enc.load_state_dict(ck["encoder"])
    enc.eval()
    t0 = time.time()
    n_all = 0
    for bi, d in enumerate(a.sim):
        for k, f in enumerate(sorted(glob.glob(os.path.join(d, "shard_*.npz")))):
            name = os.path.join(a.out, f"sim_{os.path.basename(os.path.dirname(d.rstrip('/')))}_{k:02d}.npz")
            if os.path.exists(name):
                continue
            with np.load(f) as z:
                feat = encode(enc, z["rgb"], z["depth"], noise_seed=bi * 1000 + k)
                door_id = bi * 100000 + z["env_id"].astype(np.int64)
                np.savez(name, feat=feat, aux=z["aux"], truth=z["truth"], stage=z["stage"], door_id=door_id)
            n_all += len(feat)
            print(f"[feat] {name}: {len(feat)} 枚（累計 {n_all}・{n_all / (time.time() - t0):.0f} 枚/秒）", flush=True)
    for f in a.real:
        name = os.path.join(a.out, "real_" + os.path.basename(f))
        with np.load(f) as z:
            feat = encode(enc, z["rgb"], z["depth"], noise_seed=None)
            np.savez(name, feat=feat, aux=z["aux"], truth=z["truth"], stage=z["stage"], seq=z["seq"], t_capture=z["t_capture"])
        print(f"[feat] {name}: {len(feat)} 枚", flush=True)
    print(f"[feat] 終わり {time.time() - t0:.0f} 秒", flush=True)


if __name__ == "__main__":
    main()
