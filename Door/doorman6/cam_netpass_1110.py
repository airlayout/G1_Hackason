"""11-10: vision_dump の枚（網に入れるのと同じ 10 Hz・カラー 192×108・深度 54×96・扉の頭への手掛かり 9・扉の値の正解 17）を、
RGBD の網（pretrain_rgbd_1106_1117.pt の encoder＝ResNet18 のカラーの枝＋小さな CNN の深度の枝→特徴 128、head＝MLP→扉の値 17）に
通し、扉の値の当てを netpass.npz に書く。実機（PC2 の door_vision_node.py）と同じ順の計算。深度の揺れは足さない（描いたままの深度）。

使い方（OMEN・VIRAL6_vis で）: PYTHONPATH=. python cam_netpass_1110.py <vision_dump の置き場> <網の重み .pt>
"""
from __future__ import annotations

import glob
import sys

import numpy as np
import torch

from gr00t.rl.envs.door.door_vision_feat import DOOR_TRUTH_NAMES, DoorHead, load_encoder


def main() -> None:
    d, path = sys.argv[1], sys.argv[2]
    fs = sorted(glob.glob(f"{d}/shard_*.npz"))
    if not fs:
        raise SystemExit(f"記録が無い: {d}")
    z = {k: np.concatenate([np.load(f)[k] for f in fs]) for k in np.load(fs[0]).files}
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    enc = load_encoder(path, 128, dev)
    head = DoorHead(128).to(dev).eval()
    head.load_state_dict(torch.load(path, map_location="cpu", weights_only=False)["head"])
    est, feats = [], []
    with torch.no_grad():
        for i in range(0, len(z["rgb"]), 256):
            rgb = torch.as_tensor(z["rgb"][i : i + 256], device=dev)
            depth = torch.as_tensor(z["depth"][i : i + 256], device=dev).float()
            aux = torch.as_tensor(z["aux"][i : i + 256], device=dev).float()
            f = enc(rgb, depth)
            feats.append(f.cpu().numpy())
            est.append(head(f, aux).cpu().numpy())
    est, feats = np.concatenate(est), np.concatenate(feats)
    truth = z["truth"]
    names = list(DOOR_TRUTH_NAMES)
    out = {k: z[k] for k in z if k not in ("rgb", "depth")}
    np.savez(f"{d}/netpass.npz", est=est, feat=feats, names=np.array(names), **out)
    err = np.abs(est - truth)
    for k in ("angle", "pos_x", "pos_y"):
        j = names.index(k)
        scale = 57.2958 if k == "angle" else 100.0
        unit = "°" if k == "angle" else "cm"
        print(f"[netpass] {k}: 誤差の中央 {np.median(err[:, j]) * scale:.1f} {unit}・90% 点 {np.percentile(err[:, j], 90) * scale:.1f} {unit}"
              f"（{len(est)} 枚）")


if __name__ == "__main__":
    main()
