"""11-10 手順 R2 の続き: 画像の網を固めたまま、扉の頭（DoorHead）だけを、シムの特徴に実物の特徴を混ぜて学び直す（教師あり）。

R2（網の全体を学び直す）は、実物の画が止まった 3 か所しか無く、その画を覚え込んで別の立ち位置（18:59）の左の片寄りを
−3 → −10 cm に広げた。一方、今の網の左の片寄りは 18:33 の地点 0 と 18:59 でどちらも約 −3 cm で、シムと実物の差による一定の
ずれに見える。覚え込める余地を小さくして、一定のずれだけを直す:
  --mode head  頭の全部（3 層）
  --mode last  頭の最後の層だけ
  --mode bias  頭の出口のずれ（最後の層のバイアス）だけ ＝ 扉の値 17 にそれぞれ一定の値を足すだけ

特徴は feat_cache_1110.py が貯めたもの（網は固めたまま・シムの深度の揺れは束ごとに 1 回だけ引いた）。CPU で回る。
シムの確かめの扉の分け方は door_vision_pretrain.py と同じ（扉の番号・種 0・1 割）。

出力: <out>.pt（頭の重み・door_vision_pretrain.py の出力と同じ形で網は init のまま）と <out>_realval.npz（実物の確かめの 1 枚ごとの当て。
real_eval_1110.py --pred で距離の帯ごとの表にする）と <out>.json（外れの表）

使い方（OMEN・VIRAL6_vis で PYTHONPATH=.）:
  python head_adapt_1110.py --feat <feat の置き場> --init init_e2b.pt --train-real real_183336_S.npz \
      --val-real real_185929_S.npz real_183336_S.npz --mode last --out <出力>
  --train-real・--val-real は実物の束の名前（feat の置き場の real_<名前>）
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import time

import numpy as np
import torch

from gr00t.rl.envs.door.door_vision_feat import DOOR_TRUTH_NAMES, DOOR_TRUTH_TOL, DoorHead

UNIT = np.array([180.0 / math.pi] + [100.0] * 3 + [1.0] * 6 + [100.0] * 4 + [1.0] * 3)  # 度・cm・そのまま（pretrain の evaluate と同じ）


def load_feats(paths: list[str]) -> dict[str, np.ndarray]:
    parts = []
    for p in paths:
        with np.load(p) as z:
            parts.append({k: z[k] for k in z.files})
    keys = set.intersection(*(set(p) for p in parts))
    return {k: np.concatenate([p[k] for p in parts]) for k in keys}


def split_by_door(door_id: np.ndarray, val_frac: float = 0.1, seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """door_vision_pretrain.split_by_door と同じ（同じ扉が確かめに回る）"""
    doors = np.unique(door_id)
    rng = np.random.default_rng(seed)
    val_doors = rng.choice(doors, size=max(1, int(round(len(doors) * val_frac))), replace=False)
    is_val = np.isin(door_id, val_doors)
    return np.nonzero(~is_val)[0], np.nonzero(is_val)[0]


def aux_noise(aux: torch.Tensor, gen: torch.Generator) -> torch.Tensor:
    """door_vision_pretrain.aux_noise と同じ揺れ"""
    n = torch.randn(aux.shape, generator=gen)
    out = aux.clone()
    hg = aux[:, :3] + n[:, :3] * 0.02 + torch.randn(aux.shape[0], 3, generator=gen) * 0.015
    out[:, :3] = hg / hg.norm(dim=-1, keepdim=True).clamp_min(1e-6)
    out[:, 6:9] = aux[:, 6:9] + n[:, 6:9] * 0.01
    return out


def trainable(head: DoorHead, mode: str) -> list[torch.nn.Parameter]:
    for p in head.parameters():
        p.requires_grad_(False)
    last = head.net[-1]
    sel = {"head": list(head.parameters()), "last": [last.weight, last.bias], "bias": [last.bias]}[mode]
    for p in sel:
        p.requires_grad_(True)
    return sel


def predict(head: DoorHead, d: dict[str, np.ndarray], idx: np.ndarray | None = None) -> np.ndarray:
    idx = np.arange(len(d["truth"])) if idx is None else idx
    with torch.no_grad():
        return head(torch.from_numpy(d["feat"][idx]).float(), torch.from_numpy(d["aux"][idx]).float()).numpy()


def mae(pred: np.ndarray, truth: np.ndarray) -> dict[str, float]:
    return dict(zip(DOOR_TRUTH_NAMES, (np.abs(pred - truth).mean(0) * UNIT).tolist()))


def fmt(m: dict[str, float]) -> str:
    return (f"角度 {m['angle']:.2f}° 位置 {m['pos_x']:.1f}/{m['pos_y']:.1f}/{m['pos_z']:.1f} cm 向き {m['rot6d_0']:.3f} "
            f"幅 {m['width']:.1f} レバー高 {m['handle_height']:.1f} バックセット {m['backset']:.1f} cm")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--feat", required=True)
    ap.add_argument("--init", required=True)
    ap.add_argument("--train-real", nargs="*", default=[])
    ap.add_argument("--val-real", nargs="*", default=[])
    ap.add_argument("--mode", choices=("head", "last", "bias"), default="last")
    ap.add_argument("--real-frac", type=float, default=0.25)
    ap.add_argument("--epochs", type=int, default=4)
    ap.add_argument("--batch", type=int, default=256)
    ap.add_argument("--lr", type=float, default=1e-4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    torch.manual_seed(a.seed)
    t0 = time.time()
    sim = load_feats(sorted(glob.glob(os.path.join(a.feat, "sim_*.npz"))))
    tr, va = split_by_door(sim["door_id"])
    real = load_feats([os.path.join(a.feat, "real_" + n) for n in a.train_real]) if a.train_real else None
    vals = {n: load_feats([os.path.join(a.feat, "real_" + n)]) for n in a.val_real}
    print(f"[head] シム {len(sim['truth'])} 枚（学習 {len(tr)}・確かめ {len(va)}）・実物の学習 {0 if real is None else len(real['truth'])} 枚・"
          f"確かめ {[(n, len(v['truth'])) for n, v in vals.items()]}・{time.time() - t0:.0f} 秒", flush=True)

    ck = torch.load(a.init, map_location="cpu", weights_only=False)
    head = DoorHead(128)
    head.load_state_dict(ck["head"])
    params = trainable(head, a.mode)
    opt = torch.optim.Adam(params, lr=a.lr)
    tol = torch.tensor(DOOR_TRUTH_TOL)
    n_real = int(round(a.batch * a.real_frac)) if real is not None else 0
    sim_bs = a.batch - n_real
    gen = torch.Generator()
    gen.manual_seed(a.seed)
    rng = np.random.default_rng(a.seed)

    def report(tag: str) -> dict:
        res = {"sim_val": mae(predict(head, sim, va), sim["truth"][va])}
        line = [f"[head] {tag} シムの確かめ: {fmt(res['sim_val'])}"]
        for n, v in vals.items():
            res[f"real_val:{n}"] = mae(predict(head, v), v["truth"])
            line.append(f"[head] {tag} 実物 {n}: {fmt(res[f'real_val:{n}'])}")
        print("\n".join(line), flush=True)
        return res

    history = [{"epoch": -1, **report("始め")}]
    steps = max(1, len(tr) // sim_bs)
    for ep in range(a.epochs):
        perm = rng.permutation(tr)
        for s in range(steps):
            j = perm[s * sim_bs : (s + 1) * sim_bs]
            f = [sim["feat"][j]]
            x = [sim["aux"][j]]
            y = [sim["truth"][j]]
            if n_real:
                jr = rng.integers(0, len(real["truth"]), size=n_real)
                f.append(real["feat"][jr])
                x.append(real["aux"][jr])
                y.append(real["truth"][jr])
            feat = torch.from_numpy(np.concatenate(f)).float()
            aux = aux_noise(torch.from_numpy(np.concatenate(x)).float(), gen)
            yt = torch.from_numpy(np.concatenate(y)).float()
            loss = (((head(feat, aux) - yt) / tol) ** 2).mean()
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step()
        history.append({"epoch": ep, **report(f"ep {ep}")})

    torch.save({"encoder": ck["encoder"], "head": head.state_dict(), "encoder_kind": ck.get("encoder_kind", "rgbd"),
                "src": {"init": a.init, "head_adapt": vars(a)}}, a.out + ".pt")
    np.savez(a.out + "_realval.npz", names=np.array(list(vals)),
             **{f"pred:{n}": predict(head, v) for n, v in vals.items()}, **{f"truth:{n}": v["truth"] for n, v in vals.items()})
    with open(a.out + ".json", "w") as fh:
        json.dump({"args": vars(a), "history": history}, fh, indent=1, ensure_ascii=False)
    print(f"[head] 書いた: {a.out}.pt・_realval.npz・.json（{time.time() - t0:.0f} 秒）", flush=True)


if __name__ == "__main__":
    main()
