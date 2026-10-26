"""実験 A（10-21）: V2 に扉の寸法（幅・レバーの高さ・バックセット）の出力を足し、学習に使わなかった扉で当たるかを確かめる。

なぜ: 生徒は扉の寸法を観測で受け取る（door_state_sensor.geometry）。今は実機で巻き尺の値を入れているが、
画像から寸法も出せれば、測っていない扉にも使える（ユーザーの提案、10-21）。

正解: V2 の学習の記録（v2data/pitch*、records/v2dagger_1016/v2/vision）は、同じ回の records/static.npz（親の static.npz）に
台ごとの寸法を残している。画像の env の番号で引く。扉は 96 種類（記録 6 本 × 16 台）。
検証: 扉ごとに K 個に分け（--fold/--folds）、学習に使わなかった扉の画像で測る（同じ扉の別の画像で測ると丸暗記でも当たる）。
読み方: 寸法は扉ごとに変わらないので、近づく間の推定の中央値を 1 つにまとめた「回ごと」「扉ごと」の誤差が本番の使い方。
  生徒は学習で σ 1 cm の寸法の誤差を混ぜていたので、扉ごとの誤差 1 cm 以内なら十分（実験 B で線を決め直す）

モデル: door_vision_train.py の Estimator と同じ（ResNet18＋腰 3 関節・重力 → MLP）。出力を 4 → 7 に（位置・向き・角度は残す。
寸法を足して位置の推定が悪くならないかも見るため）。扉の高さは頭のカメラに写らない（床から約 1 m まで）ので出さない。

使い方（OMEN）: python door_vision_geom_train.py --dirs v2data/pitch32 ... records/v2dagger_1016/v2/vision --fold 0 --folds 5 --out v2model/geom_1021/fold0
"""
from __future__ import annotations

import argparse
import glob
import json
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision

from door_vision_train import augment

TARGETS = ("x", "y", "yaw", "angle", "width", "lever_h", "backset")
GEOM = slice(4, 7)
APPROACH_DIST = (0.8, 1.6)  # [m] 回ごとにまとめるときに使う距離（レバーが写るのは 1.0 m より遠く）


def static_path(d: str) -> Path:
    p = Path(d) / "records" / "static.npz"
    return p if p.exists() else Path(d).parent / "static.npz"


def load_all(dirs: list[str]) -> dict:
    """画像は先に入れ物を確保して詰める（つなげると一時的にメモリが倍になる。画像だけで約 3.9 GB）"""
    files = [(run, f) for run, d in enumerate(dirs) for f in sorted(glob.glob(f"{d}/shard_*.npz"))]
    if not files:
        raise FileNotFoundError(f"shard_*.npz が無い: {dirs}")
    sizes = []
    for _, f in files:
        with np.load(f) as z:
            sizes.append(len(z["env"]))
    n = int(sum(sizes))
    with np.load(files[0][1]) as z:
        h, w = z["img"].shape[1:]
    out = {"img": np.empty((n, h, w), np.uint8), "proprio": np.empty((n, 6), np.float32), "y": np.empty((n, 7), np.float32),
           "door": np.empty(n, np.int64), "group": np.empty(n, np.int64), "stage": np.empty(n, np.int64)}
    geo = []
    for d in dirs:
        s = np.load(static_path(d))
        geo.append(np.stack([s["door_width"], s["door_handle_height"], s["door_handle_width"]], 1).astype(np.float32))
    i = 0
    for (run, f), k in zip(files, sizes):
        with np.load(f) as z:
            env = z["env"].astype(np.int64)
            out["img"][i:i + k] = z["img"]
            out["proprio"][i:i + k] = np.concatenate([z["waist"], z["gravity"]], axis=1)
            out["y"][i:i + k, :3] = z["pose"]
            out["y"][i:i + k, 3] = z["angle"]
            out["y"][i:i + k, 4:7] = geo[run][env]
            out["door"][i:i + k] = run * 100 + env
            out["group"][i:i + k] = run * 100000 + env * 1000 + z["episode"].astype(np.int64)
            out["stage"][i:i + k] = z["stage"]
        i += k
    return out


class GeomEstimator(nn.Module):
    """forward(gray_u8 [N, H, W], proprio [N, 6]) → [N, 7]（x, y [m], yaw, angle [rad], 幅・レバーの高さ・バックセット [m]）"""

    def __init__(self, y_mean: torch.Tensor, y_std: torch.Tensor, p_mean: torch.Tensor, p_std: torch.Tensor) -> None:
        super().__init__()
        net = torchvision.models.resnet18(weights=torchvision.models.ResNet18_Weights.IMAGENET1K_V1)
        net.fc = nn.Identity()
        self.backbone = net
        self.head = nn.Sequential(nn.Linear(512 + 6, 256), nn.ReLU(), nn.Linear(256, 128), nn.ReLU(), nn.Linear(128, len(TARGETS)))
        for name, v in (("y_mean", y_mean), ("y_std", y_std), ("p_mean", p_mean), ("p_std", p_std)):
            self.register_buffer(name, v)

    def normalized(self, img01: torch.Tensor, proprio: torch.Tensor) -> torch.Tensor:
        x = ((img01 - 0.45) / 0.225).unsqueeze(1).expand(-1, 3, -1, -1)
        return self.head(torch.cat([self.backbone(x), (proprio - self.p_mean) / self.p_std], dim=1))

    def forward(self, gray_u8: torch.Tensor, proprio: torch.Tensor) -> torch.Tensor:
        return self.normalized(gray_u8.float() / 255.0, proprio) * self.y_std + self.y_mean


def abs_err(pred: np.ndarray, y: np.ndarray) -> np.ndarray:
    """[N, 7] の絶対誤差（長さは cm、yaw と angle は度）"""
    e = np.abs(pred - y)
    e[:, 2] = np.abs(np.degrees(np.arctan2(np.sin(pred[:, 2] - y[:, 2]), np.cos(pred[:, 2] - y[:, 2]))))
    e[:, 3] = np.degrees(e[:, 3])
    e[:, [0, 1, 4, 5, 6]] *= 100.0
    return e


def stats(x: np.ndarray) -> dict:
    return {"mae": float(x.mean()), "p50": float(np.median(x)), "p90": float(np.percentile(x, 90)), "max": float(x.max())}


def summarize(pred: np.ndarray, data: dict, idx: np.ndarray) -> dict:
    y, st = data["y"][idx], data["stage"][idx]
    e = abs_err(pred, y)
    out = {"n": int(len(idx)), "per_image": {n: stats(e[:, j]) for j, n in enumerate(TARGETS)}}
    dist = np.hypot(y[:, 0], y[:, 1])
    out["by_distance"] = {}
    for lo, hi in ((0.0, 0.5), (0.5, 0.8), (0.8, 1.0), (1.0, 1.3), (1.3, 1.6), (1.6, 9.0)):
        m = (dist >= lo) & (dist < hi)
        if m.sum():
            out["by_distance"][f"{lo}-{hi}m"] = {"n": int(m.sum()), **{n: float(e[m, j].mean()) for j, n in enumerate(TARGETS)}}
    # 回ごと: 近づく間（区間 0〜3）で距離が APPROACH_DIST の画像の推定の中央値を、その回の寸法の推定とする
    groups, doors = data["group"][idx], data["door"][idx]
    per_ep, per_ep_door = [], []
    for gid in np.unique(groups):
        m = (groups == gid) & (st <= 3) & (dist >= APPROACH_DIST[0]) & (dist < APPROACH_DIST[1])
        if m.sum() < 3:
            continue
        est = np.median(pred[m][:, GEOM], axis=0)
        per_ep.append(np.abs(est - y[m][0, GEOM]) * 100.0)
        per_ep_door.append(doors[m][0])
    if per_ep:
        pe = np.array(per_ep)
        out["per_episode"] = {"n": len(pe), **{n: stats(pe[:, j]) for j, n in enumerate(TARGETS[4:])}}
        pd_ = np.array(per_ep_door)
        door_err = np.array([pe[pd_ == d].mean(0) for d in np.unique(pd_)])  # 扉ごと（その扉の回の平均）
        out["per_door"] = {"n": len(door_err), **{n: stats(door_err[:, j]) for j, n in enumerate(TARGETS[4:])}}
    return out


@torch.no_grad()
def predict(model: GeomEstimator, data: dict, idx: np.ndarray, dev, bs: int = 256) -> np.ndarray:
    model.eval()
    out = []
    for i in range(0, len(idx), bs):
        j = idx[i:i + bs]
        with torch.autocast("cuda", dtype=torch.bfloat16, enabled=dev.type == "cuda"):
            out.append(model(torch.from_numpy(data["img"][j]).to(dev), torch.from_numpy(data["proprio"][j]).to(dev)).float().cpu().numpy())
    return np.concatenate(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dirs", nargs="+", required=True)
    ap.add_argument("--fold", type=int, required=True)
    ap.add_argument("--folds", type=int, default=5)
    ap.add_argument("--out", required=True)
    ap.add_argument("--epochs", type=int, default=8)
    ap.add_argument("--bs", type=int, default=128)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--geom-weight", type=float, default=1.0, help="寸法 3 つの損失の重み（位置・向き・角度は 1）")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(a.seed)
    t0 = time.time()
    data = load_all(a.dirs)
    doors = np.unique(data["door"])
    rng = np.random.default_rng(a.seed)
    order = rng.permutation(doors)
    val_doors = set(np.array_split(order, a.folds)[a.fold].tolist())
    is_val = np.isin(data["door"], list(val_doors))
    tr, va = np.nonzero(~is_val)[0], np.nonzero(is_val)[0]
    print(f"[geom] 扉 {len(doors)} 種類のうち検証 {len(val_doors)}（fold {a.fold}/{a.folds}）・学習 {len(tr)} 枚・検証 {len(va)} 枚・"
          f"読み込み {time.time() - t0:.0f} s", flush=True)

    y = torch.from_numpy(data["y"][tr])
    p = torch.from_numpy(data["proprio"][tr])
    model = GeomEstimator(y.mean(0), y.std(0).clamp(min=1e-3), p.mean(0), p.std(0).clamp(min=1e-3)).to(dev)
    opt = torch.optim.AdamW(model.parameters(), lr=a.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=a.lr, total_steps=a.epochs * (len(tr) // a.bs), pct_start=0.1)
    g = torch.Generator(device=dev).manual_seed(a.seed)
    imgs, ys, ps = torch.from_numpy(data["img"]), torch.from_numpy(data["y"]), torch.from_numpy(data["proprio"])
    w = torch.tensor([1.0, 1.0, 1.0, 1.0] + [a.geom_weight] * 3, device=dev)
    history = []
    for ep in range(a.epochs):
        model.train()
        perm = rng.permutation(tr)
        tot, cnt = 0.0, 0
        for i in range(0, len(perm) - a.bs + 1, a.bs):
            j = torch.from_numpy(perm[i:i + a.bs])
            img = augment(imgs[j].to(dev, non_blocking=True).float() / 255.0, g)
            target = (ys[j].to(dev) - model.y_mean) / model.y_std
            with torch.autocast("cuda", dtype=torch.bfloat16, enabled=dev.type == "cuda"):
                pred = model.normalized(img, ps[j].to(dev))
            loss = (F.smooth_l1_loss(pred.float(), target, beta=0.1, reduction="none") * w).mean() / w.mean()
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step()
            sched.step()
            tot += float(loss) * len(j)
            cnt += len(j)
        v = summarize(predict(model, data, va, dev), data, va)
        pi = v["per_image"]
        history.append({"epoch": ep + 1, "train_loss": tot / cnt, "val_mae": {k: pi[k]["mae"] for k in TARGETS},
                        "per_door_mae": {k: v.get("per_door", {}).get(k, {}).get("mae") for k in TARGETS[4:]}})
        pdm = history[-1]["per_door_mae"]
        print(f"[geom] epoch {ep + 1}/{a.epochs} loss {tot / cnt:.4f}  1 枚ごと: x {pi['x']['mae']:.1f}・y {pi['y']['mae']:.1f} cm・"
              f"幅 {pi['width']['mae']:.1f}・レバーの高さ {pi['lever_h']['mae']:.1f}・バックセット {pi['backset']['mae']:.1f} cm  "
              f"扉ごと: 幅 {pdm['width'] or float('nan'):.1f}・レバーの高さ {pdm['lever_h'] or float('nan'):.1f}・"
              f"バックセット {pdm['backset'] or float('nan'):.1f} cm（{time.time() - t0:.0f} s）", flush=True)
    pv = predict(model, data, va, dev)
    report = {"dirs": a.dirs, "fold": a.fold, "folds": a.folds, "val_doors": sorted(int(d) for d in val_doors),
              "history": history, "val": summarize(pv, data, va)}
    np.savez_compressed(out / "pred_val.npz", pred=pv, y=data["y"][va], stage=data["stage"][va], door=data["door"][va],
                        group=data["group"][va], idx=va)
    (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=1))
    torch.save(model.state_dict(), out / "state_dict.pt")
    print(f"[geom] 書き出した: {out}（{time.time() - t0:.0f} s）", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
