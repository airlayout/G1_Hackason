"""V2（10-16）: 頭のカメラの灰色の画像から扉の状態 [x, y, yaw, angle] を推定する小さいモデルを学習し、TorchScript で書き出す。

入力: exp_v2_collect_1016.sh が集めた shard_*.npz（img・pose・angle・waist・gravity・episode・env）
  x, y, yaw  扉の座標での pelvis の床の上の姿勢（原点＝開口の中心の床・+x＝押して入る向き）
  angle      扉の開いた角度 [rad]
モデル: ResNet18（ImageNet の重みから）→ 512 ＋ 関節で分かる値 6（腰 3・重力の向き 3）→ MLP → 4。出力は物理の単位
評価: 学習に使わない回（val）と、学習に使わない俯角（--test）。誤差を距離・区間・扉の角度で分けて出す

使い方（OMEN、Isaac Sim を止めてから）:
  ~/miniforge3/envs/doorman6/bin/python door_vision_train.py --train v2data/pitch32 v2data/pitch38 ... \\
      --test v2data/pitch41 --out v2model/run1 [--epochs 8]
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision

TARGETS = ("x", "y", "yaw", "angle")


def load_dirs(dirs: list[str]) -> dict:
    parts = []
    for run, d in enumerate(dirs):
        files = sorted(glob.glob(f"{d}/shard_*.npz"))
        if not files:
            raise FileNotFoundError(f"shard_*.npz が無い: {d}")
        for f in files:
            z = np.load(f)
            parts.append({
                "img": z["img"],
                "proprio": np.concatenate([z["waist"], z["gravity"]], axis=1).astype(np.float32),
                "y": np.concatenate([z["pose"], z["angle"][:, None]], axis=1).astype(np.float32),
                "group": (run * 100000 + z["env"] * 1000 + z["episode"]).astype(np.int64),
                "stage": z["stage"].astype(np.int64),
            })
    return {k: np.concatenate([p[k] for p in parts]) for k in parts[0]}


class Estimator(nn.Module):
    """forward(gray_u8 [N, H, W], proprio [N, 6]) → [N, 4]（x [m], y [m], yaw [rad], angle [rad]）"""

    def __init__(self, y_mean: torch.Tensor, y_std: torch.Tensor, p_mean: torch.Tensor, p_std: torch.Tensor) -> None:
        super().__init__()
        net = torchvision.models.resnet18(weights=torchvision.models.ResNet18_Weights.IMAGENET1K_V1)
        net.fc = nn.Identity()
        self.backbone = net
        self.head = nn.Sequential(nn.Linear(512 + 6, 256), nn.ReLU(), nn.Linear(256, 128), nn.ReLU(), nn.Linear(128, 4))
        self.register_buffer("y_mean", y_mean)
        self.register_buffer("y_std", y_std)
        self.register_buffer("p_mean", p_mean)
        self.register_buffer("p_std", p_std)

    def normalized(self, img01: torch.Tensor, proprio: torch.Tensor) -> torch.Tensor:
        x = ((img01 - 0.45) / 0.225).unsqueeze(1).expand(-1, 3, -1, -1)
        f = self.backbone(x)
        p = (proprio - self.p_mean) / self.p_std
        return self.head(torch.cat([f, p], dim=1))

    def forward(self, gray_u8: torch.Tensor, proprio: torch.Tensor) -> torch.Tensor:
        img = gray_u8.float() / 255.0
        return self.normalized(img, proprio) * self.y_std + self.y_mean


def augment(img: torch.Tensor, g: torch.Generator) -> torch.Tensor:
    """見た目のばらつき（実機の IR は描画と違うので強めに）。形は変えない（変えると正解の姿勢がずれる）"""
    n = img.shape[0]
    dev = img.device
    r = lambda lo, hi: lo + (hi - lo) * torch.rand(n, 1, 1, device=dev, generator=g)  # noqa: E731
    m = img.mean(dim=(1, 2), keepdim=True)
    img = (img - m) * r(0.6, 1.5) + m * r(0.5, 1.7)  # コントラストと明るさ
    img = img.clamp(0, 1) ** r(0.7, 1.4)  # ガンマ
    blur = torch.rand(n, device=dev, generator=g) < 0.3
    if blur.any():
        b = F.avg_pool2d(img[blur].unsqueeze(1), 3, stride=1, padding=1).squeeze(1)
        img = img.clone()
        img[blur] = b
    img = img + torch.randn(img.shape, device=dev, generator=g) * r(0.0, 0.06)
    return img.clamp(0, 1)


def errors(pred: np.ndarray, y: np.ndarray) -> np.ndarray:
    """[N, 4] の絶対誤差（x, y は cm、yaw と angle は度）"""
    e = np.abs(pred - y)
    e[:, 2] = np.abs(np.degrees(np.arctan2(np.sin(pred[:, 2] - y[:, 2]), np.cos(pred[:, 2] - y[:, 2]))))
    e[:, 3] = np.degrees(e[:, 3])
    e[:, :2] *= 100.0
    return e


def summarize(pred: np.ndarray, data: dict, idx: np.ndarray) -> dict:
    y = data["y"][idx]
    e = errors(pred, y)
    out = {"n": int(len(idx))}
    for j, name in enumerate(TARGETS):
        out[name] = {"mae": float(e[:, j].mean()), "p50": float(np.percentile(e[:, j], 50)), "p95": float(np.percentile(e[:, j], 95))}
    dist = np.hypot(y[:, 0], y[:, 1])
    out["by_distance"] = {}
    for lo, hi in ((0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 9.0)):
        m = (dist >= lo) & (dist < hi)
        if m.sum():
            out["by_distance"][f"{lo}-{hi}m"] = {"n": int(m.sum()), **{n: float(e[m, j].mean()) for j, n in enumerate(TARGETS)}}
    out["by_door_angle"] = {}
    ang = np.degrees(y[:, 3])
    for lo, hi in ((-1, 5), (5, 30), (30, 70), (70, 180)):
        m = (ang >= lo) & (ang < hi)
        if m.sum():
            out["by_door_angle"][f"{lo}-{hi}deg"] = {"n": int(m.sum()), **{n: float(e[m, j].mean()) for j, n in enumerate(TARGETS)}}
    st = data["stage"][idx]
    out["by_stage"] = {int(s): {"n": int((st == s).sum()), **{n: float(e[st == s, j].mean()) for j, n in enumerate(TARGETS)}}
                       for s in np.unique(st)}
    return out


@torch.no_grad()
def predict(model: Estimator, data: dict, idx: np.ndarray, dev, bs: int = 256) -> np.ndarray:
    model.eval()
    out = []
    for i in range(0, len(idx), bs):
        j = idx[i:i + bs]
        img = torch.from_numpy(data["img"][j]).to(dev)
        p = torch.from_numpy(data["proprio"][j]).to(dev)
        with torch.autocast("cuda", dtype=torch.bfloat16, enabled=dev.type == "cuda"):
            out.append(model(img, p).float().cpu().numpy())
    return np.concatenate(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--train", nargs="+", required=True)
    ap.add_argument("--test", nargs="*", default=[])
    ap.add_argument("--out", required=True)
    ap.add_argument("--epochs", type=int, default=8)
    ap.add_argument("--bs", type=int, default=128)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--val-frac", type=float, default=0.15)
    ap.add_argument("--seed", type=int, default=0)
    # run2（10-16）: 閉ループで集めた画像を足す・扉が大きく開いた画像を多めに使う・扉の角度の誤差を重く数える。
    # run1 は閉ループで扉が大きく開いた所の角度を 3° ほど小さく見積もり、生徒が 90° まで押し切った
    ap.add_argument("--extra", nargs="*", default=[], help="学習だけに足す記録（閉ループの vision/ など）")
    ap.add_argument("--open-deg", type=float, default=50.0)
    ap.add_argument("--open-oversample", type=int, default=1, help="扉の角度が --open-deg 以上の画像を何倍使うか")
    ap.add_argument("--angle-weight", type=float, default=1.0)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(a.seed)

    t0 = time.time()
    data = load_dirs(a.train)
    groups = np.unique(data["group"])
    rng = np.random.default_rng(a.seed)
    val_groups = set(rng.choice(groups, size=max(1, int(len(groups) * a.val_frac)), replace=False).tolist())
    is_val = np.array([g in val_groups for g in data["group"]])
    tr, va = np.nonzero(~is_val)[0], np.nonzero(is_val)[0]
    if a.extra:  # 学習だけに足す（確認の val には入れない）
        ex = load_dirs(a.extra)
        n0 = len(data["y"])
        data = {k: np.concatenate([data[k], ex[k]]) for k in data}
        tr = np.concatenate([tr, np.arange(n0, len(data["y"]))])
        print(f"[v2] 学習に足した: {len(ex['y'])} 枚（{a.extra}）", flush=True)
    if a.open_oversample > 1:
        opened = tr[np.degrees(data["y"][tr, 3]) >= a.open_deg]
        tr = np.concatenate([tr] + [opened] * (a.open_oversample - 1))
        print(f"[v2] 扉 {a.open_deg:.0f}° 以上の {len(opened)} 枚を {a.open_oversample} 倍に", flush=True)
    print(f"[v2] 学習 {len(tr)} 枚・val {len(va)} 枚（回 {len(groups)} のうち {len(val_groups)}）・画像 {data['img'].shape[1:]}・読み込み {time.time() - t0:.0f} s", flush=True)

    y = torch.from_numpy(data["y"][tr])
    p = torch.from_numpy(data["proprio"][tr])
    model = Estimator(y.mean(0), y.std(0).clamp(min=1e-3), p.mean(0), p.std(0).clamp(min=1e-3)).to(dev)
    opt = torch.optim.AdamW(model.parameters(), lr=a.lr, weight_decay=1e-4)
    steps = a.epochs * (len(tr) // a.bs)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=a.lr, total_steps=steps, pct_start=0.1)
    g = torch.Generator(device=dev).manual_seed(a.seed)
    imgs = torch.from_numpy(data["img"])  # CPU の uint8。バッチごとに GPU へ
    ys = torch.from_numpy(data["y"])
    ps = torch.from_numpy(data["proprio"])
    history = []
    step = 0
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
            w = torch.tensor([1.0, 1.0, 1.0, a.angle_weight], device=dev)
            loss = (F.smooth_l1_loss(pred.float(), target, beta=0.1, reduction="none") * w).mean() / w.mean()
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step()
            sched.step()
            step += 1
            tot += float(loss) * len(j)
            cnt += len(j)
        v = summarize(predict(model, data, va, dev), data, va)
        history.append({"epoch": ep + 1, "train_loss": tot / cnt, "val": {k: v[k]["mae"] for k in TARGETS}})
        print(f"[v2] epoch {ep + 1}/{a.epochs} loss {tot / cnt:.4f}  val MAE: x {v['x']['mae']:.1f} cm・y {v['y']['mae']:.1f} cm・"
              f"yaw {v['yaw']['mae']:.2f}°・angle {v['angle']['mae']:.2f}°  （{time.time() - t0:.0f} s）", flush=True)

    pv = predict(model, data, va, dev)
    report = {"train_dirs": a.train, "test_dirs": a.test, "history": history, "val": summarize(pv, data, va)}
    np.savez_compressed(out / "pred_val.npz", pred=pv, y=data["y"][va], stage=data["stage"][va], idx=va)
    del imgs, data
    if a.test:
        test = load_dirs(a.test)
        idx = np.arange(len(test["y"]))
        pt = predict(model, test, idx, dev)
        report["test"] = summarize(pt, test, idx)
        pick = np.linspace(0, len(idx) - 1, 400).astype(int)  # 図に使う画像（400 枚）
        np.savez_compressed(out / "pred_test.npz", pred=pt, y=test["y"], stage=test["stage"], pick=pick, img=test["img"][pick])
        t = report["test"]
        print(f"[v2] test（学習に無い俯角）: x {t['x']['mae']:.1f} cm・y {t['y']['mae']:.1f} cm・yaw {t['yaw']['mae']:.2f}°・angle {t['angle']['mae']:.2f}°", flush=True)
    (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=1))

    model.eval().float()
    ex = (torch.zeros(2, 135, 240, dtype=torch.uint8, device=dev), torch.zeros(2, 6, device=dev))
    ts = torch.jit.trace(model, ex)
    ts.save(str(out / "estimator.pt"))
    cpu = torch.jit.trace(model.cpu(), tuple(x.cpu() for x in ex))
    cpu.save(str(out / "estimator_cpu.pt"))
    torch.save(model.state_dict(), out / "state_dict.pt")
    print(f"[v2] 書き出した: {out}/estimator.pt（{time.time() - t0:.0f} s）", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
