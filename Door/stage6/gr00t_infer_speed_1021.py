"""段 6 (a)（10-21）: GR00T N1.6 の推論の速さを OMEN（RTX 5060 Ti 16 GB）で測る。合格の線は 10 Hz 以上（計画書 D11）。

観測（画像・状態・言葉）の形は、重みに付いた設定（modality config）から読み取って作る。中身は乱数とゼロ（速さを測るだけ）。
1 回の推論で 50 歩ぶんの動き（action_horizon）が出る。

使い方（OMEN、~/Isaac-GR00T の .venv で）:
  .venv/bin/python ~/work_inoue/gr00t_1021/gr00t_infer_speed_1021.py <重みの場所> [--n 50] [--hw 480 640]
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import torch

from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.policy.gr00t_policy import Gr00tPolicy


def make_obs(policy: Gr00tPolicy, hw: tuple[int, int], state_dims: dict[str, int]) -> dict:
    cfg = policy.get_modality_config()
    obs: dict = {"video": {}, "state": {}, "language": {}}
    for modality, mc in cfg.items():
        t = len(mc.delta_indices)
        for key in mc.modality_keys:
            name = key.split(".")[-1]
            if modality == "video":
                obs["video"][name] = np.random.randint(0, 255, (1, t, hw[0], hw[1], 3), dtype=np.uint8)
            elif modality == "state":
                obs["state"][name] = np.zeros((1, t, state_dims.get(name, 7)), np.float32)
            elif modality == "language":
                obs["language"][key] = [["open the door"]]  # 言葉は項目名を略さない（annotation.human.task_description）
    return obs


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("model_path")
    ap.add_argument("--n", type=int, default=50)
    ap.add_argument("--hw", type=int, nargs=2, default=[480, 640])
    ap.add_argument("--modality-json", default="", help="データの meta/modality.json（状態の次元を読む）")
    a = ap.parse_args()
    dims: dict[str, int] = {}
    if a.modality_json:
        import json
        with open(a.modality_json) as f:
            m = json.load(f)
        dims = {k: v["end"] - v["start"] for k, v in m.get("state", {}).items()}
    t0 = time.time()
    policy = Gr00tPolicy(model_path=a.model_path, embodiment_tag=EmbodimentTag.UNITREE_G1, device="cuda:0", strict=False)
    print(f"[infer] 読み込み {time.time() - t0:.1f} s・GPU {torch.cuda.memory_allocated() / 2**30:.2f} GiB")
    cfg = policy.get_modality_config()
    print("[infer] 観測の形:", {k: (list(v.modality_keys), list(v.delta_indices)) for k, v in cfg.items()})
    obs = make_obs(policy, tuple(a.hw), dims)
    for _ in range(5):
        out = policy.get_action(obs)
    torch.cuda.synchronize()
    ts = []
    for _ in range(a.n):
        s = time.perf_counter()
        out = policy.get_action(obs)
        torch.cuda.synchronize()
        ts.append(time.perf_counter() - s)
    act = out[0] if isinstance(out, tuple) else out
    ts = np.array(ts) * 1000
    print(f"[infer] 1 回 中央 {np.median(ts):.1f} ms・9 割 {np.percentile(ts, 90):.1f} ms・最大 {ts.max():.1f} ms → {1000 / np.median(ts):.1f} Hz"
          f"・GPU の山 {torch.cuda.max_memory_allocated() / 2**30:.2f} GiB")
    print("[infer] 出る動き:", {k: tuple(v.shape) for k, v in act.items()})


if __name__ == "__main__":
    main()
