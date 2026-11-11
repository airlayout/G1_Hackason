"""パッチ 0103（11-10・画像の網の学び直しに実物の画を混ぜる）の試験。Isaac Sim は起こさない。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_vis && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- load_real: 実物の束（Door/real/real_shard_1110.py の npz）を 1 つにする。項目が欠けた束・形がシムの束と違う束は止める
- mixed_batch: 1 回の歩みのシムの番号は並べ替えた順の続き、実物は戻して n_real 枚を引く（実物が無ければ 0 枚）
- 小さな束で台本を通す: 実物を混ぜて 1 エポック学び、シムの確かめの最良・最後のエポックの重み・実物の確かめの 1 枚ごとの当てを書く。
  --eval_only は重みを測るだけで学ばない
"""
from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

SCRIPT = Path("gr00t/rl/scripts/door_vision_pretrain.py")
DIM = 17


def _load_script():
    spec = importlib.util.spec_from_file_location("door_vision_pretrain", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _frames(n: int, rng: np.random.Generator, stage: float) -> dict:
    return {
        "rgb": rng.integers(0, 255, size=(n, 108, 192, 3), dtype=np.uint8),
        "depth": rng.uniform(0.3, 2.5, size=(n, 54, 96)).astype(np.float16),
        "aux": np.tile(np.array([0.6, 0, -0.8, 0, 0, -1, 0, 0, 0], np.float32), (n, 1)),
        "truth": rng.normal(size=(n, DIM)).astype(np.float32),
        "stage": np.full(n, stage, np.float32),
    }


def _write_sim(dirpath: Path, n: int, rng: np.random.Generator) -> None:
    dirpath.mkdir(parents=True)
    f = _frames(n, rng, 0.0)
    f["env_id"] = (np.arange(n) % 10).astype(np.float32)  # 扉 10 枚
    np.savez(dirpath / "shard_00000.npz", **f)


def _write_real(path: Path, n: int, rng: np.random.Generator) -> None:
    f = _frames(n, rng, -1.0)
    f["env_id"] = np.zeros(n, np.float32)
    np.savez(path, **f)


def test_load_real_concatenates_and_checks(tmp_path):
    mod = _load_script()
    rng = np.random.default_rng(0)
    _write_real(tmp_path / "r1.npz", 4, rng)
    _write_real(tmp_path / "r2.npz", 3, rng)
    real = mod.load_real([str(tmp_path / "r1.npz"), str(tmp_path / "r2.npz")])
    assert real["rgb"].shape == (7, 108, 192, 3)
    assert real["truth"].shape == (7, DIM)
    assert mod.load_real([]) is None

    bad = _frames(2, rng, -1.0)
    bad["depth"] = bad["depth"][:, :50]
    np.savez(tmp_path / "bad.npz", **bad)
    with pytest.raises(ValueError, match="形"):
        mod.load_real([str(tmp_path / "bad.npz")])
    miss = _frames(2, rng, -1.0)
    del miss["aux"]
    np.savez(tmp_path / "miss.npz", **miss)
    with pytest.raises(ValueError, match="aux"):
        mod.load_real([str(tmp_path / "miss.npz")])


def test_mixed_batch_draws_real_with_replacement():
    mod = _load_script()
    rng = np.random.default_rng(0)
    sim, real = mod.mixed_batch(np.array([9, 3, 5]), 64, 10, rng)
    assert sim.tolist() == [3, 5, 9]
    assert len(real) == 64 and real.min() >= 0 and real.max() < 10
    assert np.all(np.diff(real) >= 0)
    sim, real = mod.mixed_batch(np.array([1, 0]), 0, 10, rng)
    assert len(real) == 0


def test_script_runs_with_real_frames_and_eval_only(tmp_path):
    rng = np.random.default_rng(1)
    _write_sim(tmp_path / "sim" / "frames", 40, rng)
    _write_real(tmp_path / "real.npz", 10, rng)
    _write_real(tmp_path / "val.npz", 6, rng)
    out = tmp_path / "net.pt"
    base = [sys.executable, str(SCRIPT), "--data", str(tmp_path / "sim" / "frames"), "--device", "cpu", "--batch", "8",
            "--eval_batch", "8", "--real_val", str(tmp_path / "val.npz")]
    r = subprocess.run(base + ["--out", str(out), "--epochs", "1", "--max_steps", "2", "--real_data", str(tmp_path / "real.npz"),
                               "--real_frac", "0.25"], capture_output=True, text=True, timeout=600)
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
    assert "シム 6＋実物 2" in r.stdout
    assert out.exists() and (tmp_path / "net_last.pt").exists()
    dump = np.load(tmp_path / "net_realval.npz")
    assert dump["pred:val.npz"].shape == (6, DIM)
    assert np.array_equal(dump["truth:val.npz"], np.load(tmp_path / "val.npz")["truth"])
    hist = json.loads((tmp_path / "net_history.json").read_text())
    assert "real_val:val.npz" in hist[0] and "real_train_sub" in hist[0]

    out2 = tmp_path / "eval.pt"
    r = subprocess.run(base + ["--out", str(out2), "--eval_only", "--init", str(out)], capture_output=True, text=True, timeout=600)
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
    assert "測るだけ 終わり" in r.stdout and not out2.exists()
    assert "real_val:val.npz" in json.loads((tmp_path / "eval_eval.json").read_text())


def test_occlude_pastes_boxes_and_near_depth():
    import torch

    mod = _load_script()
    gen = torch.Generator()
    gen.manual_seed(0)
    x = torch.full((64, 3, 108, 192), 0.5)
    d = torch.full((64, 54, 96), 2.0)
    x0, d0 = mod.occlude(x, d, 0.0, gen)
    assert torch.equal(x0, x) and torch.equal(d0, d)
    x1, d1 = mod.occlude(x, d, 1.0, gen)
    assert x1.shape == x.shape and d1.shape == d.shape
    changed = (x1 != x).any(dim=1).float().mean(dim=(1, 2))
    assert (changed > 0).all()  # 割合 1 なら全部の台に四角が貼られる
    near = d1 != d
    assert near.any() and ((d1[near] > 0.2) & (d1[near] < 0.75)).all()  # 深度を変えた所は近い物（0.25〜0.7 m）
    assert torch.equal(x, torch.full_like(x, 0.5))  # 元は書き換えない
