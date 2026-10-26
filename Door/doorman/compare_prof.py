"""prof_1023.sh の測定（tests/prof_<名前>_<時刻>）を並べて比べる。最初のフォルダが比べる基準。

速さ: 実際の経過時間で数える（iters.csv の、学習の回数が SKIP 回に届いた時刻から最後に増えた時刻までの傾き）。
  学習のログの「Computation」は gc.collect 3 か所のうち 2 か所の時間を含まないので、参考に並べるだけ。
時間の内訳: py-spy の標本（学習の繰り返しの中だけ）を、いちばん奥の行の中身と呼び出し元で分ける。
  GPU は非同期なので、GPU の計算時間はそれを待つ行（.item()・.any()・正規分布の乱数など）に表れる。
GPU のメモリ: 学習を起こす前（gpu.csv の最初の行）を引いた山（0.25 秒ごと）。
画: 確かめの記録の PNG のくっきり具合（ラプラシアンの分散）と明るさの中央（analyze_rendertest.py と同じ測り方）。

使い方: python compare_prof.py <作業場所（VIRAL6_speed）> <測定のフォルダ>...
"""
from __future__ import annotations

import re
import sys
from collections import Counter
from pathlib import Path

import numpy as np

from analyze_rendertest import sharpness

SKIP = 5  # 始めの回は重みの読み込み・描画の立ち上がりで遅いので除く
STEPS_PER_ENV = 32  # algo.config.num_steps_per_env
ANSI = re.compile(r"\x1b\[[0-9;]*m")
FRAME = re.compile(r"^(.*) \((.*):(\d+)\)$")


class SourceLines:
    """py-spy の枠（関数 (ファイル:行)）の行の中身を読む。py-spy は sys.path に入ったフォルダのファイルを
    名前だけで書く（例: ppo_trainer.py:1408）ので、相対パスで見つからなければ作業場所の中を名前で探す"""

    def __init__(self, repo: Path) -> None:
        self.repo = repo
        self.cache: dict[str, list[str]] = {}

    def _resolve(self, name: str) -> Path | None:
        path = Path(name)
        if path.is_absolute():
            return path
        if (self.repo / path).exists():
            return self.repo / path
        return next(iter(sorted(self.repo.rglob(path.name))), None)

    def text(self, frame: str) -> str:
        m = FRAME.match(frame)
        if not m:
            return ""
        key = m.group(2)
        if key not in self.cache:
            path = self._resolve(key)
            try:
                self.cache[key] = path.read_text(errors="replace").splitlines() if path else []
            except OSError:
                self.cache[key] = []
        lines, n = self.cache[key], int(m.group(3))
        return lines[n - 1] if 0 < n <= len(lines) else ""


def classify(frames: list[str], src: SourceLines) -> str:
    leaf_text = src.text(frames[-1])
    if "gc.collect" in leaf_text:
        return "後片付け（gc.collect）"
    if "empty_cache" in leaf_text or frames[-1].startswith("empty_cache ("):
        return "後片付け（empty_cache）"
    joined = ";".join(frames)
    if "ensure_isaac_rtx_render_update (" in joined:
        return "集める段：描画の待ち"
    if any(f.startswith("_physics_step (") for f in frames):  # _post_physics_step は物理の後の計算なので入れない
        return "集める段：物理"
    if "_rollout_step (" in joined:
        if "rollout_with_obj_pred (" in joined:
            return "集める段：方策（GPU の待ちを含む）"
        if "_compute_reward (" in joined:
            return "集める段：報酬"
        if "_compute_observations (" in joined:
            return "集める段：観測（描画以外）"
        return "集める段：その他"
    if "_get_train_metrics (" in joined:
        return "学習の段：指標"
    return "学習の段：順伝播・逆伝播・更新（GPU の計算を含む）"


def shares(prof: Path, src: SourceLines) -> Counter:
    cats: Counter = Counter()
    for line in prof.read_text(errors="replace").splitlines():
        stack, _, n = line.rpartition(" ")
        if not n.isdigit():
            continue
        frames = stack.split(";")
        if not any(f.startswith("train (") and "ppo_trainer.py" in f for f in frames):
            continue
        cats[classify(frames, src)] += int(n)
    return cats


def wall_rate(d: Path) -> tuple[float, float] | None:
    """（1 秒あたりの学習の回数, 測った回数）。iters.csv が無ければ None"""
    f = d / "iters.csv"
    if not f.exists():
        return None
    rows = []
    for line in f.read_text().splitlines():
        t, _, c = line.partition(",")
        if c.strip().isdigit():
            rows.append((float(t), int(c)))
    first = next(((t, c) for t, c in rows if c >= SKIP), None)
    if first is None:
        return None
    last_c = rows[-1][1]
    last_t = next(t for t, c in rows if c == last_c)  # 最後の回に届いた時刻
    if last_c <= first[1] or last_t <= first[0]:
        return None
    return (last_c - first[1]) / (last_t - first[0]), float(last_c - first[1])


def log_rate(d: Path) -> float:
    txt = ANSI.sub("", (d / "train.log").read_text(errors="replace"))
    vals = [float(v) for v in re.findall(r"Computation: (\d+) steps/s", txt)][SKIP:]
    return float(np.median(vals)) if vals else float("nan")


def num_envs(d: Path) -> int:
    m = re.search(r" n=(\d+) ", (d / "prof.log").read_text())
    return int(m.group(1)) if m else 16


def gpu_peak(d: Path) -> float:
    mem = []
    for line in (d / "gpu.csv").read_text().splitlines():
        parts = [p.strip() for p in line.split(",")]
        if len(parts) >= 2 and parts[1].isdigit():
            mem.append(int(parts[1]))
    return float(max(mem) - mem[0]) if mem else float("nan")


def main() -> None:
    repo, dirs = Path(sys.argv[1]), [Path(p) for p in sys.argv[2:]]
    src = SourceLines(repo)
    rows, all_shares = [], {}
    for d in dirs:
        name = d.name.split("_")[1]
        n = num_envs(d)
        wr = wall_rate(d)
        sps = wr[0] * n * STEPS_PER_ENV if wr else float("nan")
        cam = d / "cam"
        sh, br, m = sharpness(str(cam)) if any(cam.glob("*.png")) else (float("nan"), float("nan"), 0)
        rows.append((name, n, sps, wr[1] if wr else 0, log_rate(d), gpu_peak(d), sh, br, m))
        all_shares[name] = shares(d / "prof.txt", src)
    ref_sps, ref_sh = rows[0][2], rows[0][6]
    print("| 測定 | 台数 | 歩/秒（経過時間） | 基準との比 | 測った回数 | 歩/秒（ログ） | GPU の山 MiB | 画のくっきり（比） | 明るさ |")
    print("|---|---|---|---|---|---|---|---|---|")
    for name, n, sps, k, lr, gp, sh, br, m in rows:
        print(f"| {name} | {n} | {sps:.0f} | {sps / ref_sps:.2f} | {k:.0f} | {lr:.0f} | {gp:.0f} | {sh:.1f}（{sh / ref_sh:.2f}） | {br:.1f} |")
    cats = sorted({c for s in all_shares.values() for c in s})
    print("\n時間の内訳（学習の繰り返しの中の割合 %）\n| 分類 | " + " | ".join(all_shares) + " |")
    print("|---|" + "---|" * len(all_shares))
    for c in cats:
        vals = [100 * s[c] / max(1, sum(s.values())) for s in all_shares.values()]
        print(f"| {c} | " + " | ".join(f"{v:.1f}" for v in vals) + " |")


if __name__ == "__main__":
    main()
