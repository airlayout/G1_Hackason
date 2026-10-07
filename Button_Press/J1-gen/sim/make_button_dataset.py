"""エレベーターの呼びボタンの YOLO の学習データを、Yada の評価環境（MuJoCo）の頭カメラの画像から作る。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/make_button_dataset.py                      # 種 0〜299（basic と realistic を交互）
    $P Button_Press/J1-gen/sim/make_button_dataset.py --seeds 1000 --start 5000 --workers 8

作るもの（_local/button_press/button_yolo/dataset/。git の対象外）:
- images/{train,val}/*.png、labels/{train,val}/*.txt（Ultralytics の YOLO 形式: クラス 中心x 中心y 幅 高さ、0〜1）
- data.yaml（クラス 0: up（▲）、1: down（▼））

正解の枠は、MuJoCo のセグメンテーション（画素ごとにどの物体が写っているか）から作る。手でラベルを付けない。
一般用も車いす用も、▲ は up、▼ は down にする（一般用かどうかは common/elevator_buttons.py が位置の並びで決める）。

画像を増やす工夫（どれも評価環境の作り方の範囲）:
- 試行の種ごとに、乗り場の寸法・明るさ・ボタンの大きさが変わる（評価環境の randomize）
- 半分は realistic（深度とカラーのノイズ、カメラの取り付けの誤差、体の揺れ、立ち位置のずれ）
- 腰の角度を少し変えて、見る向きを変える
- 3 枚に 1 枚は右腕をボタンの近くまで上げ、手が写り込んだ画像にする（押す直前に撮る画像に近づける）

⚠️ シミュレーションの画像だけで学習した重みは、本物のエレベーターではそのままでは使えない見込み。
実機の画像（real/record.py で撮る）に手でラベルを付けて足し、学習し直す（README の「ボタンの YOLO」）。
"""

from __future__ import annotations

import argparse
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

FEATURE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = FEATURE_DIR.parents[1]
YADA_DIR = FEATURE_DIR.parent / "Yada"
OUT_DIR = REPO_ROOT / "_local" / "button_press" / "button_yolo" / "dataset"
CLASSES = ("up", "down")
MIN_VISIBLE_PX = 30
MIN_BOX_PX = 8
MIN_FILL = 0.4


def _yada_imports():
    """Yada の評価環境を読み込む（Yada も J1-gen もパッケージ名が common なので、別のプロセスでだけ使う）。"""
    for p in (str(YADA_DIR / "sim" / "mujoco"), str(YADA_DIR)):
        if p not in sys.path:
            sys.path.insert(0, p)
    from common.config import load_config
    from contest.robots.mujoco_robot import MujocoRobot
    from contest.task import make_trial
    from mujoco_hall import button_body, button_cap

    return load_config, MujocoRobot, make_trial, button_body, button_cap


def render_one(seed: int, split: str, out_dir: str) -> tuple[int, int]:
    """1 枚作る。(書いた枠の数, 書いた画像の数) を返す。"""
    import mujoco
    from PIL import Image

    load_config, MujocoRobot, make_trial, button_body, button_cap = _yada_imports()
    cfg = load_config("contest.yaml")
    rng = np.random.default_rng(seed + 777)
    eval_set = "realistic" if seed % 2 else "basic"
    trial = make_trial(seed, cfg, eval_set=eval_set)
    robot = MujocoRobot(trial.scene_cfg, cfg, realism=trial.realism)
    seg = mujoco.Renderer(robot.model, height=480, width=640)
    try:
        seg.enable_segmentation_rendering()
        m, d = robot.model, robot.data
        # 腰（yaw, roll, pitch = motor 12〜14）を少し変える。3 枚に 1 枚は右腕（motor 22〜28）を前に上げる
        q = robot.q_des.copy()
        q[12:15] += rng.uniform([-0.15, -0.05, -0.05], [0.15, 0.05, 0.15])
        if seed % 3 == 0:
            q[22:29] = rng.uniform([-1.4, -0.6, -0.3, -0.2, -0.5, -0.5, -0.5], [-0.6, 0.0, 0.5, 0.8, 0.5, 0.5, 0.5])
        robot.q_des = q
        robot.advance(0.6)
        obs = robot.observe(0.6)
        seg.update_scene(d, camera=robot._cam)
        ids = seg.render()
        lines = []
        for b in robot.scene.buttons:
            gids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
                    for n in (button_cap(b.name), f"{button_body(b.name)}_symbol")]
            mask = np.isin(ids[..., 0], gids) & (ids[..., 1] == mujoco.mjtObj.mjOBJ_GEOM)
            if int(mask.sum()) < MIN_VISIBLE_PX:
                continue
            v, u = np.nonzero(mask)
            x1, y1, x2, y2 = u.min(), v.min(), u.max() + 1, v.max() + 1
            # 手にほとんど隠れたボタン（細い切れ端だけが写る）は、枠を付けない（学習を乱すため）
            if min(x2 - x1, y2 - y1) < MIN_BOX_PX or mask.sum() < MIN_FILL * (x2 - x1) * (y2 - y1):
                continue
            cls = CLASSES.index("down" if b.name.endswith("down") else "up")
            lines.append(f"{cls} {(x1 + x2) / 1280:.6f} {(y1 + y2) / 960:.6f} {(x2 - x1) / 640:.6f} {(y2 - y1) / 480:.6f}")
        out = Path(out_dir)
        stem = f"seed{seed:05d}_{eval_set}"
        Image.fromarray(obs.rgb).save(out / "images" / split / f"{stem}.png")
        (out / "labels" / split / f"{stem}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
        return len(lines), 1
    finally:
        seg.close()
        robot.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, default=300, help="作る枚数（種の数）")
    ap.add_argument("--start", type=int, default=0, help="最初の種（評価に使う練習用の種 0〜19 と分けたいときに変える）")
    ap.add_argument("--val-every", type=int, default=10, help="この枚数に 1 枚を検証用にする")
    ap.add_argument("--workers", type=int, default=0, help="並列数（0 なら使えるコアの数 − 1）")
    ap.add_argument("--out", default=str(OUT_DIR))
    args = ap.parse_args()

    import os

    out = Path(args.out)
    for split in ("train", "val"):
        (out / "images" / split).mkdir(parents=True, exist_ok=True)
        (out / "labels" / split).mkdir(parents=True, exist_ok=True)
    (out / "data.yaml").write_text(
        f"# make_button_dataset.py が作った（種 {args.start}〜{args.start + args.seeds - 1}）\n"
        f"path: {out}\ntrain: images/train\nval: images/val\nnames:\n"
        + "".join(f"  {i}: {c}\n" for i, c in enumerate(CLASSES)))
    seeds = list(range(args.start, args.start + args.seeds))
    splits = ["val" if (s - args.start) % args.val_every == 0 else "train" for s in seeds]
    workers = args.workers or max(1, (os.cpu_count() or 2) - 1)
    print(f"[dataset] {len(seeds)} 枚を作る（並列数 {workers}）: {out}")
    boxes = images = 0
    with ProcessPoolExecutor(max_workers=workers) as ex:
        for k, (nb, ni) in enumerate(ex.map(render_one, seeds, splits, [str(out)] * len(seeds)), start=1):
            boxes += nb
            images += ni
            if k % 25 == 0 or k == len(seeds):
                print(f"[dataset] {k}/{len(seeds)} 枚（枠 {boxes} 個）")
    print(f"[dataset] ✅ {images} 枚、枠 {boxes} 個。data.yaml: {out / 'data.yaml'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
