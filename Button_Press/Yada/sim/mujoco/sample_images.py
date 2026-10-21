"""評価環境のサンプルの画像を作る（MuJoCo）。説明の資料や、条件の確認に使う。

    P=~/miniconda3/envs/lerobot/bin/python
    $P Button_Press/Yada/sim/mujoco/sample_images.py                  # 種 0, 1, 2
    $P Button_Press/Yada/sim/mujoco/sample_images.py --seeds 3 7 12

作るもの（_local/button_press_yada/samples/）:
- sample_sheet.png: 種ごとに 1 行。全体のカメラ / 頭カメラのカラーと深度（basic）/ 同じ（realistic）を並べる
- pressed_seed<N>.png: 見本のエージェントがボタンを押して点灯した瞬間（全体のカメラと頭カメラ）
- 1 枚ずつの画像（<種>_<名前>.png）

深度は、近い（0.2 m）ほど明るい色、遠い（1.5 m）ほど暗い色にした。測れない画素（穴）は黒。
"""

from __future__ import annotations

import argparse
import io
import sys
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from common.config import FEATURE_DIR, REPO_ROOT, load_config  # noqa: E402
from common.viz import DEPTH_FAR, DEPTH_NEAR, depth_to_rgb  # noqa: E402
from contest.interface import load_agent  # noqa: E402
from contest.robots.mujoco_robot import MujocoRobot  # noqa: E402
from contest.runner import run_episode  # noqa: E402
from contest.task import make_trial  # noqa: E402
from mujoco_hall import OVERVIEW_CAMERA  # noqa: E402

OUT_DIR = REPO_ROOT / "_local" / "button_press_yada" / "samples"
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
TILE_W, TILE_H = 400, 300


def render_overview(robot: MujocoRobot) -> np.ndarray:
    import mujoco

    r = mujoco.Renderer(robot.model, height=720, width=960)
    try:
        r.update_scene(robot.data, camera=OVERVIEW_CAMERA)
        return r.render().copy()
    finally:
        r.close()


def head_images(seed: int, cfg: dict, eval_set: str) -> dict[str, np.ndarray]:
    trial = make_trial(seed, cfg, eval_set=eval_set)
    robot = MujocoRobot(trial.scene_cfg, cfg, realism=trial.realism)
    try:
        robot.advance(0.3)
        obs = robot.observe(0.3)
        return {"overview": render_overview(robot), "rgb": obs.rgb, "depth": depth_to_rgb(obs.depth),
                "target": trial.target, "instruction": trial.instruction}
    finally:
        robot.close()


def pressed_images(seed: int, cfg: dict) -> dict[str, np.ndarray] | None:
    """見本のエージェントで押させ、点灯した瞬間の画像（basic）。押せなければ None。"""
    trial = make_trial(seed, cfg)
    robot = MujocoRobot(trial.scene_cfg, cfg)
    agent = load_agent(FEATURE_DIR / "contest" / "example_agent")
    try:
        with redirect_stdout(io.StringIO()):
            res = run_episode(robot, agent, trial, cfg)
        if res.outcome != "success":
            return None
        obs = robot.observe(res.time_s or 0.0)
        return {"overview": render_overview(robot), "rgb": obs.rgb, "time_s": res.time_s, "target": trial.target}
    finally:
        robot.close()


def _tile(img: np.ndarray, w: int = TILE_W, h: int = TILE_H):
    from PIL import Image

    return Image.fromarray(img).resize((w, h), Image.LANCZOS)


def make_sheet(seeds: list[int], cfg: dict) -> Path:
    from PIL import Image, ImageDraw, ImageFont

    font = ImageFont.truetype(FONT, 20)
    small = ImageFont.truetype(FONT, 16)
    cols = ["全体のカメラ", "頭カメラ（basic）", "深度（basic）", "頭カメラ（realistic）", "深度（realistic）"]
    head_h, label_w, pad = 40, 170, 6
    W = label_w + len(cols) * (TILE_W + pad)
    H = head_h + len(seeds) * (TILE_H + pad) + 50
    sheet = Image.new("RGB", (W, H), (245, 245, 245))
    d = ImageDraw.Draw(sheet)
    for j, c in enumerate(cols):
        d.text((label_w + j * (TILE_W + pad) + 8, 8), c, fill=(20, 20, 20), font=font)
    for i, seed in enumerate(seeds):
        b = head_images(seed, cfg, "basic")
        r = head_images(seed, cfg, "realistic")
        y = head_h + i * (TILE_H + pad)
        d.text((10, y + 10), f"seed {seed}", fill=(20, 20, 20), font=font)
        d.text((10, y + 40), f"押す: {'▲' if b['target'] == 'up' else '▼'}", fill=(20, 20, 20), font=small)
        # 指示の文は、左の欄の幅で折り返す
        line, ty = "", y + 64
        for ch in b["instruction"]:
            if d.textlength(line + ch, font=small) > label_w - 16:
                d.text((10, ty), line, fill=(90, 90, 90), font=small)
                line, ty = ch.lstrip(), ty + 20
            else:
                line += ch
        d.text((10, ty), line, fill=(90, 90, 90), font=small)
        for j, img in enumerate([b["overview"], b["rgb"], b["depth"], r["rgb"], r["depth"]]):
            sheet.paste(_tile(img), (label_w + j * (TILE_W + pad), y))
        for name, img in (("overview", b["overview"]), ("head_basic", b["rgb"]), ("depth_basic", b["depth"]),
                          ("head_realistic", r["rgb"]), ("depth_realistic", r["depth"])):
            Image.fromarray(img).save(OUT_DIR / f"seed{seed}_{name}.png")
    d.text((10, H - 40), f"深度: 近い（{DEPTH_NEAR} m）ほど明るい黄色、遠い（{DEPTH_FAR} m）ほど暗い青、測れない画素は黒。"
           "realistic は、深度のノイズ・穴、カラーのノイズ、カメラの取り付けの誤差、体の揺れ、立ち位置のずれが入る。",
           fill=(60, 60, 60), font=small)
    path = OUT_DIR / "sample_sheet.png"
    sheet.save(path)
    return path


def make_pressed(seed: int, cfg: dict) -> Path | None:
    from PIL import Image, ImageDraw, ImageFont

    p = pressed_images(seed, cfg)
    if p is None:
        print(f"[sample] seed {seed} は見本のエージェントが押せなかったので、押したところの画像は作らない")
        return None
    font = ImageFont.truetype(FONT, 22)
    ov = Image.fromarray(p["overview"]).resize((640, 480), Image.LANCZOS)
    hd = Image.fromarray(p["rgb"])
    sheet = Image.new("RGB", (640 + 640 + 6, 480 + 44), (245, 245, 245))
    d = ImageDraw.Draw(sheet)
    d.text((8, 8), f"全体のカメラ（seed {seed}、{p['time_s']:.2f} 秒で{'▲' if p['target'] == 'up' else '▼'}が点灯）",
           fill=(20, 20, 20), font=font)
    d.text((652, 8), "頭カメラ", fill=(20, 20, 20), font=font)
    sheet.paste(ov, (0, 44))
    sheet.paste(hd, (646, 44))
    path = OUT_DIR / f"pressed_seed{seed}.png"
    sheet.save(path)
    return path


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    ap.add_argument("--pressed-seed", type=int, default=1, help="押したところの画像を作る種")
    args = ap.parse_args()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    cfg = load_config("contest.yaml")
    print(f"[sample] {make_sheet(args.seeds, cfg)}")
    p = make_pressed(args.pressed_seed, cfg)
    if p:
        print(f"[sample] {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
