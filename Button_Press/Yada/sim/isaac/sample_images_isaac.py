"""評価環境のサンプルの画像を作る（Isaac Sim）。sample_images_isaac.sh から起動する（直接は起動しない）。

    bash Button_Press/Yada/sim/isaac/sample_images_isaac.sh              # 種 1
    bash Button_Press/Yada/sim/isaac/sample_images_isaac.sh --seed 3

作るもの（_local/button_press_yada/samples/）:
- overview_one_isaac.png: 1 枚のまとめ（乗り場の全体 / 頭カメラ / 深度 / 押した瞬間 / 押した瞬間の頭カメラ /
  同じ種の MuJoCo の頭カメラ（比較。sim/mujoco/sample_images.py で作った画像があれば））
- 1 枚ずつの画像（isaac_seed<N>_<名前>.png）

見本のエージェント（contest/example_agent）に押させて、点灯した瞬間を撮る。評価セットは basic
（realistic の乱しは、Isaac Sim にはまだ入れていない）。
"""

from __future__ import annotations

import argparse
import io
import sys
from contextlib import redirect_stdout
from pathlib import Path

# --- Isaac Sim の起動は他の import より先に行う ---
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Isaac Sim のサンプルの画像")
parser.add_argument("--seed", type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- ここから下は Isaac Sim 起動後にのみ import できる ---
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from common.config import FEATURE_DIR, REPO_ROOT, load_config  # noqa: E402
from common.viz import depth_to_rgb  # noqa: E402
from contest.interface import load_agent  # noqa: E402
from contest.robots.isaac_robot import IsaacRobot  # noqa: E402
from contest.runner import run_episode  # noqa: E402
from contest.task import make_trial  # noqa: E402
from isaac_world import OVERVIEW_CAMERA, camera_rgb  # noqa: E402

OUT_DIR = REPO_ROOT / "_local" / "button_press_yada" / "samples"
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"


def compose(tiles: list[tuple[Image.Image, str, str]], title: str, footer: str, path: Path) -> None:
    """2 段 × 3 列のまとめの画像（MuJoCo 版の overview_one.png と同じ並べ方）。"""
    t_font, c_font, s_font = ImageFont.truetype(FONT_BOLD, 34), ImageFont.truetype(FONT_BOLD, 22), ImageFont.truetype(FONT, 17)
    tw, th, pad, top, cap_h = 600, 450, 16, 80, 64
    W, H = 3 * tw + 4 * pad, top + 2 * (th + cap_h) + 3 * pad + 30
    img = Image.new("RGB", (W, H), (248, 248, 246))
    d = ImageDraw.Draw(img)
    d.text((pad, 20), title, fill=(25, 25, 25), font=t_font)
    for k, (t, c, s) in enumerate(tiles):
        r, cl = divmod(k, 3)
        x, y = pad + cl * (tw + pad), top + r * (th + cap_h + pad)
        img.paste(t.convert("RGB").resize((tw, th), Image.LANCZOS), (x, y))
        d.text((x, y + th + 6), c, fill=(25, 25, 25), font=c_font)
        d.text((x, y + th + 36), s, fill=(90, 90, 90), font=s_font)
    d.text((pad, H - 34), footer, fill=(110, 110, 110), font=s_font)
    img.save(path)


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    cfg = load_config("contest.yaml")
    seed = args.seed
    trial = make_trial(seed, cfg)
    agent = load_agent(FEATURE_DIR / "contest" / "example_agent")
    robot = IsaacRobot(trial.scene_cfg, cfg, device=args.device, overview=True)
    try:
        robot.advance(0.3)
        before = {"overview": camera_rgb(robot.w.cameras[OVERVIEW_CAMERA])}
        obs = robot.observe(0.0)
        before["head"], before["depth"] = obs.rgb, depth_to_rgb(obs.depth)
        with redirect_stdout(io.StringIO()):
            res = run_episode(robot, agent, trial, cfg)
        print(f"[sample] 見本のエージェント: {res.outcome}（{res.time_s} 秒）")
        after = {"overview": camera_rgb(robot.w.cameras[OVERVIEW_CAMERA]), "head": robot.observe(res.time_s or 0.0).rgb}
    finally:
        robot.close()
        agent.close()

    for name, img in (("overview", before["overview"]), ("head", before["head"]), ("depth", before["depth"]),
                      ("pressed_overview", after["overview"]), ("pressed_head", after["head"])):
        Image.fromarray(img).save(OUT_DIR / f"isaac_seed{seed}_{name}.png")
    sym = "▲" if trial.target == "up" else "▼"
    mj = OUT_DIR / f"seed{seed}_head_basic.png"
    tiles = [
        (Image.fromarray(before["overview"]), "乗り場の全体", "G1（腰を固定）とエレベーター乗り場。寸法は MuJoCo と同じ"),
        (Image.fromarray(before["head"]), "頭カメラ", "エージェントが見るカラー画像（RTX で描画）"),
        (Image.fromarray(before["depth"]), "深度", "近いほど黄色、遠いほど青"),
        (Image.fromarray(after["overview"]), "押した瞬間",
         f"見本のエージェントが {res.time_s:.2f} 秒で {sym} を点灯させた" if res.outcome == "success" else f"結果: {res.outcome}"),
        (Image.fromarray(after["head"]), "押した瞬間の頭カメラ", f"中指で押した {sym} が光る"),
    ]
    if mj.exists():
        tiles.append((Image.open(mj), "比較: MuJoCo の頭カメラ（同じ種）", "同じシーン・同じカメラの位置。描画の仕方だけが違う"))
    path = OUT_DIR / "overview_one_isaac.png"
    compose(tiles, "エレベーターのボタン押し：シミュレーションの評価環境（Isaac Sim）",
            f"Button_Press/Yada（seed {seed}、指示「{trial.instruction}」）。エージェントのコードは MuJoCo と同じ", path)
    print(f"[sample] {path}")


if __name__ == "__main__":
    main()
    # main() の外で閉じる（finally で閉じると例外が隠れる。リポジトリ直下の CLAUDE.md）
    simulation_app.close()
