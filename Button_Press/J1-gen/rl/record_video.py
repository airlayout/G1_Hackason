"""学習した方策の動きを MuJoCo で描いて mp4（実時間、25 fps）に残す。画面は要らない（EGL で描く）。

    ~/rl_env/bin/python Button_Press/J1-gen/rl/record_video.py <runs/名前>/model_latest.zip --episodes 6 --name v3_tipfix
    ~/rl_env/bin/python Button_Press/J1-gen/rl/record_video.py <model>.zip --seeds 10040 10046 10048 --name v3_failures

- 配置は evaluate.py と同じ seed（EVAL_SEED0 から順）。--seeds で選んだ配置だけを撮ることもできる（失敗した配置の確認用）
- 画の左上に 名前・seed・経過時間・押す力・結果、右下に押す点までの距離などを出す（cv2 は日本語を描けないので英語）
- 保存先: _local/rl/videos/<名前>.mp4（git の対象外）。1 本に複数のエピソードをつなぐ
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import cv2  # noqa: E402
import imageio.v2 as imageio  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rl.evaluate import EVAL_SEED0, load_policy  # noqa: E402
from rl.press_env import PressEnv  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
OUT_DIR = REPO / "_local" / "rl" / "videos"
FPS = 25
W, H = 640, 480


def _text(img: np.ndarray, s: str, xy: tuple[int, int], scale: float = 0.6, color=(255, 255, 255)) -> None:
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("model", type=Path, help="model_*.zip")
    ap.add_argument("--episodes", type=int, default=6, help="EVAL_SEED0 から何配置撮るか")
    ap.add_argument("--seeds", type=int, nargs="*", help="撮る seed を直接指定する（--episodes より優先）")
    ap.add_argument("--name", default="", help="ファイル名（既定は記録の名前）")
    ap.add_argument("--azimuth", type=float, default=210.0)
    ap.add_argument("--elevation", type=float, default=-25.0)
    ap.add_argument("--distance", type=float, default=1.5)
    args = ap.parse_args()

    import mujoco

    model, vecnorm, cfg = load_policy(args.model)
    env = PressEnv(cfg)
    renderer = mujoco.Renderer(env.model, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.azimuth, cam.elevation, cam.distance = args.azimuth, args.elevation, args.distance
    seeds = args.seeds if args.seeds else [EVAL_SEED0 + k for k in range(args.episodes)]
    name = args.name or args.model.parent.name
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / f"{name}.mp4"
    every = max(1, int(round(env.cfg["control_hz"] / FPS)))
    writer = imageio.get_writer(str(out), fps=FPS, codec="libx264", quality=7, macro_block_size=None)

    def frame(label: str, color=(255, 255, 255), t=0.0, info=None) -> np.ndarray:
        cam.lookat[:] = env.data.body("pelvis").xpos + np.array([0.3, -0.1, -0.05])
        renderer.update_scene(env.data, camera=cam)
        img = renderer.render().copy()
        _text(img, f"{name}  seed {label}", (10, 28))
        _text(img, f"t = {t:4.1f} s", (10, 56))
        if info:
            _text(img, f"force {info['force']:5.1f} N   to target {info['dist'] * 1000:5.1f} mm", (10, H - 14))
            if info["outcome"]:
                _text(img, info["outcome"].upper(), (W - 220, 40), 1.0, color)
        return img

    colors = {"success": (80, 255, 80), "collision": (255, 80, 80), "timeout": (255, 200, 60)}
    results = []
    try:
        for seed in seeds:
            obs, _ = env.reset(seed=seed)
            info: dict = {}
            writer.append_data(frame(str(seed)))
            while True:
                action, _ = model.predict(vecnorm.normalize_obs(obs[None]), deterministic=True)
                obs, _, term, trunc, info = env.step(action[0])
                if env._steps % every == 0:
                    writer.append_data(frame(str(seed), t=env._steps / env.cfg["control_hz"], info=info))
                if term or trunc:
                    break
            last = frame(str(seed), colors.get(info["outcome"], (255, 255, 255)), env._steps / env.cfg["control_hz"], info)
            for _ in range(FPS):  # 結果を 1 秒止めて見せる
                writer.append_data(last)
            results.append((seed, info["outcome"], env._steps / env.cfg["control_hz"]))
            print(f"[record_video] seed {seed}: {info['outcome']} {results[-1][2]:.1f} 秒")
    finally:
        writer.close()
    print(f"[record_video] 保存: {out}（{len(results)} 配置）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
