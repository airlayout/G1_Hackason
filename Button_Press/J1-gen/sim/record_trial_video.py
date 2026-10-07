"""Yada の評価環境の 1 試行を、実際の速さで再生される動画（25 fps、mp4）にする。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/record_trial_video.py                         # 種 0、basic
    $P Button_Press/J1-gen/sim/record_trial_video.py --seed 3 --set realistic

evaluate.py --view の画面は、計算が間に合わないとスローモーションになる（ノート PC で約 14 倍遅かった）。
この動画はシミュレーションの時間で 1 コマずつ撮るので、G1 の本当の速さが見える（撮るのに 1 本 2 分ほど）。

左は横から見た G1、右は頭のカメラ。左上に経過時間と、エージェントの段階（phase）を出す。
保存先: _local/button_press_yada/videos/seed<種>_<評価セット>.mp4（git の対象外）。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

FEATURE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = FEATURE_DIR.parents[1]
YADA_DIR = FEATURE_DIR.parent / "Yada"
OUT_DIR = REPO_ROOT / "_local" / "button_press_yada" / "videos"
FPS = 25
W, H = 640, 480


def _text(img: np.ndarray, s: str, xy: tuple[int, int], scale: float = 0.8, color=(255, 255, 255)) -> None:
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--set", default="basic", help="評価セット（basic / realistic）")
    ap.add_argument("--agent", default=str(FEATURE_DIR / "contest_agent"))
    ap.add_argument("--out", default="", help="既定は _local/button_press_yada/videos/seed<種>_<評価セット>.mp4")
    ap.add_argument("--azimuth", type=float, default=-60.0, help="横から見るカメラの向き [度]")
    ap.add_argument("--elevation", type=float, default=-15.0)
    ap.add_argument("--distance", type=float, default=1.0)
    args = ap.parse_args()

    # Yada の評価環境は、自分のフォルダを import の検索先に入れて使う（evaluate.py と同じ）
    sys.path.insert(0, str(YADA_DIR))
    import mujoco
    from common.config import load_config
    from contest.interface import load_agent
    from contest.robots.mujoco_robot import MujocoRobot
    from contest.runner import run_episode
    from contest.task import make_trial

    out = Path(args.out) if args.out else OUT_DIR / f"seed{args.seed}_{args.set}.mp4"
    out.parent.mkdir(parents=True, exist_ok=True)
    cfg = load_config("contest.yaml")
    trial = make_trial(args.seed, cfg, eval_set=args.set)
    robot = MujocoRobot(trial.scene_cfg, cfg, realism=trial.realism)
    agent = load_agent(args.agent)
    m, d = robot.model, robot.data
    side = mujoco.Renderer(m, height=H, width=W)
    head = mujoco.Renderer(m, height=H, width=W)
    cam = mujoco.MjvCamera()
    pelvis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    cam.lookat[:] = d.xpos[pelvis] + np.array([0.25, -0.05, 0.25])  # 体の右前（右腕とボタンの間）を見る
    cam.azimuth, cam.elevation, cam.distance = args.azimuth, args.elevation, args.distance

    writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (2 * W, H))
    state = {"t0": None, "next": 0.0, "frames": 0}
    phases: list[tuple[str, float]] = []

    def frame() -> np.ndarray:
        side.update_scene(d, camera=cam)
        head.update_scene(d, camera=robot._cam)
        img = np.hstack([side.render(), head.render()])[:, :, ::-1].copy()
        _text(img, f"t = {d.time - state['t0']:5.2f} s   phase: {getattr(agent.ctl, 'phase', '')}", (12, 30))
        _text(img, "head camera", (W + 12, 30))
        return img

    orig_advance = robot.advance

    def advance(dt: float):
        info = orig_advance(dt)
        if state["t0"] is None:  # 置いた直後の揺れを収める部分（エージェントを呼ぶ前）は撮らない
            state["t0"] = d.time
            return info
        ph = getattr(agent.ctl, "phase", "")
        if not phases or phases[-1][0] != ph:
            phases.append((ph, d.time - state["t0"]))
        while d.time - state["t0"] >= state["next"] - 1e-9:
            writer.write(frame())
            state["frames"] += 1
            state["next"] += 1.0 / FPS
        return info

    robot.advance = advance
    wall = time.time()
    try:
        res = run_episode(robot, agent, trial, cfg)
        # 点灯した瞬間で終わると分かりにくいので、最後の画を 1.5 秒保つ
        last = frame()
        _text(last, f"{res.outcome}  {res.time_s} s", (12, H - 20), 1.2, (0, 255, 0))
        for _ in range(int(1.5 * FPS)):
            writer.write(last)
    finally:
        writer.release()
        side.close()
        head.close()
        robot.close()
    print("[video] 段階が始まった時刻: " + "、".join(f"{p} {t:.2f} s" for p, t in phases))
    print(f"[video] {res.outcome}（{res.time_s} s）。{state['frames']} コマ、撮るのにかかった時間 {time.time() - wall:.0f} s")
    print(f"[video] ✅ {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
