"""Isaac Sim（DoorMan）で記録した機体と扉の状態を、Mac の MuJoCo で再生して動画にする。

OMEN のドライバでは Isaac Sim 5.1 のカメラの描画が落ちるので、評価の動画の代わりに使う。
記録は VIRAL29 の gr00t/rl/utils/state_recorder.py（env.config.record_states）。物理は回さず、毎コマの姿勢を置いて描くだけ。

扉は DoorMan の door.py と同じ寸法・同じ開き方で作る:
  - 扉の面は x = 0、機体は x < 0 から +x へ押して通る
  - door_open_lr = +1: 蝶番は y = −幅/2（機体から見て右）、レバーは +y 側（左）。−1 はその逆
  - 蝶番の角度が正 = +x へ開く。レバーの角度が正 = レバーが下がる

使い方:
    python replay_states.py --record <記録のディレクトリ> --mjcf <unitree_mujoco>/unitree_robots/g1/scene_29dof.xml \
        --env 0 --out-dir <出力先> [--max-episodes 3] [--min-stage 0]
"""

import argparse
import glob
import json
from pathlib import Path

import mujoco
import numpy as np

from homie_mujoco import write_video

PANEL_HALF_THICKNESS = 0.02
FRAME_WIDTH = 0.3
HANDLE_RADIUS = 0.013
HANDLE_LENGTH = 0.125
AXLE_LENGTH = 0.195
VIDEO_SIZE = (854, 480)  # 幅・高さ


def load_record(record_dir: Path) -> tuple[dict, dict]:
    static = dict(np.load(record_dir / "static.npz"))
    chunks = [dict(np.load(p)) for p in sorted(glob.glob(str(record_dir / "states_*.npz")))]
    if not chunks:
        raise FileNotFoundError(f"states_*.npz が無い: {record_dir}")
    states = {k: np.concatenate([c[k] for c in chunks]) for k in chunks[0]}
    return static, states


def add_door(spec: mujoco.MjSpec, width: float, height: float, handle_height: float, handle_width: float, lr: int) -> None:
    half_w = width / 2
    no_collide = {"contype": 0, "conaffinity": 0}
    frame_rgba = [0.55, 0.55, 0.6, 1.0]
    root = spec.worldbody.add_body(name="door_root")
    for side in (1, -1):
        root.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.05, FRAME_WIDTH / 2, (height + 0.3) / 2],
                      pos=[-0.02, side * (half_w + FRAME_WIDTH / 2), (height + 0.3) / 2], rgba=frame_rgba, **no_collide)
    root.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.05, half_w + FRAME_WIDTH, 0.15],
                  pos=[-0.02, 0, height + 0.15], rgba=frame_rgba, **no_collide)

    hinge_y = -half_w * lr
    panel = root.add_body(name="door_panel", pos=[PANEL_HALF_THICKNESS, hinge_y, 0])
    panel.add_joint(name="door_hinge", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, -lr])
    panel.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[PANEL_HALF_THICKNESS, half_w - 0.002, height / 2 - 0.002],
                   pos=[-PANEL_HALF_THICKNESS, half_w * lr, height / 2], rgba=[0.12, 0.12, 0.14, 1.0], **no_collide)

    handle_y = (width - handle_width) * lr
    handle = panel.add_body(name="door_handle", pos=[-PANEL_HALF_THICKNESS, handle_y, handle_height])
    handle.add_joint(name="door_handle_joint", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[lr, 0, 0])
    gold = [0.85, 0.7, 0.3, 1.0]
    handle.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[HANDLE_RADIUS, AXLE_LENGTH / 2],
                    quat=[0.7071068, 0, 0.7071068, 0], rgba=gold, **no_collide)
    for x in (-AXLE_LENGTH / 2, AXLE_LENGTH / 2):
        handle.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[HANDLE_RADIUS, HANDLE_LENGTH / 2],
                        pos=[x, -HANDLE_LENGTH / 2 * lr, 0], quat=[0.7071068, 0.7071068, 0, 0], rgba=gold, **no_collide)


def split_episodes(episode_length: np.ndarray) -> list[tuple[int, int]]:
    """episode_length が減った所でエピソードを切る（[開始, 終了) の組）。"""
    starts = [0] + [i for i in range(1, len(episode_length)) if episode_length[i] < episode_length[i - 1]]
    ends = starts[1:] + [len(episode_length)]
    return [(s, e) for s, e in zip(starts, ends) if e - s > 10]


def render_episode(model: mujoco.MjModel, data: mujoco.MjData, states: dict, env: int, span: tuple[int, int],
                   qadr: np.ndarray, hinge_adr: int, handle_adr: int, out: Path, fps: int, view: tuple[float, float, float]) -> None:
    renderer = mujoco.Renderer(model, height=VIDEO_SIZE[1], width=VIDEO_SIZE[0])
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [-0.4, 0.0, 0.9]
    cam.azimuth, cam.elevation, cam.distance = view
    frames = []
    for t in range(*span):
        data.qpos[:3] = states["root_pos"][t, env]
        x, y, z, w = states["root_quat_xyzw"][t, env]
        data.qpos[3:7] = [w, x, y, z]
        data.qpos[qadr] = states["dof_pos"][t, env]
        door = states["door_dof_pos"][t, env]
        data.qpos[hinge_adr] = door[0]
        data.qpos[handle_adr] = door[1] if len(door) > 1 else 0.0
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, cam)
        frames.append(renderer.render())
    write_video(out, frames, fps=fps)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--record", type=Path, required=True)
    ap.add_argument("--mjcf", type=Path, required=True)
    ap.add_argument("--env", type=int, default=0)
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--max-episodes", type=int, default=3)
    ap.add_argument("--min-stage", type=int, default=0, help="この区間まで進んだエピソードだけ書き出す")
    # 既定は廊下側（機体の左後ろ）から。部屋の側から見ると機体が扉の陰に隠れる。部屋の側は --azimuth 215
    ap.add_argument("--azimuth", type=float, default=35.0)
    ap.add_argument("--elevation", type=float, default=-20.0)
    ap.add_argument("--distance", type=float, default=4.0)
    args = ap.parse_args()

    static, states = load_record(args.record)
    k = args.env
    spec = mujoco.MjSpec.from_file(str(args.mjcf))
    add_door(spec, float(static["door_width"][k]), float(static["door_height"][k]),
             float(static["door_handle_height"][k]), float(static["door_handle_width"][k]),
             int(round(float(static["door_open_lr"][k]))))
    spec.visual.global_.offwidth = max(spec.visual.global_.offwidth, VIDEO_SIZE[0])
    spec.visual.global_.offheight = max(spec.visual.global_.offheight, VIDEO_SIZE[1])
    model = spec.compile()
    data = mujoco.MjData(model)
    names = [str(n) for n in static["dof_names"]]
    qadr = np.array([model.joint(n).qposadr[0] for n in names])
    hinge_adr = model.joint("door_hinge").qposadr[0]
    handle_adr = model.joint("door_handle_joint").qposadr[0]
    fps = int(round(1.0 / float(static["dt"])))

    args.out_dir.mkdir(parents=True, exist_ok=True)
    episodes = split_episodes(states["episode_length"][:, k])
    summary = []
    for n, (s, e) in enumerate(episodes):
        max_stage = int(states["stage"][s:e, k].max())
        max_hinge_deg = float(np.degrees(states["door_dof_pos"][s:e, k, 0].max()))
        summary.append({"episode": n, "steps": e - s, "max_stage": max_stage, "max_hinge_deg": round(max_hinge_deg, 1)})
    picked = [r for r in sorted(summary, key=lambda r: (r["max_stage"], r["max_hinge_deg"]), reverse=True)
              if r["max_stage"] >= args.min_stage][: args.max_episodes]
    for r in picked:
        s, e = episodes[r["episode"]]
        out = args.out_dir / f"env{k}_ep{r['episode']:03d}_stage{r['max_stage']}_hinge{r['max_hinge_deg']:.0f}deg.mp4"
        render_episode(model, data, states, k, (s, e), qadr, hinge_adr, handle_adr, out, fps,
                       (args.azimuth, args.elevation, args.distance))
        r["video"] = str(out)
    print("[replay_states] " + json.dumps({"episodes": len(episodes), "written": picked}, ensure_ascii=False))


if __name__ == "__main__":
    main()
