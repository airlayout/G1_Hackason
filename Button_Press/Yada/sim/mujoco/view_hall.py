"""MuJoCo のエレベーター乗り場のシーンを見る・確かめる。

    # 画面で見る（ボタンは Ctrl + 右ドラッグで押せる。押すと点灯する。R で消灯）
    python Button_Press/Yada/sim/mujoco/view_hall.py
    # 画像を保存する（頭カメラと全体の 2 枚。画面は開かない）
    python Button_Press/Yada/sim/mujoco/view_hall.py --png _local/button_press_yada/hall
    # ボタンを外から押して、沈み量と点灯を確かめる（画面は開かない）
    python Button_Press/Yada/sim/mujoco/view_hall.py --press up --force 6

ロボット（胴体固定）の関節は毎ステップ初期の姿勢に戻す（歩行も腕の制御もまだ無いため）。
MuJoCo の描画に画面（DISPLAY）を使うので、画面の無い端末では MUJOCO_GL=egl などを指定する。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from common.config import REPO_ROOT, load_config  # noqa: E402
from common.j1gen_bridge import j1gen, j1gen_config  # noqa: E402
from common.scene_spec import build_hall_scene  # noqa: E402
from mujoco_hall import OVERVIEW_CAMERA, HallMujoco, build_hall_model  # noqa: E402


def make(scene_file: str):
    import mujoco

    cfg = load_config(scene_file)
    scene = build_hall_scene(cfg)
    robot_cfg = j1gen_config("robot.yaml")
    model = build_hall_model(scene, robot_cfg, cfg.get("fingertip"))
    data = mujoco.MjData(model)
    rm = j1gen("robot_model")
    for idx, deg in cfg.get("initial_pose_deg", {}).items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, rm.JOINT_NAMES[int(idx)])
        data.qpos[model.jnt_qposadr[jid]] = np.radians(float(deg))
    mujoco.mj_forward(model, data)
    return scene, robot_cfg, model, data, HallMujoco(model, data, scene)


def hold_robot(data, idx: np.ndarray, q0: np.ndarray) -> None:
    data.qpos[idx] = q0
    data.qvel[idx] = 0.0


def save_png(model, data, robot_cfg, out_prefix: Path) -> list[Path]:
    import mujoco
    from PIL import Image

    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    cam = robot_cfg["head_camera"]
    paths = []
    for name, (w, h) in ((cam["name"], (cam["width"], cam["height"])), (OVERVIEW_CAMERA, (960, 720))):
        r = mujoco.Renderer(model, height=h, width=w)
        r.update_scene(data, camera=name)
        p = out_prefix.with_name(f"{out_prefix.name}_{name}.png")
        Image.fromarray(r.render()).save(p)
        r.close()
        paths.append(p)
    return paths


def press_test(model, data, hall: HallMujoco, name: str, force: float, seconds: float = 0.5) -> None:
    """ボタンの body に +x（壁に向かう向き）の力を seconds 秒かけ、離して seconds 秒待つ。"""
    import mujoco

    idx = hall.robot_qpos_slice()
    q0 = data.qpos[idx].copy()
    bid = hall.body_id(name)
    steps = int(seconds / model.opt.timestep)
    max_depth = 0.0
    for phase, f in (("押す", force), ("離す", 0.0)):
        for _ in range(steps):
            data.xfrc_applied[bid, :3] = [f, 0.0, 0.0]
            mujoco.mj_step(model, data)
            hold_robot(data, idx, q0)
            for n in hall.update():
                print(f"[press] {n} が押された（t = {data.time:.3f} s）")
            max_depth = max(max_depth, hall.depth(name))
        print(f"[press] {phase}: 力 {f:.1f} N、沈み量 {hall.depth(name) * 1000:.2f} mm、"
              f"点灯 {hall.lit(name)}（最大の沈み量 {max_depth * 1000:.2f} mm）")
    data.xfrc_applied[bid, :3] = 0.0


def view(model, data, hall: HallMujoco) -> None:
    import mujoco
    import mujoco.viewer

    idx = hall.robot_qpos_slice()
    q0 = data.qpos[idx].copy()
    reset = {"flag": False}

    def key_cb(keycode: int) -> None:
        if keycode == ord("R"):
            reset["flag"] = True

    with mujoco.viewer.launch_passive(model, data, key_callback=key_cb) as v:
        while v.is_running():
            t0 = time.time()
            if reset["flag"]:
                hall.reset_lights()
                reset["flag"] = False
                print("[view] 消灯した")
            mujoco.mj_step(model, data)
            hold_robot(data, idx, q0)
            for n in hall.update():
                print(f"[view] {n} が押された")
            v.sync()
            time.sleep(max(0.0, model.opt.timestep - (time.time() - t0)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", default="elevator_prod.yaml",
                    help="シーンの設定（configs/ 基準）。elevator_prod.yaml = 本番に似た乗り場、elevator_hall.yaml = 前の盤")
    ap.add_argument("--png", help="画像を保存する（パスの先頭。_<カメラ名>.png を付けて保存）")
    ap.add_argument("--press", help="このボタンを外から押して確かめる（up / down）")
    ap.add_argument("--force", type=float, default=6.0, help="--press で押す力 [N]")
    args = ap.parse_args()

    scene, robot_cfg, model, data, hall = make(args.scene)
    print(f"[hall] ボタン: {[b.name for b in scene.buttons]}、"
          f"面の中心（pelvis 座標）: {[b.face_center.round(3).tolist() for b in scene.buttons]}")
    if args.press:
        press_test(model, data, hall, args.press, args.force)
    if args.png:
        prefix = Path(args.png)
        if not prefix.is_absolute():
            prefix = REPO_ROOT / prefix
        for p in save_png(model, data, robot_cfg, prefix):
            print(f"[png] {p}")
    if not args.press and not args.png:
        view(model, data, hall)
    return 0


if __name__ == "__main__":
    sys.exit(main())
