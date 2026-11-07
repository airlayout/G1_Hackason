"""離れた位置からゴミ箱を見つけて、立つ位置まで歩く流れを MuJoCo で試す（lib/approach.py の確認用）。

    G1_HuggingFace/venv/bin/python Trash_Bin/sim/approach_sim.py
    G1_HuggingFace/venv/bin/python Trash_Bin/sim/approach_sim.py --start 0.75 0.10 20 --seed 3
    G1_HuggingFace/venv/bin/python Trash_Bin/sim/approach_sim.py --then-push        # 着いたら押す流れまで続ける
    G1_HuggingFace/venv/bin/python Trash_Bin/sim/approach_sim.py --view             # 画面で見る
    G1_HuggingFace/venv/bin/python Trash_Bin/sim/approach_sim.py --view --then-push --back-wall-gap 0.3   # 近づいて押すまでを画面で

J1-gen のシミュレーションは pelvis を固定していて歩けない。そこで、ロボットが歩いた分だけゴミ箱を逆向きに動かして
（見え方は同じになる）、「頭カメラで測る → 少し歩く → 止まって測り直す」を繰り返す。
歩行のずれはわざと入れる（--walk-error: 進む量の倍率のばらつき、--slip: 横すべり・回り過ぎ）。測るのは毎回、
頭カメラの深度の画像から（lib/locate_bin.py）で、正解の値は使わない（最後の評価にだけ使う）。
"""

from __future__ import annotations

import argparse
import copy
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from push_bin_sim import BinSimBackend  # noqa: E402  （J1-gen と lib/ のパスも通る）
from approach import ApproachConfig, approach_step, stand_error, measured_motion, walk_summary # noqa: E402
from bin_geometry import BinPose, pose_from_config  # noqa: E402
from bin_scene import load_scene_config, start_pose  # noqa: E402
from common.camera_geometry import HeadCameraTransform  # noqa: E402
from common.config import load_config  # noqa: E402
from common.sim_camera import SimHeadCamera  # noqa: E402
from locate_bin import LocateError, locate_bin  # noqa: E402
from realism import Realism, corrupt, mirror_mask  # noqa: E402


def set_bin_pose(backend: BinSimBackend, pose: BinPose) -> None:
    """シミュレーションのゴミ箱を、pelvis 座標の置き方 pose に動かす（ロボットが歩いた代わり）。"""
    import mujoco

    m, d = backend.model, backend.data
    b = m.body("bin_body").id
    pel = d.body("pelvis").xpos
    m.body_pos[b] = [pel[0] + pose.center_xy[0], pel[1] + pose.center_xy[1], m.body_pos[b][2]]
    m.body_quat[b] = [np.cos(pose.yaw / 2), 0.0, 0.0, np.sin(pose.yaw / 2)]
    mujoco.mj_forward(m, d)


def animate_bin(backend: BinSimBackend, start: BinPose, end: BinPose, duration_s: float, dt: float = 0.02) -> None:
    """画面で見るとき用: ゴミ箱を start から end へ duration_s かけてなめらかに動かす（歩いて近づく様子の代わり）。"""
    n = max(1, int(duration_s / dt))
    for i in range(1, n + 1):
        r = i / n
        set_bin_pose(backend, BinPose(center_xy=(1 - r) * np.asarray(start.center_xy) + r * np.asarray(end.center_xy),
                                      yaw=float((1 - r) * start.yaw + r * end.yaw)))
        backend.tick(dt)
        time.sleep(dt)


def walk(pose: BinPose, vx: float, vy: float, vyaw: float, t: float, rng: np.random.Generator,
         walk_error: float, slip: float) -> tuple[BinPose, np.ndarray]:
    """ロボットが (vx, vy, vyaw) で t 秒歩いたとき、ゴミ箱が pelvis 座標でどう見えるか（歩行のずれ込み）。

    歩いた量（ずれ込み）M = (dx, dy, dyaw) に対して、新しい置き方 = M⁻¹ · 古い置き方。
    """
    scale = 1.0 + rng.uniform(-walk_error, walk_error, size=3)
    dx, dy, dyaw = vx * t * scale[0], vy * t * scale[1], vyaw * t * scale[2]
    dy += rng.normal(0.0, slip) * abs(vx * t)        # 前へ歩くと横へずれる
    dyaw += rng.normal(0.0, slip) * abs(vx * t) * 2  # 前へ歩くと少し曲がる
    c, s = np.cos(dyaw), np.sin(dyaw)
    rinv = np.array([[c, s], [-s, c]])
    center = rinv @ (np.asarray(pose.center_xy) - [dx, dy])
    return BinPose(center_xy=center, yaw=float(pose.yaw - dyaw)), np.array([dx, dy, dyaw])


def add_realism_args(p: argparse.ArgumentParser) -> None:
    """実機に近づける乱しの引数（sim/realism.py）。approach_sim.py と push_bin_sim.py で共通。"""
    p.add_argument("--depth-dropout", type=float, default=0.0, help="上面（鏡面）の深度を欠けさせる割合（0〜1）")
    p.add_argument("--depth-noise", type=float, default=0.0, help="深度のノイズ（1 m あたりの標準偏差 [m]。D435 は約 0.01）")
    p.add_argument("--cam-pitch-true", type=float,
                   help="シミュレーションの頭カメラの本当の下向きの角度 [deg]。位置の計算は robot.yaml の値（47.6°）のまま")
    p.add_argument("--calib-error-m", type=float, default=0.0, help="頭カメラの位置の計算に入れるずれ（各軸 ±この値の一様乱数）[m]")
    p.add_argument("--back-wall-gap", type=float,
                   help="ゴミ箱の背面から後ろの壁までの距離 [m]（既定はシーンの設定。実物は壁にほぼ付けて置いてある）")


def apply_scene_args(scene: dict, args: argparse.Namespace) -> None:
    """乱しの引数のうち、シーン（ゴミ箱の周り）に効くものを設定に当てる。"""
    if getattr(args, "back_wall_gap", None) is not None:
        scene["bin"]["back_wall_gap_m"] = args.back_wall_gap


def setup_realism(args: argparse.Namespace, robot: dict, rng: np.random.Generator) -> tuple[Realism, dict, Any]:
    """乱しの設定。(乱し, シミュレーションのカメラ用の robot 設定, 位置の計算に使う HeadCameraTransform) を返す。"""
    real = Realism(depth_dropout=args.depth_dropout, depth_noise=args.depth_noise)
    robot_true = copy.deepcopy(robot)
    if args.cam_pitch_true is not None:
        robot_true["head_camera"]["rpy"][1] = float(np.radians(args.cam_pitch_true))
    offset = rng.uniform(-args.calib_error_m, args.calib_error_m, size=3) if args.calib_error_m > 0 else np.zeros(3)
    tf = HeadCameraTransform(robot, offset)  # 計算は「知っている」値（robot.yaml）で行う
    if real.active() or args.cam_pitch_true is not None or args.calib_error_m > 0:
        print(f"[realism] 深度の欠け {args.depth_dropout:.0%}、ノイズ {args.depth_noise} m/m²、"
              f"カメラの本当の角度 {args.cam_pitch_true or np.degrees(robot['head_camera']['rpy'][1]):.1f}°"
              f"（計算は {np.degrees(robot['head_camera']['rpy'][1]):.1f}°）、較正のずれ {np.round(offset * 100, 1)} cm")
    return real, robot_true, tf


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scene", help="ゴミ箱の設定（既定は Trash_Bin/configs/bin_scene.yaml）")
    p.add_argument("--start", type=float, nargs=3, default=[0.70, 0.05, 15.0], metavar=("FRONT_X", "CENTER_Y", "YAW"),
                   help="最初のゴミ箱の置き方（正面の x [m]、中心の y [m]、向き [deg]。pelvis 座標）")
    p.add_argument("--walk-error", type=float, default=0.15, help="歩いた量の倍率のばらつき（0.15 なら ±15%%）")
    p.add_argument("--slip", type=float, default=0.10, help="前へ歩いたときの横すべり・回り過ぎ（歩いた距離に対する割合の標準偏差）")
    p.add_argument("--seed", type=int, default=0)
    add_realism_args(p)
    p.add_argument("--then-push", action="store_true", help="着いたら、押す流れ（push_bin_sim.py --locate）まで続ける")
    p.add_argument("--view", action="store_true", help="画面で見る")
    args = p.parse_args()

    scene = load_scene_config(args.scene)
    apply_scene_args(scene, args)
    target = pose_from_config(scene)  # 立つ位置 = ゴミ箱がこの置き方に見える位置
    cfg = ApproachConfig.from_dict(scene.get("approach"))
    d = float(scene["bin"]["depth"])
    truth = BinPose(center_xy=np.array([args.start[0] + d / 2, args.start[1]]), yaw=float(np.radians(args.start[2])))

    robot, arm = load_config("robot.yaml"), load_config("arm.yaml")
    rng = np.random.default_rng(args.seed)
    real, robot_true, tf = setup_realism(args, robot, rng)
    sc0 = copy.deepcopy(scene)
    sim = dict(arm["sim"])
    sim.update(scene=sc0, initial_q=list(start_pose(scene, np.zeros(29))), realtime=False, viewer=args.view)
    backend = BinSimBackend(robot_true, sim, hold_kp=np.asarray(arm["lowcmd"]["hold_kp"], float),
                            hold_kd=np.asarray(arm["lowcmd"]["hold_kd"], float))
    backend.open()
    camera = SimHeadCamera(backend.model, backend.data, robot_true)
    set_bin_pose(backend, truth)
    print(f"[approach] 最初の置き方（正解）: 中心 [{truth.center_xy[0]:+.3f}, {truth.center_xy[1]:+.3f}] m、"
          f"向き {np.degrees(truth.yaw):+.1f}°。立つ位置までのずれ（正解）: {np.round(stand_error(truth, target)[:2] * 100, 1)} cm、"
          f"{np.degrees(stand_error(truth, target)[2]):+.1f}°")

    prior, lost, ok = None, 0, False
    last = None  # (前回測った置き方, 指令した量, 本当に動いた量)
    walked, truths = [], []
    for k in range(1, cfg.max_steps + 1):
        backend.tick(0.02)
        frame = camera.render()
        if real.active():
            frame = corrupt(frame, mirror_mask(backend.model, backend.data, camera._renderer, camera.name), real, rng)
        q_waist = backend.read_state().q[12:15]
        t0 = time.monotonic()
        try:
            det = locate_bin(frame, tf, q_waist, scene, prior=prior)
        except LocateError as e:
            lost += 1
            print(f"[approach] {k} 回目: 見つからない（{e}）")
            if lost >= 2:
                print("[approach] ❌ 2 回続けて見つからないので止める")
                break
            prior = None
            continue
        lost = 0
        if last is not None:
            walked.append((last[1], measured_motion(last[0], det.pose)))
            truths.append(last[2])
        last = None
        prior = det.pose
        meas_err = np.linalg.norm(det.pose.center_xy - truth.center_xy) * 100
        step = approach_step(det.pose, target, cfg)
        print(f"[approach] {k} 回目（測る {time.monotonic() - t0:.2f} 秒、測定の誤差 {meas_err:.1f} cm"
              f"{'、一部だけ' if det.partial else ''}）: {step.summary()}")
        if step.done:
            ok = True
            break
        new_truth, moved = walk(truth, step.vx, step.vy, step.vyaw, step.duration_s, rng, args.walk_error, args.slip)
        last = (det.pose, np.array([step.vx, step.vy, step.vyaw]) * step.duration_s, moved)
        if backend.viewer_running():
            animate_bin(backend, truth, new_truth, step.duration_s)
            time.sleep(0.5)  # 止まって測る間
        truth = new_truth
        set_bin_pose(backend, truth)
        # 次に測るときは、歩いた（つもりの）分だけ前回の置き方を動かして、探す範囲の中心にする
        planned = np.array([step.vx, step.vy, step.vyaw]) * step.duration_s
        prior, _ = walk(prior, step.vx, step.vy, step.vyaw, step.duration_s, np.random.default_rng(0), 0.0, 0.0)
        del planned, moved

    if walked:
        print("[approach] 歩いた量（指令 と 頭カメラで測った量。real/approach_bin.py と同じ集計）:")
        for line in walk_summary(walked):
            print(f"[approach]   {line}")
        err = np.array([m - t for (_, m), t in zip(walked, truths)])
        print(f"[approach]   測った量と本当に動いた量の差（最大）: 前 {np.abs(err[:, 0]).max() * 100:.1f} cm・"
              f"左 {np.abs(err[:, 1]).max() * 100:.1f} cm・{np.degrees(np.abs(err[:, 2]).max()):.1f}°")
    final = stand_error(truth, target)
    print(f"[approach] {'着いた' if ok else '着かなかった'}。最後の立つ位置のずれ（正解）: 前 {final[0] * 100:+.1f} cm・"
          f"左 {final[1] * 100:+.1f} cm・向き {np.degrees(final[2]):+.1f}°")
    camera.close()
    if backend.viewer_running() and not (ok and args.then_push):
        print("[approach] 画面を閉じると終わる")
        while backend.viewer_running():
            backend.tick(0.02)
    backend.close()

    if ok and args.then_push:
        # 着いた置き方（正解）で、押す流れを最初から（深度で測るところから）流す
        import push_bin_sim

        front = truth.center_xy[0] - d / 2
        sys.argv = [sys.argv[0], "--locate", "--bin-front", f"{front:.4f}", "--bin-y", f"{truth.center_xy[1]:.4f}",
                    "--bin-yaw", f"{np.degrees(truth.yaw):.3f}", "--label", f"after_approach{args.seed}", "--seed", str(args.seed + 1000),
                    "--depth-dropout", str(args.depth_dropout), "--depth-noise", str(args.depth_noise),
                    "--calib-error-m", str(args.calib_error_m)]
        if args.cam_pitch_true is not None:
            sys.argv += ["--cam-pitch-true", str(args.cam_pitch_true)]
        if args.back_wall_gap is not None:
            sys.argv += ["--back-wall-gap", str(args.back_wall_gap)]
        if args.view:
            sys.argv += ["--view"]
        print(f"[approach] 押す流れへ: {' '.join(sys.argv[1:])}")
        return push_bin_sim.main()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
