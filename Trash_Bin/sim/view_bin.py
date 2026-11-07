"""ゴミ箱を押す様子を MuJoCo の画面で見る（確認用。胴体固定・右腕）。

    G1_HuggingFace/venv/bin/python Trash_Bin/sim/view_bin.py                 # 右斜め前のゴミ箱を押す（設定の既定）
    G1_HuggingFace/venv/bin/python Trash_Bin/sim/view_bin.py --no-move       # 置いた様子だけ見る
    G1_HuggingFace/venv/bin/python Trash_Bin/sim/view_bin.py --camera head_camera   # 頭カメラの視点で見る
    G1_HuggingFace/venv/bin/python Trash_Bin/sim/view_bin.py --bin-front 0.15 --bin-y -0.125 --inset 0.035 --tilt 0 0
                                                                             # 正面に置く（押せない様子が見える）
    G1_HuggingFace/venv/bin/python Trash_Bin/sim/view_bin.py --save-images _local/trash_bin/view
                                                                             # 押し切った瞬間の画像も保存する

push_bin_sim.py（J1-gen の PressPipeline）は、計画が安全基準を満たさないと腕を動かさない。こちらは**基準を満たさなくても、
IK で解いた姿勢へそのまま動かして見せる**。どこがどれだけ足りないか（枠との距離、関節の可動域の端）は端末に出す。
実機の動作には使わないこと（安全の確認を飛ばしている）。

画面の操作:
- マウス: 左ドラッグで回転、右ドラッグで平行移動、ホイールで拡大・縮小
- ] / [ キー: カメラを切り替える（head_camera にすると頭カメラの見え方になる。Esc で自由な視点に戻る）
- 赤い小さな球が押す点。接触している点も表示する
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from push_bin_sim import BinSimBackend  # noqa: E402  （J1-gen と lib/ を import できるようにパスも通る）
from bin_task import finger_axis  # noqa: E402
from bin_scene import CONTENT, FLAP, bin_obstacles, flap_angle_deg, load_scene_config, push_point, start_pose  # noqa: E402
from common.arm.types import JointCommand  # noqa: E402
from hand_shape import install as install_hand_shape  # noqa: E402
from common.collision import CollisionChecker  # noqa: E402
from common.config import load_config  # noqa: E402
from common.kinematics import ArmKinematics  # noqa: E402

DT = 0.02  # 指令の周期 [秒]（50 Hz）
WAIST_PITCH = 14  # motor 番号


def solve(kin: ArmKinematics, checker: CollisionChecker, point: np.ndarray, axis: np.ndarray,
          seeds: list[np.ndarray], what: str) -> tuple[np.ndarray, bool]:
    """IK を初期値ごとに解き、一番よいもの（と安全基準を満たすか）を返す。満たさなくても返す（表示用）。"""
    best = None
    for s in seeds:
        r = kin.ik(point, s, axis)
        near = checker.contacts(r.q)
        score = (not r.success, r.pos_err + np.degrees(r.axis_err) * 1e-3, len(near))
        if best is None or score < best[0]:
            best = (score, r, near)
    _, r, near = best
    ok = r.success and not near
    msg = f"[view] {what}: {'OK' if ok else 'NG'}（位置の誤差 {r.pos_err * 1000:.1f} mm、指の向きの誤差 {np.degrees(r.axis_err):.1f}°"
    if r.at_limit:
        msg += f"、可動域の端: {', '.join(r.at_limit)}"
    msg += "）"
    print(msg)
    for c in sorted(near, key=lambda c: c.dist)[:3]:
        print(f"[view]   近すぎる: {c}")
    return r.q, ok


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scene", help="シーンの設定（既定は Trash_Bin/configs/bin_scene.yaml）")
    p.add_argument("--bin-front", type=float, help="ゴミ箱の正面の x [m]（pelvis 座標）")
    p.add_argument("--bin-y", type=float, help="ゴミ箱の中心の y [m]（pelvis 座標。右が −）")
    p.add_argument("--waist-pitch", type=float, help="押す前に腰を前に傾ける角度 [deg]（既定はシーンの設定の lean_waist_pitch_deg。0 なら傾けない）")
    p.add_argument("--lift-m", type=float, help="ゴミ箱を載せる台の高さ [m]（既定はシーンの設定の bin.stand_height_m）")
    p.add_argument("--half", choices=["right", "left"], help="フラップのどちらの半分を押すか（既定は右）")
    p.add_argument("--inset", type=float, help="開口の手前の縁から押す点までの奥行 [m]")
    p.add_argument("--from-axis", type=float, help="フラップの軸から押す点までの距離 [m]")
    p.add_argument("--depth-mm", type=float, help="押し込む深さ [mm]")
    p.add_argument("--approach-mm", type=float, help="押す点の何 mm 上から真下へ下ろすか（既定は設定の approach_m）")
    p.add_argument("--tilt", type=float, nargs=2, metavar=("BACK", "RIGHT"),
                   help="指の向き: 真下から手前へ BACK°、右へ RIGHT°（負なら前・左）。既定は設定の finger_tilts_deg の最初")
    p.add_argument("--via", help='経由する姿勢（右腕の 7 つの角度 deg の JSON。例: "[60,-20,0,-20,0,0,0]"）')
    p.add_argument("--cam-pitch-deg", type=float, help="頭カメラの下向きの角度 [deg]（既定は robot.yaml の 47.6°）")
    p.add_argument("--cam-fovy-deg", type=float,
                   help="頭カメラの縦の画角 [deg]（既定は robot.yaml の 42.6 = D435 のカラー。深度・赤外線は約 58）")
    p.add_argument("--cam-size", type=int, nargs=2, metavar=("W", "H"),
                   help="頭カメラの画像の大きさ（既定は robot.yaml の 640 480。D435 のカラーは 16:9 の 848 480 で横の画角が広い）")
    p.add_argument("--speed-mm-s", type=float, help="押し込みの速さ [mm/s]（既定は設定の speed_m_s）")
    p.add_argument("--stop-lag-mm", type=float,
                   help="押し込みを止める遅れ [mm]（指令した指先が実際の指先より先に出た量の増え。既定は設定の stop_lag_m）")
    p.add_argument("--no-move", action="store_true", help="腕を動かさず、置いた様子だけ見る")
    p.add_argument("--camera", help="この視点で画面を開く（head_camera: 頭カメラ、wrist_camera: 手首カメラ）。] キーでも切り替えられる")
    p.add_argument("--save-images", help="押し切った瞬間の画像（外からの視点・頭カメラ）を保存するフォルダ")
    p.add_argument("--no-viewer", action="store_true", help="画面を開かずに同じ処理をする（結果の数字だけ見る）")
    args = p.parse_args()

    scene = load_scene_config(args.scene)
    if args.bin_front is not None:
        scene["bin"]["front_x"] = args.bin_front
    if args.bin_y is not None:
        scene["bin"]["center_y"] = args.bin_y
    if args.waist_pitch is not None:
        scene["lean_waist_pitch_deg"] = args.waist_pitch
    if args.lift_m is not None:
        scene["bin"]["stand_height_m"] = args.lift_m  # ゴミ箱を載せる台の高さ
    for key, val in (("half", args.half), ("inset_m", args.inset), ("from_axis_m", args.from_axis)):
        if val is not None:
            scene["push"][key] = val
    if args.depth_mm is not None:
        scene["push"]["depth_m"] = args.depth_mm / 1000.0

    robot, arm_cfg, press = load_config("robot.yaml"), load_config("arm.yaml"), load_config("press.yaml")
    if args.cam_pitch_deg is not None:
        robot["head_camera"]["rpy"][1] = float(np.radians(args.cam_pitch_deg))
    if args.cam_fovy_deg is not None:
        robot["head_camera"]["fovy_deg"] = args.cam_fovy_deg
    if args.cam_size is not None:
        robot["head_camera"]["width"], robot["head_camera"]["height"] = args.cam_size
    side = "right"
    if scene.get("hand_shape", "box") == "parts":
        install_hand_shape(robot)
    sim_cfg = dict(arm_cfg["sim"])
    sim_cfg.update(scene=scene, initial_q=list(start_pose(scene, np.zeros(29))), realtime=not args.no_viewer,
                   viewer=not args.no_viewer)
    backend = BinSimBackend(robot, sim_cfg, hold_kp=np.asarray(arm_cfg["lowcmd"]["hold_kp"], dtype=float),
                            hold_kd=np.asarray(arm_cfg["lowcmd"]["hold_kd"], dtype=float))
    backend.open()
    import mujoco

    if backend._viewer is not None:
        with backend._viewer.lock():
            backend._viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 1
            if args.camera:
                backend._viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                backend._viewer.cam.fixedcamid = backend.model.camera(args.camera).id
    target = push_point(scene, side)
    print(f"[view] 押す点（pelvis 座標）: {np.round(target, 3).tolist()} m、床から {target[2] + scene['pelvis_height_m']:.3f} m")

    if not args.no_move:
        run_motion(args, scene, robot, arm_cfg, press, backend, target)

    print("[view] 画面を閉じると終わる（Ctrl+C でも終わる）")
    try:
        while backend.viewer_running():
            backend.tick(DT)  # フラップなどが戻る様子も見えるように進め続ける
    except KeyboardInterrupt:
        pass
    backend.close()
    return 0


def run_motion(args: argparse.Namespace, scene: dict, robot: dict, arm_cfg: dict, press: dict,
               backend: BinSimBackend, target: np.ndarray) -> None:
    side = "right"
    kin = ArmKinematics(robot, side, press["ik"])
    checker = CollisionChecker(robot, side, float(press["collision"]["clearance_m"]), bin_obstacles(scene))
    idx = list(kin.arm_idx)
    down = np.array([0.0, 0.0, -1.0])
    approach_m = args.approach_mm / 1000.0 if args.approach_mm is not None else float(scene["push"].get("approach_m", 0.08))
    speed = args.speed_mm_s / 1000.0 if args.speed_mm_s is not None else float(scene["push"].get("speed_m_s", 0.02))
    approach = target + np.array([0, 0, approach_m])
    end = target + down * float(scene["push"]["depth_m"])

    q0 = backend.read_state().q.copy()
    # 押す前に腰を前に傾ける（lib/lean.py と同じ）。IK は傾けたあとの腰の角度で解く
    lean = float(np.radians(float(scene.get("lean_waist_pitch_deg") or 0.0)))
    q_lean = q0.copy()
    q_lean[WAIST_PITCH] += lean
    zero = q_lean.copy()
    zero[idx] = 0.0
    seeds = [q_lean, zero]
    # 経由する姿勢（押す流れと同じ。--via が無ければ設定の push.via_pose）。開始 → 経由 → 押す点の上 の順に動く
    via_cfg = json.loads(args.via) if args.via else scene["push"].get("via_pose")
    q_via = None
    if via_cfg is not None:
        q_via = q_lean.copy()
        q_via[idx] = 0.0 if via_cfg == "zero" else np.radians(np.asarray(via_cfg, dtype=float))
        seeds.insert(0, q_via)
    # 指の向きの候補を順に試し、押す点の上の姿勢が安全基準を満たす最初のものを使う（計画の流れと同じ）
    tilts = [args.tilt] if args.tilt else (scene["push"].get("finger_tilts_deg") or [[0.0, 0.0]])
    for tilt in tilts:
        print(f"[view] 指の向き: 真下から手前へ {tilt[0]:.0f}°、右へ {tilt[1]:.0f}°")
        axis = finger_axis(*tilt)
        q_app, ok = solve(kin, checker, approach, axis, seeds, f"押す点の {approach_m * 1000:.0f} mm 上")
        if ok:
            break
    # 真下への直線（上 → 押し込み）を、前の点の解を初期値にして順に解く
    n = max(2, int(np.ceil(np.linalg.norm(end - approach) / (speed * DT))))
    line, q = [], q_app
    worst = (np.inf, "")
    for i in range(1, n + 1):
        r = kin.ik(approach + (end - approach) * i / n, q, axis, max_iter=50)
        q = r.q
        line.append(q)
        if i % 10 == 0 or i == n:
            for c in checker.contacts(q):
                if c.dist < worst[0]:
                    worst = (c.dist, str(c))
    print(f"[view] 押し込みの直線（{n} 点）: 最後の点の位置の誤差 {r.pos_err * 1000:.1f} mm"
          + (f"、可動域の端: {', '.join(r.at_limit)}" if r.at_limit else ""))
    if worst[1]:
        print(f"[view]   一番近づいた組: {worst[1]}")

    # 腕の指令（右腕だけ、arm_sdk の weight = 1）
    gains = arm_cfg["gains"] if "gains" in arm_cfg else {}
    kp = np.zeros(29)
    kd = np.zeros(29)
    for j, i in enumerate(idx):
        g = (gains.get("wrist") if j >= 4 else gains.get("arm")) or {"kp": 60.0, "kd": 1.5}
        kp[i], kd[i] = float(g["kp"]), float(g["kd"])
    wh = arm_cfg["arm_sdk"]["waist_hold"]  # 腰は J1-gen と同じ保持のゲインで指令する
    kp[WAIST_PITCH], kd[WAIST_PITCH] = float(wh["kp"]), float(wh["kd"])
    cmd = JointCommand(q=q0.copy(), kp=kp, kd=kd, weight=1.0)

    hits: dict[str, float] = {}

    def step() -> None:
        backend.send(cmd)
        backend.tick(DT)
        record_contacts()

    def go(q_to: np.ndarray, sec: float) -> None:
        q_from = cmd.q.copy()
        k = max(1, int(sec / DT))
        for s in range(1, k + 1):
            a = 0.5 - 0.5 * np.cos(np.pi * s / k)
            cmd.q = q_from + (q_to - q_from) * a
            step()

    def hold(sec: float) -> None:
        for _ in range(int(sec / DT)):
            step()

    def waist_to(target: float) -> None:
        """腰の pitch を、最大 5°/s でゆっくり動かす（lib/lean.py と同じ速さ）。"""
        p0 = float(cmd.q[WAIST_PITCH])
        sec = max(1.0, 1.5 * abs(target - p0) / np.radians(5.0))
        k = max(1, int(sec / DT))
        for s in range(1, k + 1):
            a = (s / k) ** 2 * (3 - 2 * s / k)
            cmd.q[WAIST_PITCH] = p0 + (target - p0) * a
            step()
        hold(1.0)

    def record_contacts() -> None:
        m, d = backend.model, backend.data
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            n1, n2 = m.geom(c.geom1).name, m.geom(c.geom2).name
            names = [x for x in (n1, n2) if x.startswith("bin")]
            if not names:
                continue
            other = m.body(m.geom_bodyid[c.geom2 if n1 in names else c.geom1]).name
            mujoco_contact_force(m, d, i, f6)
            key = f"{other} ↔ {names[0]}"
            hits[key] = max(hits.get(key, 0.0), abs(float(f6[0])))

    print("[view] 動かす: 開始の姿勢 → 腰を傾ける → 経由の姿勢 → 押す点の上 → 真下へ押し込む → 保持 → 戻る → 腰を戻す")
    hold(1.5)
    if lean:
        print(f"[view] 腰を前に {np.degrees(lean):.0f}° 傾ける")
        waist_to(q_lean[WAIST_PITCH])
    if q_via is not None:
        go(q_via, 3.0)
    go(q_app, 3.0)
    # 押し込み: 指先が進まなくなったら（指令した指先が実際の指先より押す向きに lag_m 以上先に出たら）それ以上押さず、
    # 今の実際の姿勢を保つ。J1-gen の contact_stop と同じ考え方（決めた深さまで押し続けると、フラップが逃げたあとも
    # 腕が押し付けて接触力が 80 N まで上がった。2026-11-05）
    lag_m = args.stop_lag_mm / 1000.0 if args.stop_lag_mm is not None else float(scene["push"]["stop_lag_m"])
    baseline = None
    for qq in line:
        cmd.q = qq
        step()
        q_meas = backend.read_state().q
        lag = float((kin.fk_pos(cmd.q) - kin.fk_pos(q_meas)) @ down)
        baseline = lag if baseline is None else baseline
        if lag - baseline > lag_m:
            print(f"[view] 指先が進まなくなった（遅れの増え {(lag - baseline) * 1000:.1f} mm）。ここで止めて保持する")
            cmd.q = q_meas.copy()
            break
    hold(1.0)
    m, d = backend.model, backend.data
    tip = kin.fk_pos(backend.read_state().q)
    print(f"[view] 押し込んだ位置: 指先の目標とのずれ {np.linalg.norm(tip - end) * 1000:.1f} mm、"
          f"フラップ {abs(flap_angle_deg(m, d)):.1f}°（途中の最大 {backend.flap_max_deg:.1f}°）")
    print(f"[view] 頭カメラに写った中身の画素数: {visible_pixels(backend, robot)}")
    if args.save_images:
        save_images(backend, robot, Path(args.save_images))
    for k, f in sorted(hits.items(), key=lambda kv: -kv[1]):
        mark = "" if FLAP in k else "  ← フラップ以外に当たっている"
        print(f"[view]   接触 {k}: 最大 {f:.1f} N{mark}")
    hold(1.0)
    go(q_app, 2.0)
    q_back = q0.copy()
    q_back[WAIST_PITCH] = cmd.q[WAIST_PITCH]  # 腕を戻すあいだは、腰は傾けたまま
    if q_via is not None:
        q_v = q_via.copy()
        q_v[WAIST_PITCH] = cmd.q[WAIST_PITCH]
        go(q_v, 3.0)
    go(q_back, 3.0)
    if lean:
        print("[view] 腰を戻す")
        waist_to(q0[WAIST_PITCH])


def mujoco_contact_force(m, d, i: int, out: np.ndarray) -> None:  # noqa: ANN001
    import mujoco

    mujoco.mj_contactForce(m, d, i, out)


def save_images(backend: BinSimBackend, robot: dict, out_dir: Path) -> None:
    """今の状態を、外からの視点（右斜め前・真上）と頭カメラで描いて PNG に保存する。"""
    import cv2
    import mujoco

    m, d = backend.model, backend.data
    out_dir.mkdir(parents=True, exist_ok=True)
    r = mujoco.Renderer(m, 480, 640)
    pelvis = d.body("pelvis").xpos
    views = {"front_right": (1.4, 220.0, -20.0), "top": (1.2, 180.0, -80.0)}
    for name, (dist, azim, elev) in views.items():
        cam = mujoco.MjvCamera()
        cam.lookat[:] = pelvis + np.array([0.1, -0.3, -0.25])
        cam.distance, cam.azimuth, cam.elevation = dist, azim, elev
        r.update_scene(d, camera=cam)
        cv2.imwrite(str(out_dir / f"{name}.png"), cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR))
    r.close()
    cam_cfg = robot["head_camera"]
    r = mujoco.Renderer(m, int(cam_cfg["height"]), int(cam_cfg["width"]))
    r.update_scene(d, camera=cam_cfg["name"])
    cv2.imwrite(str(out_dir / "head_camera.png"), cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR))
    r.close()
    print(f"[view] 画像を保存した: {out_dir}（front_right.png / top.png / head_camera.png）")


def visible_pixels(backend: BinSimBackend, robot: dict) -> dict[str, int]:
    """頭カメラの画像で、中身の geom が写っている画素の数。"""
    import mujoco

    m, d = backend.model, backend.data
    cam = robot["head_camera"]
    r = mujoco.Renderer(m, int(cam["height"]), int(cam["width"]))
    r.enable_segmentation_rendering()
    r.update_scene(d, camera=cam["name"])
    seg = r.render()
    r.close()
    out = {}
    for g in range(m.ngeom):
        name = m.geom(g).name
        if name.startswith(CONTENT):
            out[name[len(CONTENT):]] = int(((seg[:, :, 0] == g) & (seg[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))).sum())
    return out


if __name__ == "__main__":
    sys.exit(main())
