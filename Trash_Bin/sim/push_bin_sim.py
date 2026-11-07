"""ゴミ箱のフラップを真下に押す動作を MuJoCo で試す（胴体固定・右腕）。

    G1_HuggingFace/venv/bin/python Trash_Bin/sim/push_bin_sim.py
    G1_HuggingFace/venv/bin/python Trash_Bin/sim/push_bin_sim.py --depth-mm 35 --inset 0.05
    G1_HuggingFace/venv/bin/python Trash_Bin/sim/push_bin_sim.py --view          # 画面で動きを見る
    G1_HuggingFace/venv/bin/python Trash_Bin/sim/push_bin_sim.py --cam-pitch-deg 60

押す流れ（IK・衝突の確認・経由の姿勢・押し込み・戻り）は Button_Press/J1-gen の PressPipeline をそのまま使う
（import するだけで、J1-gen のファイルは変えない）。ここで足すのは次の 3 つ:
- シーン: 机とボトルの代わりにゴミ箱（sim/bin_scene.py、configs/bin_scene.yaml）
- 対象: 検出の代わりに、設定から求めた押す点（manual）。押す向きは真下
- 測定: 押している間のフラップの角度と、押し切ったときに頭カメラに写った中身の画素数

記録は _local/trash_bin/runs/ に出る（J1-gen と同じ形。頭カメラの画像は frames/ の下）。
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "Button_Press" / "J1-gen"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(REPO / "Trash_Bin" / "lib"))

from bin_scene import CONTENT, add_bin_scene, flap_angle_deg, load_scene_config, pose_from_config, start_pose  # noqa: E402
from bin_task import BinPipeline, apply_measured_pelvis_height, configure_pipeline, finger_axis, install_leaning  # noqa: E402,F401
from common import sim_scene  # noqa: E402
from common.arm.backend_sim import SimBackend  # noqa: E402
from common.camera_geometry import HeadCameraTransform  # noqa: E402
from common.pipeline import PipelineResult  # noqa: E402
from common.pipeline_cli import add_common_args, load_pipeline_config, make_logger  # noqa: E402
from common.press_planner import PressPlan  # noqa: E402
from common.sim_camera import SimHeadCamera  # noqa: E402
from hand_shape import install as install_hand_shape  # noqa: E402
from locate_bin import LocateError, locate_bin  # noqa: E402


class BinSimBackend(SimBackend):
    """J1-gen の SimBackend の、シーンをゴミ箱にしたもの。押している間のフラップの角度も記録する。

    SimBackend.open() は、シーンの組み立てに common.sim_scene.add_scene（机とボトル）を呼ぶ。
    J1-gen のファイルは変えずにシーンだけ差し替えるため、open() の間だけその関数をゴミ箱のものに置き換える。
    """

    def open(self) -> None:
        orig = sim_scene.add_scene
        sim_scene.add_scene = add_bin_scene
        try:
            super().open()
        finally:
            sim_scene.add_scene = orig
        # 頭カメラを 16:9（848×480 など）で描けるように、画面外で描く大きさの上限を広げる（既定は 640×480）
        self.model.vis.global_.offwidth = max(int(self.model.vis.global_.offwidth), 1280)
        self.model.vis.global_.offheight = max(int(self.model.vis.global_.offheight), 960)
        self.flap_max_deg = 0.0
        self.bin_contacts: dict[str, float] = {}  # フラップ以外のゴミ箱と体の接触（組ごとの最大の力 [N]）

    def tick(self, dt: float) -> None:
        super().tick(dt)
        self.flap_max_deg = max(self.flap_max_deg, abs(flap_angle_deg(self.model, self.data)))
        self._record_contacts()

    def _record_contacts(self) -> None:
        import mujoco

        m, d = self.model, self.data
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            n1, n2 = m.geom(c.geom1).name, m.geom(c.geom2).name
            bin_name = next((x for x in (n1, n2) if x.startswith("bin") and not x.startswith("bin_flap")), None)
            if bin_name is None:
                continue
            other = m.geom_bodyid[c.geom2 if n1 == bin_name else c.geom1]
            mujoco.mj_contactForce(m, d, i, f6)
            key = f"{m.body(other).name}<->{bin_name}"
            self.bin_contacts[key] = max(self.bin_contacts.get(key, 0.0), round(abs(float(f6[0])), 1))


class BinSimPipeline(BinPipeline):
    """lib/bin_task.py の BinPipeline に、シミュレーションでの測定を足したもの:
    押し切った時点（段階 end）で、フラップの角度と、頭カメラに写った中身の画素数を測る。
    """

    def __init__(self, *args: Any, camera: SimHeadCamera, sim: BinSimBackend, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.camera, self.sim = camera, sim

    def _on_stage(self, stage: str, plan: PressPlan) -> None:
        super()._on_stage(stage, plan)
        if stage == "end":
            m = self.sim.model
            px = {}
            for g in range(m.ngeom):
                name = m.geom(g).name
                if name.startswith(CONTENT):
                    px[name[len(CONTENT):]] = self._visible_pixels(g)
            self.measure.update({"flap_deg_at_end": round(abs(flap_angle_deg(m, self.sim.data)), 1),
                                 "visible_px_at_end": px})
            self.logger.event("bin_measure", **self.measure)

    def _visible_pixels(self, geom_id: int) -> int:
        """頭カメラの画像で、その geom が写っている画素の数（手やフラップに隠れた分は数えない）。"""
        import mujoco

        r = self.camera._renderer
        r.enable_segmentation_rendering()
        r.update_scene(self.sim.data, camera=self.camera.name)
        seg = r.render()
        r.disable_segmentation_rendering()
        return int(((seg[:, :, 0] == geom_id) & (seg[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))).sum())


def run_sim(args: argparse.Namespace) -> tuple[PipelineResult, BinSimBackend, Any, dict[str, Any]]:
    scene = load_scene_config(args.scene)
    from approach_sim import apply_scene_args

    apply_scene_args(scene, args)
    if args.waist_pitch is not None:
        scene["lean_waist_pitch_deg"] = args.waist_pitch
    if args.lift_m is not None:
        scene["bin"]["stand_height_m"] = args.lift_m  # ゴミ箱を載せる台の高さ
    if args.bin_front is not None:
        scene["bin"]["front_x"] = args.bin_front
    if args.bin_y is not None:
        scene["bin"]["center_y"] = args.bin_y
    if args.bin_yaw is not None:
        scene["bin"]["yaw_deg"] = args.bin_yaw
    if args.inset is not None:
        scene["push"]["inset_m"] = args.inset
    if args.from_axis is not None:
        scene["push"]["from_axis_m"] = args.from_axis
    if args.depth_mm is not None:
        scene["push"]["depth_m"] = args.depth_mm / 1000.0
    if args.via is not None:
        scene["push"]["via_pose"] = json.loads(args.via) if args.via.startswith("[") else (args.via or None)

    # J1-gen は既定で実機日の机の箱（configs/obstacles.yaml）を障害物に読む。シミュレーションには机が無いので、
    # --obstacles-file で指定したときだけ読む（空文字は「読まない」）
    if args.obstacles_file is None:
        args.obstacles_file = ""
    cfg = load_pipeline_config(args)
    if scene.get("hand_shape", "box") == "parts":
        install_hand_shape(cfg.robot)
    side = cfg.pipeline.get("arm") or cfg.arm["arm"]
    if args.cam_pitch_deg is not None:
        cfg.robot["head_camera"]["rpy"][1] = float(np.radians(args.cam_pitch_deg))
    # 実機に近づける乱し（sim/realism.py）。シミュレーションのカメラは本当の向き、位置の計算は知っている向き
    from approach_sim import setup_realism
    from realism import corrupt, mirror_mask

    rng = np.random.default_rng(getattr(args, "seed", 0))
    real, robot_true, locate_tf = setup_realism(args, cfg.robot, rng)
    if args.obstacle_steps is not None:
        scene["push"]["obstacle_steps"] = args.obstacle_steps
    if args.max_joint_speed is not None:
        scene["push"]["max_joint_speed_rad_s"] = args.max_joint_speed
    if args.approach_mm is not None:
        scene["push"]["approach_m"] = args.approach_mm / 1000.0
    if args.clearance_mm is not None:
        cfg.press["collision"]["clearance_m"] = args.clearance_mm / 1000.0
    cfg.pipeline["log_dir"] = args.log_dir or "_local/trash_bin/runs"

    sim_cfg = dict(cfg.arm["sim"])
    # --pelvis-true: シミュレーションの世界だけ本当の pelvis の高さにする（計画と位置の計算は設定の値のまま。実機の高さの不確かさを試す）
    world_scene = copy.deepcopy(scene)
    if args.pelvis_true is not None:
        world_scene["pelvis_height_m"] = args.pelvis_true
    sim_cfg.update(scene=world_scene, initial_q=list(start_pose(scene, np.zeros(29))),
                   realtime=args.view, viewer=args.view)
    backend = BinSimBackend(robot_true, sim_cfg,
                            hold_kp=np.asarray(cfg.arm["lowcmd"]["hold_kp"], dtype=float),
                            hold_kd=np.asarray(cfg.arm["lowcmd"]["hold_kd"], dtype=float),
                            uses_weight=sim_cfg.get("emulate", "arm_sdk") == "arm_sdk")
    backend.open()
    camera = SimHeadCamera(backend.model, backend.data, robot_true)

    # ゴミ箱の置き方: 設定の値（正解）か、頭カメラの深度から測った値（--locate。実機と同じやり方）
    truth = pose_from_config(scene)
    located = None
    if args.locate:
        backend.tick(cfg.arm["control_hz"] and 1.0 / float(cfg.arm["control_hz"]))
        frame = camera.render()
        if real.active():
            frame = corrupt(frame, mirror_mask(backend.model, backend.data, camera._renderer, camera.name), real, rng)
        q_waist = backend.read_state().q[12:15]
        # シミュレーションのカメラは較正のずれが無いので、較正の補正（localize.yaml）は使わない（--calib-error-m で入れられる）
        try:
            det = locate_bin(frame, locate_tf, q_waist, scene)
            apply_measured_pelvis_height(scene, det)
        except LocateError as e:
            print(f"[bin] ❌ 深度からゴミ箱を見つけられない: {e}")
            camera.close()
            backend.close()
            res = PipelineResult(code=2, message=f"ゴミ箱を見つけられない: {e}")
            return res, backend, None, {"result": res.message, "code": 2, "located": None}
        err_cm = float(np.linalg.norm(det.pose.center_xy - truth.center_xy) * 100)
        err_deg = float(np.degrees(det.pose.yaw - truth.yaw))
        print(f"[bin] 深度からゴミ箱を見つけた: {det.summary()}")
        print(f"[bin]   正解との差: 中心 {err_cm:.1f} cm、向き {err_deg:+.1f}°")
        located = {"center_xy": det.pose.center_xy.round(4).tolist(), "yaw_deg": round(float(np.degrees(det.pose.yaw)), 2),
                   "center_err_cm": round(err_cm, 2), "yaw_err_deg": round(err_deg, 2), "partial": det.partial}
        pose = det.pose
    else:
        pose = truth
    target = configure_pipeline(cfg, scene, side, pose, keep_speed=getattr(args, "speed_mm_s", None) is not None)
    depth = float(scene["push"]["depth_m"])

    logger = make_logger(cfg, args.label or "bin_sim", {"kind": "sim", "scene": scene, "located": located})
    lean_rad = install_leaning(scene)
    pipe = BinSimPipeline(cfg, backend, logger, camera.render, None, confirm=args.confirm, camera=camera, sim=backend,
                          lean_rad=lean_rad, extra_seeds_deg=scene["push"].get("ik_seeds_deg"),
                          finger_tilts_deg=(json.loads(args.tilts) if args.tilts
                                            else scene["push"].get("finger_tilts_deg") or [[0, 0]]))
    try:
        res = pipe.run()
    finally:
        camera.close()
    summary = {"result": "成功" if res.code == 0 else res.message, "code": res.code,
               "push_point_pelvis_m": target.round(4).tolist(), "depth_m": depth, "located": located,
               "flap_max_deg": round(backend.flap_max_deg, 1), "bin_contacts_N": backend.bin_contacts, **pipe.measure}
    return res, backend, logger, summary


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p)
    p.add_argument("--scene", help="シーンの設定（既定は Trash_Bin/configs/bin_scene.yaml）")
    p.add_argument("--inset", type=float, help="開口の手前の縁から押す点までの奥行 [m]（既定はシーンの設定）")
    p.add_argument("--from-axis", type=float, help="フラップの軸から押す点までの距離 [m]（既定はシーンの設定）")
    p.add_argument("--via", help='経由する姿勢: zero | "" (直接) | "[7 つの角度 deg]"（既定はシーンの設定）')
    p.add_argument("--approach-mm", type=float, help="押す点の何 mm 手前（上）まで関節空間で動くか（既定は press.yaml の 50）")
    p.add_argument("--obstacle-steps", type=int, help="計画で、すり鉢の斜面を何段の階段で近似するか（既定 4）")
    p.add_argument("--clearance-mm", type=float, help="計画で腕と障害物の間に空ける距離 [mm]（既定は press.yaml の 10）")
    p.add_argument("--tilts", help='指の向きの候補（JSON。例: "[[-45, 0], [-60, 0]]"。既定はシーンの設定）')
    p.add_argument("--max-joint-speed", type=float, help="関節の最大速度 [rad/s]（既定はシーンの設定。無ければ arm.yaml の 0.5）")
    from approach_sim import add_realism_args

    add_realism_args(p)
    p.add_argument("--seed", type=int, default=0, help="乱し（--depth-dropout など）の乱数の種")
    p.add_argument("--locate", action="store_true",
                   help="ゴミ箱の置き方を、頭カメラの深度から測る（lib/locate_bin.py。実機と同じやり方）。無ければ設定の値")
    p.add_argument("--bin-yaw", type=float, help="ゴミ箱の向き [deg]（左回りが正。既定はシーンの設定の 0）")
    p.add_argument("--bin-y", type=float, help="ゴミ箱の中心の y [m]（pelvis 座標。右が −）")
    p.add_argument("--bin-front", type=float, help="ゴミ箱の正面の x [m]（pelvis 座標。既定はシーンの設定の 0.15）")
    p.add_argument("--waist-pitch", type=float, help="押す前に腰を前に傾ける角度 [deg]（既定はシーンの設定の lean_waist_pitch_deg。0 なら傾けない）")
    p.add_argument("--lift-m", type=float, help="ゴミ箱を載せる台の高さ [m]（既定はシーンの設定の bin.stand_height_m）")
    p.add_argument("--pelvis-true", type=float, help="シミュレーションの世界の本当の pelvis の高さ [m]（計画は設定の pelvis_height_m のまま）")
    p.add_argument("--cam-pitch-deg", type=float, help="頭カメラの下向きの角度 [deg]（既定は robot.yaml の 47.6°）")
    p.add_argument("--confirm", action="store_true", help="各段階で Enter を待つ")
    p.add_argument("--view", action="store_true", help="MuJoCo の画面を開いて、動きを実時間で見る")
    args = p.parse_args()
    res, backend, logger, summary = run_sim(args)
    print("[push_bin_sim] 結果: " + json.dumps(summary, ensure_ascii=False))
    if logger is not None:
        print(f"[push_bin_sim] 記録: {logger.dir}")
    if backend.viewer_running():
        print("[push_bin_sim] 終わった。画面を閉じると終わる（Ctrl+C でも終わる）")
        try:
            while backend.viewer_running():
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
    backend.close()
    return res.code


if __name__ == "__main__":
    sys.exit(main())
