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
import dataclasses
import functools
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

from bin_scene import (  # noqa: E402
    CONTENT, add_bin_scene, bin_obstacles, flap_angle_deg, load_scene_config, pose_from_config, push_point, start_pose,
)
from common.camera_geometry import HeadCameraTransform  # noqa: E402
from locate_bin import locate_bin  # noqa: E402
from common import sim_scene  # noqa: E402
from hand_shape import install as install_hand_shape  # noqa: E402
from common.arm.backend_sim import SimBackend  # noqa: E402
from common import pipeline as j1_pipeline  # noqa: E402
from common.pipeline import PipelineResult, PressPipeline  # noqa: E402
from lean import LeaningCommander  # noqa: E402
from common.pipeline_cli import add_common_args, load_pipeline_config, make_logger  # noqa: E402
from common.press_planner import PressPlan, UnreachableError  # noqa: E402
from common.sim_camera import SimHeadCamera  # noqa: E402


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

    def tick(self, dt: float) -> None:
        super().tick(dt)
        self.flap_max_deg = max(self.flap_max_deg, abs(flap_angle_deg(self.model, self.data)))


def finger_axis(back_deg: float, side_deg: float) -> np.ndarray:
    """指の向き（pelvis 座標）。真下から、手前（−x、ロボット側）へ back_deg、右（−y）へ side_deg 傾ける。"""
    b, s = np.radians(back_deg), np.radians(side_deg)
    v = np.array([-np.sin(b) * np.cos(s), -np.sin(s), -np.cos(b) * np.cos(s)])
    return v / np.linalg.norm(v)


class BinPipeline(PressPipeline):
    """J1-gen の PressPipeline に、次の 2 つを足したもの。

    - 指の向きの候補を順に試す。J1-gen の全体の流れは指を押す向き（真下）にそろえるが、真下だと手の本体が
      指先から手前へ張り出し、開口の手前の縁に近づいて計画が通らなかった（2026-11-05）。
      指先を手前へ傾けると、手の本体は奥へ逃げる
    - 押し切った時点（段階 end）で、フラップの角度と、頭カメラに写った中身の画素数を測る
    """

    def __init__(self, *args: Any, camera: SimHeadCamera, sim: BinSimBackend,
                 finger_tilts_deg: list[list[float]], lean_rad: float = 0.0, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.lean_rad = lean_rad
        self.camera, self.sim = camera, sim
        self.measure: dict[str, Any] = {}
        self._tilts = finger_tilts_deg
        self._axis: np.ndarray | None = None
        plan = self.planner.plan
        # 計画の関数に、今試している指の向きを渡す（J1-gen の make_plan は渡さないため）
        # 立て直し（replan）は前の計画の指の向きを自分で渡すので、そのときはそちらを使う
        self.planner.plan = lambda *a, **k: plan(*a, **{"finger_axis": self._axis, **k})

    def make_plan(self, st: Any, target: np.ndarray) -> PressPlan:
        if self.lean_rad:
            # 腕を動かす前（腰はまだ直立）に立てる計画も、傾けたあとの腰の角度で立てる。
            # 押す手前では、J1-gen が実測の腰の角度で計画を立て直す
            q = st.q.copy()
            q[14] += self.lean_rad
            st = dataclasses.replace(st, q=q)
        errors = []
        for back, side in self._tilts:
            self._axis = finger_axis(back, side)
            print(f"[bin] 指の向き: 手前へ {back:.0f}°、右へ {side:.0f}°")
            try:
                plan = super().make_plan(st, target)
            except UnreachableError as e:
                errors.append(f"[手前 {back:.0f}°, 右 {side:.0f}°] {e}")
                continue
            self.measure["finger_tilt_deg"] = [back, side]
            self.logger.event("bin_finger_tilt", back_deg=back, side_deg=side)
            return plan
        raise UnreachableError("どの指の向きでも計画できない: " + " // ".join(errors))

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
    if args.waist_pitch is not None:
        scene["lean_waist_pitch_deg"] = args.waist_pitch
    if args.lift_m:
        # ゴミ箱を台に載せて lift_m 高くする（床からの高さを全部上げるのと同じなので、pelvis の高さを下げて表す）
        scene["pelvis_height_m"] = float(scene["pelvis_height_m"]) - args.lift_m
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
    # 押す向きは真下、深さはシーンの設定（J1-gen の上限 3 cm は外す）。押す点は、ゴミ箱の置き方を決めてから下で求める
    cfg.pipeline["target"]["source"] = "manual"
    cfg.pipeline["push"]["direction"] = list(scene["push"]["direction"])
    cfg.pipeline["via_pose"] = scene["push"].get("via_pose")
    depth = float(scene["push"]["depth_m"])
    cfg.press["press"]["press_depth_m"] = depth
    if args.obstacle_steps is not None:
        scene["push"]["obstacle_steps"] = args.obstacle_steps
    if args.clearance_mm is not None:
        cfg.press["collision"]["clearance_m"] = args.clearance_mm / 1000.0
    if args.approach_mm is not None:
        cfg.press["press"]["approach_distance_m"] = args.approach_mm / 1000.0
    elif scene["push"].get("approach_m"):
        cfg.press["press"]["approach_distance_m"] = float(scene["push"]["approach_m"])
    if getattr(args, "speed_mm_s", None) is None and scene["push"].get("speed_m_s"):
        cfg.press["press"]["press_speed_m_s"] = float(scene["push"]["speed_m_s"])
    # 関節の最大速度（J1-gen の arm.yaml の safety.max_joint_speed_rad_s）。計画（1 周期の動きの上限）と、腕を動かすときの
    # 安全確認の両方に効く。正面から押す直線は J1-gen の 0.5 rad/s をわずかに超える（最大 4%）ので、シーンの設定で上書きできる
    if args.max_joint_speed is not None:
        scene["push"]["max_joint_speed_rad_s"] = args.max_joint_speed
    if scene["push"].get("max_joint_speed_rad_s"):
        cfg.arm["safety"]["max_joint_speed_rad_s"] = float(scene["push"]["max_joint_speed_rad_s"])
    # 手先が入ってよい箱（J1-gen の press.yaml の workspace。ボタン押し用の仮の値で、x の下限 0.10 m は体の横のゴミ箱に合わない）
    ws = scene["push"].get("workspace")
    if ws:
        cfg.press["workspace"][side] = {"min": list(ws["min"]), "max": list(ws["max"])}
    # 押し込みを止める遅れ（J1-gen の contact_stop.lag_m）。ゴミ箱のフラップは 5 mm では開き切る前に止まる
    if "stop_lag_m" in scene["push"] and "contact_stop" in cfg.press["press"]:
        cfg.press["press"]["contact_stop"]["lag_m"] = float(scene["push"]["stop_lag_m"])
    cfg.press["press"]["max_press_depth_m"] = max(depth, float(cfg.press["press"]["max_press_depth_m"]))
    # J1-gen の立つ位置の勧め（ボトルの机用）はゴミ箱には合わないので使わない
    cfg.pipeline["stand"] = None
    cfg.pipeline["log_dir"] = args.log_dir or "_local/trash_bin/runs"

    sim_cfg = dict(cfg.arm["sim"])
    sim_cfg.update(scene=scene, initial_q=list(start_pose(scene, np.zeros(29))),
                   realtime=args.view, viewer=args.view)
    backend = BinSimBackend(cfg.robot, sim_cfg,
                            hold_kp=np.asarray(cfg.arm["lowcmd"]["hold_kp"], dtype=float),
                            hold_kd=np.asarray(cfg.arm["lowcmd"]["hold_kd"], dtype=float),
                            uses_weight=sim_cfg.get("emulate", "arm_sdk") == "arm_sdk")
    backend.open()
    camera = SimHeadCamera(backend.model, backend.data, cfg.robot)

    # ゴミ箱の置き方: 設定の値（正解）か、頭カメラの深度から測った値（--locate。実機と同じやり方）
    truth = pose_from_config(scene)
    located = None
    if args.locate:
        backend.tick(cfg.arm["control_hz"] and 1.0 / float(cfg.arm["control_hz"]))
        frame = camera.render()
        q_waist = backend.read_state().q[12:15]
        # シミュレーションのカメラは較正のずれが無いので、較正の補正（localize.yaml）は使わない
        det = locate_bin(frame, HeadCameraTransform(cfg.robot), q_waist, scene)
        err_cm = float(np.linalg.norm(det.pose.center_xy - truth.center_xy) * 100)
        err_deg = float(np.degrees(det.pose.yaw - truth.yaw))
        print(f"[bin] 深度からゴミ箱を見つけた: {det.summary()}")
        print(f"[bin]   正解との差: 中心 {err_cm:.1f} cm、向き {err_deg:+.1f}°")
        located = {"center_xy": det.pose.center_xy.round(4).tolist(), "yaw_deg": round(float(np.degrees(det.pose.yaw)), 2),
                   "center_err_cm": round(err_cm, 2), "yaw_err_deg": round(err_deg, 2), "partial": det.partial}
        pose = det.pose
    else:
        pose = truth
    target = push_point(scene, side, pose=pose)
    cfg.pipeline["target"]["manual"]["point_pelvis_m"] = target.tolist()
    # ゴミ箱を計画の障害物に入れる（測った置き方で）
    cfg.press["obstacles"] = list(cfg.press.get("obstacles") or []) + bin_obstacles(
        scene, steps=int(scene["push"].get("obstacle_steps", 4)), pose=pose)

    logger = make_logger(cfg, args.label or "bin_sim", {"kind": "sim", "scene": scene, "located": located})
    # 腰を傾ける: J1-gen の流れが使う ArmCommander を、腰を傾けて戻すもの（lib/lean.py）に差し替える
    lean_rad = float(np.radians(float(scene.get("lean_waist_pitch_deg") or 0.0)))
    lean_cfg = scene.get("lean") or {}
    j1_pipeline.ArmCommander = functools.partial(
        LeaningCommander, lean_rad=lean_rad, max_speed_deg_s=float(lean_cfg.get("max_speed_deg_s", 5.0)),
        settle_s=float(lean_cfg.get("settle_s", 1.0)), max_error_deg=float(lean_cfg.get("max_error_deg", 2.0)))
    pipe = BinPipeline(cfg, backend, logger, camera.render, None, confirm=args.confirm, camera=camera, sim=backend,
                       lean_rad=lean_rad,
                       finger_tilts_deg=(json.loads(args.tilts) if args.tilts
                                         else scene["push"].get("finger_tilts_deg") or [[0, 0]]))
    try:
        res = pipe.run()
    finally:
        camera.close()
    summary = {"result": "成功" if res.code == 0 else res.message, "code": res.code,
               "push_point_pelvis_m": target.round(4).tolist(), "depth_m": depth, "located": located,
               "flap_max_deg": round(backend.flap_max_deg, 1), **pipe.measure}
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
    p.add_argument("--locate", action="store_true",
                   help="ゴミ箱の置き方を、頭カメラの深度から測る（lib/locate_bin.py。実機と同じやり方）。無ければ設定の値")
    p.add_argument("--bin-yaw", type=float, help="ゴミ箱の向き [deg]（左回りが正。既定はシーンの設定の 0）")
    p.add_argument("--bin-y", type=float, help="ゴミ箱の中心の y [m]（pelvis 座標。右が −）")
    p.add_argument("--bin-front", type=float, help="ゴミ箱の正面の x [m]（pelvis 座標。既定はシーンの設定の 0.15）")
    p.add_argument("--waist-pitch", type=float, help="押す前に腰を前に傾ける角度 [deg]（既定はシーンの設定の lean_waist_pitch_deg。0 なら傾けない）")
    p.add_argument("--lift-m", type=float, default=0.0, help="ゴミ箱を台に載せて高くする量 [m]（どれだけ足りないかを調べる用）")
    p.add_argument("--cam-pitch-deg", type=float, help="頭カメラの下向きの角度 [deg]（既定は robot.yaml の 47.6°）")
    p.add_argument("--confirm", action="store_true", help="各段階で Enter を待つ")
    p.add_argument("--view", action="store_true", help="MuJoCo の画面を開いて、動きを実時間で見る")
    args = p.parse_args()
    res, backend, logger, summary = run_sim(args)
    print("[push_bin_sim] 結果: " + json.dumps(summary, ensure_ascii=False))
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
