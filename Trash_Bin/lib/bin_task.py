"""ゴミ箱を押す流れの組み立て。sim（sim/push_bin_sim.py）と real（real/push_bin.py）で共通。

Button_Press/J1-gen の PressPipeline をそのまま使い（J1-gen のファイルは変えない）、ここで次を足す:
- configure_pipeline: シーンの設定（configs/bin_scene.yaml の push など）を、J1-gen の設定（cfg.press / pipeline / arm）に当てる
- install_leaning: J1-gen の流れが使う ArmCommander を、腰を傾けて戻すもの（lib/lean.py）に差し替える
- BinPipeline: 指の向きの候補を順に試す。腕を動かす前の計画を、傾けたあとの腰の角度で立てる
"""

from __future__ import annotations

import dataclasses
import functools
from typing import Any

import numpy as np

from bin_geometry import BinPose, bin_obstacles, push_point
from common import pipeline as j1_pipeline
from common.pipeline import PipelineConfig, PressPipeline
from common.press_planner import PressPlan, UnreachableError
from lean import WAIST_PITCH, LeaningCommander
from locate_bin import LocateError


def finger_axis(back_deg: float, side_deg: float) -> np.ndarray:
    """指の向き（pelvis 座標）。真下から、手前（−x、ロボット側）へ back_deg、右（−y）へ side_deg 傾ける（負なら前・左）。"""
    b, s = np.radians(back_deg), np.radians(side_deg)
    v = np.array([-np.sin(b) * np.cos(s), -np.sin(s), -np.cos(b) * np.cos(s)])
    return v / np.linalg.norm(v)


def apply_measured_pelvis_height(scene: dict[str, Any], det: Any, max_change_m: float = 0.04) -> float | None:
    """深度で測った床から pelvis の高さを求め、scene["pelvis_height_m"] を置き換えて返す（床が測れていなければ None で、そのまま）。

    実機の pelvis の高さは資料で 0.73〜0.76 m と割れている。設定より実機が高いと押す点が浅くなり、フラップが開き切らない
    （2 cm 高いと成功が約半分。2026-11-07、MuJoCo）。押す点と障害物はこの値から求まるので、configure_pipeline より前に呼ぶ。
    設定から max_change_m より離れていたら、カメラか床の測り間違いかもしれないので LocateError で止める。
    """
    level = det.info.get("floor_level")
    if level is None:
        return None
    dz = float(level["dz_m"])  # 測った床を設定の高さへ動かす量。+ なら床が設定より低い = pelvis が高い
    if abs(dz) > max_change_m:
        raise LocateError(f"測った床の高さが設定と {dz * 100:+.1f} cm 違う（許容 ±{max_change_m * 100:.0f} cm）。カメラの角度か立ち方を確かめる")
    height = float(scene["pelvis_height_m"]) + dz
    print(f"[bin] 床から求めた pelvis の高さ: {height:.3f} m（設定 {scene['pelvis_height_m']:.3f} m との差 {dz * 100:+.1f} cm）")
    scene["pelvis_height_m"] = height
    return height


def configure_pipeline(cfg: PipelineConfig, scene: dict[str, Any], side: str, pose: BinPose,
                       keep_speed: bool = False) -> np.ndarray:
    """シーンの設定を J1-gen の設定に当て、押す点（pelvis 座標）を返す。pose はゴミ箱の置き方（設定の値か、深度で測った値）。

    keep_speed: True なら押し込みの速さはコマンドラインの値（J1-gen の --speed-mm-s）のままにする
    """
    p = scene["push"]
    target = push_point(scene, side, pose=pose)
    # 対象は求めた押す点（manual）、押す向きは真下
    cfg.pipeline["target"]["source"] = "manual"
    cfg.pipeline["target"]["manual"]["point_pelvis_m"] = target.tolist()
    cfg.pipeline["push"]["direction"] = list(p["direction"])
    cfg.pipeline["via_pose"] = p.get("via_pose")
    # J1-gen の立つ位置の勧め（ボトルの机用）はゴミ箱には合わないので使わない
    cfg.pipeline["stand"] = None
    # 押し込みの深さ（J1-gen の上限 3 cm は外す）・手前の距離・速さ
    depth = float(p["depth_m"])
    cfg.press["press"]["press_depth_m"] = depth
    cfg.press["press"]["max_press_depth_m"] = max(depth, float(cfg.press["press"]["max_press_depth_m"]))
    if p.get("approach_m"):
        cfg.press["press"]["approach_distance_m"] = float(p["approach_m"])
    # 押し込んだまま保持する時間（J1-gen の既定 0.5 秒）。この間に画像を撮る（lib/lean.py の follow）
    if p.get("hold_s"):
        cfg.press["press"]["hold_s"] = float(p["hold_s"])
    if p.get("speed_m_s") and not keep_speed:
        cfg.press["press"]["press_speed_m_s"] = float(p["speed_m_s"])
    # 押し込みを止める遅れ（J1-gen の contact_stop.lag_m）。ゴミ箱のフラップは 5 mm では開き切る前に止まる
    if "stop_lag_m" in p and "contact_stop" in cfg.press["press"]:
        cfg.press["press"]["contact_stop"]["lag_m"] = float(p["stop_lag_m"])
    # 関節の最大速度（J1-gen の arm.yaml の safety.max_joint_speed_rad_s）。計画と、腕を動かすときの安全確認の両方に効く
    if p.get("max_joint_speed_rad_s"):
        cfg.arm["safety"]["max_joint_speed_rad_s"] = float(p["max_joint_speed_rad_s"])
    # 手先が入ってよい箱（J1-gen の press.yaml の workspace はボタン押し用の仮の値）
    ws = p.get("workspace")
    if ws:
        cfg.press["workspace"][side] = {"min": list(ws["min"]), "max": list(ws["max"])}
    # ゴミ箱を計画の障害物に入れる（置き方 pose で）
    cfg.press["obstacles"] = list(cfg.press.get("obstacles") or []) + bin_obstacles(
        scene, steps=int(p.get("obstacle_steps", 4)), pose=pose)
    return target


def install_leaning(scene: dict[str, Any]) -> float:
    """J1-gen の流れが使う ArmCommander を、腰を傾けて戻すもの（lib/lean.py）に差し替え、傾ける角度 [rad] を返す。"""
    lean_rad = float(np.radians(float(scene.get("lean_waist_pitch_deg") or 0.0)))
    lc = scene.get("lean") or {}
    j1_pipeline.ArmCommander = functools.partial(
        LeaningCommander, lean_rad=lean_rad, max_speed_deg_s=float(lc.get("max_speed_deg_s", 5.0)),
        settle_s=float(lc.get("settle_s", 1.0)), max_error_deg=float(lc.get("max_error_deg", 2.0)))
    return lean_rad


class BinPipeline(PressPipeline):
    """J1-gen の PressPipeline に、次の 2 つを足したもの。

    - 指の向きの候補を順に試す。J1-gen の全体の流れは指を押す向き（真下）にそろえるが、正面のゴミ箱では
      真下だと手首の可動域が足りず、前へ 30〜40° 傾けると届く（2026-11-05）
    - 腕を動かす前（腰はまだ直立）に立てる計画を、傾けたあとの腰の角度で立てる。押す手前では、J1-gen が
      実測の腰の角度で計画を立て直す
    """

    def __init__(self, *args: Any, finger_tilts_deg: list[list[float]], lean_rad: float = 0.0,
                 extra_seeds_deg: list[list[float]] | None = None, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.lean_rad = lean_rad
        self.extra_seeds_deg = extra_seeds_deg or []
        self.measure: dict[str, Any] = {}
        self._tilts = finger_tilts_deg
        self._axis: np.ndarray | None = None
        plan = self.planner.plan
        # 計画の関数に、今試している指の向きを渡す（J1-gen の make_plan は渡さないため）。
        # 立て直し（replan）は前の計画の指の向きを自分で渡すので、そのときはそちらを使う
        self.planner.plan = lambda *a, **k: plan(*a, **{"finger_axis": self._axis, **k})

    def _on_stage(self, stage: str, plan: PressPlan) -> None:
        """J1-gen の段階ごとの記録に加えて、押し込みが止まったら（end）、保持する間にも画像を撮るようにする。"""
        super()._on_stage(stage, plan)
        arm = getattr(self, "_arm", None)
        if stage == "end" and self.grab_frame is not None and arm is not None and hasattr(arm, "hold_capture"):
            arm.hold_capture = lambda i: self.logger.save_frame(f"hold{i + 1}", self.grab_frame())

    def _seeds(self, st: Any) -> list[tuple[str, np.ndarray | None]]:
        """J1-gen の IK の初期値（教えた姿勢・今の姿勢・ゼロ姿勢）に、設定の初期値を足す。
        J1-gen の初期値だけだと、台に載せたゴミ箱の上の姿勢の解を見つけられなかった（2026-11-07）。"""
        seeds = super()._seeds(st)
        for i, deg in enumerate(self.extra_seeds_deg):
            q = st.q.copy()
            q[self.planner.kin.arm_idx] = np.radians(np.asarray(deg, dtype=float))
            seeds.append((f"設定の初期値 {i}", q))
        return seeds

    def make_plan(self, st: Any, target: np.ndarray) -> PressPlan:
        if self.lean_rad:
            q = st.q.copy()
            q[WAIST_PITCH] += self.lean_rad
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
