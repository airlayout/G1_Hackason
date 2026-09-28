"""全体をつなぐ流れ（タスク6）: 検出 → 目標 → IK → 手前の姿勢 → 押し込み → 戻る。sim / real 共通。

    PressPipeline(...).run()   # 終了コードを返す（0: 成功）

段階（どれも RunLogger に記録する。dry-run でも記録する）:
1. lowstate を待ち、相手（実機 / 模擬ロボット）と mode_machine を確かめる
2. 対象の位置（腕はまだ動かさない）
   - depth: 複数フレームで検出 → 深度 → pelvis 座標。**較正値（localize.yaml）は Locator が必ず足す**。
     ばらつきが大きい・求まらないフレームが多いときは中止。手が対象を隠していないかも確かめる
   - manual: 設定の点、または教えた姿勢の中指の先 + 定規で測ったずれ
3. 計画: 教えた姿勢を IK の初期値にして、押し込みの軌道を作る。届かない・ぶつかるならここで中止（何も送らない）
4. 実行（ArmCommander。確認モードなら段階ごとに Enter）: 手前の姿勢 → **実測の腰の角度で必ず計算し直す** →
   押し込み → 保持 → 戻り → 押したあとの確認（押し直し）→ 開始姿勢へ戻る
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .arm import (
    ArmCommander,
    NoMotionError,
    StateTimeoutError,
    StopRequested,
    UnsafeTargetError,
    WaistDeviationError,
    joint_limits,
)
from .arm.backend import ArmBackend
from .arm.gravity import GravityModel, needs_gravity_model
from .arm.types import JointState
from .camera_geometry import HeadCameraTransform
from .dds import PeerMismatchError
from .localize import Detector, Locator
from .post_press import NoCheck, PostPressCheck
from .press import execute_press
from .press_planner import PressPlan, PressPlanner, UnreachableError
from .realday import FingertipFK, load_taught_poses, project_to_image, taught_q_seed
from .rgbd_protocol import RgbdFrame
from .robot_model import WAIST_IDX
from .run_logger import LoggingBackend, RunLogger

# 終了コード
EXIT_OK = 0
EXIT_TARGET = 2  # 対象の位置が求まらない、または届かない・ぶつかる（何も送っていない）
EXIT_NO_MOTION = 3
EXIT_STATE = 4  # lowstate が届かない・途切れた、mode_machine・相手の食い違い、モータが無効
EXIT_WAIST = 5
EXIT_STOPPED = 130


class TargetError(RuntimeError):
    """対象の位置が求まらない。"""


@dataclass
class PipelineConfig:
    robot: dict[str, Any]
    arm: dict[str, Any]
    press: dict[str, Any]
    localize: dict[str, Any]
    pipeline: dict[str, Any]


@dataclass
class PipelineResult:
    code: int
    message: str = ""
    target: np.ndarray | None = None
    plan: PressPlan | None = None
    checks: list[Any] = field(default_factory=list)


def fmt(v: Any, scale: float = 1.0, digits: int = 3) -> str:
    """ベクトルを見やすく表示する（例: [+0.388, -0.194, +0.041]）。"""
    return "[" + ", ".join(f"{x * scale:+.{digits}f}" for x in np.asarray(v, dtype=float)) + "]"


def resolve_push_dir(direction: Any) -> np.ndarray:
    if direction == "forward":
        return np.array([1.0, 0.0, 0.0])
    v = np.asarray(direction, dtype=float)
    if v.shape != (3,) or np.linalg.norm(v) < 1e-9:
        raise ValueError(f"push.direction は forward か [x, y, z]: {direction}")
    return v / np.linalg.norm(v)


class PressPipeline:
    def __init__(
        self,
        cfg: PipelineConfig,
        backend: ArmBackend,
        logger: RunLogger,
        grab_frame: Callable[[], RgbdFrame | None] | None,
        detector: Detector | None,
        confirm: bool,
        input_fn: Callable[[str], str] = input,
        post_check: PostPressCheck | None = None,
    ) -> None:
        self.cfg = cfg
        self.side = cfg.pipeline.get("arm") or cfg.arm["arm"]
        cfg.arm["arm"] = self.side
        self.logger = logger
        self.backend = LoggingBackend(backend, logger)
        self.grab_frame = grab_frame
        self.detector = detector
        self.confirm = confirm
        self.input = input_fn
        self.post_check = post_check or NoCheck()
        self.planner = PressPlanner.from_config(cfg.robot, cfg.press, cfg.arm, self.side)
        self.fk = FingertipFK(cfg.robot, cfg.press["ik"])
        self.calib = np.asarray(cfg.localize["calibration"]["offset_pelvis_m"], dtype=float)
        self.transform = HeadCameraTransform(cfg.robot, self.calib)

    # ---- 1. lowstate ------------------------------------------------------------

    def wait_state(self, timeout_s: float = 10.0) -> JointState:
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:
            st = self.backend.read_state()
            if st is not None:
                self.backend.verify_peer(st)
                return st
            time.sleep(0.05)
        raise StateTimeoutError(f"{timeout_s} 秒待っても lowstate が届かない（接続・NIC 名・電源を確認）")

    # ---- 2. 対象の位置 ------------------------------------------------------------

    def locate_depth(self) -> np.ndarray:
        c = self.cfg.pipeline["target"]["depth"]
        if self.grab_frame is None or self.detector is None:
            raise TargetError("target.source = depth だが、カメラか検出器が無い")
        if not np.any(self.calib):
            print("[pipeline] ⚠️ 較正値（localize.yaml の calibration.offset_pelvis_m）が 0 のまま（未較正）")
            self.logger.event("warning", message="未較正（較正値が 0）")
        locator = Locator(self.cfg.robot, self.cfg.localize, self.detector)
        pts = []
        for k in range(int(c["frames"])):
            f = self.grab_frame()
            st = self.backend.read_state()
            if f is None or st is None:
                self.logger.event("locate_frame", index=k, ok=False, reason="フレームか lowstate が無い")
                continue
            qw = st.q[list(WAIST_IDX)]
            found = locator.locate(f, qw)
            best = locator.best(found)
            self.logger.save_frame(f"locate{k}", f, waist_q=qw)
            self.logger.event("locate_frame", index=k, ok=best is not None, waist_q=qw,
                              detections=[{"class": x.class_name, "confidence": x.confidence, "bbox": x.bbox,
                                           "pixel": x.pixel, "depth_m": x.depth_m, "p_pelvis": x.p_pelvis,
                                           "reason": x.reason} for x in found])
            if best is None:
                continue
            self._check_hand_in_view(best.bbox, st, f, c["hand_in_view"])
            self._save_detection_image(f, found, k)
            pts.append(best.p_pelvis)
        if len(pts) < int(c["min_valid_frames"]):
            raise TargetError(f"対象の位置が求まったフレームが {len(pts)} 枚（必要 {c['min_valid_frames']} 枚）")
        arr = np.array(pts)
        spread = arr.max(axis=0) - arr.min(axis=0)
        target = np.median(arr, axis=0)
        self.logger.event("target_depth", target=target, spread_m=spread, n=len(pts), calibration_offset=self.calib)
        print(f"[pipeline] 対象（深度）: {fmt(target)} m（{len(pts)} 枚、ばらつき {fmt(spread, 1000, 1)} mm、"
              f"較正値 {fmt(self.calib, 1000, 1)} mm を含む）")
        if np.any(spread > float(c["max_spread_m"])):
            raise TargetError(f"フレームごとの位置のばらつきが大きい: {fmt(spread, 1000, 1)} mm"
                              f"（上限 {float(c['max_spread_m']) * 1000:.0f} mm）")
        return target

    def _check_hand_in_view(self, bbox: tuple[float, ...], st: JointState, f: RgbdFrame, action: str) -> None:
        x1, y1, x2, y2 = bbox
        for side, p in self.fk.positions(st.q).items():
            uv = project_to_image(p, st.q[list(WAIST_IDX)], self.transform, f.intrinsics)
            if uv is not None and x1 <= uv[0] <= x2 and y1 <= uv[1] <= y2:
                msg = f"{side} の手（指先）が対象の枠の中に写っている。腕をカメラの視野から外すこと"
                self.logger.event("hand_in_view", side=side, pixel=uv, bbox=bbox)
                if action == "abort":
                    raise TargetError(msg)
                print(f"[pipeline] ⚠️ {msg}")

    def _save_detection_image(self, f: RgbdFrame, found: list[Any], k: int) -> None:
        import cv2

        img = f.color_bgr.copy()
        for x in found:
            x1, y1, x2, y2 = (int(round(t)) for t in x.bbox)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0) if x.ok else (0, 0, 255), 2)
            cv2.circle(img, (int(x.pixel[0]), int(x.pixel[1])), 5, (0, 255, 255), -1)
        self.logger.save_image(f"detect{k}", img)

    def locate_manual(self) -> np.ndarray:
        m = self.cfg.pipeline["target"]["manual"]
        if m.get("point_pelvis_m") is not None:
            target = np.asarray(m["point_pelvis_m"], dtype=float)
            how = "設定の点（point_pelvis_m）"
        elif m.get("taught_pose"):
            poses = load_taught_poses()
            if m["taught_pose"] not in poses:
                raise TargetError(f"教えた姿勢 {m['taught_pose']!r} が configs/taught_poses.yaml に無い")
            base = np.asarray(poses[m["taught_pose"]]["fingertip_pelvis_m"], dtype=float)
            target = base + np.asarray(m.get("offset_m", [0, 0, 0]), dtype=float)
            how = f"教えた姿勢 {m['taught_pose']} の中指の先 + ずれ {m.get('offset_m')}"
        else:
            raise TargetError("target.source = manual だが、manual.point_pelvis_m も manual.taught_pose も無い")
        self.logger.event("target_manual", target=target, how=how)
        print(f"[pipeline] 対象（manual、{how}）: {fmt(target)} m（較正値は使わない）")
        if self.grab_frame is not None:
            self.logger.save_frame("manual_target", self.grab_frame())
        return target

    # ---- 3. 計画 -----------------------------------------------------------------

    def _seeds(self, st: JointState) -> list[tuple[str, np.ndarray | None]]:
        """IK の初期値の候補（順に試す）: 教えた姿勢 → 今の姿勢 → 腕のゼロ姿勢。"""
        seeds: list[tuple[str, np.ndarray | None]] = []
        name = self.cfg.pipeline.get("seed_pose")
        if name:
            try:
                side, q = taught_q_seed(name, st.q)
                if side == self.side:
                    seeds.append((f"教えた姿勢 {name}", q))
                else:
                    print(f"[pipeline] ⚠️ 教えた姿勢 {name} は {side} 腕のもの（動かすのは {self.side}）。使わない")
            except KeyError:
                print(f"[pipeline] ⚠️ 教えた姿勢 {name!r} が configs/taught_poses.yaml に無い")
        seeds.append(("今の姿勢", None))
        zero = st.q.copy()
        zero[self.planner.kin.arm_idx] = 0.0
        seeds.append(("腕のゼロ姿勢（肘を約 90° 曲げて前に出した姿勢）", zero))
        return seeds

    def _via_pose(self) -> np.ndarray | None:
        """経由する姿勢（片腕 7 関節）: null（直接）| zero（腕のゼロ姿勢）| 教えた姿勢の名前 | 右腕の 7 つの角度 [deg]。"""
        via = self.cfg.pipeline.get("via_pose")
        if via is None or via == "" or via is False:
            return None
        if via == "zero":
            return np.zeros(len(self.planner.kin.arm_idx))
        if isinstance(via, (list, tuple)):
            q = np.radians(np.asarray(via, dtype=float))
            if q.shape != (7,):
                raise TargetError(f"via_pose の角度は 7 つ: {via}")
            if self.side == "left":
                q[[1, 2, 4, 6]] *= -1.0  # 左右の反転（roll と yaw の向きが逆）
            return q
        poses = load_taught_poses()
        if via not in poses:
            raise TargetError(f"経由の姿勢 {via!r} が configs/taught_poses.yaml に無い")
        if poses[via]["side"] != self.side:
            raise TargetError(f"経由の姿勢 {via!r} は {poses[via]['side']} 腕のもの（動かすのは {self.side}）")
        return np.asarray(poses[via]["arm_q_rad"], dtype=float)

    def make_plan(self, st: JointState, target: np.ndarray) -> PressPlan:
        push = resolve_push_dir(self.cfg.pipeline["push"]["direction"])
        via = self._via_pose()
        errors = []
        for label, q_seed in self._seeds(st):
            try:
                plan = self.planner.plan(st.q, target, push, q_seed=q_seed, q_via_arm=via)
            except UnreachableError as e:
                errors.append(f"{label}: {e}")
                self.logger.event("plan_failed", seed=label, reason=str(e))
                print(f"[pipeline] IK の初期値「{label}」では計画できない: {e}")
                continue
            self.logger.save_json("plan_initial.json", self._plan_dict(plan))
            self.logger.event("plan", summary=plan.summary(), seed=label)
            print(f"[pipeline] 計画（IK の初期値: {label}）: {plan.summary()}")
            return plan
        raise UnreachableError("どの IK の初期値でも計画できない: " + " / ".join(errors))

    @staticmethod
    def _plan_dict(plan: PressPlan) -> dict[str, Any]:
        return {"side": plan.side, "target_point": plan.target_point, "approach_point": plan.approach_point,
                "end_point": plan.end_point, "push_dir": plan.push_dir, "depth_m": plan.depth,
                "waist_q": plan.waist_q, "max_ik_err_m": plan.max_ik_err_m, "q_via": plan.q_via,
                "q_approach": plan.q_approach,
                "press_in": plan.press_in, "press_out": plan.press_out}

    # ---- 全体 -----------------------------------------------------------------

    def _on_stage(self, stage: str, plan: PressPlan) -> None:
        self.logger.event("stage", stage=stage, depth_m=plan.depth, waist_q=plan.waist_q)
        if stage.endswith("replanned"):
            self.logger.save_json(f"plan_{stage}.json", self._plan_dict(plan))
        if self.grab_frame is not None and stage in ("approach", "end", "retreated"):
            self.logger.save_frame(stage, self.grab_frame())

    def run(self) -> PipelineResult:
        res = PipelineResult(code=EXIT_OK)
        self._arm: ArmCommander | None = None
        try:
            res = self._run()
        except (TargetError, UnreachableError, UnsafeTargetError) as e:
            if self._arm is not None and self._arm.started:
                res = PipelineResult(EXIT_TARGET, f"腕を動かしたあとで目標を拒否した（安全に止めた）: {e}")
            else:
                res = PipelineResult(EXIT_TARGET, f"目標を拒否した（何も送っていない）: {e}")
        except NoMotionError as e:
            res = PipelineResult(EXIT_NO_MOTION, f"❌ {e}")
        except WaistDeviationError as e:
            res = PipelineResult(EXIT_WAIST, f"❌ {e}")
        except (StateTimeoutError, PeerMismatchError) as e:
            res = PipelineResult(EXIT_STATE, f"❌ {e}")
        except StopRequested as e:
            res = PipelineResult(EXIT_STOPPED, f"中止: {e}")
        except RuntimeError as e:  # モータが無効、mode_machine が違う、など（ArmCommander.start）
            res = PipelineResult(EXIT_STATE, f"❌ {e}")
        print(f"[pipeline] 結果: {'成功' if res.code == EXIT_OK else res.message}（終了コード {res.code}）")
        self.logger.event("result", code=res.code, message=res.message)
        self.logger.close(code=res.code, message=res.message, target=res.target)
        return res

    def _run(self) -> PipelineResult:
        cfg = self.cfg
        self.logger.event("start", side=self.side, backend=self.backend.name, dry_run=self.backend.dry_run,
                          target_source=cfg.pipeline["target"]["source"], calibration_offset=self.calib)
        st = self.wait_state()
        expected = int(cfg.arm["expected_mode_machine"])
        if st.mode_machine != expected:
            raise RuntimeError(f"mode_machine が {st.mode_machine}（期待値 {expected}）。機体構成が違うので中止する")
        self.logger.event("lowstate", mode_machine=st.mode_machine, waist_q=st.q[list(WAIST_IDX)])

        src = cfg.pipeline["target"]["source"]
        if src == "depth":
            target = self.locate_depth()
        elif src == "manual":
            target = self.locate_manual()
        else:
            raise TargetError(f"target.source は depth か manual: {src}")

        st = self.wait_state()
        plan = self.make_plan(st, target)
        lower, upper = joint_limits(cfg.robot)
        gravity = GravityModel(cfg.robot) if needs_gravity_model(cfg.arm) else None
        pc = cfg.press.get("post_check", {})
        with ArmCommander(self.backend, cfg.arm, lower, upper, fk=self.planner.kin.fk_pos,
                          workspace=self.planner.workspace, confirm=self.confirm, input_fn=self.input,
                          gravity=gravity) as arm:
            self._arm = arm
            q_start = arm.commanded_arm_q
            plan, checks = execute_press(
                arm, plan, hold_s=float(cfg.press["press"]["hold_s"]), planner=self.planner, return_to=q_start,
                on_stage=self._on_stage, post_check=self.post_check,
                max_retries=int(pc.get("max_retries", 0)), grab_frame=self.grab_frame,
            )
            for i, c in enumerate(checks, start=1):
                self.logger.event("post_check", attempt=i, ok=c.ok, deeper_m=c.deeper_m, message=c.message, data=c.data)
        if arm.stop_reason:
            raise StopRequested(arm.stop_reason)
        return PipelineResult(EXIT_OK, "成功", target=target, plan=plan, checks=checks)
