"""推論・HTTPから独立した腕の制御状態機械。時刻はG1側のmonotonicに統一する。"""

from __future__ import annotations

import secrets
from collections import deque

import numpy as np

from .config import finite_array


class ArmControl:
    """呼出側のロック下で使用する。停止後の推論結果はsessionで排除する。"""

    def __init__(self, config: dict, *, execute: bool = False):
        self.config = config
        self.execute = execute
        self.q = np.zeros(29)
        self.dq = np.zeros(29)
        self.state_at = float("-inf")
        self.camera_at = float("-inf")
        self.target = np.zeros(14)
        self.anchor = np.zeros(14)
        self.weight = 0.0
        self.session = None
        self.phase = "idle"
        self.reason = ""
        self.started_at = 0.0
        self.command_at = 0.0
        self.chunk_at = 0.0
        self.chunk = None
        self.action_hz = 30.0
        self.observations = deque(maxlen=256)
        self.last_observation_id = None

    def update_state(self, q, dq, *, now: float) -> None:
        self.q = finite_array(q, (29,), "q")
        self.dq = finite_array(dq, (29,), "dq")
        self.state_at = now

    def observe(self, *, now: float) -> str:
        self._require_fresh(now)
        identifier = secrets.token_hex(12)
        self.observations.append((identifier, now, self.camera_at))
        return identifier

    def _require_fresh(self, now: float) -> None:
        if now - self.state_at > self.config["state_timeout_s"]:
            raise ValueError("関節状態が途絶しました")
        if now - self.camera_at > self.config["camera_timeout_s"]:
            raise ValueError("カメラ画像が途絶しました")

    def start(self, *, now: float, max_task_s: float) -> str:
        self._require_fresh(now)
        if self.phase != "idle" or self.weight > 0:
            raise ValueError("腕は実行中または停止処理中です")
        q = self.q[15:29]
        if np.any(q < self.config["lower_rad"]) or np.any(q > self.config["upper_rad"]):
            raise ValueError("開始姿勢が設定の可動範囲外です")
        self.anchor = q.copy()
        self.target = q.copy()
        self.chunk = None
        self.started_at = self.command_at = now
        self.deadline = now + max_task_s
        self.session = secrets.token_hex(16)
        self.last_observation_id = None
        self.phase, self.reason = "running", ""
        return self.session

    def accept(self, session: str, observation_id: str, positions, action_hz: float, *, now: float) -> None:
        if self.phase != "running" or not secrets.compare_digest(session, self.session or ""):
            raise ValueError("この実行は停止済みです")
        self._require_fresh(now)
        if observation_id == self.last_observation_id:
            raise ValueError("同じ観測からの指令は再実行できません")
        observed = next((x for x in self.observations if x[0] == observation_id), None)
        if observed is None or now - min(observed[1:]) > self.config["prediction_timeout_s"]:
            raise ValueError("推論元の観測が古すぎます")
        chunk = np.asarray(positions, dtype=np.float64)
        if chunk.ndim != 2 or not 1 <= len(chunk) <= 40 or chunk.shape[1] != 14:
            raise ValueError("指令は1..40ステップ×14関節です")
        if not np.isfinite(chunk).all() or not np.isfinite(action_hz) or not 1 <= action_hz <= 60:
            raise ValueError("指令には有限値と1..60 Hzが必要です")
        if np.any(chunk < self.config["lower_rad"]) or np.any(chunk > self.config["upper_rad"]):
            raise ValueError("予測が関節可動範囲外です")
        if np.any(np.abs(chunk - self.anchor) > self.config["max_excursion_rad"]):
            raise ValueError("予測が開始姿勢からの移動上限を超えました")
        self.chunk, self.chunk_at, self.command_at = chunk.copy(), now, now
        self.action_hz = action_hz
        self.last_observation_id = observation_id
        self.observations = deque((item for item in self.observations if item[1] > observed[1]), maxlen=256)

    def stop(self, reason: str = "停止") -> None:
        self.session = None
        self.chunk = None
        self.reason = reason
        self.phase = "stopping" if self.weight > 0 else "idle"

    def tick(self, *, now: float, dt: float) -> tuple[np.ndarray, float]:
        if self.phase == "running":
            try:
                self._require_fresh(now)
                if now >= self.deadline:
                    raise ValueError("実行時間の上限に達しました")
                if now - self.command_at > self.config["command_timeout_s"]:
                    raise ValueError("推論指令が途絶しました")
                if np.max(np.abs(self.target - self.q[15:29])) > self.config["max_tracking_error_rad"]:
                    raise ValueError("腕が目標に追従していません")
            except ValueError as error:
                self.stop(str(error))
        # ループが遅延しても一度に大きな位置・weight変化を送らない。
        step = min(max(dt, 0), 2 / self.config["control_hz"])
        if self.phase == "running":
            if self.chunk is not None:
                index = min(int((now - self.chunk_at) * self.action_hz), len(self.chunk) - 1)
                delta = self.chunk[index] - self.target
                limit = self.config["max_velocity_rad_s"] * step
                self.target += np.clip(delta, -limit, limit)
            self.weight = min(1.0, self.weight + step / self.config["ramp_s"])
        else:
            self.weight = max(0.0, self.weight - step / self.config["ramp_s"])
            if self.weight == 0:
                self.phase = "idle"
        return self.target.copy(), self.weight

    def status(self, *, now: float) -> dict:
        return {
            "phase": self.phase,
            "reason": self.reason,
            "execute": self.execute,
            "weight": round(self.weight, 3),
            "state_age_s": self._age(now, self.state_at),
            "camera_age_s": self._age(now, self.camera_at),
            "q": self.q.tolist(),
            "target": self.target.tolist(),
        }

    @staticmethod
    def _age(now: float, timestamp: float) -> float | None:
        age = now - timestamp
        return round(age, 3) if np.isfinite(age) else None
