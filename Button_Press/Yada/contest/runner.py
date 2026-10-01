"""1 試行を動かす（観測 → act() → 安全のための処理 → 送る → 採点）。シミュレーションと実機で共通。

結果:
- success     指定のボタンが点灯し、もう一方は点灯していない
- wrong       違うボタンが点灯した（同じ周期に両方が点灯した場合も含む）
- timeout     制限時間内に点灯しなかった
- gave_up     エージェントが done を返したが、点灯していない
- error       エージェントが例外を出した、または出力の形が違う
"""

from __future__ import annotations

import time
import traceback
from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

from .interface import UPPER_BODY_IDX, Action, Agent, TaskInfo
from .robots.base import Robot


@dataclass
class EpisodeResult:
    seed: int
    sim: str
    target: str
    instruction: str
    outcome: str
    time_s: float | None = None  # 点灯までの時間（success / wrong のとき）
    steps: int = 0
    max_contact_force_n: float | None = None  # 壁・扉・盤との接触力の最大
    clipped_limit_steps: int = 0  # 可動範囲で丸めた周期の数
    clipped_rate_steps: int = 0  # 1 周期の最大移動量で丸めた周期の数
    ignored_base_cmd_steps: int = 0  # 下半身の指令が効かない環境で、0 でない指令を返した周期の数
    act_time_mean_ms: float = 0.0  # act() にかかった時間の平均
    error: str = ""
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class SafetyFilter:
    """上半身の目標を、関節の可動範囲と 1 周期の最大移動量で丸める（J1-gen の common/arm/safety.py と同じ考え）。"""

    def __init__(self, lo29: np.ndarray, hi29: np.ndarray, margin: float, max_step: float):
        idx = list(UPPER_BODY_IDX)
        self.lo = lo29[idx] + margin
        self.hi = hi29[idx] - margin
        self.max_step = float(max_step)
        self.prev: np.ndarray | None = None

    def reset(self, q_upper_now: np.ndarray) -> None:
        self.prev = np.clip(np.asarray(q_upper_now, dtype=float), self.lo, self.hi)

    def __call__(self, q_target: np.ndarray) -> tuple[np.ndarray, bool, bool]:
        """(丸めた目標, 可動範囲で丸めたか, 移動量で丸めたか)。"""
        assert self.prev is not None
        q = np.clip(q_target, self.lo, self.hi)
        by_limit = not np.allclose(q, q_target)
        step = np.clip(q - self.prev, -self.max_step, self.max_step)
        by_rate = not np.allclose(step, q - self.prev)
        self.prev = self.prev + step
        return self.prev.copy(), by_limit, by_rate


def _check_action(a: Any) -> Action:
    if not isinstance(a, Action):
        raise TypeError(f"act() が Action を返さない: {type(a)}")
    q = np.asarray(a.q_target, dtype=float).reshape(-1)
    if q.shape != (len(UPPER_BODY_IDX),):
        raise ValueError(f"q_target の形が違う: {q.shape}（{len(UPPER_BODY_IDX)} 個が要る）")
    if not np.all(np.isfinite(q)):
        raise ValueError("q_target に NaN か inf がある")
    b = np.asarray(a.base_cmd, dtype=float).reshape(-1)
    if b.shape != (3,) or not np.all(np.isfinite(b)):
        raise ValueError(f"base_cmd は (vx, vy, yaw_rate) の 3 個: {b}")
    return Action(q_target=q, base_cmd=b, done=bool(a.done))


def run_episode(robot: Robot, agent: Agent, trial: Any, contest_cfg: dict[str, Any], verbose: bool = False) -> EpisodeResult:
    """1 試行を動かす。trial は contest.task.Trial。"""
    dt = 1.0 / float(contest_cfg["control_hz"])
    limit = float(contest_cfg["time_limit_s"])
    kp, kd = robot.upper_gains()
    task = TaskInfo(instruction=trial.instruction, target=trial.target, sim=robot.name, time_limit=limit,
                    control_dt=dt, base_enabled=robot.base_enabled, upper_kp=kp, upper_kd=kd)
    res = EpisodeResult(seed=trial.seed, sim=robot.name, target=trial.target, instruction=trial.instruction,
                        outcome="timeout")
    s = contest_cfg["safety"]
    lo, hi = robot.joint_limits()
    safety = SafetyFilter(lo, hi, float(s["limit_margin_rad"]), float(s["max_joint_speed_rad_s"]) * dt)

    # 置いた直後の揺れを収める（エージェントは呼ばない）
    robot.advance(float(contest_cfg.get("settle_s", 0.0)))
    obs = robot.observe(0.0)
    safety.reset(obs.q[list(UPPER_BODY_IDX)])
    act_times: list[float] = []
    try:
        agent.reset(task)
    except Exception:
        res.outcome, res.error = "error", traceback.format_exc()
        return res

    t = 0.0
    max_f = 0.0
    while t < limit:
        try:
            t0 = time.perf_counter()
            action = _check_action(agent.act(obs))
            act_times.append(time.perf_counter() - t0)
        except Exception:
            res.outcome, res.error = "error", traceback.format_exc()
            break
        q_safe, by_limit, by_rate = safety(action.q_target)
        res.clipped_limit_steps += int(by_limit)
        res.clipped_rate_steps += int(by_rate)
        base = action.base_cmd
        if not robot.base_enabled and np.any(base != 0.0):
            res.ignored_base_cmd_steps += 1
            base = np.zeros(3)
        robot.command(q_safe, base)
        info = robot.advance(dt)
        t += dt
        res.steps += 1
        if info.max_contact_force is not None:
            max_f = max(max_f, info.max_contact_force)
        lit = [n for n, on in info.lit.items() if on]
        if lit:
            res.outcome = "success" if lit == [trial.target] else "wrong"
            res.time_s = round(t, 3)
            if verbose:
                print(f"[run] seed {trial.seed}: {lit} が点灯（t = {t:.2f} s）")
            break
        if action.done:
            res.outcome = "gave_up"
            break
        obs = robot.observe(t)

    res.max_contact_force_n = round(max_f, 2)
    res.act_time_mean_ms = round(1000.0 * float(np.mean(act_times)), 2) if act_times else 0.0
    return res
