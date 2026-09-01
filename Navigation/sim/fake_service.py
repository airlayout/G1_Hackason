"""偽の `slam_operate`。プロセス内で動き、標準ライブラリしか使わない。

`nav/mission.py` の相手をこれに差し替えると、実機もPC2もDDSも無しで
プロトコルと状態遷移を検証できる。**実機と同じJSONを組み立ててから
`nav/protocol.py` でパースさせている**ので、シリアライズ層も一緒に試せる。

2つのモードを1つの実装で兼ねる:

| モード | `kinematics` | 何を見るか |
|---|---|---|
| mock | `False` | プロトコル・状態遷移・507リトライ・到達待ち。数ポーリングで即到達する |
| sim | `True` | 上に加えて幾何。目標へ向かって等速で動き、障害物の前で止まる |

**時計は仮想時間。** `sleep()` は実際には待たず、内部時刻を進めて運動を積分する。
おかげで30分の巡回が1秒でテストできる。

再現している実機の癖:

- 静止中も `vx/vy/vyaw` が0にならない（実測値をそのまま入れている）
- 1804前は `info` も `ctrName` も `"not init"`、`state` は `"ready"`
- 地図が読めないときは `errorCode 507 / "Load pcd failed."` の一種類だけ
- `task_result` はタスク実行時にしか流れない
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

from nav.geometry import Pose2D, yaw_to_quaternion
from nav.protocol import (
    API_CLOSE_SLAM,
    API_END_MAPPING,
    API_INIT_POSE,
    API_NAVIGATE_POSE,
    API_PAUSE,
    API_RESUME,
    API_START_MAPPING,
    CTRL_NAME_NOT_INIT,
    ERROR_LOAD_PCD_FAILED,
    ERROR_OK,
    TYPE_CTRL_INFO,
    TYPE_TASK_RESULT,
    parse_ctrl_info,
    parse_response,
    parse_task_result,
)
from nav.transport import SlamTransport

# 運動を積分する刻み[s]。細かすぎると遅く、粗いと障害物をすり抜ける。
INTEGRATION_STEP_S = 0.05

# 障害物を見つけて止まる距離[m]。実機のLiDARの検知に相当する。
LOOKAHEAD_M = 0.8


@dataclass(frozen=True)
class Obstacle:
    """円い障害物。時間で現れたり消えたりできる（人が横切る状況を作るため）。"""

    x: float
    y: float
    radius: float = 0.5
    appears_at_s: float = 0.0
    disappears_at_s: float = float("inf")

    def is_active(self, now: float) -> bool:
        return self.appears_at_s <= now < self.disappears_at_s

    def blocks(self, x: float, y: float) -> bool:
        return math.hypot(x - self.x, y - self.y) <= self.radius


@dataclass(frozen=True)
class FakeOptions:
    kinematics: bool = True
    """False なら運動を積分せず、`mock_arrival_polls` 回のポーリングで到達する。"""

    speed_mps: float = 0.5
    turn_rate_rps: float = 1.0
    arrival_tolerance_m: float = 0.15
    mock_arrival_polls: int = 3

    known_maps: frozenset[str] = frozenset()
    """1804が成功する地図パス。ここに無いaddressは実機同様507になる。"""

    init_failures: int = 0
    """地図が既知でも最初のN回の1804を507で落とす。リトライの検証用。

    1802の書き込み完了と1804のレースを模したもの。
    """

    obstacles: tuple[Obstacle, ...] = ()

    reject_navigate: bool = False
    """1102を受理しない。異常系の検証用。"""

    freeze_after_s: float | None = None
    """この秒数を過ぎたら進捗が止まる。停滞検知の検証用。"""


class FakeTransport(SlamTransport):
    """`SlamTransport` の偽実装。"""

    def __init__(self, start: Pose2D, options: FakeOptions | None = None) -> None:
        self._options = options or FakeOptions()
        self._pose = start
        self._time = 0.0
        self._target: Pose2D | None = None
        self._segment_start: Pose2D | None = None
        self._initialized = False
        self._init_attempts = 0
        self._paused = False
        self._mapping = False
        self._arrived = False
        self._polls_since_command = 0
        self._task_results: list = []
        self._blocked = False
        self._blocked_since: float | None = None
        self.calls: list[tuple[int, dict]] = []
        """投げられたAPIの記録。テストで順序を確かめるのに使う。"""

    # ---------------------------------------------------------- SlamTransport

    def call(self, api_id: int, request: dict):
        self.calls.append((api_id, request))
        handler = _HANDLERS.get(api_id)
        if handler is None:
            return _response(False, ERROR_OK, f"unknown api id {api_id}")
        return handler(self, request.get("data") or {})

    def latest_ctrl_info(self):
        return parse_ctrl_info(json.dumps(self._ctrl_info_frame()))

    def take_task_results(self) -> list:
        taken, self._task_results = self._task_results, []
        return taken

    def now(self) -> float:
        return self._time

    def sleep(self, seconds: float) -> None:
        """実際には待たず、仮想時間を進めながら運動を積分する。"""

        self._polls_since_command += 1
        remaining = seconds
        while remaining > 0.0:
            step = min(INTEGRATION_STEP_S, remaining)
            self._time += step
            self._advance(step)
            remaining -= step

    # ------------------------------------------------------------ 状態の参照

    @property
    def pose(self) -> Pose2D:
        return self._pose

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def is_blocked(self) -> bool:
        return self._blocked

    # ------------------------------------------------------------ APIの実装

    def _handle_start_mapping(self, _data: dict):
        self._mapping = True
        return _response(True, ERROR_OK, "")

    def _handle_end_mapping(self, data: dict):
        address = data.get("address", "")
        if not self._mapping or not address:
            return _response(False, ERROR_LOAD_PCD_FAILED, "Load pcd failed.")
        self._mapping = False
        return _response(True, ERROR_OK, "")

    def _handle_init_pose(self, data: dict):
        """1804。実機と同じく、失敗は507の一種類しか返さない。"""

        self._init_attempts += 1
        if data.get("address") not in self._options.known_maps:
            return _response(False, ERROR_LOAD_PCD_FAILED, "Load pcd failed.")
        if self._init_attempts <= self._options.init_failures:
            return _response(False, ERROR_LOAD_PCD_FAILED, "Load pcd failed.")
        self._pose = Pose2D(float(data["x"]), float(data["y"]), self._pose.yaw)
        self._initialized = True
        return _response(True, ERROR_OK, "")

    def _handle_navigate(self, data: dict):
        if self._options.reject_navigate:
            return _response(False, ERROR_OK, "rejected by test")
        if not self._initialized:
            return _response(False, ERROR_LOAD_PCD_FAILED, "Load pcd failed.")
        target = data["targetPose"]
        self._target = Pose2D(float(target["x"]), float(target["y"]), self._pose.yaw)
        self._segment_start = self._pose
        self._arrived = False
        self._polls_since_command = 0
        return _response(True, ERROR_OK, "")

    def _handle_pause(self, _data: dict):
        self._paused = True
        return _response(True, ERROR_OK, "")

    def _handle_resume(self, _data: dict):
        self._paused = False
        return _response(True, ERROR_OK, "")

    def _handle_close(self, _data: dict):
        self._initialized = False
        self._target = None
        return _response(True, ERROR_OK, "")

    # -------------------------------------------------------------- 運動と状態

    def _advance(self, step: float) -> None:
        if self._target is None or self._paused or self._arrived:
            return
        if not self._options.kinematics:
            if self._polls_since_command >= self._options.mock_arrival_polls:
                self._arrive()
            return
        if self._options.freeze_after_s is not None and self._time >= self._options.freeze_after_s:
            return

        distance = self._pose.distance_to(self._target)
        if distance <= self._options.arrival_tolerance_m:
            self._arrive()
            return

        heading = self._pose.heading_to(self._target)
        if self._obstacle_ahead(heading):
            self._blocked = True
            return
        self._blocked = False
        travel = min(self._options.speed_mps * step, distance)
        self._pose = Pose2D(
            self._pose.x + math.cos(heading) * travel,
            self._pose.y + math.sin(heading) * travel,
            heading,
        )

    def _obstacle_ahead(self, heading: float) -> bool:
        """進行方向 LOOKAHEAD_M 以内に有効な障害物があるか。"""

        active = [o for o in self._options.obstacles if o.is_active(self._time)]
        if not active:
            return False
        steps = int(LOOKAHEAD_M / INTEGRATION_STEP_S)
        for index in range(steps + 1):
            distance = LOOKAHEAD_M * index / steps
            x = self._pose.x + math.cos(heading) * distance
            y = self._pose.y + math.sin(heading) * distance
            if any(obstacle.blocks(x, y) for obstacle in active):
                return True
        return False

    def _arrive(self) -> None:
        self._pose = Pose2D(self._target.x, self._target.y, self._target.yaw)
        self._arrived = True
        self._target = None
        self._blocked = False
        self._task_results.append(
            parse_task_result(
                json.dumps(
                    {
                        "type": TYPE_TASK_RESULT,
                        "errorCode": ERROR_OK,
                        "info": "",
                        "data": {"targetNodeName": 0, "is_arrived": True},
                    }
                )
            )
        )

    def _ctrl_info_frame(self) -> dict:
        """実機が流すのと同じ形のフレームを組み立てる。"""

        name = "pid" if self._initialized else CTRL_NAME_NOT_INIT
        return {
            "type": TYPE_CTRL_INFO,
            "errorCode": ERROR_OK,
            "info": "running" if self._initialized else CTRL_NAME_NOT_INIT,
            "data": {
                "stateMachine": {
                    "state": "follow" if self._target else "ready",
                    "ctrName": name,
                    "isOpenPlan": False,
                    "isBack": False,
                    "isRotate": False,
                    "isClimbStairs": False,
                    "isPause": self._paused,
                    # 実機は静止中も0にならない。速度0で停止判定させないための再現
                    "vx": 0.004,
                    "vy": 0.005,
                    "vyaw": 0.065,
                },
                "currentPose": {
                    "x": self._pose.x, "y": self._pose.y, "z": 0.0,
                    "roll": 0.0, "pitch": 0.0, "yaw": self._pose.yaw,
                },
                "is_arrived": self._arrived,
                "targetNodeName": 0,
                "obsInfo": {"state": self._blocked, "time": self._blocked_seconds()},
                "progress": {
                    "used_time": self._time,
                    "last_time": 0.0,
                    "completion_percentage": self._completion(),
                },
                "total_distance": -1.0,
            },
        }

    def _blocked_seconds(self) -> float:
        if not self._blocked:
            self._blocked_since = None
            return 0.0
        if self._blocked_since is None:
            self._blocked_since = self._time
        return self._time - self._blocked_since

    def _completion(self) -> float:
        if self._target is None or self._segment_start is None:
            return 0.0
        total = self._segment_start.distance_to(self._target)
        if total <= 0.0:
            return 1.0
        return min(1.0, (total - self._pose.distance_to(self._target)) / total)


def _response(succeed: bool, error_code: int, info: str):
    """実機と同じ封筒でJSONを作り、protocol.py に読ませる。"""

    return parse_response(
        json.dumps({"succeed": succeed, "errorCode": error_code, "info": info, "data": {}})
    )


_HANDLERS = {
    API_START_MAPPING: FakeTransport._handle_start_mapping,
    API_END_MAPPING: FakeTransport._handle_end_mapping,
    API_INIT_POSE: FakeTransport._handle_init_pose,
    API_NAVIGATE_POSE: FakeTransport._handle_navigate,
    API_PAUSE: FakeTransport._handle_pause,
    API_RESUME: FakeTransport._handle_resume,
    API_CLOSE_SLAM: FakeTransport._handle_close,
}
