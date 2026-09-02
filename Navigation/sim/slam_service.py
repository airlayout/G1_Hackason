"""sim 側の `slam_operate`。実機の PC1 が担っている仕事をここで代わりに行う。

`nav/mission.py` から見た相手はここ。1804/1102/1201/1202 を受け、
**目標との差から速度指令を作って `sim/g1_walker.py` に渡す**。
到達したら `is_arrived` を立て、`task_result` を流す。

前の実装（`sim/fake_service.py`、削除済み）との違いはここだけ:

| | 旧 | 新 |
|---|---|---|
| 運動 | 目標へ等速直線で進む自作の積分 | 学習済みポリシー + MuJoCo の物理 |
| 障害物 | 円との距離を自作で判定 | `mujoco_lidar` で Mid-360 を撃つ |
| 時計 | 仮想時間 | MuJoCo の sim 時間 |

**再現している実機の癖**（`Navigation/README.md`「実測で確定した挙動」）:

- 静止中も `vx/vy/vyaw` が 0 にならない
- 1804 前は `info` も `ctrName` も `"not init"`
- 地図が読めないときは `errorCode 507 / "Load pcd failed."` の一種類だけ
- `task_result` はタスク実行時にしか流れない

**再現していないもの**: 1801/1802 の建図（sim に地図は既にある）。
形だけ受けて成功を返す。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass

import numpy as np

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
    Pose2D,
    parse_ctrl_info,
    parse_response,
    parse_task_result,
)
from nav.transport import SlamTransport
from sim.g1_walker import MAX_VX_MPS, MAX_WZ_RPS, G1Walker, Mid360, build_model
from sim.rooms import DynamicObstacle, Room

# 速度指令を作り直す間隔[s]。実機の `ctrl_info` が約 5Hz、ポリシーが 50Hz なので、
# その間の 10Hz にする。細かくしても歩容は 50Hz でしか変わらない。
CONTROL_TICK_S = 0.1

# --------------------------------------------------------------- 到達の判定

# 目標にこれだけ近づいたら「着いた」[m]。
# 歩幅が約 0.3m あるので、これより厳しくすると目標をまたいで往復する。
ARRIVAL_TOLERANCE_M = 0.25

# 到達後に向きを合わせる許容[rad]。ポリシーの旋回は 0.46rad/s なので
# 15° の詰めに 0.6 秒かかる。厳しくするほど巡回が伸びるだけで得は無い。
ARRIVAL_YAW_TOLERANCE_RAD = math.radians(15.0)

# ------------------------------------------------------------- 速度の作り方

# 進行方向と目標方位のずれがこれを超えたら、**前進をやめてその場旋回する**[rad]。
# 歩きながら曲がると内側に膨らんで壁に寄る。実測で曲線歩行(0.3,0,0.5)は
# 5 秒で 1.10m しか進まず 129° 回るので、大きく向きを変えるなら止まって回る方が速い。
TURN_IN_PLACE_RAD = math.radians(35.0)

# 速度指令のゲイン。**制御としては P だけ**で足りる。
# ポリシー自身が加減速を吸収するので、こちらで積分項を持つと振動する。
LINEAR_GAIN = 1.2  # [1/s] 距離[m] -> 前進速度[m/s]
ANGULAR_GAIN = 1.5  # [1/s] 方位差[rad] -> 旋回速度[rad/s]

# 方位がずれているぶん前進を落とす係数。1.0 でずれ 0、0 で ±90°。
# これが無いと、曲がりきる前に目標を通り過ぎて回り込みを繰り返す。
def _forward_scale(heading_error: float) -> float:
    return max(0.0, math.cos(heading_error))


# ------------------------------------------------------- 障害物とみなす条件

# 進行方向のこの距離までを見る[m]。実機が障害物の手前で止まる距離に相当する。
LOOKAHEAD_M = 1.2

# 進路の半幅[m]。G1 の肩幅 0.45m の半分 0.225m に歩容の揺れを足した。
# 広げすぎると壁に沿って歩くだけで「塞がれた」になる。
CORRIDOR_HALF_WIDTH_M = 0.35

# 障害物とみなす高さの帯[m]（地面から）。`nav/occupancy.py` と同じ理由で
# 床と天井を除く。ただし sim は地面が平らなので下限は低くてよい。
OBSTACLE_Z_MIN = 0.15
OBSTACLE_Z_MAX = 1.80

# これだけの点が条件を満たしたら「塞がれた」。1 点だとノイズで止まる。
OBSTACLE_MIN_POINTS = 8

# 地図で説明できる点を捨てるときの余裕[m]。
# LiDAR の当たった点は壁の面から数 cm ずれるし、地図の格子も 0.10m 刻みなので、
# 壁のセルを少しだけ太らせてから照合する。大きくしすぎると壁ぎわの本物の
# 障害物まで「地図のせい」にしてしまうので、格子 1〜2 セルぶんに留める。
KNOWN_MAP_TOLERANCE_M = 0.15


@dataclass(frozen=True)
class SimOptions:
    """sim の相手の振る舞い。異常系はここから注入する。"""

    known_maps: frozenset[str] = frozenset()
    """1804 が成功する地図パス。ここに無い address は実機同様 507 になる。"""

    init_failures: int = 0
    """地図が既知でも最初の N 回の 1804 を 507 で落とす。リトライの検証用。"""

    reject_navigate: bool = False
    """1102 を受理しない。異常系の検証用。"""

    obstacles: tuple[DynamicObstacle, ...] = ()

    arrival_tolerance_m: float = ARRIVAL_TOLERANCE_M
    arrival_yaw_tolerance_rad: float = ARRIVAL_YAW_TOLERANCE_RAD
    lookahead_m: float = LOOKAHEAD_M

    lidar: bool = True
    """False にすると LiDAR を回さない（1 スキャン約 4ms を節約できる）。

    切ると**地図に無い障害物に気付けなくなる**ので、既定は入り。
    経路計画だけを見たいときの逃げ道として残してある。
    """


class SimTransport(SlamTransport):
    """MuJoCo の中の G1 を相手にした `SlamTransport`。

    `nav/mission.py` はこのクラスと `RealTransport` の区別を知らない。
    """

    def __init__(self, room: Room, options: SimOptions | None = None) -> None:
        self._room = room
        self._options = options or SimOptions()
        model = build_model(room, self._options.obstacles)
        self._walker = G1Walker(model)
        self._lidar = Mid360(model) if self._options.lidar else None
        # 「地図で説明できる点」を捨てるための壁の一覧。膨らませるのは照合の
        # 誤差ぶん(0.15m)だけで、経路計画で使う機体半径ぶん(0.40m)ではない。
        self._known = room.grid(inflation=KNOWN_MAP_TOLERANCE_M)
        self._mocap_ids = self._find_mocap_ids(model)

        self._target: Pose2D | None = None
        self._segment_start: Pose2D | None = None
        self._initialized = False
        self._init_attempts = 0
        self._paused = False
        self._mapping = False
        self._arrived = False
        self._blocked = False
        self._blocked_since: float | None = None
        self._task_results: list = []
        self.calls: list[tuple[int, dict]] = []
        """投げられた API の記録。テストで順序を確かめるのに使う。"""

        self._sync_obstacles()

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
        """MuJoCo の sim 時間。実時間の約 20〜80 倍で進む。"""

        return self._walker.sim_time

    def sleep(self, seconds: float) -> None:
        """時間を進める。**その間ずっと歩かせる。**

        `mission.py` は 0.2 秒ごとにここを呼んで状態を見に来る。
        制御はその中でさらに細かく（`CONTROL_TICK_S`）回す。
        """

        remaining = seconds
        while remaining > 1e-9:
            tick = min(CONTROL_TICK_S, remaining)
            self._sync_obstacles()
            self._drive(tick)
            remaining -= tick

    # ------------------------------------------------------------ 状態の参照

    @property
    def pose(self) -> Pose2D:
        return self._walker.pose

    @property
    def walker(self) -> G1Walker:
        return self._walker

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def is_blocked(self) -> bool:
        return self._blocked

    @property
    def has_fallen(self) -> bool:
        return self._walker.has_fallen

    # ------------------------------------------------------------ API の実装

    def _handle_start_mapping(self, _data: dict):
        self._mapping = True
        return _response(True, ERROR_OK, "")

    def _handle_end_mapping(self, data: dict):
        if not self._mapping or not data.get("address"):
            return _response(False, ERROR_LOAD_PCD_FAILED, "Load pcd failed.")
        self._mapping = False
        return _response(True, ERROR_OK, "")

    def _handle_init_pose(self, data: dict):
        """1804。**機体は動かさない。**

        実機の 1804 は「地図を読んで、いまここに居ると教える」であって、
        そこへ移動させる命令ではない。sim でも同じにしないと、
        経路の出発点と物理の位置がずれたまま気付けなくなる。
        """

        self._init_attempts += 1
        if data.get("address") not in self._options.known_maps:
            return _response(False, ERROR_LOAD_PCD_FAILED, "Load pcd failed.")
        if self._init_attempts <= self._options.init_failures:
            return _response(False, ERROR_LOAD_PCD_FAILED, "Load pcd failed.")
        self._initialized = True
        return _response(True, ERROR_OK, "")

    def _handle_navigate(self, data: dict):
        if self._options.reject_navigate:
            return _response(False, ERROR_OK, "rejected by test")
        if not self._initialized:
            return _response(False, ERROR_LOAD_PCD_FAILED, "Load pcd failed.")
        target = data["targetPose"]
        self._target = Pose2D(
            float(target["x"]), float(target["y"]), _yaw_of_quaternion(target)
        )
        self._segment_start = self._walker.pose
        self._arrived = False
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

    # ------------------------------------------------------------- 運動制御

    def _drive(self, tick: float) -> None:
        """1 tick ぶん、目標へ向かう速度指令を作って歩かせる。

        これが実機の PC1 がやっていること。**位置で閉じている**ので、
        ポリシーが指令どおりの速度で歩かなくても（実測: 前進 0.5 指令で 0.45）
        目標には着く。
        """

        if self._target is None or self._paused or self._arrived:
            # 目標が無くても足踏みは続ける。制御を止めると転ぶ
            self._walker.stop()
            self._walker.step(tick)
            return

        here = self._walker.pose
        distance = math.hypot(self._target.x - here.x, self._target.y - here.y)
        if distance <= self._options.arrival_tolerance_m and self._align_yaw(here, tick):
            self._arrive()
            return

        bearing = (
            math.atan2(self._target.y - here.y, self._target.x - here.x)
            if distance > 1e-6
            else here.yaw
        )
        heading_error = _wrap(bearing - here.yaw)

        if self._update_blocked(here, bearing):
            self._walker.stop()
            self._walker.step(tick)
            return

        if abs(heading_error) > TURN_IN_PLACE_RAD:
            forward = 0.0
        else:
            forward = min(MAX_VX_MPS, LINEAR_GAIN * distance) * _forward_scale(heading_error)
        self._walker.set_command(forward, 0.0, _clamp(ANGULAR_GAIN * heading_error))
        self._walker.step(tick)

    def _align_yaw(self, here: Pose2D, tick: float) -> bool:
        """位置は着いた。向きも合っていれば True。合っていなければ回して False。"""

        error = _wrap(self._target.yaw - here.yaw)
        if abs(error) <= self._options.arrival_yaw_tolerance_rad:
            return True
        self._walker.set_command(0.0, 0.0, _clamp(ANGULAR_GAIN * error))
        self._walker.step(tick)
        return False

    def _arrive(self) -> None:
        self._arrived = True
        self._target = None
        self._blocked = False
        self._blocked_since = None
        self._walker.stop()
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

    # --------------------------------------------------------- 障害物の検知

    def _update_blocked(self, here: Pose2D, bearing: float) -> bool:
        """進路に障害物があるか。**判定は LiDAR の点群だけから作る。**

        壁は常に見えているので、点の位置を「進行方向の細い帯の中か」で絞る。
        実機の避障も同じことをしている（`obsInfo` は有無と秒数しか返さない）。
        """

        blocked = self._lidar is not None and self._points_in_corridor(here, bearing)
        if blocked and self._blocked_since is None:
            self._blocked_since = self._walker.sim_time
        elif not blocked:
            self._blocked_since = None
        self._blocked = blocked
        return blocked

    def _points_in_corridor(self, here: Pose2D, bearing: float) -> bool:
        points = self._unmapped_points()
        if len(points) == 0:
            return False
        # 進行方向を x 軸にした座標へ移す
        delta = points[:, :2] - np.array([here.x, here.y])
        forward = delta[:, 0] * math.cos(bearing) + delta[:, 1] * math.sin(bearing)
        lateral = -delta[:, 0] * math.sin(bearing) + delta[:, 1] * math.cos(bearing)
        in_corridor = (
            (forward > 0.0)
            & (forward < self._options.lookahead_m)
            & (np.abs(lateral) < CORRIDOR_HALF_WIDTH_M)
            & (points[:, 2] > OBSTACLE_Z_MIN)
            & (points[:, 2] < OBSTACLE_Z_MAX)
        )
        return int(in_corridor.sum()) >= OBSTACLE_MIN_POINTS

    def _unmapped_points(self) -> np.ndarray:
        """LiDAR が当てた点のうち、**地図で説明できないもの**だけを返す。

        壁は常に見えているので、そのまま数えると「壁に向かって歩いている」だけで
        塞がれた判定になり、壁ぎわの巡回地点へ行けなくなる。実機の避障も
        「いま見えているもの」から「地図に載っているもの」を引いた差分を見ている。

        引き算に使う壁は、照合の誤差ぶん(`KNOWN_MAP_TOLERANCE_M` = 0.15m)しか
        膨らませない。経路計画で使う機体半径ぶんの膨張(0.40m)まで「地図のせい」に
        すると、**壁から 0.4m 以内に立っている本物の障害物を見落とす。**
        """

        points = self._lidar.scan(self._walker.data)
        if len(points) == 0:
            return points
        spec = self._known.spec
        col, row = spec.to_cell(points[:, 0], points[:, 1])
        inside = (col >= 0) & (col < spec.width) & (row >= 0) & (row < spec.height)
        # 地図の外に出た点は「地図で説明できない」側に寄せる（見落とすより止まる）
        explained = np.zeros(len(points), bool)
        explained[inside] = self._known.blocked[row[inside], col[inside]]
        return points[~explained]

    def _find_mocap_ids(self, model) -> list[int]:
        import mujoco

        ids = []
        for index in range(len(self._options.obstacles)):
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"obstacle_{index}")
            ids.append(int(model.body_mocapid[body_id]))
        return ids

    def _sync_obstacles(self) -> None:
        """現れる／消える障害物を今の時刻の位置へ動かす。"""

        now = self._walker.sim_time
        for mocap_id, obstacle in zip(self._mocap_ids, self._options.obstacles):
            self._walker.data.mocap_pos[mocap_id] = obstacle.position_at(now)

    # ------------------------------------------------------------ トピック

    def _ctrl_info_frame(self) -> dict:
        """実機が流すのと同じ形のフレームを組み立てる。"""

        pose = self._walker.pose
        return {
            "type": TYPE_CTRL_INFO,
            "errorCode": ERROR_OK,
            "info": "running" if self._initialized else CTRL_NAME_NOT_INIT,
            "data": {
                "stateMachine": {
                    "state": "follow" if self._target else "ready",
                    "ctrName": "pid" if self._initialized else CTRL_NAME_NOT_INIT,
                    "isOpenPlan": False,
                    "isBack": False,
                    "isRotate": False,
                    "isClimbStairs": False,
                    "isPause": self._paused,
                    # 実機は静止中も 0 にならない。速度 0 で停止判定させないための再現
                    "vx": 0.004,
                    "vy": 0.005,
                    "vyaw": 0.065,
                },
                "currentPose": {
                    "x": pose.x, "y": pose.y, "z": self._walker.height,
                    "roll": 0.0, "pitch": 0.0, "yaw": pose.yaw,
                },
                "is_arrived": self._arrived,
                "targetNodeName": 0,
                "obsInfo": {"state": self._blocked, "time": self._blocked_seconds()},
                "progress": {
                    "used_time": self._walker.sim_time,
                    "last_time": 0.0,
                    "completion_percentage": self._completion(),
                },
                "total_distance": -1.0,
            },
        }

    def _blocked_seconds(self) -> float:
        if self._blocked_since is None:
            return 0.0
        return self._walker.sim_time - self._blocked_since

    def _completion(self) -> float:
        if self._target is None or self._segment_start is None:
            return 0.0
        here = self._walker.pose
        total = math.hypot(
            self._target.x - self._segment_start.x, self._target.y - self._segment_start.y
        )
        if total <= 0.0:
            return 1.0
        left = math.hypot(self._target.x - here.x, self._target.y - here.y)
        return float(min(1.0, max(0.0, (total - left) / total)))


def _response(succeed: bool, error_code: int, info: str):
    """実機と同じ封筒で JSON を作り、`nav/protocol.py` に読ませる。

    直接 `ServiceResponse` を組むのではなくパーサを通すのは、
    **シリアライズ層も一緒に試す**ため。
    """

    return parse_response(
        json.dumps({"succeed": succeed, "errorCode": error_code, "info": info, "data": {}})
    )


def _yaw_of_quaternion(target: dict) -> float:
    from nav.protocol import quaternion_to_yaw

    return quaternion_to_yaw(
        float(target.get("q_x", 0.0)),
        float(target.get("q_y", 0.0)),
        float(target.get("q_z", 0.0)),
        float(target.get("q_w", 1.0)),
    )


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float) -> float:
    return max(-MAX_WZ_RPS, min(MAX_WZ_RPS, value))


_HANDLERS = {
    API_START_MAPPING: SimTransport._handle_start_mapping,
    API_END_MAPPING: SimTransport._handle_end_mapping,
    API_INIT_POSE: SimTransport._handle_init_pose,
    API_NAVIGATE_POSE: SimTransport._handle_navigate,
    API_PAUSE: SimTransport._handle_pause,
    API_RESUME: SimTransport._handle_resume,
    API_CLOSE_SLAM: SimTransport._handle_close,
}
