"""`slam_operate` (SERVICE_NAME="slam_operate", VERSION="1.0.0.1") のJSON層。

リクエストの組み立てとレスポンス/トピックのパースだけを担当する。
**通信手段は一切知らない**（DDSもsshもここには出てこない）。
そのおかげでmock/sim/実機のどれでも同じコードが動く。

出典と検証状況:

- 形式は公式ドキュメント `slam_navigation_services_interface`（更新2026-07-20）。
  原文の写しは `docs/G1＿Hackthon/_原文_G1_SLAM導航服務接口_20260720.txt`（git管理外）
- `ctrl_info` の実測値は `Navigation/README.md`「実測で確定した挙動」節。
  **実機は仕様書のスーパーセットを流す**（`ctrl_info` に仕様書に無い `currentPose` と
  `total_distance` が入る）ため、パーサは未知フィールドを無視し、
  仕様書にあるフィールドの欠落も許容する

未検証: 1802/1804/1102 の成功パスは一度も通っていない（Phase 5で確認する）。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any

from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class Pose2D:
    """平面上の姿勢。yawの単位はラジアン。

    G1のナビは室内平地が適用条件で、roll/pitch は制御対象ではない。
    z も 1102 に渡す値は「地図の高さ」であって指示できる自由度ではないため持たない。
    """

    x: float
    y: float
    yaw: float = 0.0


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """yaw[rad] -> (q_x, q_y, q_z, q_w)。Z軸まわりの回転のみ。"""

    return tuple(Rotation.from_euler("z", yaw).as_quat())


def quaternion_to_yaw(q_x: float, q_y: float, q_z: float, q_w: float) -> float:
    """(q_x,q_y,q_z,q_w) -> yaw[rad]。roll/pitch は捨てる。

    正規化されていない四元数も受ける（実機のJSONを直接食わせるため）。
    ノルムが0に近い四元数は scipy が ValueError にする。
    """

    return float(Rotation.from_quat([q_x, q_y, q_z, q_w]).as_euler("zyx")[0])

SERVICE_NAME = "slam_operate"
SERVICE_VERSION = "1.0.0.1"

# --- api-id（公式「三、服务数据说明」。unitree_slam_example/keyDemo.cpp と一致） ---
API_START_MAPPING = 1801
API_END_MAPPING = 1802
API_INIT_POSE = 1804
API_NAVIGATE_POSE = 1102
API_PAUSE = 1201
API_RESUME = 1202
API_CLOSE_SLAM = 1901

# --- 固定値。公式で「固定值」と明記されているもの ---
SLAM_TYPE_INDOOR = "indoor"
NAVIGATE_MODE = 1  # 绕障(回避)モードはG1に無い。障害物に遭遇したら停止する

# --- errorCode。公式に一覧が無く、実測で判明したものだけを定数化する ---
ERROR_OK = 0
ERROR_LOAD_PCD_FAILED = 507  # "Load pcd failed." ファイル不在/形式不正/権限を区別できない総称エラー

# --- トピック（公式「四、话题数据说明」） ---
TOPIC_SLAM_INFO = "rt/slam_info"
TOPIC_SLAM_KEY_INFO = "rt/slam_key_info"
TOPIC_RELOCATION_GLOBAL_MAP = "rt/unitree/slam_relocation/global_map"
TOPIC_RELOCATION_POINTS = "rt/unitree/slam_relocation/points"
TOPIC_RELOCATION_ODOM = "rt/unitree/slam_relocation/odom"
TOPIC_MAPPING_POINTS = "rt/unitree/slam_mapping/points"
TOPIC_MAPPING_ODOM = "rt/unitree/slam_mapping/odom"

# --- rt/slam_info の type ---
TYPE_ROBOT_DATA = "robot_data"
TYPE_POS_INFO = "pos_info"
TYPE_MAPPING_INFO = "mapping_info"  # 建図中は pos_info ではなくこちら
TYPE_CTRL_INFO = "ctrl_info"
TYPE_TASK_RESULT = "task_result"  # rt/slam_key_info

# 1804 を投げる前の `ctrl_info.info` / `stateMachine.ctrName`。
# 「地図を読み込んでいない」ことの実測上の目印（README「待機時の状態」）。
CTRL_NAME_NOT_INIT = "not init"


# ---------------------------------------------------------------- リクエスト


def start_mapping_request() -> dict[str, Any]:
    """1801 建図開始。"""

    return {"data": {"slam_type": SLAM_TYPE_INDOOR}}


def end_mapping_request(address: str) -> dict[str, Any]:
    """1802 建図終了・保存。

    address は **PC1(运控PC 192.168.123.161) のファイルシステム上のパス**。
    PC2に置いたファイルは 1804 から見えない（README「address は PC1 の…」節）。
    公式はディスク圧迫を避けるため test1.pcd 〜 test10.pcd の使い回しを推奨している。
    """

    return {"data": {"address": address}}


def init_pose_request(address: str, pose: Pose2D, z: float = 0.0) -> dict[str, Any]:
    """1804 初期位姿（地図読込＋自己位置設定）。"""

    q_x, q_y, q_z, q_w = yaw_to_quaternion(pose.yaw)
    return {
        "data": {
            "x": pose.x,
            "y": pose.y,
            "z": z,
            "q_x": q_x,
            "q_y": q_y,
            "q_z": q_z,
            "q_w": q_w,
            "address": address,
        }
    }


def navigate_request(target: Pose2D, z: float = 0.0) -> dict[str, Any]:
    """1102 位姿導航。

    **現在位置との距離が10mを超えてはならない**（公式の注意書き）。
    分割は `nav.route` の責任で、ここでは検査しない。
    リクエスト単体では現在位置を知らないため、ここで弾くと嘘の安心を与える。
    """

    q_x, q_y, q_z, q_w = yaw_to_quaternion(target.yaw)
    return {
        "data": {
            "targetPose": {
                "x": target.x,
                "y": target.y,
                "z": z,
                "q_x": q_x,
                "q_y": q_y,
                "q_z": q_z,
                "q_w": q_w,
            },
            "mode": NAVIGATE_MODE,
        }
    }


def pause_request() -> dict[str, Any]:
    """1201 一時停止。"""

    return {"data": {}}


def resume_request() -> dict[str, Any]:
    """1202 再開。"""

    return {"data": {}}


def close_slam_request() -> dict[str, Any]:
    """1901 SLAM終了。"""

    return {"data": {}}


# ---------------------------------------------------------------- レスポンス


@dataclass(frozen=True)
class ServiceResponse:
    """全APIに共通のレスポンス封筒。"""

    succeed: bool
    error_code: int
    info: str
    data: dict[str, Any] = field(default_factory=dict)

    @property
    def is_load_pcd_failure(self) -> bool:
        return self.error_code == ERROR_LOAD_PCD_FAILED


def parse_response(payload: str | bytes | dict[str, Any]) -> ServiceResponse:
    """レスポンスJSONを読む。壊れたJSONは ValueError にする（黙って握らない）。"""

    body = _as_dict(payload, "レスポンス")
    return ServiceResponse(
        succeed=bool(body.get("succeed", False)),
        error_code=int(body.get("errorCode", -1)),
        info=str(body.get("info", "")),
        data=body.get("data") or {},
    )


# ------------------------------------------------------------ トピックの中身


@dataclass(frozen=True)
class ObstacleInfo:
    """`ctrl_info.data.obsInfo`。state=障害物の有無、time=遭遇してからの秒数。"""

    blocked: bool = False
    blocked_seconds: float = 0.0


@dataclass(frozen=True)
class Progress:
    """`ctrl_info.data.progress`。completion_percentage の値域は実測で未確認。"""

    used_time: float = 0.0
    last_time: float = 0.0
    completion_percentage: float = 0.0


@dataclass(frozen=True)
class CtrlInfo:
    """`rt/slam_info` の `type:"ctrl_info"`。約5Hzで流れる（実測）。

    `initialized` は「1804が通って地図を読み込んだか」の判定に使う。
    速度が0かどうかでは停止を判定できない（実測: 静止中も vx/vy/vyaw が0にならない）。
    """

    state: str = ""
    ctrl_name: str = ""
    is_paused: bool = False
    is_arrived: bool = False
    target_node_name: int = 0
    current_pose: Pose2D | None = None
    target_pose: Pose2D | None = None
    obstacle: ObstacleInfo = field(default_factory=ObstacleInfo)
    progress: Progress = field(default_factory=Progress)
    error_code: int = 0
    info: str = ""

    @property
    def initialized(self) -> bool:
        """1804が成功して地図が載っているか。

        待機時の実測は info も ctrName も "not init"。どちらか一方だけが
        変わる可能性を考え、**両方が "not init" でないこと**を条件にはしない。
        """

        return CTRL_NAME_NOT_INIT not in (self.ctrl_name, self.info)


@dataclass(frozen=True)
class TaskResult:
    """`rt/slam_key_info` の `type:"task_result"`。タスク実行時のみ流れる。"""

    is_arrived: bool
    target_node_name: int = 0
    error_code: int = 0
    info: str = ""


def parse_slam_info(payload: str | bytes | dict[str, Any]) -> tuple[str, dict[str, Any]]:
    """`rt/slam_info` を (type, data) に割る。type別のパースは呼び側で選ぶ。"""

    body = _as_dict(payload, "slam_info")
    return str(body.get("type", "")), (body.get("data") or {})


def parse_ctrl_info(payload: str | bytes | dict[str, Any]) -> CtrlInfo:
    """`ctrl_info` を読む。欠けたフィールドは既定値で埋める。

    実機は仕様書に無い `currentPose` と `total_distance` を追加で流してくる。
    仕様書側にしか無い `startPose` を使う予定は今のところ無い。
    """

    body = _as_dict(payload, "ctrl_info")
    data = body.get("data") or {}
    machine = data.get("stateMachine") or {}
    obstacle = data.get("obsInfo") or {}
    progress = data.get("progress") or {}
    return CtrlInfo(
        state=str(machine.get("state", "")),
        ctrl_name=str(machine.get("ctrName", "")),
        is_paused=bool(machine.get("isPause", False)),
        is_arrived=bool(data.get("is_arrived", False)),
        target_node_name=int(data.get("targetNodeName", 0) or 0),
        current_pose=parse_pose(data.get("currentPose")),
        target_pose=parse_pose(data.get("targetPose")),
        obstacle=ObstacleInfo(
            blocked=bool(obstacle.get("state", False)),
            blocked_seconds=float(obstacle.get("time", 0.0) or 0.0),
        ),
        progress=Progress(
            used_time=float(progress.get("used_time", 0.0) or 0.0),
            last_time=float(progress.get("last_time", 0.0) or 0.0),
            completion_percentage=float(progress.get("completion_percentage", 0.0) or 0.0),
        ),
        error_code=int(body.get("errorCode", 0) or 0),
        info=str(body.get("info", "")),
    )


def parse_task_result(payload: str | bytes | dict[str, Any]) -> TaskResult:
    """`rt/slam_key_info` を読む。"""

    body = _as_dict(payload, "task_result")
    data = body.get("data") or {}
    return TaskResult(
        is_arrived=bool(data.get("is_arrived", False)),
        target_node_name=int(data.get("targetNodeName", 0) or 0),
        error_code=int(body.get("errorCode", 0) or 0),
        info=str(body.get("info", "")),
    )


def parse_pose(raw: dict[str, Any] | None) -> Pose2D | None:
    """四元数形式とroll/pitch/yaw形式のどちらのPoseも受ける。

    `ctrl_info` は roll/pitch/yaw、`pos_info` と 1102 の入力は四元数、と
    同じ「Pose」でも表現が違うため、両方を受けてPose2Dに正規化する。
    x/y すら無いものは Pose ではないので None を返す。
    """

    if not raw:
        return None
    if "x" not in raw or "y" not in raw:
        return None
    x = float(raw["x"])
    y = float(raw["y"])
    if "q_w" in raw:
        yaw = quaternion_to_yaw(
            float(raw.get("q_x", 0.0)),
            float(raw.get("q_y", 0.0)),
            float(raw.get("q_z", 0.0)),
            float(raw["q_w"]),
        )
    else:
        yaw = float(raw.get("yaw", 0.0))
    return Pose2D(x, y, yaw)


def _as_dict(payload: str | bytes | dict[str, Any], label: str) -> dict[str, Any]:
    if isinstance(payload, dict):
        return payload
    text = payload.decode("utf-8") if isinstance(payload, bytes) else payload
    try:
        body = json.loads(text)
    except json.JSONDecodeError as error:
        raise ValueError(f"{label}のJSONを解釈できない: {error}") from error
    if not isinstance(body, dict):
        raise ValueError(f"{label}のJSONがオブジェクトではない: {type(body).__name__}")
    return body
