"""UI と航法側の境界。

⚠️ **このファイルに ROS の語彙を持ち込まないこと。** UI は ROS を知らない、というのが
この構成の前提。ROS のトピック名・メッセージ型は `ros_adapter`（Docker 内で動く別プロセス）
の中だけに閉じ込め、UI へは下の `NavState` を JSON にしたものだけが渡る。

座標系は地図座標（ROS でいう `map` フレーム）。単位は m と度。
UI 側は TF ツリーを一切知らなくてよい（`map->base_link` の合成はアダプタの仕事）。
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field


@dataclass
class Pose:
    """地図座標系での位置と向き。"""

    x: float
    y: float
    yaw_deg: float = 0.0

    def as_dict(self) -> dict[str, float]:
        return {"x": round(self.x, 3), "y": round(self.y, 3), "yaw_deg": round(self.yaw_deg, 1)}


@dataclass
class NavState:
    """UI が1画面を描くのに必要な航法側の情報のすべて。

    `connected` が False のときは他のフィールドを信用しないこと（アダプタに繋がって
    いない、あるいは航法側が落ちている）。
    """

    connected: bool = False
    # 現在地。取れていなければ None
    pose: Pose | None = None
    # いま向かっている目的地。巡回中なら次の巡回点
    goal: Pose | None = None
    # Nav2 が立てた経路（地図座標の点列）
    plan: list[tuple[float, float]] = field(default_factory=list)
    # 巡回路そのもの（教示・記録された点列）
    route: list[tuple[float, float]] = field(default_factory=list)
    # 走行状態。DISCONNECTED / STANDBY / READY / NAVIGATING / FAULT / E_STOP
    bridge_state: str = "DISCONNECTED"
    # 巡回状態。⚠️ 実機の patrol_node が出す語を使う:
    # IDLE / RUNNING / HOLD / DONE / TEACH（"PAUSED" という状態は無い）
    patrol_state: str = "IDLE"
    patrol_index: int = 0
    patrol_total: int = 0
    # 直近の一言（断り文やエラーをそのまま出す）
    message: str = ""

    def as_dict(self) -> dict[str, object]:
        return {
            "connected": self.connected,
            "pose": self.pose.as_dict() if self.pose else None,
            "goal": self.goal.as_dict() if self.goal else None,
            "plan": [[round(x, 2), round(y, 2)] for x, y in self.plan],
            "route": [[round(x, 2), round(y, 2)] for x, y in self.route],
            "bridge_state": self.bridge_state,
            "patrol": {
                "state": self.patrol_state,
                "index": self.patrol_index,
                "total": self.patrol_total,
            },
            "message": self.message,
        }


@dataclass
class CommandResult:
    """ボタンを押した結果。`ok` が False なら `message` に断り文が入る。"""

    ok: bool
    message: str = ""

    def as_dict(self) -> dict[str, object]:
        return {"ok": self.ok, "message": self.message}


# UI が投げられる操作の一覧。
# ⚠️ **目的地を「送る」操作は意図的に入れていない。** UI 経由で機体が歩き出す経路を
# 作らない、という設計上の決定（2026-09-20）。地図のクリックでゴールを打てるように
# したくなったら、ここに足す前に安全側の影響を必ず見直すこと。
COMMANDS: tuple[str, ...] = (
    "enable_navigation",
    "disable_navigation",
    "stop",
    "clear_fault",
    "patrol_start",
    "patrol_pause",
    "patrol_stop",
    "patrol_skip",
)


class NavSource(ABC):
    """航法情報の取得元。モックと実機（HTTP アダプタ経由）を差し替えるための境界。"""

    @abstractmethod
    def get_state(self) -> NavState:
        """いまの状態を返す。例外を投げず、取れないときは connected=False を返すこと。"""
        raise NotImplementedError

    @abstractmethod
    def send_command(self, name: str) -> CommandResult:
        """操作を1件実行する。`name` は COMMANDS のいずれか。"""
        raise NotImplementedError

    def map_info(self) -> dict[str, object] | None:
        """地図のメタ情報（resolution / origin / width / height）。無ければ None。"""
        return None

    def map_png(self) -> bytes | None:
        """地図を PNG にしたもの。無ければ None。"""
        return None

    @property
    def is_mock(self) -> bool:
        """作り物かどうか。UI に「モックで動作中」と出させるために使う。

        ⚠️ **画面にはっきり出すこと。** モックの数字を実機のものと取り違えるのが
        いちばん危ない。
        """
        return False

    def close(self) -> None:
        """後始末。既定では何もしない。"""
