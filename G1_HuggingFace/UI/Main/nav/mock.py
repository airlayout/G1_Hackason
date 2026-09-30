"""実機も ROS も無しで UI を組むためのモック。

**実在の地図（既定は `room_a_map_20260911`）の上を、実在の通路に沿って歩く。**
巡回路の4点は地図の自由空間から選んであり、全区間で壁から 0.42m 以上離れている
（2026-09-20 に距離変換で確認）。地図の上に線を引いたときに壁を突き抜けないので、
画面の見た目がそのまま実機の見た目に近くなる。

⚠️ **ここで再現しているのは「状態の移り変わり」だけで、物理は無い。**
歩行の揺れ・自己位置の飛び・Nav2 の経路の曲がりは再現していない。モックで見た目が
整ったことは、実機で整うことの保証にはならない。

⚠️ 状態機械は実機の断り方に合わせてある（`patrol_ctl.sh` の表と同じ）:
`patrol_start` は `bridge_state` が NAVIGATING でないと断る。UI に断り文を出す経路を
モックの段階で作り込んでおくため。
"""
from __future__ import annotations

import math
import threading
import time
from pathlib import Path

from mapimage import MapImage, load_map

from .base import CommandResult, NavSource, NavState, Pose

# 地図の自由空間から求めた往復巡回路（片道 17.7m）。
# ⚠️ **これはモック専用の作り物。** 実機の巡回路は現地で教示するか
# `record_waypoints.py` で記録すること（地図座標を手で書かない、が Navigation 側の方針）。
# 状態を進める周期。実機の Nav2 が 20Hz で指令を出すのに合わせてある
_TICK_S = 0.05

MOCK_ROUTE: tuple[tuple[float, float], ...] = (
    (10.25, -4.40),
    (11.65, 1.00),
    (11.85, 8.50),
    (12.15, 13.10),
)


class MockNavSource(NavSource):
    """巡回路の上を一定速度で往復し、ボタンに状態機械で応答するモック。"""

    def __init__(self, map_yaml: Path, speed_mps: float = 0.35,
                 route: tuple[tuple[float, float], ...] = MOCK_ROUTE) -> None:
        self._map: MapImage = load_map(map_yaml)
        self._speed = float(speed_mps)
        self._route = list(route)
        self._lock = threading.Lock()

        # 現在地は1点目から始める。向きは2点目を向く
        self._x, self._y = self._route[0]
        self._yaw = self._heading(0, 1)
        self._leg = 0           # いま何番目の点へ向かっているか
        self._direction = 1     # +1: 順方向、-1: 折り返し

        self._bridge_state = "STANDBY"
        self._patrol_state = "IDLE"
        self._message = "モックで動作中。実機には繋がっていません"
        # STANDBY は TF/センサーが揃うまでの状態。数秒で READY に上がるのを真似る
        self._ready_at = time.monotonic() + 3.0

        # ⚠️ **時間を進めるのは専用スレッドの仕事。** get_state() の中で進めると、
        # ブラウザのポーリング周期で歩く速さが変わってしまう（1Hz で問い合わせると
        # 速度が半分になる、を実際に踏んだ）。実機は誰も見ていなくても歩くので、
        # 取得とは独立に回す。
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="mock-nav", daemon=True)
        self._thread.start()

        print(f"[モック] 巡回路 {len(self._route)} 点、速度 {self._speed} m/s で開始する")

    def _run(self) -> None:
        """一定周期で状態を進める。"""
        while not self._stop.wait(_TICK_S):
            with self._lock:
                now = time.monotonic()
                if self._bridge_state == "STANDBY" and now >= self._ready_at:
                    self._bridge_state = "READY"
                    self._message = "TF・センサーは健全。走行許可を出してください"
                if self._bridge_state == "NAVIGATING" and self._patrol_state == "RUNNING":
                    self._advance(_TICK_S)

    # ---- 内部 ---------------------------------------------------------------

    def _heading(self, i: int, j: int) -> float:
        (x1, y1), (x2, y2) = self._route[i], self._route[j]
        return math.degrees(math.atan2(y2 - y1, x2 - x1))

    def _target_index(self) -> int:
        return self._leg

    def _advance(self, dt: float) -> None:
        """巡回路に沿って dt 秒ぶん進める。端に着いたら折り返す。"""
        tx, ty = self._route[self._leg]
        dx, dy = tx - self._x, ty - self._y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            self._next_leg()
            return

        # 向きは進行方向へ滑らかに寄せる（毎周期で飛ばない程度に）
        want = math.degrees(math.atan2(dy, dx))
        diff = (want - self._yaw + 180.0) % 360.0 - 180.0
        self._yaw += max(-90.0 * dt, min(90.0 * dt, diff))

        step = self._speed * dt
        if step >= dist:
            self._x, self._y = tx, ty
            self._next_leg()
        else:
            self._x += dx / dist * step
            self._y += dy / dist * step

    def _next_leg(self) -> None:
        """次の点へ。端まで行ったら向きを反転して往復する。"""
        nxt = self._leg + self._direction
        if nxt >= len(self._route) or nxt < 0:
            self._direction *= -1
            nxt = self._leg + self._direction
        self._leg = max(0, min(len(self._route) - 1, nxt))

    # ---- NavSource ----------------------------------------------------------

    def get_state(self) -> NavState:
        with self._lock:
            pose = Pose(self._x, self._y, self._yaw)
            goal: Pose | None = None
            plan: list[tuple[float, float]] = []
            if self._patrol_state in ("RUNNING", "HOLD"):
                gx, gy = self._route[self._leg]
                goal = Pose(gx, gy, 0.0)
                # モックの「経路」は次の点までの直線。Nav2 の曲がった経路は再現しない
                plan = [(self._x, self._y), (gx, gy)]

            return NavState(
                connected=True,
                pose=pose,
                goal=goal,
                plan=plan,
                route=list(self._route),
                bridge_state=self._bridge_state,
                patrol_state=self._patrol_state,
                patrol_index=self._leg,
                patrol_total=len(self._route),
                message=self._message,
            )

    def send_command(self, name: str) -> CommandResult:
        with self._lock:
            if name == "enable_navigation":
                if self._bridge_state == "FAULT":
                    return CommandResult(False, "FAULT のままです。先に clear_fault を。")
                if self._bridge_state == "STANDBY":
                    return CommandResult(False, "STANDBY です。TF とセンサーがまだ揃っていません。")
                self._bridge_state = "NAVIGATING"
                self._message = "走行許可を出した"
                return CommandResult(True, "走行を許可しました")

            if name == "disable_navigation":
                self._bridge_state = "READY"
                self._patrol_state = "IDLE" if self._patrol_state == "RUNNING" else self._patrol_state
                self._message = "走行許可を取り消した"
                return CommandResult(True, "走行許可を取り消しました")

            if name == "stop":
                self._patrol_state = "HOLD" if self._patrol_state == "RUNNING" else self._patrol_state
                self._message = "停止した"
                return CommandResult(True, "停止しました")

            if name == "clear_fault":
                if self._bridge_state != "FAULT":
                    return CommandResult(False, "FAULT ではありません")
                self._bridge_state = "READY"
                self._message = "FAULT を解除した"
                return CommandResult(True, "FAULT を解除しました")

            if name == "patrol_start":
                # ⚠️ 実機と同じ断り方をする（patrol_ctl.sh の表）
                if self._bridge_state != "NAVIGATING":
                    return CommandResult(
                        False, f"bridge が {self._bridge_state} です。"
                               "NAVIGATING でないと巡回は始められません")
                if not self._route:
                    return CommandResult(False, "巡回路が空です。現地で教示してください")
                self._patrol_state = "RUNNING"
                self._message = "巡回を開始した"
                return CommandResult(True, "巡回を開始しました")

            if name == "patrol_pause":
                if self._patrol_state != "RUNNING":
                    return CommandResult(False, "巡回していません")
                # ⚠️ 実機の patrol_node は "HOLD" にする。"PAUSED" ではない
                self._patrol_state = "HOLD"
                self._message = "巡回を一時停止した"
                return CommandResult(True, "一時停止しました")

            if name == "patrol_stop":
                self._patrol_state = "IDLE"
                self._leg = 0
                self._direction = 1
                self._x, self._y = self._route[0]
                self._message = "巡回を止めて1点目に戻した"
                return CommandResult(True, "巡回を停止しました")

            if name == "patrol_skip":
                if self._patrol_state not in ("RUNNING", "HOLD"):
                    return CommandResult(False, "巡回していません")
                self._next_leg()
                self._message = "次の点へ飛ばした"
                return CommandResult(True, "次の点へ進みました")

            return CommandResult(False, f"知らない操作です: {name}")

    @property
    def is_mock(self) -> bool:
        return True

    def map_info(self) -> dict[str, object]:
        return self._map.as_dict()

    def map_png(self) -> bytes:
        return self._map.png

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
