"""1804 → 1102連打 → 到達待ち、の状態機械。

`SlamTransport` 越しにしか外と話さないので、mock / sim / 実機で同じコードが動く。

やること:

1. **1804で地図を読ませ自己位置を合わせる**（507はリトライする）
2. **経路を区間に割って1102を順に投げる**（割るのは `nav/route.py`）
3. **到達を待つ**。塞がれたら待ち、待っても駄目なら迂回する
4. **巡回する**（周回数ぶん繰り返す）

やらないこと: 速度制御（`speed`パラメータが無い）、バランス制御（PC1の仕事）。

⚠️ **到達判定は実機で未検証。** `rt/slam_key_info` の `task_result` を一次情報とし、
`ctrl_info.is_arrived` は補助に使う。前の区間の`is_arrived`が残っていると
次の区間を即座に「到達」と誤判定するので、**一度Falseを見てから**しか採用しない。
Phase 5で実機の挙動を見て詰める。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from .geometry import Pose2D
from .occupancy import DEFAULT_INFLATION_M, OccupancyGrid
from .protocol import (
    API_INIT_POSE,
    API_NAVIGATE_POSE,
    API_PAUSE,
    API_RESUME,
    ServiceResponse,
    init_pose_request,
    navigate_request,
    pause_request,
    resume_request,
)
from .route import MAX_SEGMENT_M, RouteError, Segment, plan_route


class Outcome(Enum):
    """ミッションの終わり方。"""

    COMPLETED = "completed"
    FAILED_INIT = "failed_init"  # 1804が通らなかった
    FAILED_ROUTE = "failed_route"  # 経路を引けなかった
    FAILED_REJECTED = "failed_rejected"  # 1102が受理されなかった
    FAILED_TIMEOUT = "failed_timeout"  # 想定時間を超えても着かない
    FAILED_STALL = "failed_stall"  # 進捗が動かない
    FAILED_BLOCKED = "failed_blocked"  # 塞がれ続けて迂回もできない


class _SegmentResult(Enum):
    ARRIVED = "arrived"
    BLOCKED = "blocked"
    TIMEOUT = "timeout"
    STALL = "stall"
    REJECTED = "rejected"


@dataclass(frozen=True)
class MissionOptions:
    """ミッションの調整つまみ。既定値の根拠はそれぞれのコメントに書く。"""

    map_address: str
    """PC1上の地図PCDのパス。1802が書いた場所を指す（PC2ではない）。"""

    laps: int = 1
    """周回数。巡回デモ用。"""

    max_segment_m: float = MAX_SEGMENT_M

    init_backoff_s: tuple[float, ...] = (0.5, 1.5)
    """1804が507で落ちたときの待ち時間。要素数＋1回まで試す。

    507は総称エラーで原因が分からず、応答も0.01秒で返る（＝ファイルI/Oの前に
    落ちている）ので、粘っても同じ結果になる可能性が高い。
    1802直後の書き込み完了とのレースだけを拾えれば十分なので数回で諦める。
    無限リトライは原因を隠すだけなのでやらない。
    """

    poll_interval_s: float = 0.2
    """状態の見に行く間隔。`ctrl_info` が実測で約5Hzなのでそれに合わせる。"""

    assumed_speed_mps: float = 0.5
    """想定移動速度。`speed`は指定できないので、打ち切り時間の見積りにだけ使う。"""

    arrival_timeout_factor: float = 3.0
    arrival_timeout_margin_s: float = 20.0
    """打ち切り時間 = 区間距離/想定速度 * factor + margin。

    factorを3倍も取るのは、回頭・加減速・定位のばらつきが読めないため。
    実機で1102を計測したら詰める。
    """

    stall_timeout_s: float = 45.0
    """`progress.completion_percentage` が動かないまま経過したら打ち切る秒数。

    速度が0かどうかでは停止を判定できない（実測: 静止中も vx/vy/vyaw が0にならない）。
    """

    obstacle_wait_s: float = 15.0
    """障害物で止まってから迂回に切り替えるまで待つ秒数。

    人が横切っただけなら数秒で消える。それを待たずに迂回すると
    巡回経路が無用に伸びる。
    """

    detour_limit: int = 3
    """1ミッションで許す迂回の回数。これを超えたら諦めて報告する。"""

    detour_ahead_m: float = 1.5
    detour_obstacle_radius_m: float = 0.6
    """迂回時に地図へ書き込む仮の障害物。機体の前方 detour_ahead_m の位置に、
    半径 detour_obstacle_radius_m + 機体半径 の円を置く。

    `slam_operate` は障害物の位置を教えてくれない（`obsInfo` は有無と経過秒のみ）
    ので推定するしかない。LiDARの生点群から実位置を取るのは Phase 5 以降の課題。

    前方1.5mなのは、機体が障害物を検知して止まる位置がその手前だから。
    実機で1102を計測したら詰める。
    """

    detour_self_clearance_m: float = 0.3
    """仮の障害物を機体自身から離す最小距離[m]。

    これが無いと、書き込んだ円が機体の現在位置を飲み込み、
    「出発点が障害物の中にある」として経路を引けなくなる。
    機体が現にそこに立っている以上、そこは通行可能でなければならない。
    """

    trust_ctrl_info_arrival: bool = True
    """`ctrl_info.is_arrived` を到達判定に使うか。

    一次情報は `rt/slam_key_info` の `task_result`。実機で `task_result` が
    確実に飛ぶと分かったらFalseにしてよい。
    """


@dataclass
class MissionReport:
    """走らせた結果。失敗しても「どこまで行けたか」が分かるようにする。"""

    outcome: Outcome
    waypoints_reached: int = 0
    segments_executed: int = 0
    detours: int = 0
    elapsed_s: float = 0.0
    message: str = ""
    log: list[str] = field(default_factory=list)

    @property
    def succeeded(self) -> bool:
        return self.outcome is Outcome.COMPLETED


class Mission:
    """1つの巡回を最後まで走らせる。"""

    def __init__(
        self,
        transport,
        grid: OccupancyGrid,
        waypoints: list[Pose2D],
        options: MissionOptions,
    ) -> None:
        if len(waypoints) < 2:
            raise ValueError(f"ウェイポイントは出発点を含めて2点以上必要: {len(waypoints)}点")
        if options.laps < 1:
            raise ValueError(f"周回数は1以上: {options.laps}")
        self._transport = transport
        self._grid = grid
        self._waypoints = list(waypoints)
        self._options = options
        self._report = MissionReport(outcome=Outcome.COMPLETED)

    def run(self) -> MissionReport:
        started = self._transport.now()
        try:
            self._run()
        finally:
            self._report.elapsed_s = self._transport.now() - started
        return self._report

    def pause(self) -> ServiceResponse:
        """1201。別スレッドから呼ぶ想定。"""

        return self._transport.call(API_PAUSE, pause_request())

    def resume(self) -> ServiceResponse:
        """1202。"""

        return self._transport.call(API_RESUME, resume_request())

    # ------------------------------------------------------------------ 中身

    def _run(self) -> None:
        if not self._initialize():
            return
        current = self._waypoints[0]
        remaining = self._waypoints[1:] * self._options.laps
        while remaining:
            segments = self._plan(current, remaining)
            if segments is None:
                return
            current, remaining, keep_going = self._walk(segments, current, remaining)
            if not keep_going:
                return

    def _initialize(self) -> bool:
        """1804。失敗したら理由を残して終わる。"""

        request = init_pose_request(self._options.map_address, self._waypoints[0])
        waits = (*self._options.init_backoff_s, None)
        for attempt, wait in enumerate(waits, start=1):
            response = self._transport.call(API_INIT_POSE, request)
            if response.succeed:
                self._note(f"1804成功 ({attempt}回目): {self._options.map_address}")
                return True
            self._note(f"1804失敗 ({attempt}回目) errorCode={response.error_code} {response.info}")
            if wait is not None:
                self._transport.sleep(wait)
        self._fail(
            Outcome.FAILED_INIT,
            f"1804が{len(waits)}回とも失敗した。"
            f"地図 {self._options.map_address} がPC1(192.168.123.161)上に存在するか、"
            "1802で保存済みかを確認すること（507はファイル不在・形式不正・権限を区別しない）",
        )
        return False

    def _plan(self, current: Pose2D, remaining: list[Pose2D]) -> list[Segment] | None:
        try:
            return plan_route(
                self._grid, [current, *remaining], max_segment_m=self._options.max_segment_m
            )
        except RouteError as error:
            self._fail(Outcome.FAILED_ROUTE, str(error))
            return None

    def _walk(
        self, segments: list[Segment], current: Pose2D, remaining: list[Pose2D]
    ) -> tuple[Pose2D, list[Pose2D], bool]:
        """区間を順に実行する。迂回が要るときは経路を引き直すため途中で戻る。"""

        for segment in segments:
            result = self._execute(segment)
            self._report.segments_executed += 1
            if result is _SegmentResult.ARRIVED:
                current = segment.target
                if segment.is_waypoint:
                    remaining = remaining[1:]
                    self._report.waypoints_reached += 1
                    self._note(f"ウェイポイント到達 ({current.x:.2f}, {current.y:.2f})")
                continue
            if result is _SegmentResult.BLOCKED:
                return self._detour(segment, remaining)
            self._fail(_OUTCOME_OF[result], f"区間{segment.index}で{result.value}")
            return current, remaining, False
        return current, remaining, True

    def _detour(
        self, segment: Segment, remaining: list[Pose2D]
    ) -> tuple[Pose2D, list[Pose2D], bool]:
        """塞がれた場所を地図に書き込み、経路を引き直す。"""

        if self._report.detours >= self._options.detour_limit:
            self._fail(
                Outcome.FAILED_BLOCKED,
                f"迂回を{self._options.detour_limit}回試したが通れない。"
                "障害物が動かないか、通路が機体幅より狭い",
            )
            return segment.start, remaining, False

        here = self._current_pose(segment.start)
        heading = here.heading_to(segment.target)
        radius = self._options.detour_obstacle_radius_m + DEFAULT_INFLATION_M
        ahead = max(
            self._options.detour_ahead_m, radius + self._options.detour_self_clearance_m
        )
        blocked_x, blocked_y = _ahead_of(here, heading, ahead)
        self._grid = self._grid.with_extra_obstacle(blocked_x, blocked_y, radius)
        self._report.detours += 1
        self._note(
            f"迂回{self._report.detours}回目: ({blocked_x:.2f}, {blocked_y:.2f}) を塞がれたとみなす"
        )
        return here, remaining, True

    def _execute(self, segment: Segment) -> _SegmentResult:
        """1区間ぶんの1102を投げて到達を待つ。"""

        response = self._transport.call(API_NAVIGATE_POSE, navigate_request(segment.target))
        if not response.succeed:
            self._note(f"1102が受理されなかった errorCode={response.error_code} {response.info}")
            return _SegmentResult.REJECTED

        watch = _SegmentWatch(self._transport, self._options, segment)
        while True:
            self._transport.sleep(self._options.poll_interval_s)
            result = watch.step()
            if result is not None:
                return result

    def _current_pose(self, fallback: Pose2D) -> Pose2D:
        info = self._transport.latest_ctrl_info()
        if info is not None and info.current_pose is not None:
            return info.current_pose
        return fallback

    def _note(self, line: str) -> None:
        self._report.log.append(f"[{self._transport.now():8.2f}s] {line}")

    def _fail(self, outcome: Outcome, message: str) -> None:
        self._report.outcome = outcome
        self._report.message = message
        self._note(f"中断: {message}")


class _SegmentWatch:
    """1区間の到達・障害物・停滞を見張る。

    障害物で止まっている時間は打ち切り時間に数えない。
    人が横切っただけで巡回全体を落とすのは行き過ぎだから。
    """

    def __init__(self, transport, options: MissionOptions, segment: Segment) -> None:
        self._transport = transport
        self._options = options
        self._started = transport.now()
        self._budget_s = (
            segment.length / options.assumed_speed_mps * options.arrival_timeout_factor
            + options.arrival_timeout_margin_s
        )
        self._blocked_total_s = 0.0
        self._blocked_since: float | None = None
        self._last_progress = -1.0
        self._last_progress_at = self._started
        self._seen_not_arrived = False

    def step(self) -> _SegmentResult | None:
        """1回ぶん見る。まだ決まらなければ None。"""

        now = self._transport.now()
        if any(result.is_arrived for result in self._transport.take_task_results()):
            return _SegmentResult.ARRIVED

        info = self._transport.latest_ctrl_info()
        if info is not None:
            if self._is_arrival(info):
                return _SegmentResult.ARRIVED
            blocked = self._track_obstacle(info, now)
            if blocked:
                return _SegmentResult.BLOCKED
            self._track_progress(info, now)

        moving_s = now - self._started - self._blocked_elapsed(now)
        if moving_s > self._budget_s:
            return _SegmentResult.TIMEOUT
        if now - self._last_progress_at - self._blocked_elapsed(now) > self._options.stall_timeout_s:
            return _SegmentResult.STALL
        return None

    def _is_arrival(self, info) -> bool:
        if not self._options.trust_ctrl_info_arrival:
            return False
        if not info.is_arrived:
            self._seen_not_arrived = True
            return False
        # 前の区間の到達フラグが残っている可能性があるので、
        # 一度Falseを見るまでは信用しない
        return self._seen_not_arrived

    def _track_obstacle(self, info, now: float) -> bool:
        if not info.obstacle.blocked:
            if self._blocked_since is not None:
                self._blocked_total_s += now - self._blocked_since
                self._blocked_since = None
            return False
        if self._blocked_since is None:
            self._blocked_since = now
            return False
        return now - self._blocked_since >= self._options.obstacle_wait_s

    def _track_progress(self, info, now: float) -> None:
        percentage = info.progress.completion_percentage
        if percentage > self._last_progress:
            self._last_progress = percentage
            self._last_progress_at = now

    def _blocked_elapsed(self, now: float) -> float:
        if self._blocked_since is None:
            return self._blocked_total_s
        return self._blocked_total_s + (now - self._blocked_since)


_OUTCOME_OF = {
    _SegmentResult.TIMEOUT: Outcome.FAILED_TIMEOUT,
    _SegmentResult.STALL: Outcome.FAILED_STALL,
    _SegmentResult.REJECTED: Outcome.FAILED_REJECTED,
}


def _ahead_of(pose: Pose2D, heading: float, distance: float) -> tuple[float, float]:
    return pose.x + math.cos(heading) * distance, pose.y + math.sin(heading) * distance
