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

import numpy as np

from .occupancy import DEFAULT_INFLATION_M, OccupancyGrid
from .protocol import (
    API_INIT_POSE,
    API_NAVIGATE_POSE,
    API_PAUSE,
    API_RESUME,
    Pose2D,
    ServiceResponse,
    init_pose_request,
    navigate_request,
    pause_request,
    resume_request,
)
from .route import MAX_SEGMENT_M, RouteError, Segment, needs_command, plan_route


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
    """迂回時に「ここが塞がれている」と推定する円。機体の前方 detour_ahead_m の位置に、
    半径 `detour_obstacle_radius_m + DEFAULT_INFLATION_M`（＝0.6 + 0.40 = 1.0m）で置く。

    足すのは `DEFAULT_INFLATION_M`（機体半径 0.25m + 定位誤差 + 歩容の揺れ）であって、
    機体半径そのものではない。地図の他のセルと同じ膨張を掛けておかないと、
    この円の縁だけ機体半径ぶんの余裕が無い経路が引ける。

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
        on_note=None,
    ) -> None:
        """`on_note` を渡すと、記録した1行が起きたその場で呼ばれる。

        報告書（`MissionReport.log`）は最後まで溜まるので、走っている間は
        何も分からない。実時間で走らせる sim では 100 秒以上黙ることになり、
        「動いているのか固まっているのか」が見えない。
        """
        if len(waypoints) < 2:
            raise ValueError(f"ウェイポイントは出発点を含めて2点以上必要: {len(waypoints)}点")
        if options.laps < 1:
            raise ValueError(f"周回数は1以上: {options.laps}")
        self._transport = transport
        self._base_grid = grid
        """実測の地図。**ここは絶対に書き換えない。**"""

        self._guessed_obstacles: list[tuple[float, float, float]] = []
        """迂回のときに「ここが塞がれている」と推定した円 (x, y, 半径)。

        地図へ直接書き込まずに一覧で持つのは、**憶測を取り消せるようにする**ため。
        `slam_operate` は障害物の位置を教えてくれない（`obsInfo` は有無と経過秒だけ）
        ので、これはあくまで推定であって観測ではない。
        """

        self._waypoints = list(waypoints)
        self._options = options
        self._on_note = on_note
        self._report = MissionReport(outcome=Outcome.COMPLETED)

    def run(self) -> MissionReport:
        started = self._transport.now()
        try:
            self._run()
        finally:
            self._report.elapsed_s = self._transport.now() - started
        return self._report

    @property
    def waypoints_reached(self) -> int:
        """いま何個のウェイポイントを通過したか。走っている最中でも読める。

        報告書は `run()` が返るまで手に入らないので、ビューアに
        「次はどこを目指しているのか」を描くのにこれが要る。
        """

        return self._report.waypoints_reached

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
            remaining = self._drop_satisfied(current, remaining)
            if not remaining:
                return
            segments = self._plan(current, remaining)
            if segments is None:
                return
            if not segments:
                # ここに来るのは `_drop_satisfied` と `plan_route` の判定が
                # 食い違ったとき。放っておくと同じ計画を延々と繰り返して
                # **無限ループする**（実際に踏んで 11 分回り続けた）。
                # 進めないなら黙って回らず、理由を付けて止まる。
                self._fail(
                    Outcome.FAILED_ROUTE,
                    f"残り{len(remaining)}地点に対して区間が1つも作れなかった。"
                    f"現在地({current.x:.2f}, {current.y:.2f})と"
                    f"次の目標({remaining[0].x:.2f}, {remaining[0].y:.2f})が"
                    "近すぎるか、判定の閾値が食い違っている",
                )
                return
            current, remaining, keep_going = self._walk(segments, current, remaining)
            if not keep_going:
                return

    def _drop_satisfied(self, current: Pose2D, remaining: list[Pose2D]) -> list[Pose2D]:
        """もう立っているウェイポイントを、到達済みとして先頭から取り除く。

        `nav/route.py` は「動く必要が無い」区間を作らない（`MIN_SEGMENT_M` 未満で
        向き直しも要らないもの）。区間が作られないと `_walk` が到達を数えられず、
        そのウェイポイントが `remaining` から永久に外れない。結果として
        同じ計画を作り直し続けて**無限ループする**（実測: 11 分回り続けた）。

        判定は `route.needs_command` と共有する。別々の閾値を持つと必ず食い違う。
        """

        while remaining and not needs_command(current, remaining[0]):
            reached = remaining[0]
            remaining = remaining[1:]
            self._report.waypoints_reached += 1
            self._note(
                f"ウェイポイント({reached.x:.2f}, {reached.y:.2f})は"
                "既にその場に立っているので到達済みとする"
            )
        return remaining

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
                self._working_grid(current),
                [current, *remaining],
                max_segment_m=self._options.max_segment_m,
            )
        except RouteError as error:
            self._fail(Outcome.FAILED_ROUTE, str(error))
            return None

    def _working_grid(self, current: Pose2D) -> OccupancyGrid:
        """実測の地図に、迂回の憶測を重ねた格子。**足元だけは憶測から除く。**

        迂回のあとの歩行は、計画した直線をそのままなぞるわけではない
        （歩容の揺れと定位のずれがある）。憶測で置いた円のふちを、
        回り込む途中でかすめて中に入ることが実際に起きる
        （sim で実測: 1 回目の迂回で置いた円の中心へ 17 秒後の自分が 0.95m まで入った）。
        そのまま計画すると「出発点が障害物の中にある」で経路を引けない。

        逃げ道は 2 通りあって、**円ごと取り下げるのは駄目**だった。
        取り下げると真下にある本物の障害物へまっすぐ戻ってしまい、
        同じ場所で迂回を繰り返して上限に達する（sim で実測）。
        機体はふちをかすめただけで、その先が塞がっているという推定は正しい。

        なので**足元のぶんだけ憶測に穴を空ける**。穴は憶測の層にしか空けないので、
        実測の壁は 1 セルも消えない。機体が本物の壁の膨張域に入り込んでいる場合は
        足元が塞がったままになり、経路が引けずに理由付きで止まる（それが正しい）。
        """

        if not self._guessed_obstacles:
            return self._base_grid

        guesses = np.zeros_like(self._base_grid.blocked)
        for x, y, radius in self._guessed_obstacles:
            guesses |= self._base_grid.disk(x, y, radius)

        escape = self._escape_radius(current)
        if escape > 0.0:
            guesses &= ~self._base_grid.disk(current.x, current.y, escape)
            self._note(
                f"足元 ({current.x:.2f}, {current.y:.2f}) が迂回の憶測の中に入ったので、"
                f"半径{escape:.2f}mだけ憶測から除いた（実測の地図は触っていない）"
            )
        return self._base_grid.with_extra_blocked(guesses)

    def _escape_radius(self, current: Pose2D) -> float:
        """足元を憶測から抜け出させるのに要る半径[m]。中に入っていなければ 0。

        いちばん深く入り込んでいる円のふちを、`detour_self_clearance_m` だけ
        越えるところまで。円の中心に立っているなら円がまるごと消えるが、
        それは「そこは歩けた」ことの証明なので消えてよい。
        """

        deepest = 0.0
        for x, y, radius in self._guessed_obstacles:
            distance = math.hypot(current.x - x, current.y - y)
            if distance < radius:
                deepest = max(deepest, radius - distance)
        if deepest <= 0.0:
            return 0.0
        return deepest + self._options.detour_self_clearance_m

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
        heading = _heading(here, segment.target)
        radius = self._options.detour_obstacle_radius_m + DEFAULT_INFLATION_M
        ahead = max(
            self._options.detour_ahead_m, radius + self._options.detour_self_clearance_m
        )
        blocked_x, blocked_y = _ahead_of(here, heading, ahead)
        self._guessed_obstacles.append((blocked_x, blocked_y, radius))
        self._report.detours += 1
        self._note(
            f"迂回{self._report.detours}回目: ({blocked_x:.2f}, {blocked_y:.2f}) を塞がれたとみなす"
        )
        return here, remaining, True

    def _execute(self, segment: Segment) -> _SegmentResult:
        """1区間ぶんの1102を投げて到達を待つ。"""

        # 「どこからどこへ向かっているのか」を、投げた時点で残す。
        # 到達したときだけ記録していると、走っている間はどこを目指しているのか
        # 分からない（実際に sim を見ていて分からなかった）。
        self._note(
            f"1102 #{segment.index}: "
            f"({segment.start.x:.2f}, {segment.start.y:.2f}) -> "
            f"({segment.target.x:.2f}, {segment.target.y:.2f})  {segment.length:.2f}m"
            + ("  ★ウェイポイント" if segment.is_waypoint else "")
        )
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
        entry = f"[{self._transport.now():8.2f}s] {line}"
        self._report.log.append(entry)
        if self._on_note is not None:
            self._on_note(entry)

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
        self._blocked_at_last_progress = 0.0
        """進捗が最後に動いた時点での、塞がれていた時間の累計[s]。

        停滞の判定は「最後に進捗が動いてから」を測るので、引く塞がれ時間も
        同じ起点から数えないと合わない。区間開始からの累計をそのまま引くと、
        **進捗が動く前に塞がれていた時間を二重に割り引く**ことになり、
        塞がれた回数が多い区間ほど本物の停滞に気付けなくなる。
        """

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

        # 打ち切り時間は区間開始からを測るので、引くのも区間開始からの累計
        moving_s = now - self._started - self._blocked_elapsed(now)
        if moving_s > self._budget_s:
            return _SegmentResult.TIMEOUT
        # 停滞は「最後に進捗が動いてから」を測るので、引くのもその時点からの分だけ
        blocked_since_progress = self._blocked_elapsed(now) - self._blocked_at_last_progress
        stalled_s = now - self._last_progress_at - blocked_since_progress
        if stalled_s > self._options.stall_timeout_s:
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
            self._blocked_at_last_progress = self._blocked_elapsed(now)

    def _blocked_elapsed(self, now: float) -> float:
        if self._blocked_since is None:
            return self._blocked_total_s
        return self._blocked_total_s + (now - self._blocked_since)


_OUTCOME_OF = {
    _SegmentResult.TIMEOUT: Outcome.FAILED_TIMEOUT,
    _SegmentResult.STALL: Outcome.FAILED_STALL,
    _SegmentResult.REJECTED: Outcome.FAILED_REJECTED,
}


def _heading(origin: Pose2D, target: Pose2D) -> float:
    """origin から target を向く方位[rad]。同じ点なら origin の向きのまま。

    `nav/geometry.py` の `Pose2D.heading_to` を 1 行に畳んだもの。
    geometry.py は scipy.Rotation に置き換えて削除したが、
    この 1 行に OSS の代替は無い。
    """

    dx, dy = target.x - origin.x, target.y - origin.y
    if math.hypot(dx, dy) < 1e-9:
        return origin.yaw
    return math.atan2(dy, dx)


def _ahead_of(pose: Pose2D, heading: float, distance: float) -> tuple[float, float]:
    return pose.x + math.cos(heading) * distance, pose.y + math.sin(heading) * distance
