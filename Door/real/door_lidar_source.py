"""段 5・L1（2026-10-23）: door_run_real.py の中で、ライダーの推定器（door_lidar_node.py）の電文を受けて生徒に渡す扉の値を作る。numpy だけ。

2 つの使い方（--lidar-mode）:
  track  毎回の測定で当て直す（L2 の (a)）。電文はライダーの姿勢を点群の時刻で出すので、その時刻から今までの動きを
         脚の odometry で足して 50 Hz にする（0.1 s 分なので odometry の倍率の誤差は 1 cm 未満）
  still  STAND で止まっている間の電文から最初の姿勢を決め、以後は odometry だけで運ぶ（L2 の (c)。比べる元）
どちらも扉の角度はライダーの値（測れない回は前の値を保ち、古さが増える。シミュレータの door_angle_lidar と同じ形）。

時刻: 電文の時刻は推定器の time.time()（点群を集めた真ん中あたり）。制御は time.monotonic() で動くので、電文ごとに
「受け取った monotonic − (受け取った time.time() − 電文の時刻)」で点群の monotonic の時刻に直す（起動時に 1 回だけ差を取ると、
時計が途中で動いたときに黙ってずれる。10-23 のレビュー）。推定器は同じ PC2 で動かす前提。

pelvis への直しは、点群の時刻の腰の関節・pelvis の傾きで行う（今の値を古い点群に当てると、腰が 0.3 rad/s で動くと 0.1 s で
1.7°、pelvis が 3° 揺れると 2.4 cm ずれる。10-23 のレビュー）。odometry と一緒に時刻つきで覚えておく（note_state）。

見張り（立ったまま止める理由を返す。止めるのは呼ぶ側）:
  電文が LOST_S（0.5 s）届かない／推定器が見失った（受け入れた回が 0.5 s 無い）／電文の年齢がおかしい（時計の食い違い）
  odometry か関節の値が ODOM_GAP_S（0.3 s）続けて無い（点群の時刻から今まで運べない）
  ライダーの動きと odometry の動きが、直近 1 s で DISAGREE_M（8 cm）か DISAGREE_DEG（5°）を超えて食い違う
  （odometry は横に 7〜10% 短いので、1 s・0.36 m/s で 3 cm は正常。外れて跳んだ時だけ掛かる線）
"""
from __future__ import annotations

import math
import socket
import threading
import time
from collections import deque
from typing import Deque, List, Optional, Tuple

import numpy as np

import door_frame as dfr
import door_lidar_msg as msg

LOST_S = 0.5
ODOM_GAP_S = 0.3
DISAGREE_WINDOW_S = 1.0
DISAGREE_M = 0.08
DISAGREE_DEG = 5.0
STATE_KEEP_S = 6.0
STATE_MATCH_S = 0.05  # 点群の時刻にいちばん近い関節・odometry の値がこれより離れていたら「無い」
MAX_AGE_S = 1.0  # door_angle_lidar の古さの頭打ち（シミュレータの door_sensor.max_age_s）
LATENCY_S = 0.1  # 測った直後の古さ（シミュレータの latency_s。初めの値も 0.1）
STILL_MSGS = 10  # still: 最初の姿勢を決める電文の数（中央値）
MSG_AGE_MAX_S = 1.0  # 受け取った時点で電文の年齢がこれを越える・負なら、時計が食い違っている
REF_TIMEOUT_S = 15.0  # REF の後、推定器から電文がこれだけ来なければ「応えない」

Odom = Tuple[float, float, float]
Waist = Tuple[float, float, float]


class _Rx:
    """受け取った電文と、その点群の monotonic の時刻"""

    __slots__ = ("m", "t_scan", "rx")

    def __init__(self, m: msg.LidarMsg, t_scan: float, rx: float) -> None:
        self.m, self.t_scan, self.rx = m, t_scan, rx


class LidarDoorSource:
    def __init__(self, port: int = msg.PORT, head_xy: Tuple[float, float] = (0.0, 0.0), mode: str = "track") -> None:
        if mode not in ("track", "still"):
            raise ValueError(mode)
        self.mode = mode
        self.head_xy = head_xy
        self.port = port
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("127.0.0.1", port))
        self.rx.settimeout(0.2)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._lock = threading.Lock()
        self.latest: Optional[_Rx] = None
        self.recent: Deque[_Rx] = deque(maxlen=50)
        self.bad = 0
        self.clock_bad = 0
        self.ref_t: Optional[float] = None
        self.states: Deque[Tuple[float, Optional[Odom], Waist, float, float]] = deque()  # (t, odom, 腰, roll, pitch)
        self.state_gap_since: Optional[float] = None
        self.angle_held = 0.0
        self.angle_t: Optional[float] = None  # 最後に測れた角度の点群の時刻（monotonic）
        self.angle_seen = -1  # 角度を取り込んだ最後の電文の連番
        self.angle_valid = 1.0
        self.still_pose: Optional[dfr.PlanarPose] = None
        self.still_odom: Optional[dfr.PlanarPose] = None
        self.scan_pelvis: Deque[Tuple[float, dfr.PlanarPose, Odom]] = deque()  # 食い違いの見張り用（点群の時刻・pelvis・odom）
        self._stop = threading.Event()
        threading.Thread(target=self._recv_loop, daemon=True).start()

    # --- 受け取り（別の糸） ---
    def _recv_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self.rx.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                return
            rx_mono, rx_wall = time.monotonic(), time.time()
            m = msg.unpack(data)
            with self._lock:
                if m is None:
                    self.bad += 1
                    continue
                age = rx_wall - m.stamp
                if not (-0.05 <= age <= MSG_AGE_MAX_S):
                    self.clock_bad += 1
                    continue
                r = _Rx(m, rx_mono - age, rx_mono)
                self.latest = r
                self.recent.append(r)

    def close(self) -> None:
        self._stop.set()
        self.rx.close()

    def request_ref(self) -> None:
        """推定器に「今から止まって扉と地図を作れ」と送る（STAND で止まってから）"""
        self.tx.sendto(b"REF", ("127.0.0.1", self.port + 1))
        with self._lock:
            self.latest, self.recent = None, deque(maxlen=50)
        self.ref_t = time.monotonic()
        self.still_pose = self.still_odom = None
        self.scan_pelvis.clear()
        self.angle_t, self.angle_seen, self.angle_valid, self.angle_held = None, -1, 1.0, 0.0

    def snapshot(self) -> Tuple[Optional[msg.LidarMsg], float]:
        with self._lock:
            r = self.latest
        return (r.m, r.rx) if r is not None else (None, 0.0)

    def ref_status(self, now: float) -> str:
        """REF の後の様子（STAND の表示用）"""
        m, _ = self.snapshot()
        if self.ref_t is None:
            return "REF を送っていない"
        if m is None:
            late = now - self.ref_t > REF_TIMEOUT_S
            return (f"⚠️ 推定器が {now - self.ref_t:.0f} s 応えない（lidar_start.sh で起きているか・推定器のログ）" if late
                    else "準備中（扉と地図を作っている）")
        return "準備よし（A で扉の方策へ）" if self.ready(now) else "電文は来ているが受け入れた推定が無い"

    # --- 制御の 1 周期ごと ---
    def note_state(self, now: float, odom: Optional[Odom], q_waist: Waist, roll: float, pitch: float) -> None:
        """odometry（無ければ None）・腰の関節・pelvis の傾きを時刻つきで覚える"""
        self.states.append((now, odom, q_waist, roll, pitch))
        while self.states and now - self.states[0][0] > STATE_KEEP_S:
            self.states.popleft()

    def _state_at(self, t: float) -> Optional[Tuple[Optional[Odom], Waist, float, float]]:
        if not self.states:
            return None
        best = min(self.states, key=lambda e: abs(e[0] - t))
        return best[1:] if abs(best[0] - t) <= STATE_MATCH_S else None

    def ready(self, now: float) -> bool:
        m, rx = self.snapshot()
        return m is not None and m.pose_ok and now - rx < 0.3

    def _pelvis(self, m: msg.LidarMsg, q_waist: Waist, roll: float, pitch: float) -> dfr.PlanarPose:
        x, y, yaw = msg.lidar_to_pelvis(m.x, m.y, m.yaw, q_waist, roll, pitch, self.head_xy)
        return dfr.PlanarPose(x, y, yaw)

    def _last_ok(self) -> Optional[_Rx]:
        with self._lock:
            for r in reversed(self.recent):
                if r.m.pose_ok:
                    return r
        return None

    def take_still(self, now: float) -> Optional[dfr.PlanarPose]:
        """still: 直近の受け入れた電文を、それぞれ点群の時刻の関節で pelvis に直し、odometry で今へ運んでから中央値を取る。
        以後は今の odometry を基準に運ぶ"""
        cur = self._state_at(now)
        if cur is None or cur[0] is None:
            return None
        with self._lock:
            ok = [r for r in self.recent if r.m.pose_ok][-STILL_MSGS:]
        poses: List[dfr.PlanarPose] = []
        for r in ok:
            st = self._state_at(r.t_scan)
            if st is None or st[0] is None:
                continue
            p = self._pelvis(r.m, st[1], st[2], st[3])
            poses.append(dfr.compose_planar(p, dfr.PlanarPose(*st[0]), dfr.PlanarPose(*cur[0])))
        if len(poses) < STILL_MSGS // 2:
            return None
        yaw = math.atan2(float(np.median([math.sin(p.yaw) for p in poses])), float(np.median([math.cos(p.yaw) for p in poses])))
        self.still_pose = dfr.PlanarPose(float(np.median([p.x for p in poses])), float(np.median([p.y for p in poses])), yaw)
        self.still_odom = dfr.PlanarPose(*cur[0])
        return self.still_pose

    def _odom_gap(self, now: float, have: bool) -> str:
        if have:
            self.state_gap_since = None
            return ""
        if self.state_gap_since is None:
            self.state_gap_since = now
        if now - self.state_gap_since > ODOM_GAP_S:
            return f"odometry が {now - self.state_gap_since:.1f} s 途切れた（枠を運べない）"
        return ""

    def pose(self, now: float, odom: Optional[Odom]) -> Tuple[Optional[dfr.PlanarPose], str]:
        """生徒に渡す pelvis の姿勢と、立ったまま止める理由（無ければ ""）。先に note_state で今の値を入れておく"""
        if self.mode == "still":
            gap = self._odom_gap(now, odom is not None)
            if gap:
                return None, gap
            if self.still_pose is None or self.still_odom is None:
                return None, "最初の姿勢が無い（still。A を押した時に決まる）"
            if odom is None:
                return None, ""  # 短い途切れ。呼ぶ側は前の姿勢を保つ
            return dfr.compose_planar(self.still_pose, self.still_odom, dfr.PlanarPose(*odom)), ""
        m, rx = self.snapshot()
        if m is None or now - rx > LOST_S:
            return None, (f"ライダーの推定が {now - rx:.1f} s 届かない" if m is not None else "ライダーの推定がまだ無い")
        base = self._last_ok()
        if base is None or now - base.t_scan > LOST_S + 0.1:
            return None, "ライダーが扉を見失った（受け入れた推定が 0.5 s 無い）"
        st = self._state_at(base.t_scan)
        gap = self._odom_gap(now, st is not None and st[0] is not None and odom is not None)
        if gap:
            return None, gap
        if st is None or st[0] is None or odom is None:
            return None, ""  # 短い途切れ（0.3 s まで）。呼ぶ側は前の姿勢を保つ
        o_scan, q_waist, roll, pitch = st
        p_scan = self._pelvis(base.m, q_waist, roll, pitch)
        p_now = dfr.compose_planar(p_scan, dfr.PlanarPose(*o_scan), dfr.PlanarPose(*odom))
        return p_now, self._check_disagree(base.t_scan, p_scan, o_scan)

    def _check_disagree(self, t_scan: float, p_scan: dfr.PlanarPose, o_scan: Odom) -> str:
        """直近 1 s のライダーの動きと odometry の動きを比べる（点群の時刻どうし。1 つの点群につき 1 回だけ覚える）"""
        if not self.scan_pelvis or self.scan_pelvis[-1][0] < t_scan:
            self.scan_pelvis.append((t_scan, p_scan, o_scan))
        while self.scan_pelvis and t_scan - self.scan_pelvis[0][0] > DISAGREE_WINDOW_S:
            self.scan_pelvis.popleft()
        if len(self.scan_pelvis) < 2 or t_scan - self.scan_pelvis[0][0] < 0.5 * DISAGREE_WINDOW_S:
            return ""
        t0, p0, o0 = self.scan_pelvis[0]
        via_odom = dfr.compose_planar(p0, dfr.PlanarPose(*o0), dfr.PlanarPose(*o_scan))
        dpos = math.hypot(via_odom.x - p_scan.x, via_odom.y - p_scan.y)
        dyaw = abs(math.degrees(math.atan2(math.sin(via_odom.yaw - p_scan.yaw), math.cos(via_odom.yaw - p_scan.yaw))))
        if dpos > DISAGREE_M or dyaw > DISAGREE_DEG:
            return f"ライダーと odometry が {t_scan - t0:.1f} s で {dpos * 100:.0f} cm・{dyaw:.1f}° 食い違った"
        return ""

    def angle_obs(self, now: float) -> np.ndarray:
        """door_angle_lidar の形 [角度 rad, 最後の測定が成功したか, 古さ/1.0 s]"""
        with self._lock:
            r = self.latest
        if r is not None and r.m.seq != self.angle_seen:
            self.angle_seen = r.m.seq
            if r.m.angle_ok:
                self.angle_held, self.angle_t, self.angle_valid = float(r.m.angle), r.t_scan, 1.0
            else:
                self.angle_valid = 0.0
                if self.angle_t is None:
                    self.angle_t = r.t_scan
        if self.angle_t is None:
            return dfr.door_closed_angle_obs()
        age = min(MAX_AGE_S, max(LATENCY_S, now - self.angle_t))
        return np.array([self.angle_held, self.angle_valid, age / MAX_AGE_S])
