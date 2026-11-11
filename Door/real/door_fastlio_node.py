"""11-10 手順 L2（計画書 docs/plan/2026-11-09-g1-door-staged-rgbd.md 9.4）: FAST-LIO（逐次にマッピング）の姿勢を、
扉の座標でのライダーの床の上の姿勢にして、door_lidar_msg の電文で走らせ役へ送る（今の受け口 door_lidar_source がそのまま使える）。

流れ（L1 の l1_fastlio_carry_1110.py と同じ計算を、走査ごとに逐次に行う）:
  1. 走らせ役から REF を受ける（UDP・port+1）。"REF2 x y yaw" は段 0 の網（c2）で測った「扉の座標でのライダーの床の上の姿勢」。
     "REF" だけなら (0, 0, 0)（扉の代わりに段 0 のライダーの真下が原点。試し用）
  2. REF の後 REF_SKIP_S 捨て、止まった STILL_SCANS 回（2.5 秒）の生の点群で床を当てる（lidar_door_est.level_from_scans）。
     同じ回の FAST-LIO の姿勢の平均を段 0 の姿勢にする
  3. 以後、FAST-LIO の出力ごとに: 段 0 からの動きを床に投げ（planar_track）、REF の姿勢に足して（compose）電文を送る
時刻: FAST-LIO の出力のヘッダはライダーの時計（日で 0.1〜2 秒ずれる）。その回の点群のヘッダ（＋1 回）に当て、
  その点群を受け取った PC2 の time.time() から「走査の終わり」の時刻（受け取り − SCAN_DELAY_S ＋ SCAN_PERIOD_S/2）を付ける。
  ライダーの時計は使わない（受け取りの時計で毎回付け直すので、時計のずれは消える）
pelvis への直し（腰の関節・pelvis の IMU・取付は URDF の計算だけ＝L3）と /dog_odom でのつなぎ（50 Hz）は受け口の側（track の形）。
L2b（9.14）: 手のひらで扉の角度を出す時も、受け口の track の形（姿勢の時刻にそろえ、その後を /dog_odom でつなぐ）が要る。

PC2 で: python3 -u door_fastlio_node.py [--iface eth0]（FAST-LIO は Mapping の fastlio_mapping を Door/real/fastlio_cfg_1110/
  fastlio_g1_vox025.yaml で起こしておく。出力は /Odometry_loc＝DDS の rt/Odometry_loc）。⚠️ 機体ではまだ試していない
記録で（Mac・OMEN）: --replay-lidar <ライダーのセッション> --replay-fastlio <FAST-LIO の再生の tf_bag> --ref-wall <PC2 の時刻>
  --ref-pose x y yaw --out <npz>（FAST-LIO の出力は走査ごとに、その回の点群の後に流す）
"""
from __future__ import annotations

import argparse
import math
import queue
import signal
import socket
import sqlite3
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Deque, Iterator, List, Optional, Sequence, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_lidar_msg as msg  # noqa: E402
import lidar_door_est as est  # noqa: E402

REF_SKIP_S = 1.0           # door_lidar_node・R0 と同じ（STAND に入った直後は姿勢が動いている）
STILL_SCANS = 25           # 床を当てる回（10 Hz で 2.5 秒）
SCAN_PERIOD_S = 0.1        # MID-360 の 1 回
SCAN_DELAY_S = 0.055       # 受け取り − 走査の真ん中（door_lidar_node と同じ）
MATCH_MAX_S = 0.02         # FAST-LIO の出力とその回のヘッダ（＋1 回）の差の上限（L1）
ANCHOR_MIN_POSES = 10      # 段 0 の回に当たった FAST-LIO の出力の最小（L1）
ANCHOR_WAIT_S = 1.0        # 段 0 の最後の回からこれだけ待っても揃わなければ失敗（REF をもう一度）
REF_TOTAL_S = REF_SKIP_S + STILL_SCANS * 0.1 + 2.5  # REF からこれだけで段 0 がそろわなければ失敗（点群が来ない時も）
PENDING_KEEP = 8           # 点群より先に届いた FAST-LIO の出力を待たせる数
ANCHOR_SHAKE_M = 0.03      # 段 0 の間の FAST-LIO の姿勢の揺れの上限（止まっていない・L1 は 0.4 cm）
LIDAR_H_RANGE_M = (1.0, 1.4)  # 床からのライダーの高さ（L1 は 1.209 m）。外なら床を取り違えた
JUMP_MPS = 1.5             # 追っている間の跳びの線（純正の歩行は 0.5 m/s まで）
JUMP_RAD = 0.5             # 1 回（0.1 s）の向きの跳び
JUMP_RISE_M = 0.3          # 段 0 からのライダーの上下（歩いても数 cm）
SCAN_KEEP = 64             # 当てるために覚えておく点群の回
ODOM_TOPIC = "rt/Odometry_loc"
CLOUD_TOPIC = "rt/utlidar/cloud_livox_mid360"
Pose = Tuple[float, float, float]


# ── L1 で確かめた計算（l1_fastlio_carry_1110.py と同じ式） ─────────────────
def mean_pose(Ts: np.ndarray) -> np.ndarray:
    """止まっている間の姿勢の平均（位置は平均・回転は SVD で直交に戻した平均）"""
    u, _, vt = np.linalg.svd(Ts[:, :3, :3].mean(0))
    out = np.eye(4)
    out[:3, :3] = u @ np.diag([1.0, 1.0, np.linalg.det(u @ vt)]) @ vt
    out[:3, 3] = Ts[:, :3, 3].mean(0)
    return out


def planar_track(Ts: np.ndarray, T_anchor: np.ndarray, level: est.Level) -> np.ndarray:
    """段 0 の水平の座標（段 0 のライダーの真下が原点・段 0 のライダーの向きが +x）で見たライダーの水平の位置と向き [n, 3]。
    世界（重力で水平）の座標 W = Level.R · (段 0 のライダーの座標)。時刻 t のライダーの姿勢は W で R_W = Level.R · M_R、
    推定器の水平の座標は「傾きだけを外した」Ry·Rx·FLIP なので、向きは R_W · FLIPᵀ の ZYX の向き（Rz を取り出す）"""
    M = np.linalg.inv(T_anchor) @ Ts
    Rw = level.R @ M[:, :3, :3]
    A = Rw @ est.FLIP.T
    yaw = np.arctan2(A[:, 1, 0], A[:, 0, 0])
    p = M[:, :3, 3] @ level.R.T
    return np.c_[p[:, 0], p[:, 1], yaw]


def compose(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """平面の姿勢 a（[3]）の上に、a の座標で見た動き b（[n,3]）を足す"""
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.c_[a[0] + c * b[:, 0] - s * b[:, 1], a[1] + s * b[:, 0] + c * b[:, 1], a[2] + b[:, 2]]


def quat_to_mat(q: Sequence[float]) -> np.ndarray:
    """四元数 (x, y, z, w) → 回転行列"""
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose_matrix(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quat_to_mat((qx, qy, qz, qw))
    T[:3, 3] = (x, y, z)
    return T


# ── 本体（点群・FAST-LIO の出どころとは切り離してある） ───────────────────
class FastlioCarry:
    """REF の後に段 0 の床と姿勢を作り、FAST-LIO の出力ごとに扉の座標でのライダーの姿勢の電文を返す。
    on_scan・on_odom はどちらも送る電文の列を返す（出力が点群より先に届いたら待たせ、その点群が来た時に当て直すので、
    on_scan からも電文が出る）。追っている間に姿勢が跳んだ（FAST-LIO の再起動・発散）・有限でない時は、追うのをやめて
    電文を止める（受け口は 0.5 s 届かなければ立ったまま止まる）。もう一度 REF が要る"""

    def __init__(self) -> None:
        self.scans: Deque[Tuple[float, float]] = deque(maxlen=SCAN_KEEP)  # (ヘッダ, 受け取り)
        self.pending: Deque[Tuple[float, np.ndarray]] = deque(maxlen=PENDING_KEEP)  # 点群より先に届いた出力
        self.last_hdr: Optional[float] = None
        self.ref_t: Optional[float] = None
        self.ref_pose: Pose = (0.0, 0.0, 0.0)
        self.anchor_xyz: List[np.ndarray] = []
        self.anchor_hdr: List[float] = []
        self.anchor_recv_last: Optional[float] = None
        self.anchor_T: List[np.ndarray] = []
        self.level: Optional[est.Level] = None
        self.T_a: Optional[np.ndarray] = None
        self.last_sent: Optional[Tuple[float, np.ndarray]] = None
        self.seq = 0
        self.last_error = ""
        self.ref_log: List[Tuple[float, float, float, float]] = []
        self.stats = {"scans": 0, "scan_dup": 0, "odom": 0, "bad": 0, "late_match": 0, "unmatched": 0, "duplicate": 0,
                      "sent": 0, "fail": 0}

    @property
    def tracking(self) -> bool:
        return self.T_a is not None

    def request_ref(self, t_wall: float, pose: Pose = (0.0, 0.0, 0.0)) -> None:
        """REF（t_wall は受け取った PC2 の time.time()）。前の段 0・追っていた値は捨てて作り直す"""
        self._reset()
        self.ref_t, self.ref_pose = t_wall, tuple(float(v) for v in pose)  # type: ignore[assignment]
        self.ref_log.append((t_wall, *self.ref_pose))
        print(f"[fastlio_node] REF（扉の座標でのライダー x {pose[0]:+.3f} y {pose[1]:+.3f} 向き {math.degrees(pose[2]):+.1f}°）: "
              f"{REF_SKIP_S:.0f} 秒待ってから {STILL_SCANS} 回で床と段 0 の姿勢を作る（止まって）", flush=True)

    def _reset(self) -> None:
        self.ref_t = None
        self.anchor_xyz, self.anchor_hdr, self.anchor_T, self.anchor_recv_last = [], [], [], None
        self.level = self.T_a = self.last_sent = None

    def _fail(self, why: str) -> None:
        self.last_error = why
        self.stats["fail"] += 1
        self._reset()
        print(f"[fastlio_node] ⚠️ {why}（電文を止めた。REF をもう一度）", flush=True)

    def wants_points(self, recv: float) -> bool:
        """この回の点群（xyz）が要るか。要らない回は点群を読まない（PC2 の CPU を使わない）"""
        return (self.ref_t is not None and not self.tracking and recv - self.ref_t >= REF_SKIP_S
                and len(self.anchor_xyz) < STILL_SCANS)

    def on_scan(self, recv: float, hdr: float, xyz: Optional[np.ndarray]) -> List[msg.LidarMsg]:
        self.stats["scans"] += 1
        if self.scans and hdr <= self.scans[-1][0]:
            self.stats["scan_dup"] += 1  # 同じ回が二度・古い回が遅れて届いた
            return []
        want = self.wants_points(recv)
        self.scans.append((hdr, recv))
        if xyz is not None and want:
            self.anchor_xyz.append(xyz)
            self.anchor_hdr.append(hdr)
            self.anchor_recv_last = recv
        out = self._retry_pending(hdr)
        if self.ref_t is not None and not self.tracking:
            if len(self.anchor_xyz) == STILL_SCANS and recv - self.anchor_recv_last > ANCHOR_WAIT_S:
                self._fail(f"段 0 の回に当たった FAST-LIO の出力が {len(self.anchor_T)}（{ANCHOR_MIN_POSES} 要る）。FAST-LIO が起きているか")
            elif recv - self.ref_t > REF_TOTAL_S:
                self._fail(f"REF から {REF_TOTAL_S:.0f} 秒で段 0 がそろわない（点群 {len(self.anchor_xyz)}/{STILL_SCANS}・"
                           f"FAST-LIO の出力 {len(self.anchor_T)}）")
        return out

    def _match(self, stamp: float) -> Optional[Tuple[float, float]]:
        if not self.scans:
            return None
        key = stamp - SCAN_PERIOD_S
        hdr, recv = min(self.scans, key=lambda s: abs(s[0] - key))
        return (hdr, recv) if abs(hdr - key) <= MATCH_MAX_S else None

    def _retry_pending(self, newest_hdr: float) -> List[msg.LidarMsg]:
        out: List[msg.LidarMsg] = []
        keep: Deque[Tuple[float, np.ndarray]] = deque(maxlen=PENDING_KEEP)
        for stamp, T in self.pending:
            hit = self._match(stamp)
            if hit is not None:
                self.stats["late_match"] += 1
                out += self._use(T, hit)
            elif stamp - SCAN_PERIOD_S < newest_hdr - MATCH_MAX_S:
                self.stats["unmatched"] += 1  # その回の点群はもう来ているはず
            else:
                keep.append((stamp, T))
        self.pending = keep
        return out

    def on_odom(self, stamp: float, T: np.ndarray) -> List[msg.LidarMsg]:
        """FAST-LIO の出力（ヘッダの時刻はライダーの時計・camera_init → body）"""
        self.stats["odom"] += 1
        T = np.asarray(T, dtype=float)
        if T.shape != (4, 4) or not np.all(np.isfinite(T)) or not math.isfinite(stamp):
            self.stats["bad"] += 1
            return []
        hit = self._match(stamp)
        if hit is None:
            newest = self.scans[-1][0] if self.scans else -math.inf
            if stamp - SCAN_PERIOD_S > newest + MATCH_MAX_S:
                self.pending.append((stamp, T))  # その回の点群がまだ届いていない
            else:
                self.stats["unmatched"] += 1
            return []
        return self._use(T, hit)

    def _finalize(self) -> None:
        if len(self.anchor_T) < ANCHOR_MIN_POSES:
            self._fail(f"段 0 の回に当たった FAST-LIO の出力が {len(self.anchor_T)}（{ANCHOR_MIN_POSES} 要る）")
            return
        try:
            level = est.level_from_scans(self.anchor_xyz)
            T_a = mean_pose(np.array(self.anchor_T))
            still = planar_track(np.array(self.anchor_T), T_a, level)
        except Exception as e:  # noqa: BLE001  numpy の LinAlgError も。落ちずに REF を待つ
            self._fail(f"段 0 の床か姿勢が作れない: {type(e).__name__}: {e}")
            return
        shake = float(np.hypot(still[:, 0], still[:, 1]).max())
        print(f"[fastlio_node] 段 0: 床の傾き {level.tilt_deg():.1f}°・高さ {level.h:.3f} m・"
              f"FAST-LIO の出力 {len(self.anchor_T)}/{STILL_SCANS}・止まった間の揺れ {100 * shake:.1f} cm", flush=True)
        if shake > ANCHOR_SHAKE_M:
            self._fail(f"段 0 の間に FAST-LIO の姿勢が {100 * shake:.1f} cm 動いた（{100 * ANCHOR_SHAKE_M:.0f} cm まで。止まっていない）")
            return
        if not (LIDAR_H_RANGE_M[0] <= level.h <= LIDAR_H_RANGE_M[1]):
            self._fail(f"床からのライダーの高さ {level.h:.3f} m が {LIDAR_H_RANGE_M} の外（床を取り違えた疑い）")
            return
        self.level, self.T_a = level, T_a

    def _use(self, T: np.ndarray, hit: Tuple[float, float]) -> List[msg.LidarMsg]:
        hdr, recv = hit
        if self.last_hdr is not None and hdr <= self.last_hdr:
            self.stats["duplicate"] += 1  # 同じ回の 2 つ目・古い回の出力
            return []
        self.last_hdr = hdr
        if self.ref_t is None:
            return []
        if not self.tracking:
            if hdr in self.anchor_hdr:
                self.anchor_T.append(T)
            if len(self.anchor_hdr) == STILL_SCANS and hdr >= self.anchor_hdr[-1]:
                self._finalize()
            if not self.tracking:
                return []
        M = np.linalg.inv(self.T_a) @ T
        rise = float((M[:3, 3] @ self.level.R.T)[2])
        p = compose(np.array(self.ref_pose), planar_track(T[None], self.T_a, self.level))[0]
        t = recv - SCAN_DELAY_S + SCAN_PERIOD_S / 2
        why = self._jump(t, p, rise)
        if why:
            self._fail(why)
            return []
        self.last_sent = (t, p)
        self.seq += 1
        self.stats["sent"] += 1
        return [msg.LidarMsg(self.seq, t, float(p[0]), float(p[1]), math.atan2(math.sin(p[2]), math.cos(p[2])), float("nan"), 0,
                             msg.STATE_OK, 0.0, float(self.level.tilt_deg()))]

    def _jump(self, t: float, p: np.ndarray, rise: float) -> str:
        """FAST-LIO の再起動・発散の見張り（空なら無事）"""
        if abs(rise) > JUMP_RISE_M:
            return f"FAST-LIO のライダーの高さが段 0 から {rise:+.2f} m 変わった（再起動か発散の疑い）"
        if self.last_sent is None:
            return ""
        t0, p0 = self.last_sent
        dt = max(t - t0, SCAN_PERIOD_S / 2)
        speed = float(np.hypot(*(p[:2] - p0[:2]))) / dt
        dyaw = abs(math.atan2(math.sin(p[2] - p0[2]), math.cos(p[2] - p0[2])))
        if speed > JUMP_MPS or dyaw > JUMP_RAD:
            return f"FAST-LIO の姿勢が跳んだ（{speed:.1f} m/s・{math.degrees(dyaw):.0f}°）。再起動か発散の疑い"
        return ""


# ── 記録を流す（Mac・OMEN） ───────────────────────────────────────
def _odom_rows(bag_dir: Path) -> List[Tuple[float, np.ndarray]]:
    from dog_odom_bag import parse_odom_cdr
    rows = []
    for db in sorted(bag_dir.glob("*.db3")):
        con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        try:
            tid = con.execute("SELECT id FROM topics WHERE name = ?", ("/Odometry_loc",)).fetchone()
            if tid is None:
                raise SystemExit(f"[fastlio_node] {db} に /Odometry_loc が無い")
            for _ts, data in con.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", tid):
                sec, nsec, *pose = parse_odom_cdr(data)
                rows.append((sec + nsec / 1e9, pose_matrix(*pose)))
        finally:
            con.close()
    return sorted(rows, key=lambda r: r[0])


def replay_events(lidar_session: Path, fastlio_bag: Path) -> Iterator[Tuple[str, float, float, object]]:
    """("scan", 受け取り, ヘッダ, 点群を読む関数) と ("odom", ヘッダ, 0, 4×4) を、ライダーの時計の順に。
    FAST-LIO の出力はその回の点群の後（ヘッダ＋1 回＋30 ms）に置く"""
    import real_truth_1110 as rt
    db = next((lidar_session / "raw" / "rosbag2").glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    try:
        tid = con.execute("SELECT id FROM topics WHERE name = ?", (rt.LIDAR_TOPIC,)).fetchone()[0]
        scans = con.execute("SELECT id, timestamp, substr(data, 5, 8) FROM messages WHERE topic_id = ? ORDER BY timestamp",
                            (tid,)).fetchall()
    finally:
        con.close()
    events: List[Tuple[float, str, float, float, object]] = []
    for mid, ts, h in scans:
        hdr = int.from_bytes(h[:4], "little", signed=True) + int.from_bytes(h[4:], "little") / 1e9
        events.append((hdr + SCAN_PERIOD_S, "scan", ts / 1e9, hdr, mid))
    for stamp, T in _odom_rows(fastlio_bag):
        events.append((stamp + 0.03, "odom", stamp, 0.0, T))
    events.sort(key=lambda e: e[0])

    def loader(mid: int):
        def load() -> np.ndarray:
            c = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
            try:
                data = c.execute("SELECT data FROM messages WHERE id = ?", (mid,)).fetchone()[0]
            finally:
                c.close()
            return rt.scan_xyz(data)
        return load
    for _, kind, a, b, payload in events:
        yield (kind, a, b, loader(payload) if kind == "scan" else payload)


def run_replay(node: FastlioCarry, events, ref_wall: float, ref_pose: Pose) -> List[msg.LidarMsg]:
    out: List[msg.LidarMsg] = []
    ref_done = False
    for kind, a, b, payload in events:
        if kind == "scan":
            if not ref_done and a >= ref_wall:
                node.request_ref(ref_wall, ref_pose)
                ref_done = True
            out += node.on_scan(a, b, payload() if node.wants_points(a) else None)
        else:
            out += node.on_odom(a, payload)
    return out


# ── 機体（PC2） ───────────────────────────────────────────────
WARN_EVERY_S = 5.0  # 同じ警告を出す間隔（遅れている時にログがあふれないように）


def open_ref_socket(port: int) -> socket.socket:
    """REF の口（127.0.0.1:port）を主の糸で開く。使われていたら起動を止める（自前の推定器 door_lidar_node と同じ口）"""
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        rx.bind(("127.0.0.1", port))
    except OSError as e:
        rx.close()
        raise SystemExit(f"[fastlio_node] REF の口 127.0.0.1:{port} が開けない（{e}）。door_lidar_node.py が動いていないか") from e
    rx.settimeout(0.2)
    return rx


def listen_ref(rx: socket.socket, inbox: "queue.Queue", stop: threading.Event) -> None:
    while not stop.is_set():
        try:
            data, _ = rx.recvfrom(128)
        except socket.timeout:
            continue
        except OSError:
            return
        pose = msg.parse_ref(data)
        if pose is None:
            print(f"[fastlio_node] ⚠️ 知らない電文を捨てた: {data[:40]!r}", flush=True)
            continue
        try:
            inbox.put(("ref", time.time(), 0.0, pose), timeout=1.0)  # REF は捨てない（1 秒待つ）
        except queue.Full:
            print("[fastlio_node] ⚠️ 箱があふれていて REF を入れられない（処理が止まっている）", flush=True)


def run_live(node: FastlioCarry, iface: str, host: str, port: int, stop: threading.Event, sent: List[msg.LidarMsg]) -> None:
    """DDS の糸は受け取って箱に入れるだけ。処理は 1 本の糸で順に（点群は段 0 の回だけ読む）。1 件の処理の例外は数えて続ける"""
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
    import door_lidar_node as dln

    rx = open_ref_socket(port + 1)
    inbox: "queue.Queue" = queue.Queue(maxsize=256)
    drops = {"n": 0, "t": 0.0}

    def put(item) -> None:
        try:
            inbox.put_nowait(item)
        except queue.Full:
            drops["n"] += 1
            if time.time() - drops["t"] > WARN_EVERY_S:
                print(f"[fastlio_node] ⚠️ 箱があふれた（処理が追いつかない・捨てた数 {drops['n']}）", flush=True)
                drops["t"] = time.time()

    def on_cloud(m) -> None:
        put(("scan", time.time(), m.header.stamp.sec + m.header.stamp.nanosec / 1e9, m))

    def on_odom(m) -> None:
        p, q = m.pose.pose.position, m.pose.pose.orientation
        put(("odom", m.header.stamp.sec + m.header.stamp.nanosec / 1e9, 0.0, pose_matrix(p.x, p.y, p.z, q.x, q.y, q.z, q.w)))

    threading.Thread(target=listen_ref, args=(rx, inbox, stop), daemon=True).start()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ChannelFactoryInitialize(0, iface)
    subs = [ChannelSubscriber(CLOUD_TOPIC, PointCloud2_), ChannelSubscriber(ODOM_TOPIC, Odometry_)]
    subs[0].Init(on_cloud, 10)
    subs[1].Init(on_odom, 10)
    print(f"[fastlio_node] {CLOUD_TOPIC}・{ODOM_TOPIC} を待つ。電文は {host}:{port} へ・REF は 127.0.0.1:{port + 1} で受ける", flush=True)
    t_report, errors, t_err = time.time(), 0, 0.0
    try:
        while not stop.is_set():
            if time.time() - t_report > 10.0:
                print(f"[fastlio_node] {node.stats}・例外 {errors}・{'送っている' if node.tracking else ('段 0 を作っている' if node.ref_t else 'REF 待ち')}",
                      flush=True)
                t_report = time.time()
            try:
                kind, a, b, payload = inbox.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                if kind == "ref":
                    out = []
                    node.request_ref(a, payload)
                elif kind == "scan":
                    out = node.on_scan(a, b, dln.cloud_xyz(payload) if node.wants_points(a) else None)
                else:
                    out = node.on_odom(a, payload)
            except Exception as e:  # noqa: BLE001  1 つの壊れた点群・出力で口ごと落とさない
                errors += 1
                if time.time() - t_err > WARN_EVERY_S:
                    print(f"[fastlio_node] ⚠️ {kind} の処理で例外（{errors} 件目）: {type(e).__name__}: {e}", flush=True)
                    t_err = time.time()
                continue
            for m in out:
                tx.sendto(msg.pack(m), (host, port))
                sent.append(m)
    finally:
        rx.close()


def save(node: FastlioCarry, out: List[msg.LidarMsg], path: Path) -> None:
    rows = np.array([[m.stamp, m.x, m.y, m.yaw, m.tilt_deg] for m in out]) if out else np.zeros((0, 5))
    np.savez_compressed(path, rows=rows, cols=np.array(["stamp", "x", "y", "yaw", "tilt_deg"]),
                        ref_log=np.array(node.ref_log) if node.ref_log else np.zeros((0, 4)), stats=str(node.stats))
    print(f"[fastlio_node] 書いた: {path}（電文 {len(out)}・{node.stats}）", flush=True)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=msg.PORT, help="電文を送る先（走らせ役）。REF は port+1 で受ける")
    ap.add_argument("--replay-lidar", type=Path, help="記録: ライダーのセッション（生の点群）")
    ap.add_argument("--replay-fastlio", type=Path, help="記録: FAST-LIO の再生の tf_bag（/Odometry_loc）")
    ap.add_argument("--ref-wall", type=float, help="記録: REF を出す PC2 の時刻（壁）")
    ap.add_argument("--ref-pose", type=float, nargs=3, default=[0.0, 0.0, 0.0], help="記録: REF の扉の座標でのライダーの姿勢")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(argv)
    node = FastlioCarry()
    if a.replay_lidar:
        if a.replay_fastlio is None or a.ref_wall is None:
            raise SystemExit("[fastlio_node] --replay-lidar には --replay-fastlio と --ref-wall が要る")
        t0 = time.time()
        out = run_replay(node, replay_events(a.replay_lidar, a.replay_fastlio), a.ref_wall, tuple(a.ref_pose))
        print(f"[fastlio_node] 再生: {time.time() - t0:.1f} 秒", flush=True)
        if a.out:
            save(node, out, a.out)
        return 0
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    sent: List[msg.LidarMsg] = []
    try:
        run_live(node, a.iface, a.host, a.port, stop, sent)
    finally:
        if a.out:
            a.out.parent.mkdir(parents=True, exist_ok=True)
            save(node, sent, a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
