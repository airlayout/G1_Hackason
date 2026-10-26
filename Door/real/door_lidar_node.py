"""段 5・L1（2026-10-23）: 頭のライダーで扉の座標でのライダーの姿勢と扉の角度を出し、UDP で door_run_real.py へ渡す（別のプロセス）。

PC2 で: python3 -u door_lidar_node.py --thickness 0.04 [--lever-right]
  1. rt/utlidar/cloud_livox_mid360 を受ける（いちばん新しい回だけを処理する。遅れをためない）
  2. 制御から「REF」（STAND に入った時に送られる）を受けたら、次の STILL_SCANS 回（10 Hz で 2.5 秒）で扉と地図を作る
     （機体は扉の正面で HOMIE で立っている・扉は閉じている前提）。作り直しは REF をもう一度
  3. 以後、1 回ごとに lidar_door_est.DoorTracker で重ね直し、door_lidar_msg の電文を送る
  rt/lowstate・rt/dog_odom は購読しない（1 kHz の受け取りで Python が詰まるのを避ける。pelvis への直しは制御の側）

Mac で記録を流す（試し・L2）: Navigation/.venv/bin/python Door/real/door_lidar_node.py --replay <ライダーのセッション> [--ref-at 0]
  記録の受け取った時刻の間隔のまま流し、同じ電文を UDP で送り、--out に全部の推定を npz で残す

止め方: Ctrl+C か SIGTERM（制御は 0.5 s 届かなければ立ったまま止まる）
"""
from __future__ import annotations

import argparse
import math
import queue
import signal
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Iterator, List, Optional, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_lidar_msg as msg  # noqa: E402
import lidar_door_est as est  # noqa: E402

STILL_SCANS = 25
REF_SKIP_S = 1.0  # REF の後、この間の回は捨てる（STAND に入った直後は既定の姿勢から HOMIE の姿勢へ動いている）
# 点群は 0.1 s かけて集める。受け取った時刻はヘッダの時刻（集め始め）より 102〜108 ms 遅い（10-21 の記録）。
# 重ね直しの姿勢は集める間の真ん中あたりのものなので、受け取った時刻から 55 ms 引いた時刻を電文に付ける
SCAN_DELAY_S = 0.055
CLOUD_TOPIC = "rt/utlidar/cloud_livox_mid360"


def cloud_xyz(m) -> np.ndarray:
    """unitree_sdk2py の PointCloud2_ から x・y・z（float32 の欄）を (n, 3) で"""
    off = {f.name: int(f.offset) for f in m.fields}
    step = int(m.point_step)
    data = m.data if isinstance(m.data, (bytes, bytearray)) else bytes(m.data)
    n = len(data) // step
    raw = np.frombuffer(data, dtype=np.uint8, count=n * step).reshape(n, step)
    cols = [raw[:, off[k]:off[k] + 4].copy().view("<f4")[:, 0] for k in ("x", "y", "z")]
    xyz = np.stack(cols, axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(1)]


class Node:
    """REF を受けて扉と地図を作り、1 回ごとに推定して送る。点群の出どころ（機体・記録）とは切り離してある"""

    def __init__(self, thickness: float, lever_side: int, host: str, port: int) -> None:
        self.thickness = thickness
        self.lever_side = lever_side
        self.addr = (host, port)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tracker: Optional[est.DoorTracker] = None
        self.collect: Optional[List[np.ndarray]] = None
        self.ref_t: Optional[float] = None
        self.seq = 0
        self.rows: List[Tuple[float, ...]] = []

    def request_ref(self) -> None:
        print(f"[lidar_node] REF を受けた: {REF_SKIP_S:.0f} 秒待ってから 2.5 秒で扉と地図を作る（扉の正面で止まって・扉は閉じて）", flush=True)
        self.tracker = None
        self.ref_t = None  # 次の回の時刻から数える（記録を早送りで流しても同じ秒数を捨てる）
        self.collect = []

    def _build(self, scans: List[np.ndarray]) -> None:
        t0 = time.perf_counter()
        try:
            tracker = est.DoorTracker.from_still(scans, self.thickness, self.lever_side)
        except Exception as e:  # noqa: BLE001  numpy の LinAlgError も（ValueError ではない）。落ちずに REF を待つ
            print(f"[lidar_node] ⚠️ 扉が測れない: {type(e).__name__}: {e}（REF をもう一度）", flush=True)
            return
        d = tracker.door
        p = d.lidar_in_door
        print(f"[lidar_node] 扉と地図を作った（{(time.perf_counter() - t0) * 1000:.0f} ms）: 幅 {d.width:.3f} m・"
              f"ライダー x {p.x:+.3f} y {p.y:+.3f} 向き {math.degrees(p.yaw):+.1f}°・地図 {len(tracker.map.points)} 点・"
              f"頭の傾き {tracker.level.tilt_deg():.1f}°・床からの高さ {tracker.level.h:.3f} m・板の傾き {d.info['slab_tilt_deg']:+.1f}°", flush=True)
        self.tracker = tracker

    def on_scan(self, stamp: float, xyz: np.ndarray) -> None:
        collect = self.collect
        if collect is not None:
            if self.ref_t is None:
                self.ref_t = stamp
            if stamp - self.ref_t < REF_SKIP_S:
                return
            collect.append(xyz)
            if len(collect) >= STILL_SCANS:
                self.collect = None
                self._build(collect)
            return
        tracker = self.tracker  # REF の糸が途中で None にしても、この回は取った方で最後まで計算する
        if tracker is None:
            return
        t0 = time.perf_counter()
        try:
            e = tracker.step(stamp, xyz)
        except Exception as ex:  # noqa: BLE001  1 回の例外で落ちない。電文は送らない（制御は 0.5 s で「届かない」と止まる）
            print(f"[lidar_node] ⚠️ 推定で例外: {type(ex).__name__}: {ex}", flush=True)
            return
        state = (msg.STATE_OK if e.accepted else 0) | (msg.STATE_ANGLE_OK if math.isfinite(e.angle_deg) else 0)
        m = msg.LidarMsg(self.seq, stamp, e.lidar.x, e.lidar.y, e.lidar.yaw, math.radians(e.angle_deg),
                         e.angle_points, state, e.match.rms_m if math.isfinite(e.match.rms_m) else -1.0, e.tilt_deg)
        self.tx.sendto(msg.pack(m), self.addr)
        self.seq += 1
        cost = time.perf_counter() - t0
        self.rows.append((stamp, e.lidar.x, e.lidar.y, e.lidar.yaw, e.angle_deg, float(e.accepted), float(e.lost),
                          e.match.matched, e.match.rms_m, e.tilt_deg, cost))
        if e.lost and self.seq % 10 == 0:
            print("[lidar_node] ⚠️ 見失った（1 秒続けて重ね直せない）", flush=True)

    def save(self, path: Path) -> None:
        if self.rows:
            cols = ("stamp", "x", "y", "yaw", "angle_deg", "accepted", "lost", "matched", "rms", "tilt_deg", "cost_s")
            r = np.array(self.rows, dtype=float)
            np.savez_compressed(path, **{c: r[:, i] for i, c in enumerate(cols)})
            print(f"[lidar_node] 記録を書いた: {path}（{len(r)} 回・計算 中央 {np.median(r[:, -1]) * 1000:.1f} ms・"
                  f"99% {np.percentile(r[:, -1], 99) * 1000:.1f} ms）", flush=True)


def listen_ref(node: Node, port: int, stop: threading.Event) -> None:
    """制御（door_run_real.py）からの REF を受ける"""
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", port))
    rx.settimeout(0.2)
    while not stop.is_set():
        try:
            data, _ = rx.recvfrom(64)
        except socket.timeout:
            continue
        if data.strip() == b"REF":
            node.request_ref()


def replay_scans(session: Path, speed: float) -> Iterator[Tuple[float, np.ndarray]]:
    """記録の回を、受け取った時刻の間隔の speed 倍の速さで返す。電文の時刻は記録の間隔のまま（早送りでも歩く速さが変わらない）。
    ⚠️ speed ≠ 1 だと電文の時刻が今からずれるので、door_run_real.py（電文の年齢を見る）と組むときは speed 1 で流す"""
    from crop_lidar_bag import iter_lidar, scan_xyz  # Mac だけ（g1_mapping を使う）

    t_rec0 = t_wall0 = None
    for recv_ns, _stamp, payload in iter_lidar(session):
        t_rec = recv_ns / 1e9
        if t_rec0 is None:
            t_rec0, t_wall0 = t_rec, time.time()
        wait = t_wall0 + (t_rec - t_rec0) / speed - time.time()
        if wait > 0:
            time.sleep(wait)
        yield t_wall0 + (t_rec - t_rec0) - SCAN_DELAY_S, scan_xyz(payload)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--thickness", type=float, required=True, help="[m] 扉の板の厚み（巻き尺）。枠の原点は厚みの真ん中")
    ap.add_argument("--lever-right", action="store_true", help="把手が右の扉（今回の扉は左なので付けない）")
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=msg.PORT, help="推定を送る先（door_run_real.py）。REF は port+1 で受ける")
    ap.add_argument("--replay", type=Path, help="Mac で記録を流す（ライダーのセッション）")
    ap.add_argument("--ref-at", type=float, default=0.0, help="--replay で REF を出す記録の始めからの秒")
    ap.add_argument("--speed", type=float, default=1.0, help="--replay の速さ")
    ap.add_argument("--out", type=Path, default=Path("logs") / f"lidar_node_{time.strftime('%Y%m%d_%H%M%S')}.npz")
    a = ap.parse_args()

    node = Node(a.thickness, -1 if a.lever_right else +1, a.host, a.port)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    a.out.parent.mkdir(parents=True, exist_ok=True)
    try:
        if a.replay:
            t_start, refd = None, False
            for stamp, xyz in replay_scans(a.replay, a.speed):
                if stop.is_set():
                    break
                t_start = stamp if t_start is None else t_start
                if node.collect is None and node.tracker is None and stamp - t_start >= a.ref_at and not refd:
                    refd = True
                    node.request_ref()
                node.on_scan(stamp, xyz)
            return 0
        threading.Thread(target=listen_ref, args=(node, a.port + 1, stop), daemon=True).start()
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

        latest: "queue.Queue[Tuple[float, object]]" = queue.Queue(maxsize=1)

        def on_cloud(m) -> None:  # DDS の糸では受け取るだけ。古い回は捨てていちばん新しい回だけ残す
            item = (time.time() - SCAN_DELAY_S, m)
            try:
                latest.put_nowait(item)
            except queue.Full:
                try:
                    latest.get_nowait()
                except queue.Empty:
                    pass
                latest.put_nowait(item)

        ChannelFactoryInitialize(0, a.iface)
        sub = ChannelSubscriber(CLOUD_TOPIC, PointCloud2_)
        sub.Init(on_cloud, 1)
        print(f"[lidar_node] {CLOUD_TOPIC} を待つ。推定は {a.host}:{a.port} へ・REF は 127.0.0.1:{a.port + 1} で受ける", flush=True)
        n_rx, t_report = 0, time.time()
        while not stop.is_set():
            try:
                stamp, m = latest.get(timeout=0.5)
            except queue.Empty:
                if time.time() - t_report > 5.0:
                    print("[lidar_node] ⚠️ 点群が 0.5 s 来ない", flush=True)
                    t_report = time.time()
                continue
            node.on_scan(stamp, cloud_xyz(m))
            n_rx += 1
            if time.time() - t_report > 10.0:
                print(f"[lidar_node] 処理した回 {n_rx}（10 秒）・状態 {'推定中' if node.tracker else ('扉を測っている' if node.collect is not None else 'REF 待ち')}", flush=True)
                n_rx, t_report = 0, time.time()
        return 0
    finally:
        node.save(a.out)


if __name__ == "__main__":
    sys.exit(main())
