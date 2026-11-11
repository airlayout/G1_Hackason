"""11-10 手順 R0: 実物の記録の画像 1 枚ごとに、頭のライダーの扉の測定から「扉の値 17」の正解を付ける（計画書 8.7 節）。

画像の網（カラー＋深度 → 扉の値 17）を実物の画で評価し学び直すための正解。ライダーは方策に入れない（答え合わせの計器としてだけ使う）。

流れ:
  1. 時計: 画像（vision_*.npz の t_capture）と走らせ役（door_stock_*.npz の t）は PC2 の monotonic、ライダーと /dog_odom の記録（db3）は
     PC2 の壁の時計（record_dds_to_bag.py の time.time_ns()）。両方にある /dog_odom の軌跡を突き合わせて差（壁 − monotonic）を出す
  2. 扉と地図: 巻き尺の地点で止まっている間の 25 回（lidar_door_est.DoorTracker.from_still、実機の REF と同じ）
  3. 追う: そこから時刻の先へ 1 回ずつ重ね直し、時刻の前へは時刻を反転して同じように追う（等速の予測は向きを問わない）
  4. pelvis へ: 実機と同じ door_lidar_msg.lidar_to_pelvis（腰の 3 関節・pelvis の IMU・取付の補正 head_xy）
     取付は巻き尺の 1 地点で合わせ（--fit-spot）、残りの地点で確かめる。--center を渡せばそれを使う（door_run_stock の lidar_center_xy）
  5. 画像の時刻へ: いちばん近い受け入れた回（0.3 s 以内）から、走らせ役の odom（/dog_odom）の差で運ぶ（door_lidar_source の track と同じ）
     扉から 0.7 m より近い回は推定器が横へ滑る（11-04）ので、最後の遠い回から odom で運んだ値も並べて出し（carry）、正解はそちらを使う
  6. 扉の値 17（実の向き）= [扉の角度, door_frame.door_frame_loc 9, door_frame.door_geometry 7] → 方策の鏡写し（obs_mirror_M）で網の側へ

巻き尺: 扉の板の廊下側の面から足先まで（toe）。pelvis = toe + 0.145 m（足のメッシュの前端。11-04 の 0.123 は当たりの球の中心）。扉の原点は板の厚みの真ん中なので、
pelvis から見た開口の中心の「前」= toe + 0.145 + 厚み/2。「左」は開口の中心が pelvis の左に何 m か。

使い方（OMEN の real_ana_1106/・doorman6 の Python。g1_mapping の rebuild.py を ~/work_inoue/Mapping/real/python/g1_mapping/ に置く）:
  python real_truth_1110.py --vision <vision_*.npz> --stock <door_stock_*.npz> --lidar <*_li> --odom <*_odom> \
      --policy door_vision_e2b.npz --door 0.94,2.05,0.99,0.064 --thickness 0.04 \
      --spot 1150:1235:0.760:0.080 --spot 1540:1610:0.210:0.070 --spot 1795:1830:1.370:0.025 --ref-spot 0 --fit-spot 0 --out truth.npz
  --spot は t_capture（monotonic 秒）の範囲・足先の距離 toe [m]・開口の中心の左 [m]
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import pickle
import re
import sqlite3
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Sequence, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import door_frame as dfr  # noqa: E402
import door_lidar_msg as dlm  # noqa: E402
import door_vision_core as dvc  # noqa: E402
import dog_odom_bag as dob  # noqa: E402
import lidar_door_est as est  # noqa: E402
from crop_lidar_bag import LIDAR_TOPIC, scan_xyz  # noqa: E402
from door_lidar_node import REF_SKIP_S, SCAN_DELAY_S, STILL_SCANS  # noqa: E402

# 純正で立った時の pelvis から足の先まで。11-04 の 0.123 は当たりの球（足首の前 0.12・半径 5 mm）の中心までだった。
# 巻き尺は足の本当の先まで測る。足のメッシュ（ankle_roll_link.STL）の前端は足首の前 0.142 m（11-10 に確かめた）→ 0.003 + 0.142
TOE_TO_PELVIS_M = 0.145
NEAR_M = 0.7  # ライダーから扉の面までがこれより近いと推定器が横へ滑る（11-04）
SCAN_MATCH_S = 0.3  # 画像の時刻からこれより離れた回しか無ければ正解を付けない
ANGLE_MATCH_S = 1.0
ANGLE_CLOSED_DEG = 3.0  # 扉に近くて角度が測れない間は、最後に測れた角度がこれ以下（閉じている）なら持ち越す
JUMP_FROM_OWN_M = 0.15  # 読んだ地図に重ね直した姿勢が、この回で測った扉の座標からこれ以上離れたら当たり違い
CARRY_MAX_PATH_M = 2.0  # odom で運ぶ道のりの上限（脚の odom は横に 7〜10%・前後に 4〜6% 短い）
CLOCK_SEARCH_S = 5.0  # 名前から出した時計の差の見込みの前後をこれだけ探す
MOVING_MPS = 0.05  # 時計を合わせるのに使う「動いている」odom の速さ
TURNING_RAD_S = 0.05  # 向きで時計を合わせるのに使う「回っている」odom の向きの速さ
SRC_NONE, SRC_LIDAR, SRC_CARRY = 0, 1, 2
WAIST = slice(12, 15)


@dataclass(frozen=True)
class Spot:
    t0: float  # t_capture（monotonic）
    t1: float
    toe: float  # 扉の板の面から足先まで [m]
    left: float  # 開口の中心が pelvis の左に [m]

    def pelvis_in_door(self, thickness: float) -> np.ndarray:
        """巻き尺から出す、扉の座標での pelvis の床の上の位置。巻き尺は扉の面に垂直（toe）・扉と平行（left）に測るので、
        機体が扉に対して回っていても扉の座標のまま比べる（11-10: 地点 2 は −4.5° 回っていて、pelvis の向きで比べると左右が 12 cm ずれた）。
        つま先 → pelvis の 0.123 m は機体の向きに沿うが、5° 回っても前後の差は 0.5 mm なので扉の垂直に足す"""
        return np.array([-(self.toe + TOE_TO_PELVIS_M + thickness / 2.0), -self.left])


@dataclass(frozen=True)
class Stock:
    t: np.ndarray  # monotonic
    q_waist: np.ndarray  # [n, 3]
    quat: np.ndarray  # [n, 4] pelvis の IMU（w, x, y, z）
    odom: np.ndarray  # [n, 3] /dog_odom の x・y・yaw（odom_relay 経由）


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vision", required=True)
    ap.add_argument("--stock", required=True)
    ap.add_argument("--lidar", required=True, type=Path, help="ライダーのセッション（*_li）")
    ap.add_argument("--odom", type=Path, help="/dog_odom のセッション（*_odom）。--clock odom のとき要る")
    ap.add_argument("--clock", choices=("odom", "lidar"), default="odom",
                    help="時計の合わせ方。odom: /dog_odom の 2 つの記録の位置／lidar: ライダーと odom の向きの変化（/dog_odom の記録が壊れているとき）")
    ap.add_argument("--policy", required=True, help="鏡写しの行列を読む方策の書き出し（door_vision_e2b.npz）")
    ap.add_argument("--door", default="0.94,2.05,0.99,0.064", help="幅・高さ・レバーの高さ・バックセット [m]（巻き尺）")
    ap.add_argument("--lever-right", action="store_true")
    ap.add_argument("--thickness", type=float, default=0.04)
    ap.add_argument("--spot", action="append", default=[], help="t0:t1:toe:left（巻き尺の地点）")
    ap.add_argument("--ref-spot", type=int, default=0, help="扉と地図を作る地点の番号（--spot の順）")
    ap.add_argument("--ref", default=None, help="扉と地図を作る t_capture の範囲 t0:t1（--ref-spot より優先）")
    ap.add_argument("--fit-spot", type=int, default=0, help="取付を合わせる地点の番号。-1 なら --center を使う")
    ap.add_argument("--center", default=None,
                    help="pelvis から見たライダーの水平の位置 前,左 [m]（--fit-spot -1 のとき）。fk なら補正なし（URDF の順運動学だけ）")
    ap.add_argument("--clock-guess", type=float, default=None, help="壁 − monotonic の見込み [s]（既定は記録の名前から）")
    ap.add_argument("--save-tracker", default=None, help="作った扉と地図を pickle で書く（同じ日の別の回で --tracker-from に渡す）")
    ap.add_argument("--tracker-from", default=None, help="別の回で作った扉と地図を読む（この回の --ref の範囲で床を当て直し、位置を探し直す）")
    ap.add_argument("--reuse", default=None, help="前の出力の npz のライダーの推定（lidar_rows）を使い、推定をやり直さない（同じ --ref のとき）")
    ap.add_argument("--out", required=True)
    return ap.parse_args(argv)


def load_stock(path: str) -> Stock:
    z = np.load(path, allow_pickle=True)
    return Stock(t=z["t"], q_waist=z["q"][:, WAIST], quat=z["quat"], odom=z["odom"])


def _name_time(name: str, pattern: str, fmt: str) -> float:
    m = re.search(pattern, name)
    if m is None:
        raise SystemExit(f"[truth] 名前 {name} から時刻が読めない（--clock-guess を渡す）")
    return datetime.strptime(m.group(0), fmt).timestamp()  # 同じ PC2 の時計の名前どうしの差だけ使うので、時間帯は何でもよい


def clock_guess(stock_path: str, stock_t0: float, odom_session: Path, odom_recv0: float) -> float:
    """記録の名前（PC2 の現地時刻・秒まで）から、壁 − monotonic の見込み"""
    t_stock = _name_time(Path(stock_path).name, r"\d{8}_\d{6}", "%Y%m%d_%H%M%S")
    t_odom = _name_time(odom_session.name, r"\d{8}T\d{6}", "%Y%m%dT%H%M%S")
    return odom_recv0 + (t_stock - t_odom) - stock_t0


def clock_offset(t_mono: np.ndarray, odom_mono: np.ndarray, t_wall: np.ndarray, odom_wall: np.ndarray,
                 guess: float, search_s: float = CLOCK_SEARCH_S) -> Tuple[float, float]:
    """壁 − monotonic [s] と、そのときの位置の食い違いの中央値 [m]。同じ /dog_odom を 2 つの時計で記録したものを突き合わせる。
    動いている所だけで比べる（止まっていると時刻が決まらない）。粗く 10 ms → 細かく 1 ms"""
    ok = np.isfinite(odom_mono).all(1)
    t, xy = t_mono[ok], odom_mono[ok, :2]
    v = np.hypot(*np.gradient(xy, t, axis=0).T)
    moving = v > MOVING_MPS
    if moving.sum() < 50:
        raise SystemExit(f"[truth] 動いている odom が {int(moving.sum())} 点しか無く、時計を合わせられない")
    t, xy = t[moving], xy[moving]

    def cost(off: float) -> float:
        tw = t + off
        inside = (tw > t_wall[0]) & (tw < t_wall[-1])
        if inside.sum() < 0.5 * len(tw):
            return math.inf
        dx = np.interp(tw[inside], t_wall, odom_wall[:, 0]) - xy[inside, 0]
        dy = np.interp(tw[inside], t_wall, odom_wall[:, 1]) - xy[inside, 1]
        return float(np.median(np.hypot(dx, dy)))

    coarse = guess + np.arange(-search_s, search_s, 0.01)
    c0 = coarse[int(np.argmin([cost(o) for o in coarse]))]
    fine = c0 + np.arange(-0.02, 0.02, 0.001)
    costs = [cost(o) for o in fine]
    k = int(np.argmin(costs))
    return float(fine[k]), float(costs[k])


def clock_from_lidar_yaw(rows: np.ndarray, stock: Stock, guess: float, search_s: float = 2.0) -> Tuple[float, float]:
    """壁 − monotonic [s] と、向きの食い違いの中央 [°]。ライダーで測った pelvis の向き（壁の時計）と走らせ役の odom の向き（monotonic）の
    変化を突き合わせる。odom の向きが動いた所（前後 5 s を足す）だけで比べ、向きの差の一定分は引く。粗く 10 ms → 細かく 1 ms"""
    od = OdomTrack.from_stock(stock)
    rate = np.abs(np.gradient(od.xyyaw[:, 2], od.t))
    turn_t = od.t[rate > TURNING_RAD_S]
    if len(turn_t) < 20:
        raise SystemExit(f"[truth] 回っている odom が {len(turn_t)} 点しか無く、向きで時計を合わせられない")
    acc = rows[:, 5] > 0
    tw, yaw_l = rows[acc, 0], np.unwrap(rows[acc, 3])

    def cost(off: float) -> float:
        tm = tw - off
        sel = (tm >= turn_t.min() - 5.0) & (tm <= turn_t.max() + 5.0)
        if sel.sum() < 20:
            return math.inf
        qi = nearest(stock.t, tm[sel])
        r = yaw_l[sel] - stock.q_waist[qi, 0] - np.interp(tm[sel], od.t, od.xyyaw[:, 2])
        return float(np.median(np.abs(r - np.median(r))))

    coarse = guess + np.arange(-search_s, search_s, 0.01)
    c0 = coarse[int(np.argmin([cost(o) for o in coarse]))]
    fine = c0 + np.arange(-0.02, 0.02, 0.001)
    costs = [cost(o) for o in fine]
    k = int(np.argmin(costs))
    return float(fine[k]), math.degrees(costs[k])


def iter_scans(session: Path, reverse: bool = False, lo: Optional[float] = None, hi: Optional[float] = None
               ) -> Iterator[Tuple[float, np.ndarray]]:
    """(点群の真ん中あたりの壁の時刻, 点) を 1 回ずつ（全部をメモリに載せない）。reverse なら新しい回から"""
    bags = sorted((session / "raw" / "rosbag2").glob("*.db3"))
    if not bags:
        raise SystemExit(f"[truth] db3 が無い: {session}")
    order = "DESC" if reverse else "ASC"
    lo_ns = int((lo + SCAN_DELAY_S) * 1e9) if lo is not None else 0
    hi_ns = int((hi + SCAN_DELAY_S) * 1e9) if hi is not None else 2 ** 62
    for bag in (bags[::-1] if reverse else bags):
        con = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
        try:
            row = con.execute("SELECT id FROM topics WHERE name = ?", (LIDAR_TOPIC,)).fetchone()
            if row is None:
                raise SystemExit(f"[truth] {bag} に {LIDAR_TOPIC} が無い")
            q = (f"SELECT timestamp, data FROM messages WHERE topic_id = ? AND timestamp BETWEEN ? AND ? "
                 f"ORDER BY timestamp {order}")
            for recv_ns, payload in con.execute(q, (row[0], lo_ns, hi_ns)):
                yield recv_ns / 1e9 - SCAN_DELAY_S, scan_xyz(payload)
        finally:
            con.close()


def build_tracker(session: Path, t0_wall: float, t1_wall: float, thickness: float, lever_side: int) -> est.DoorTracker:
    """止まっている間の回（最初の REF_SKIP_S 秒を捨てて STILL_SCANS 回）で扉と地図を作る（実機の REF と同じ）"""
    scans = [xyz for t, xyz in iter_scans(session, lo=t0_wall + REF_SKIP_S, hi=t1_wall)][:STILL_SCANS]
    if len(scans) < STILL_SCANS:
        raise SystemExit(f"[truth] 扉と地図を作る範囲に回が {len(scans)} しか無い（{STILL_SCANS} 要る）")
    return est.DoorTracker.from_still(scans, thickness, lever_side)


ROW_COLS = ("t_wall", "x", "y", "yaw", "angle_deg", "accepted", "rms", "matched", "min_eig", "why", "tilt_deg")


def run_pass(tracker: est.DoorTracker, scans: Iterator[Tuple[float, np.ndarray]], sign: float, label: str) -> np.ndarray:
    """1 回ずつ重ね直す。sign = −1 で時刻の前へ（時刻を反転して渡す）。行は ROW_COLS"""
    rows, t_start = [], time.perf_counter()
    for k, (t_wall, xyz) in enumerate(scans):
        e = tracker.step(sign * t_wall, xyz)
        rows.append((t_wall, e.lidar.x, e.lidar.y, e.lidar.yaw, e.angle_deg, float(e.accepted), e.match.rms_m,
                     e.match.matched, e.match.min_eig, float(tracker.last_why), e.tilt_deg))
        if (k + 1) % 1000 == 0:
            acc = np.mean([r[5] for r in rows[-1000:]])
            print(f"[truth] {label}: {k + 1} 回（直近 1000 回の受け入れ {100 * acc:.0f}%・{time.perf_counter() - t_start:.0f} s）", flush=True)
    return np.array(rows, dtype=float).reshape(-1, len(ROW_COLS))


def tracker_from_saved(path: str, session: Path, ref0_wall: float, ref1_wall: float) -> est.DoorTracker:
    """別の回で作った扉と地図（--save-tracker の pickle）を読み、この回の止まった範囲の回で床を当て直して、位置を探し直す。
    11-10: 18:59 は扉と地図を作り直すと扉の幅を 1.3 cm 広く出し、深度で確かめると前が 2.5〜3 cm ずれていた。同じ日・同じ扉は、
    巻き尺で確かめた回（18:33）の扉と地図を使い回して物差しをそろえる"""
    with open(path, "rb") as f:
        tracker: est.DoorTracker = pickle.load(f)
    scans = [est.valid_points(xyz) for t, xyz in iter_scans(session, lo=ref0_wall + REF_SKIP_S, hi=ref1_wall)][:STILL_SCANS]
    if len(scans) < STILL_SCANS:
        raise SystemExit(f"[truth] 位置を探し直す範囲に回が {len(scans)} しか無い（{STILL_SCANS} 要る）")
    level = est.level_from_scans(scans)
    q = np.concatenate([level.apply(s) for s in scans])
    # 初めの値はこの回の止まった範囲で測った扉の座標でのライダーの姿勢（扉と地図を作り直すと数 cm ずれるだけ）。
    # 地図の全体を総当たりで探す relocalize は、廊下の似た形に当たって横に 1.85 m ずれた（11-10）ので使わない
    own = est.door_frame_from_still([level.apply(s) for s in scans], -2.0 * tracker.door.face_x, tracker.door.lever_side)
    res = est.match_scan(tracker.map, est.band_xy(q), own.lidar_in_door, tracker.door)
    jump = own.lidar_in_door.inverse().compose(res.pose)
    if not res.ok or math.hypot(jump.x, jump.y) > JUMP_FROM_OWN_M:
        raise SystemExit(f"[truth] 読んだ地図に位置が当たらない（当たり {res.ok}・残り {res.rms_m}・当たった点 {res.matched}・"
                         f"この回の扉の座標から {math.hypot(jump.x, jump.y):.3f} m）")
    print(f"[truth] この回で測った扉の座標でのライダー x {own.lidar_in_door.x:+.3f} y {own.lidar_in_door.y:+.3f} "
          f"向き {math.degrees(own.lidar_in_door.yaw):+.1f}°（幅 {own.width:.3f} m）→ 読んだ地図に重ね直して {100 * jump.x:+.1f}・{100 * jump.y:+.1f} cm・"
          f"{math.degrees(jump.yaw):+.1f}° 動いた", flush=True)
    print(f"[truth] 扉と地図は {path} から（幅 {tracker.door.width:.3f} m）・この回の床で頭の傾き {level.tilt_deg():.1f}°・"
          f"位置を探し直した: ライダー x {res.pose.x:+.3f} y {res.pose.y:+.3f} 向き {math.degrees(res.pose.yaw):+.1f}°"
          f"（残り {1000 * res.rms_m:.1f} mm・当たった点 {res.matched}）", flush=True)
    return est.DoorTracker(tracker.door, tracker.map, level, res.pose)


def track_both_ways(session: Path, ref0_wall: float, ref1_wall: float, thickness: float, lever_side: int,
                    load_from: Optional[str] = None, save_to: Optional[str] = None) -> np.ndarray:
    """止まった範囲で扉と地図を作り（load_from があれば別の回のものを読み）、そこから時刻の先へ・前へ追った行（時刻の順）"""
    if load_from:
        tracker = tracker_from_saved(load_from, session, ref0_wall, ref1_wall)
    else:
        tracker = build_tracker(session, ref0_wall, ref1_wall, thickness, lever_side)
        d = tracker.door
        print(f"[truth] 扉と地図を作った: 幅 {d.width:.3f} m・ライダー x {d.lidar_in_door.x:+.3f} y {d.lidar_in_door.y:+.3f}"
              f" 向き {math.degrees(d.lidar_in_door.yaw):+.1f}°・頭の傾き {tracker.level.tilt_deg():.1f}°", flush=True)
    if save_to:
        with open(save_to, "wb") as f:
            pickle.dump(tracker, f)
        print(f"[truth] 扉と地図を書いた: {save_to}", flush=True)
    back = copy.deepcopy(tracker)
    fwd_rows = run_pass(tracker, iter_scans(session, lo=ref0_wall), +1.0, "先へ")
    back_rows = run_pass(back, iter_scans(session, reverse=True, hi=ref0_wall), -1.0, "前へ")
    rows = np.concatenate([back_rows, fwd_rows]) if len(back_rows) else fwd_rows
    return rows[np.argsort(rows[:, 0], kind="stable")]


def nearest(t_sorted: np.ndarray, t: np.ndarray) -> np.ndarray:
    """t_sorted の中で t にいちばん近い番号"""
    i = np.clip(np.searchsorted(t_sorted, t), 1, len(t_sorted) - 1)
    return np.where(np.abs(t_sorted[i - 1] - t) <= np.abs(t_sorted[i] - t), i - 1, i)


def stock_pose_parts(stock: Stock, t_mono: float) -> Tuple[np.ndarray, float, float, np.ndarray, np.ndarray]:
    """その時刻の (腰の 3 関節, roll, pitch, pelvis の四元数, odom)"""
    i = int(nearest(stock.t, np.array([t_mono]))[0])
    roll, pitch = dfr.roll_pitch_from_quat_wxyz(stock.quat[i])
    return stock.q_waist[i], roll, pitch, stock.quat[i], stock.odom[i]


@dataclass(frozen=True)
class OdomTrack:
    """走らせ役の odom（monotonic）を時刻で引く。path は始めからの道のり [m]"""

    t: np.ndarray
    xyyaw: np.ndarray  # [n, 3]（yaw は巻き戻しを解いた値）
    path: np.ndarray

    @classmethod
    def from_stock(cls, stock: Stock) -> "OdomTrack":
        ok = np.isfinite(stock.odom).all(1)
        o = stock.odom[ok].copy()
        o[:, 2] = np.unwrap(o[:, 2])
        return cls(stock.t[ok], o, np.r_[0.0, np.cumsum(np.hypot(*np.diff(o[:, :2], axis=0).T))])

    def at(self, t_mono: float) -> dfr.PlanarPose:
        return dfr.PlanarPose(*(float(np.interp(t_mono, self.t, self.xyyaw[:, k])) for k in range(3)))

    def path_between(self, t0: float, t1: float) -> float:
        return float(abs(np.interp(t1, self.t, self.path) - np.interp(t0, self.t, self.path)))


def pelvis_rows(rows: np.ndarray, off: float, stock: Stock, head_xy: Tuple[float, float]) -> np.ndarray:
    """ライダーの姿勢の行 → 扉の座標での pelvis の床の上の姿勢 [n, 3]（実機と同じ直し方）"""
    out = np.full((len(rows), 3), np.nan)
    for k, r in enumerate(rows):
        qw, roll, pitch, _, _ = stock_pose_parts(stock, r[0] - off)
        out[k] = dlm.lidar_to_pelvis(r[1], r[2], r[3], tuple(qw), roll, pitch, head_xy)
    return out


def fit_center(rows: np.ndarray, off: float, stock: Stock, spot: Spot, thickness: float) -> Tuple[np.ndarray, np.ndarray]:
    """巻き尺の地点で、pelvis（巻き尺）から見たライダーの水平の位置（前・左）と、その時の順運動学の分を引いた head_xy"""
    t_mono = rows[:, 0] - off
    sel = (t_mono >= spot.t0) & (t_mono <= spot.t1) & (rows[:, 5] > 0)
    if sel.sum() < 5:
        raise SystemExit(f"[truth] 取付を合わせる地点（t {spot.t0:.0f}〜{spot.t1:.0f}）に受け入れた回が {int(sel.sum())} しか無い")
    centers, heads = [], []
    pelvis_xy = spot.pelvis_in_door(thickness)
    for r in rows[sel]:
        qw, roll, pitch, _, _ = stock_pose_parts(stock, r[0] - off)
        yaw_p = r[3] - qw[0]
        cy, sy = math.cos(yaw_p), math.sin(yaw_p)
        d = np.array([r[1], r[2]]) - pelvis_xy
        center = np.array([cy * d[0] + sy * d[1], -sy * d[0] + cy * d[1]])
        centers.append(center)
        heads.append(dlm.head_xy_from_center_offset(tuple(center), tuple(qw), roll, pitch))
    return np.median(centers, axis=0), np.median(heads, axis=0)


def center_of_door_in_pelvis(pose: dfr.PlanarPose) -> np.ndarray:
    """扉の座標での pelvis の姿勢 → pelvis（の向き）から見た開口の中心（前・左）"""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return np.array([-(c * pose.x + s * pose.y), -(-s * pose.x + c * pose.y)])


def mirror_door17(p: dc.Params, vp: dvc.VisionParams, real17: np.ndarray) -> np.ndarray:
    """実の向きの扉の値 17 → 網の側（方策の鏡写し）。鏡写しの行列で扉の欄が扉の欄だけから決まることは check_mirror_isolated で見る"""
    if not p.mirror:
        return real17.copy()
    obs = dvc.insert_door(vp, np.zeros(p.obs_dim), real17)
    obs[slice(*vp.door_slices["door_angle_lidar"])] = [real17[0], 1.0, vp.angle_age]
    return dvc.door_from_obs(vp, p.obs_mirror_M @ obs + p.obs_mirror_b)


def check_mirror_isolated(p: dc.Params, vp: dvc.VisionParams) -> None:
    rows = np.concatenate([np.arange(*vp.door_slices[t]) for t in dvc.DOOR_TERMS])
    others = np.setdiff1d(np.arange(p.obs_dim), rows)
    leak = float(np.abs(p.obs_mirror_M[np.ix_(rows, others)]).max()) if p.mirror else 0.0
    if leak > 0:
        raise SystemExit(f"[truth] 鏡写しで扉の欄に扉以外の欄が混ざる（{leak}）。mirror_door17 の作りを見直す")


def angles_at(rows: np.ndarray, off: float, t_cap: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """画像の時刻ごとの扉の角度 [rad] と、持ち越したか。いちばん近い測れた角度が ANGLE_MATCH_S 以内ならそれ。
    無ければ（扉に近いと測れない）その前に最後に測れた角度が ANGLE_CLOSED_DEG 以下（閉じている）なら持ち越す。どちらも無ければ nan"""
    ok = np.isfinite(rows[:, 4])
    out = np.full(len(t_cap), np.nan)
    kept = np.zeros(len(t_cap), dtype=bool)
    if not ok.any():
        return out, kept
    t, a = rows[ok, 0] - off, rows[ok, 4]
    i = nearest(t, t_cap)
    close = np.abs(t[i] - t_cap) <= ANGLE_MATCH_S
    out[close] = np.radians(a[i[close]])
    last = np.searchsorted(t, t_cap) - 1
    carry = ~close & (last >= 0)
    carry[carry] &= np.abs(a[last[carry]]) <= ANGLE_CLOSED_DEG
    out[carry] = np.radians(a[last[carry]])
    kept[carry] = True
    return out, kept


def label_images(t_cap: np.ndarray, rows: np.ndarray, pel: np.ndarray, off: float, odom: OdomTrack
                 ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """画像 1 枚ごとの扉の座標での pelvis の姿勢。(直接 [n,3], 運んだ [n,3], 出どころ [n], 回との時刻の差 [n])
    直接: いちばん近い受け入れた回から odom で運ぶ。運んだ: 扉から NEAR_M より近い間は、最後の遠い受け入れた回から odom で運ぶ
    （道のり CARRY_MAX_PATH_M まで。立っている間は道のりが増えないので何秒でも運ぶ）"""
    acc = rows[:, 5] > 0
    t_acc, pel_acc, far_acc = rows[acc, 0] - off, pel[acc], rows[acc, 1] < -NEAR_M
    direct = np.full((len(t_cap), 3), np.nan)
    carry = np.full((len(t_cap), 3), np.nan)
    src = np.full(len(t_cap), SRC_NONE)
    dt = np.full(len(t_cap), np.nan)
    if not len(t_acc):
        return direct, carry, src, dt
    idx = nearest(t_acc, t_cap)
    far_t, far_i = t_acc[far_acc], np.nonzero(far_acc)[0]
    for j, t in enumerate(t_cap):
        i = int(idx[j])
        dt[j] = t - t_acc[i]
        if abs(dt[j]) <= SCAN_MATCH_S:
            pj = dfr.compose_planar(dfr.PlanarPose(*pel_acc[i]), odom.at(t_acc[i]), odom.at(t))
            direct[j] = (pj.x, pj.y, pj.yaw)
            if far_acc[i]:
                carry[j] = direct[j]
                src[j] = SRC_LIDAR
                continue
        k = int(np.searchsorted(far_t, t)) - 1  # 画像の時刻より前の最後の遠い回
        if k < 0 or odom.path_between(far_t[k], t) > CARRY_MAX_PATH_M:
            continue
        pj = dfr.compose_planar(dfr.PlanarPose(*pel_acc[far_i[k]]), odom.at(far_t[k]), odom.at(t))
        carry[j] = (pj.x, pj.y, pj.yaw)
        src[j] = SRC_CARRY
    return direct, carry, src, dt


def door17_rows(poses: np.ndarray, angles: np.ndarray, t_cap: np.ndarray, stock: Stock, geom: np.ndarray,
                p: dc.Params, vp: dvc.VisionParams) -> Tuple[np.ndarray, np.ndarray]:
    """姿勢 → 扉の値 17（実の向き・網の側）。姿勢か角度が無い枚は nan"""
    real = np.full((len(t_cap), dvc.DOOR_DIM), np.nan)
    mirr = np.full_like(real, np.nan)
    qi = nearest(stock.t, t_cap)
    for j in range(len(t_cap)):
        if not (np.isfinite(poses[j]).all() and math.isfinite(angles[j])):
            continue
        real[j] = np.concatenate([[angles[j]], dfr.door_frame_loc(dfr.PlanarPose(*poses[j]), stock.quat[qi[j]]), geom])
        mirr[j] = mirror_door17(p, vp, real[j])
    return real, mirr


def spot_report(name: str, spots: Sequence[Spot], t_cap: np.ndarray, poses: np.ndarray, thickness: float) -> List[Dict]:
    """巻き尺の地点ごとに、扉の座標での pelvis の位置を巻き尺と比べる（前＝扉の面からの距離・左＝開口の中心が左に何 m）"""
    out = []
    for k, s in enumerate(spots):
        sel = (t_cap >= s.t0) & (t_cap <= s.t1) & np.isfinite(poses[:, 0])
        want = s.pelvis_in_door(thickness)
        if not sel.any():
            print(f"  [{name}] 地点 {k}（toe {s.toe:.3f}）: 正解が付いた枚が無い", flush=True)
            out.append({"spot": k, "n": 0})
            continue
        got = np.median(poses[sel, :2], axis=0)
        sd = np.std(poses[sel, :2], axis=0)
        yaw = math.degrees(float(np.median(poses[sel, 2])))
        err = -(got - want)  # 前・左の向きの差（正なら正解の方が扉から遠い・左）
        print(f"  [{name}] 地点 {k}（toe {s.toe:.3f}・{int(sel.sum())} 枚・向き {yaw:+.1f}°）: 正解 前 {-got[0]:.3f}±{sd[0]:.3f} "
              f"左 {-got[1]:+.3f}±{sd[1]:.3f}・巻き尺 前 {-want[0]:.3f} 左 {-want[1]:+.3f} → 差 前 {100 * err[0]:+.1f} 左 {100 * err[1]:+.1f} cm"
              f"{'' if np.abs(err).max() <= 0.03 else '  ⚠️ 3 cm を越える'}", flush=True)
        out.append({"spot": k, "n": int(sel.sum()), "yaw_deg": yaw, "got": (-got).tolist(), "sd": sd.tolist(),
                    "tape": (-want).tolist(), "err": err.tolist()})
    return out


def spread_report(real17: np.ndarray, poses: np.ndarray) -> Dict:
    ok = np.isfinite(real17[:, 1])
    if not ok.any():
        return {}
    fwd, left = real17[ok, 1], real17[ok, 2]
    yaw = np.degrees(poses[ok, 2])
    ang = np.degrees(real17[ok, 0])
    rep = {
        "前の分布 [m]（5・25・50・75・95%）": np.percentile(fwd, [5, 25, 50, 75, 95]).round(3).tolist(),
        "左の分布 [m]": np.percentile(left, [5, 25, 50, 75, 95]).round(3).tolist(),
        "向きの分布 [°]": np.percentile(yaw, [5, 25, 50, 75, 95]).round(1).tolist(),
        "扉の角度の分布 [°]": np.percentile(ang, [5, 50, 95, 99]).round(1).tolist(),
        "扉の角度が 3° を越える枚": int(np.sum(ang > 3.0)),
        "前の帯ごとの枚 (〜0.5・〜0.8・〜1.2・〜1.6・それ以上)": np.histogram(fwd, [-9, 0.5, 0.8, 1.2, 1.6, 9])[0].tolist(),
    }
    for k, v in rep.items():
        print(f"  {k}: {v}", flush=True)
    return rep


def main(argv=None) -> int:  # noqa: C901
    a = parse_args(argv)
    spots = [Spot(*(float(x) for x in s.split(":"))) for s in a.spot]
    width, height, lever_h, backset = (float(x) for x in a.door.split(","))
    geom = dfr.door_geometry(width, height, lever_h, backset, lever_left=not a.lever_right)
    p = dc.load_params(a.policy, mirror=not a.lever_right)
    vp = dvc.load_vision(a.policy, p)
    check_mirror_isolated(p, vp)

    zv = np.load(a.vision, allow_pickle=True)
    seq, t_cap = zv["seq"], zv["t_capture"]
    stock = load_stock(a.stock)
    if a.ref is not None:
        r0, r1 = (float(x) for x in a.ref.split(":"))
    else:
        r0, r1 = spots[a.ref_spot].t0, spots[a.ref_spot].t1

    if a.clock == "odom":
        if a.odom is None:
            raise SystemExit("[truth] --clock odom には --odom（/dog_odom のセッション）が要る")
        try:
            dog = dob.load_odom(a.odom)
        except sqlite3.DatabaseError as e:
            raise SystemExit(f"[truth] /dog_odom の記録が読めない（{e}）。--clock lidar --clock-guess <同じ日の別の回の壁 − monotonic> で合わせる")
        guess = a.clock_guess if a.clock_guess is not None else clock_guess(a.stock, float(stock.t[0]), a.odom, float(dog.recv_s[0]))
        off, miss = clock_offset(stock.t, stock.odom, dog.recv_s, np.c_[dog.xyz[:, :2], dog.yaw], guess)
        print(f"[truth] 時計の差 壁 − monotonic = {off:.3f} s（/dog_odom の 2 つの記録を突き合わせ・名前からの見込みとの差 {off - guess:+.2f} s・"
              f"動いている間の位置の食い違い 中央 {1000 * miss:.1f} mm）", flush=True)
        off_ref = off
    else:
        if a.clock_guess is None:
            raise SystemExit("[truth] --clock lidar には --clock-guess（同じ日の別の回の壁 − monotonic）が要る")
        off_ref = a.clock_guess
    print(f"[truth] 画像 {len(seq)} 枚（t {t_cap[0]:.0f}〜{t_cap[-1]:.0f}）", flush=True)

    if a.reuse is not None:
        zr = np.load(a.reuse, allow_pickle=False)
        prev = json.loads(str(zr["meta_json"]))
        if prev["ref_t"] != [r0, r1] or abs(prev["clock_offset_s"] - off_ref) > 0.05:
            raise SystemExit(f"[truth] --reuse の推定は別の条件（ref {prev['ref_t']}・時計 {prev['clock_offset_s']:.3f}）で作られた")
        rows = zr["lidar_rows"]
        print(f"[truth] ライダーの推定は {a.reuse} から", flush=True)
    else:
        rows = track_both_ways(a.lidar, r0 + off_ref, r1 + off_ref, a.thickness, -1 if a.lever_right else +1,
                               load_from=a.tracker_from, save_to=a.save_tracker)
    print(f"[truth] ライダー {len(rows)} 回・受け入れ {100 * rows[:, 5].mean():.1f}%・扉の角度が測れた {100 * np.isfinite(rows[:, 4]).mean():.1f}%",
          flush=True)
    if a.clock == "lidar":
        off, miss = clock_from_lidar_yaw(rows, stock, off_ref)
        print(f"[truth] 時計の差 壁 − monotonic = {off:.3f} s（ライダーと odom の向きの変化を突き合わせ・見込みとの差 {off - off_ref:+.3f} s・"
              f"向きの食い違い 中央 {miss:.2f}°）", flush=True)

    if a.fit_spot >= 0:
        center, head_xy = fit_center(rows, off, stock, spots[a.fit_spot], a.thickness)
        print(f"[truth] 取付を地点 {a.fit_spot} の巻き尺で合わせた: pelvis から見たライダー 前 {center[0]:+.3f} 左 {center[1]:+.3f} m"
              f"（旋回の値 +0.047・−0.016／11-04 の巻き尺 −0.012・+0.013）", flush=True)
    else:
        qw, roll, pitch, _, _ = stock_pose_parts(stock, 0.5 * (r0 + r1))
        if a.center == "fk":
            head_xy = np.zeros(2)
            center = dlm.lidar_offset_xy(tuple(qw), roll, pitch, (0.0, 0.0))
        else:
            center = np.array([float(x) for x in a.center.split(",")])
            head_xy = np.array(dlm.head_xy_from_center_offset(tuple(center), tuple(qw), roll, pitch))
        print(f"[truth] 取付は {a.center}: pelvis から見たライダー 前 {center[0]:+.3f} 左 {center[1]:+.3f} m", flush=True)

    pel = pelvis_rows(rows, off, stock, tuple(head_xy))
    odom = OdomTrack.from_stock(stock)
    direct, carry, src_c, dt = label_images(t_cap, rows, pel, off, odom)
    # 正解の本線は直接（11-10: 18:33 で 3 地点とも巻き尺と 3 cm 以内。運んだ値は扉の 0.4 m 手前で脚の odom の短さの分 5 cm ずれた）。
    # 直接が無い枚（いちばん近い受け入れた回が 0.3 s より離れている）だけ運んだ値で埋める
    has_d = np.isfinite(direct[:, 0])
    pose = np.where(has_d[:, None], direct, carry)
    src = np.where(has_d, SRC_LIDAR, np.where(np.isfinite(carry[:, 0]), SRC_CARRY, SRC_NONE))
    angles, angle_kept = angles_at(rows, off, t_cap)
    real, mirr = door17_rows(pose, angles, t_cap, stock, geom, p, vp)
    real_c, mirr_c = door17_rows(carry, angles, t_cap, stock, geom, p, vp)
    print(f"[truth] 正解が付いた枚 {100 * np.isfinite(real[:, 1]).mean():.1f}%（直接 {int(np.sum(src == SRC_LIDAR))} 枚・運んで埋めた "
          f"{int(np.sum(src == SRC_CARRY))} 枚・扉の角度を持ち越した {int(angle_kept.sum())} 枚）", flush=True)
    print("[truth] 巻き尺との比べ（扉の座標・合格の線: 3 cm 以内）", flush=True)
    rep_d = spot_report("正解", spots, t_cap, pose, a.thickness)
    rep_c = spot_report("遠い回から運んだ", spots, t_cap, carry, a.thickness)
    print("[truth] 正解の散らばり", flush=True)
    spread = spread_report(real, pose)

    meta = {"clock": a.clock, "clock_offset_s": off, "clock_miss": miss, "ref_t": [r0, r1], "door": [width, height, lever_h, backset],
            "lever_right": a.lever_right, "thickness": a.thickness, "toe_to_pelvis_m": TOE_TO_PELVIS_M, "center": a.center,
            "center_xy": list(map(float, center)), "head_xy": list(map(float, head_xy)), "fit_spot": a.fit_spot,
            "spots": [s.__dict__ for s in spots], "spot_truth": rep_d, "spot_carry": rep_c, "spread": spread, "near_m": NEAR_M,
            "policy": a.policy, "vision": a.vision, "stock": a.stock, "lidar": str(a.lidar), "tracker_from": a.tracker_from}
    np.savez_compressed(a.out, seq=seq, t_capture=t_cap, truth17_m=mirr, truth17_real=real, truth17_carry_m=mirr_c,
                        truth17_carry_real=real_c, pose=pose, pose_direct=direct, pose_carry=carry, src=src, dt_scan=dt,
                        angle_kept=angle_kept, lidar_rows=rows, lidar_cols=np.array(ROW_COLS),
                        meta_json=json.dumps(meta, ensure_ascii=False))
    print(f"[truth] 書いた: {a.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
