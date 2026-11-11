"""11-10 手順 L1 (a): 段 0 の扉の位置を FAST-LIO の動きで運んだ値を、R0 の正解（ライダーの推定器の直接の値）と巻き尺と比べる（計画書 9.4 節）。

段 0〜2 の走らせ役が使う形そのもの: 扉の座標での pelvis ＝ 段 0 の扉の位置 ∘ FAST-LIO の段 0 からの動き（床に投げる）∘ URDF の livox → pelvis。
ここでは段 0 の扉の位置を R0 の正解（止まった区間の推定器の値）にして、FAST-LIO の運びの誤差だけを出す（網の外れは L1 (b)）。

流れ:
  1. FAST-LIO の再生（Mac の colima の replay_realtime.sh・fastlio_g1.yaml）の /Odometry_loc（camera_init → body。body ≡ livox_frame）を読む
  2. 時刻: FAST-LIO の時刻はライダーの時計（点群のヘッダ＋1 回の長さ）。いちばん近い点群のヘッダに当てて、その回の受け取りの時刻
     （推定器の行と同じ「受け取り − SCAN_DELAY_S」）を付ける。ヘッダとの差が 1 回の半分を越える出力は捨てる
  3. 床に投げる: 段 0（止まった区間）の点群で床を当て（lidar_door_est.level_from_scans、推定器と同じ）、その水平の座標を基準に、
     FAST-LIO の 3 次元の姿勢から各時刻のライダーの水平の位置と向き（傾きだけを外した ZYX の向き）を出す
  4. 段 0 の推定器のライダーの姿勢（止まった区間の中央値）に結び、推定器の行と同じ形（ROW_COLS）にする
  5. pelvis へ: R0 と同じ door_lidar_msg.lidar_to_pelvis（腰の 3 関節・pelvis の IMU・取付は URDF の計算だけ）
  6. 比べる: 同じ回どうしで R0 の正解（受け入れた回）と、着いた先の止まった区間で巻き尺と。参考に /dog_odom で運んだ値も並べる

使い方（OMEN・doorman6 の Python・real_ana_1106/）:
  python l1_fastlio_carry_1110.py --fastlio <再生の tf_bag> --lidar <切り出したセッション> --truth truth_183336_S.npz \\
      --anchor 1500:1525 --end 1545:1560 --spot 1540:1610:0.210:0.070 --json out.json
  --anchor・--end・--spot は monotonic 秒（画像・走らせ役の時計）。壁の時計との差は正解の meta の clock_offset_s
"""
from __future__ import annotations

import argparse
import json
import math
import sqlite3
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_frame as dfr  # noqa: E402
import lidar_door_est as est  # noqa: E402
import real_truth_1110 as rt  # noqa: E402
from dog_odom_bag import parse_odom_cdr  # noqa: E402

SCAN_PERIOD_S = 0.1  # MID-360 の 1 回
MATCH_MAX_S = 0.02  # FAST-LIO の出力とその回のヘッダ（＋1 回）の差の上限（1 回の半分にすると、どの出力もどこかの回に当たってしまう）
ODOM_TOPIC = "/Odometry_loc"  # このフォークは /Odometry を改名している（replay_realtime.sh の注記）
PASS_TRUTH_M = 0.02  # 計画書 9.4 L1 (a): R0 の正解に対して
PASS_TAPE_M = 0.05  # 巻き尺に対して


# ── FAST-LIO の出力を読む（fastlio_vs_icp_1021.load_fastlio と同じ。あちらは open3d を読み込むので OMEN の環境で使えない） ──
def quat_to_mat(q: np.ndarray) -> np.ndarray:
    """四元数 (x, y, z, w) → 回転行列"""
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_fastlio(bag_dir: Path) -> Tuple[np.ndarray, np.ndarray]:
    """/Odometry_loc を (ヘッダ時刻 [s], camera_init → body の 4×4 [n,4,4]) で"""
    dbs = sorted(bag_dir.glob("*.db3"))
    if not dbs:
        raise SystemExit(f"[l1] db3 が無い: {bag_dir}")
    stamps, Ts = [], []
    for db in dbs:
        con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        try:
            tid = con.execute("SELECT id FROM topics WHERE name = ?", (ODOM_TOPIC,)).fetchone()
            if tid is None:
                raise SystemExit(f"[l1] {db} に {ODOM_TOPIC} が無い")
            for _ts, data in con.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", tid):
                if b"camera_init\x00" not in data[:40] or b"body\x00" not in data[:48]:
                    raise SystemExit(f"[l1] frame 名が camera_init → body でない: {data[:48]!r}")
                sec, nsec, x, y, z, qx, qy, qz, qw = parse_odom_cdr(data)
                T = np.eye(4)
                T[:3, :3] = quat_to_mat(np.array([qx, qy, qz, qw]))
                T[:3, 3] = (x, y, z)
                stamps.append(sec + nsec / 1e9)
                Ts.append(T)
        finally:
            con.close()
    if not stamps:
        raise SystemExit(f"[l1] {bag_dir} の {ODOM_TOPIC} が 0 件（FAST-LIO が起きていない）")
    order = np.argsort(stamps)
    return np.array(stamps)[order], np.array(Ts)[order]


def scan_times(session: Path) -> Tuple[np.ndarray, np.ndarray]:
    """点群の (受け取りの壁の時刻 [s], ヘッダの時刻 [s])。ヘッダは PointCloud2 の CDR の先頭 4+4+4 バイト"""
    db = next((session / "raw" / "rosbag2").glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    try:
        tid = con.execute("SELECT id FROM topics WHERE name = ?", (rt.LIDAR_TOPIC,)).fetchone()[0]
        rows = con.execute("SELECT timestamp, substr(data, 5, 8) FROM messages WHERE topic_id = ? ORDER BY timestamp",
                           (tid,)).fetchall()
    finally:
        con.close()
    recv = np.array([r[0] for r in rows]) / 1e9
    hdr = np.array([int.from_bytes(r[1][:4], "little", signed=True) + int.from_bytes(r[1][4:], "little") / 1e9 for r in rows])
    return recv, hdr


def match_to_scans(fl_stamp: np.ndarray, recv: np.ndarray, hdr: np.ndarray) -> Tuple[np.ndarray, np.ndarray, Dict]:
    """FAST-LIO の出力ごとに、その姿勢の時刻（受け取りの時計）と、当たった印。
    FAST-LIO の姿勢は走査の終わり（ヘッダ＋1 回）。推定器の行の時刻「受け取り − SCAN_DELAY_S」は走査の真ん中なので、
    同じ回でも FAST-LIO の姿勢は SCAN_PERIOD_S / 2 だけ後（歩く 0.3 m/s で 1.5 cm）。比べる時は推定器の時刻へ補間する。
    同じ回に 2 つの出力が当たったら、先の 1 つだけを残す"""
    key = fl_stamp - SCAN_PERIOD_S
    i = rt.nearest(hdr, key)
    diff = key - hdr[i]
    ok = np.abs(diff) <= MATCH_MAX_S
    first = np.zeros(len(i), bool)
    first[np.unique(i, return_index=True)[1]] = True
    ok &= first
    t_wall = recv[i] - rt.SCAN_DELAY_S + SCAN_PERIOD_S / 2
    rep = {"出力": int(len(fl_stamp)), "点群": int(len(recv)), "回に当たった": int(ok.sum()),
           "出力の時刻 − (ヘッダ + 0.1 s) の中央値 [ms]": round(1000 * float(np.median(diff)), 1),
           "受け取り − ヘッダ の中央値 [s]": round(float(np.median(recv - hdr)), 3),
           "出力の最大の穴 [s]": round(float(np.diff(fl_stamp).max()), 3) if len(fl_stamp) > 1 else None}
    return t_wall, ok, rep


# ── 床に投げる ───────────────────────────────────────────────────
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
    M = np.linalg.inv(T_anchor) @ Ts  # 段 0 のライダーの座標で見た動き
    Rw = level.R @ M[:, :3, :3]
    A = Rw @ est.FLIP.T
    yaw = np.arctan2(A[:, 1, 0], A[:, 0, 0])
    p = M[:, :3, 3] @ level.R.T  # 位置（W の座標。原点は段 0 のライダー）
    return np.c_[p[:, 0], p[:, 1], yaw]


def interp_planar(t_new: np.ndarray, t: np.ndarray, p: np.ndarray) -> np.ndarray:
    """平面の姿勢の列 p（[n,3]・時刻 t の順）を t_new へ線形に補間する（向きは巻き戻しを解いてから）"""
    yaw = np.unwrap(p[:, 2])
    return np.c_[np.interp(t_new, t, p[:, 0]), np.interp(t_new, t, p[:, 1]), np.interp(t_new, t, yaw)]


def anchor_scans(session: Path, anchor: "Window", off: float) -> Tuple[list, float, float]:
    """段 0 の床を当てる回（R0 の REF と同じく始めの REF_SKIP_S を捨てて STILL_SCANS 回）と、その最初と最後の時刻（壁）。
    段 0 の FAST-LIO の平均の姿勢・推定器の中央値も、この同じ区間で取る（窓の中で頭の傾きが動いても床と姿勢が食い違わない）"""
    got = list(rt.iter_scans(session, lo=anchor.t0 + off + rt.REF_SKIP_S, hi=anchor.t1 + off))[: rt.STILL_SCANS]
    if len(got) < rt.STILL_SCANS:
        raise SystemExit(f"[l1] 段 0 の区間に回が {len(got)} しか無い（{rt.STILL_SCANS} 要る）")
    return [xyz for _, xyz in got], got[0][0], got[-1][0]


def match_rows(t_query: np.ndarray, t_rows: np.ndarray, tol_s: float = 1e-3) -> np.ndarray:
    """t_query ごとに、t_rows（時刻の順）の同じ回の番号。tol_s より離れていれば −1"""
    j = rt.nearest(t_rows, t_query)
    return np.where(np.abs(t_rows[j] - t_query) <= tol_s, j, -1)


def compose(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """平面の姿勢 a（[3]）の上に、a の座標で見た動き b（[n,3]）を足す"""
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.c_[a[0] + c * b[:, 0] - s * b[:, 1], a[1] + s * b[:, 0] + c * b[:, 1], a[2] + b[:, 2]]


def wrap(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


@dataclass(frozen=True)
class Window:
    t0: float  # monotonic
    t1: float

    @classmethod
    def parse(cls, s: str) -> "Window":
        a, b = (float(x) for x in s.split(":"))
        return cls(a, b)

    def mask(self, t_mono: np.ndarray) -> np.ndarray:
        return (t_mono >= self.t0) & (t_mono <= self.t1)


def err_stats(d: np.ndarray) -> Dict:
    """扉の座標での差 [n, 3]（x, y, 向き）→ 前後（扉の面に垂直）・横・距離・向きのまとめ [cm・°]"""
    if not len(d):
        return {"n": 0}
    r = np.hypot(d[:, 0], d[:, 1])
    return {"n": int(len(d)), "前後 中央 [cm]": round(100 * float(np.median(d[:, 0])), 2),
            "横 中央 [cm]": round(100 * float(np.median(d[:, 1])), 2), "距離 中央 [cm]": round(100 * float(np.median(r)), 2),
            "距離 95% [cm]": round(100 * float(np.percentile(r, 95)), 2), "距離 最大 [cm]": round(100 * float(r.max()), 2),
            "向き 中央 [°]": round(math.degrees(float(np.median(d[:, 2]))), 2),
            "向き 最大 [°]": round(math.degrees(float(np.abs(d[:, 2]).max())), 2)}


def odom_carry(stock: rt.Stock, t_mono: np.ndarray, start: np.ndarray, t_start: float) -> np.ndarray:
    """段 0 の pelvis を /dog_odom（走らせ役の odom・補正なし）で運んだ値 [n, 3]（参考）"""
    odom = rt.OdomTrack.from_stock(stock)
    o0 = odom.at(t_start)
    s = dfr.PlanarPose(*start)
    out = [dfr.compose_planar(s, o0, odom.at(float(t))) for t in t_mono]
    return np.array([(p.x, p.y, p.yaw) for p in out])


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fastlio", required=True, type=Path, help="再生の tf_bag（/Odometry_loc）")
    ap.add_argument("--lidar", required=True, type=Path, help="再生に使ったライダーのセッション（切り出し）")
    ap.add_argument("--truth", required=True, help="R0 の正解（real_truth_1110.py の npz）")
    ap.add_argument("--stock", default=None, help="走らせ役の記録（既定は正解の meta の stock）")
    ap.add_argument("--anchor", required=True, help="段 0 にあたる止まった区間 t0:t1（monotonic）")
    ap.add_argument("--end", required=True, help="着いた先の止まった区間 t0:t1（monotonic）")
    ap.add_argument("--spot", default=None, help="着いた先の巻き尺 t0:t1:toe:left（無ければ巻き尺とは比べない）")
    ap.add_argument("--label", default="")
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--npz", type=Path, default=None, help="回ごとの値を書く")
    return ap.parse_args(argv)


def main(argv=None) -> int:  # noqa: C901
    a = parse_args(argv)
    z = np.load(a.truth, allow_pickle=False)
    meta = json.loads(str(z["meta_json"]))
    off = float(meta["clock_offset_s"])
    head_xy = tuple(meta["head_xy"])
    thickness = float(meta["thickness"])
    truth = z["lidar_rows"]
    stock = rt.load_stock(a.stock or meta["stock"])
    anchor, end = Window.parse(a.anchor), Window.parse(a.end)

    st, Ts = load_fastlio(a.fastlio)
    recv, hdr = scan_times(a.lidar)
    t_wall, ok, health = match_to_scans(st, recv, hdr)
    st, Ts, t_wall = st[ok], Ts[ok], t_wall[ok]
    t_mono = t_wall - off
    print(f"[l1] {a.label} FAST-LIO: {health}", flush=True)

    # 段 0: 床・FAST-LIO の平均の姿勢・推定器のライダーの姿勢（受け入れた回の中央値）を同じ 25 回の区間で
    scans, ta0, ta1 = anchor_scans(a.lidar, anchor, off)
    level = est.level_from_scans(scans)
    ma = (t_wall >= ta0) & (t_wall <= ta1 + SCAN_PERIOD_S)
    if ma.sum() < 10:
        raise SystemExit(f"[l1] 段 0 の区間に FAST-LIO の出力が {int(ma.sum())} しか無い")
    T_a = mean_pose(Ts[ma])
    tm = (truth[:, 0] >= ta0) & (truth[:, 0] <= ta1) & (truth[:, 5] > 0)
    if tm.sum() < 10:
        raise SystemExit(f"[l1] 段 0 の区間に正解の受け入れた回が {int(tm.sum())} しか無い")
    d_a = np.r_[np.median(truth[tm, 1:3], axis=0), float(np.median(np.unwrap(truth[tm, 3])))]
    still = planar_track(Ts[ma], T_a, level)
    print(f"[l1] 段 0: 床の傾き {level.tilt_deg():.1f}°・高さ {level.h:.3f} m・推定器のライダー x {d_a[0]:+.3f} y {d_a[1]:+.3f} "
          f"向き {math.degrees(d_a[2]):+.1f}°・止まった間の FAST-LIO の揺れ {100 * float(np.hypot(*still[:, :2].T).max()):.1f} cm", flush=True)

    # FAST-LIO で運んだライダーの行（推定器と同じ「走査の真ん中」の時刻へ補間）→ pelvis
    lid_end = compose(d_a, planar_track(Ts, T_a, level))
    t_wall = t_wall - SCAN_PERIOD_S / 2
    lid = interp_planar(t_wall, t_wall + SCAN_PERIOD_S / 2, lid_end)
    t_mono = t_wall - off
    rows = np.full((len(lid), len(rt.ROW_COLS)), np.nan)
    rows[:, 0], rows[:, 1:4], rows[:, 5] = t_wall, lid, 1.0
    pel_fl = rt.pelvis_rows(rows, off, stock, head_xy)

    # 正解と同じ回どうし
    jt = match_rows(t_wall, truth[:, 0])
    has = jt >= 0
    acc = np.zeros(len(jt), bool)
    acc[has] = truth[jt[has], 5] > 0
    pel_tr = np.full_like(pel_fl, np.nan)
    pel_tr[acc] = rt.pelvis_rows(truth[jt[acc]], off, stock, head_xy)
    d = pel_fl - pel_tr
    d[:, 2] = wrap(d[:, 2])
    me = end.mask(t_mono) & acc
    moving = acc & ~anchor.mask(t_mono) & ~end.mask(t_mono)

    # /dog_odom で運んだ値（参考）
    m_start = (t_wall >= ta0) & (t_wall <= ta1) & acc
    start = np.median(pel_tr[m_start], axis=0)
    t_start = float(np.median(t_mono[m_start]))
    pel_od = odom_carry(stock, t_mono, start, t_start)
    do = pel_od - pel_tr
    do[:, 2] = wrap(do[:, 2])

    tr_end = np.median(pel_tr[me], axis=0) if me.any() else np.full(3, np.nan)
    tr_start = start
    motion = {"動いた距離 [m]": round(float(np.hypot(*(tr_end[:2] - tr_start[:2]))), 3),
              "回った角度 [°]": round(math.degrees(float(wrap(np.array([tr_end[2] - tr_start[2]]))[0])), 1),
              "着いた先の pelvis（扉の座標）": [round(float(v), 3) for v in tr_end[:2]],
              "時間 [s]": round(float(np.median(t_mono[me]) - t_start), 1) if me.any() else None}
    res: Dict = {"label": a.label, "FAST-LIO": health, "動き（正解）": motion,
                 "段 0 の床の傾き [°]": round(level.tilt_deg(), 2),
                 "着いた先（止まった区間）": err_stats(d[me]), "動いている間": err_stats(d[moving]), "全部": err_stats(d[acc]),
                 "参考: /dog_odom で運ぶ 着いた先": err_stats(do[me]), "参考: /dog_odom で運ぶ 動いている間": err_stats(do[moving])}
    if me.any():
        e_end = float(np.hypot(*np.median(d[me, :2], axis=0)))
        res["合否 R0 の正解に対して 2 cm"] = bool(e_end <= PASS_TRUTH_M)
    if a.spot:
        t0, t1, toe, left = (float(x) for x in a.spot.split(":"))
        sp = rt.Spot(t0, t1, toe, left)
        want = sp.pelvis_in_door(thickness)
        ms = (t_mono >= t0) & (t_mono <= t1)
        if ms.any():
            got = np.median(pel_fl[ms, :2], axis=0)
            gt = np.median(pel_tr[ms & acc, :2], axis=0) if (ms & acc).any() else np.full(2, np.nan)
            e = got - want
            res["巻き尺"] = {"n": int(ms.sum()), "巻き尺の pelvis": [round(float(v), 3) for v in want],
                            "FAST-LIO − 巻き尺 [cm]": [round(100 * float(v), 1) for v in e],
                            "正解 − 巻き尺 [cm]": [round(100 * float(v), 1) for v in gt - want],
                            "合否 5 cm": bool(float(np.hypot(*e)) <= PASS_TAPE_M)}
    for k, v in res.items():
        print(f"[l1]   {k}: {v}", flush=True)
    if a.json:
        a.json.write_text(json.dumps(res, ensure_ascii=False, indent=1))
    if a.npz:
        np.savez_compressed(a.npz, t_mono=t_mono, pel_fastlio=pel_fl, pel_truth=pel_tr, pel_odom=pel_od, lidar_fastlio=lid,
                            acc=acc, meta_json=json.dumps(res, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
