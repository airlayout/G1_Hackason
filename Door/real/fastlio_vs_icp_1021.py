"""段 1 の続き（2026-10-23）: 10-21 の記録に FAST_LIO を掛け直し、止まっている区間どうしの動きを
ライダーの ICP（正解）・脚の odometry（/dog_odom）と同じ点（robot_center）で並べる。

FAST_LIO の再生（Mac の colima の rviz コンテナ・実時間）は既製の replay_realtime.sh で行う:
  G1_RT_ENGINE=fastlio G1_BAG_SUB=raw/rosbag2 G1_BAG_DELAY=5 \\
  G1_FASTLIO_CFG_DIR=/work/G1_Hackason/Mapping/real/quickstart/pc2/fastlio_loc G1_FASTLIO_CFG=fastlio_g1.yaml \\
    bash Mapping/real/quickstart/replay_realtime.sh <ライダーのセッション> fastlio_live_cfg
  → runs/<セッション>/fastlio_live_cfg/tf_bag に /Odometry_loc（camera_init → body）
⚠️ 先に VM の受信バッファを上げること（colima ssh -- sudo sysctl -w net.core.rmem_max=33554432 net.core.rmem_default=8388608）。
   既定の 208 KB だと 1 回 440 KB の点群が best effort の購読で 11% 欠ける（01 で 598 回中 530 回。0.6 s の穴）

比べ方:
1. 止まっている区間・床の水平化 L・ICP は lidar_still_icp.py の関数をそのまま使う（同じ L を FAST_LIO にも掛ける）
2. FAST_LIO の姿勢 T(t) は camera_init → body。body → livox_frame の外部パラメータ E を掛けて camera_init → livox にし、
   区間ごとに平均した姿勢 T_k から、区間 0 に対する動き T_0⁻¹ T_k を出す。
   それを L·(…)·L⁻¹ で床の水平の座標に直し、ICP と同じ planar → lidar_to_center で robot_center の 前・左・向き にする
   （eval_traj.py の to_base_link の注記の「odometry は A∘T∘A⁻¹」と同じ形）
3. 時刻: FAST_LIO のヘッダ時刻は点群のヘッダ時刻＋1 回の長さ（lidar_end_time）＝ PC2 の受信時刻とほぼ同じ（差を出して確かめる）。
   区間は /dog_odom の受信時刻で決まっているので、そのまま突き合わせる
4. 起動直後の WARMUP_S 秒は区間の平均に入れない（静止の 01 で最初の 2 s に 3 cm 動く過渡がある）
5. ICP は odometry の初期値と FAST_LIO の初期値の両方から解き、答えが同じかを見る（初期値に引きずられていないか）

使い方: Navigation/.venv/bin/python Door/real/fastlio_vs_icp_1021.py [--names 08b 04 ...] [--subs fastlio_live_cfg ...]
出力: Door/real/logs_real/fastlio_vs_icp_1021/ に 結果の json・軌跡の csv と npz
"""
from __future__ import annotations

import argparse
import json
import math
import sqlite3
import sys
import warnings
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import door_lidar_angle as dla  # noqa: E402
import lidar_still_icp as lsi  # noqa: E402
from crop_lidar_bag import iter_lidar  # noqa: E402
from dog_odom_bag import load_odom, parse_odom_cdr  # noqa: E402
from take_quicklook import body_frame, remote_events, still_segments  # noqa: E402

warnings.filterwarnings("ignore", category=RuntimeWarning)

RUNS = HERE.parents[1] / "Mapping" / "real" / "runs"
REMOTE_DIR = RUNS / "logs_1021"
OUT_DEFAULT = HERE / "logs_real" / "fastlio_vs_icp_1021"
NAMES_DEFAULT = ["01", "02", "02b", "02c", "02d", "04", "06", "08b"]
# body → livox_frame（FAST_LIO の extrinsic_T・R。点は p_body = R·p_livox + T）
EXTRINSIC_T = {
    "fastlio_live_cfg": (0.0, 0.0, 0.0),  # pc2/fastlio_loc/fastlio_g1.yaml（PC2 の live）。推定 OFF ＝ body ≡ livox_frame
    "fastlio_pc2cfg_buf": (-0.011, -0.02329, 0.04412),  # quickstart/fastlio/g1_mid360_pc2.yaml。⚠️ 推定 ON なので名目の値
    "fastlio_live_cfg_rep2": (0.0, 0.0, 0.0),  # live と同じ設定の 2 回目の再生（実時間の再生は決定的でないので回ごとの差を見る）
}
SUBS_DEFAULT = ["fastlio_live_cfg", "fastlio_pc2cfg_buf"]
WARMUP_S = 5.0  # [s] FAST_LIO の起動直後を区間の平均から外す
# FAST_LIO の区間の窓。"all" = 区間全部（ICP・odom と同じ）、"tail3" = 最後の 3 s、"head3" = 最初の 3 s（平均の取り方への依存を見る）
FL_WINDOW = "all"
FL_WINDOW_S = 3.0
MIN_LEG_M = 0.3  # [m] これより短い動きは 1 m あたりの値を出さない
TURN_DEG = 45.0  # [°] これより回った動きは「旋回」に分ける
FRAME_AHEAD_M = 1.0  # [m] 運んだ扉の枠が着いた先でこの距離だけ前にあるとしたときの誤差も出す


# ── 小さな幾何 ─────────────────────────────────────────────────────
def quat_to_mat(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(R: np.ndarray) -> np.ndarray:
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
    return np.array([x, y, z, w])


def mean_pose(Ts: np.ndarray) -> np.ndarray:
    """止まっている間の姿勢の平均（位置は平均・回転は符号をそろえた四元数の平均）"""
    qs = np.array([mat_to_quat(T[:3, :3]) for T in Ts])
    qs[qs @ qs[0] < 0] *= -1
    q = qs.mean(0)
    out = np.eye(4)
    out[:3, :3] = quat_to_mat(q / np.linalg.norm(q))
    out[:3, 3] = Ts[:, :3, 3].mean(0)
    return out


def se2_rel(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """a から見た b（どちらも 前・左・向き[rad]）"""
    c, s = math.cos(a[2]), math.sin(a[2])
    d = b[:2] - a[:2]
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], wrap(b[2] - a[2])])


def se2_apply(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    c, s = math.cos(p[2]), math.sin(p[2])
    return p[:2] + np.array([c * q[0] - s * q[1], s * q[0] + c * q[1]])


def se2_inv(p: np.ndarray) -> np.ndarray:
    c, s = math.cos(p[2]), math.sin(p[2])
    return np.array([-(c * p[0] + s * p[1]), s * p[0] - c * p[1], -p[2]])


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# ── 読み込み ───────────────────────────────────────────────────────
def session(name: str, kind: str) -> Path:
    hits = sorted(RUNS.glob(f"20261021T*_odom_1021_{name}_{kind}"))
    if len(hits) != 1:
        raise SystemExit(f"[fl] {name}_{kind} が {len(hits)} 個")
    return hits[0]


def load_fastlio(bag_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    """/Odometry_loc を (ヘッダ時刻 [s], camera_init → body の 4×4) で。frame 名も確かめる"""
    dbs = sorted(bag_dir.glob("*.db3"))
    if not dbs:
        raise SystemExit(f"[fl] db3 が無い: {bag_dir}")
    stamps, Ts = [], []
    for db in dbs:
        con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        try:
            tid = con.execute("SELECT id FROM topics WHERE name = '/Odometry_loc'").fetchone()
            if tid is None:
                raise SystemExit(f"[fl] {db} に /Odometry_loc が無い")
            for _ts, data in con.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", tid):
                if b"camera_init\x00" not in data[:40] or b"body\x00" not in data[:48]:
                    raise SystemExit(f"[fl] frame 名が camera_init → body でない: {data[:48]!r}")
                sec, nsec, x, y, z, qx, qy, qz, qw = parse_odom_cdr(data)
                T = np.eye(4)
                T[:3, :3] = quat_to_mat(np.array([qx, qy, qz, qw]))
                T[:3, 3] = (x, y, z)
                stamps.append(sec + nsec / 1e9)
                Ts.append(T)
        finally:
            con.close()
    order = np.argsort(stamps)
    return np.array(stamps)[order], np.array(Ts)[order]


def lidar_times(li: Path) -> tuple[np.ndarray, np.ndarray]:
    rec = [(r, s) for r, s, _ in iter_lidar(li)]
    return np.array([r for r, _ in rec]) / 1e9, np.array([s for _, s in rec]) / 1e9


# ── 姿勢を robot_center の 前・左・向き に ───────────────────────────
def to_center(T_rel_livox: np.ndarray, L: np.ndarray) -> np.ndarray:
    """区間 0 のライダーから見たライダーの動き（ライダーの座標）→ 区間 0 の robot_center から見た robot_center"""
    T_lev = L @ T_rel_livox @ np.linalg.inv(L)
    return lsi.lidar_to_center(*lsi.planar(T_lev))


def odom_rel(tr, segs: list[tuple[float, float]]) -> list[np.ndarray]:
    """lidar_still_icp.py の main と同じ計算（区間ごとの平均 → 区間 0 の robot_center の座標）"""
    t = tr.recv_s - tr.recv_s[0]
    fx, fy, yaw = body_frame(tr)
    odo = []
    for sa, sb in segs:
        m = (t >= sa + lsi.EDGE_S) & (t <= sb - lsi.EDGE_S)
        odo.append(np.array([fx[m].mean(), fy[m].mean(), math.radians(float(yaw[m].mean()))]))
    f0, l0, y0 = odo[0]
    c0, s0 = math.cos(y0), math.sin(y0)
    out = []
    for fk, lk, yk in odo:
        df, dl = fk - f0, lk - l0
        out.append(np.array([c0 * df + s0 * dl, -s0 * df + c0 * dl, yk - y0]))
    return out


def init_from_center(p: np.ndarray) -> np.ndarray:
    ix, iy = lsi.center_to_lidar_init(*p)
    T = np.eye(4)
    c, s = math.cos(p[2]), math.sin(p[2])
    T[:2, :2] = [[c, -s], [s, c]]
    T[0, 3], T[1, 3] = ix, iy
    return T


# ── FAST_LIO の区間ごとの姿勢と軌跡 ────────────────────────────────
def fastlio_track(li: Path, sub: str, lr: np.ndarray) -> dict | None:
    bag = li / sub / "tf_bag"
    if not bag.exists():
        return None
    st, Tb = load_fastlio(bag)
    E = np.eye(4)
    E[:3, 3] = EXTRINSIC_T[sub]
    Tl = Tb @ E
    j = np.clip(np.searchsorted(lr, st), 1, len(lr) - 1)
    near = np.where(np.abs(st - lr[j - 1]) < np.abs(st - lr[j]), lr[j - 1], lr[j])
    gaps = np.diff(st)
    return {"stamp": st, "T_body": Tb, "T": Tl, "E": E,
            "check": {"回数": int(len(st)), "ライダーの回数": int(len(lr)),
                      "0.15 s を超える穴": int((gaps > 0.15).sum()), "最大の穴 [s]": round(float(gaps.max()), 3),
                      "ヘッダ時刻 − 受信時刻 の中央値 [ms]": round(float(np.median(st - near)) * 1000, 1)}}


def segment_pose(fl: dict, a: float, b: float, L: np.ndarray) -> tuple[np.ndarray | None, dict]:
    """区間（端を削った窓・起動直後を除く）の平均姿勢と、窓の中の揺れ（robot_center・mm）"""
    st = fl["stamp"]
    lo, hi = a + lsi.EDGE_S, b - lsi.EDGE_S
    if FL_WINDOW == "tail3":
        lo = max(lo, hi - FL_WINDOW_S)
    elif FL_WINDOW == "head3":
        hi = min(hi, max(lo, st[0] + WARMUP_S) + FL_WINDOW_S)
    m = (st >= lo) & (st <= hi) & (st >= st[0] + WARMUP_S)
    if m.sum() < 3:
        return None, {"標本": int(m.sum())}
    Tm = mean_pose(fl["T"][m])
    Tinv = np.linalg.inv(Tm)
    c = np.array([to_center(Tinv @ T, L) for T in fl["T"][m]])
    r = np.hypot(c[:, 0], c[:, 1])
    return Tm, {"標本": int(m.sum()), "揺れ xy の rms [mm]": round(float(np.sqrt((r ** 2).mean())) * 1000, 1),
                "揺れ xy の最大 [mm]": round(float(r.max()) * 1000, 1),
                "向きの揺れ の最大 [°]": round(math.degrees(float(np.abs(c[:, 2]).max())), 3)}


def save_track(out: Path, name: str, sub: str, fl: dict, L: np.ndarray, T_ref: np.ndarray, t0: float,
               segs_abs: list) -> np.ndarray:
    Tinv = np.linalg.inv(T_ref)
    c = np.array([to_center(Tinv @ T, L) for T in fl["T"]])
    c[:, 2] = np.unwrap(c[:, 2])
    stem = out / f"traj_{name}_{sub}"
    np.savez_compressed(stem.with_suffix(".npz"), stamp_s=fl["stamp"], t_s=fl["stamp"] - t0,
                        center_fwd_left_yaw=c, T_camera_init_body=fl["T_body"], T_camera_init_livox=fl["T"],
                        E_body_livox=fl["E"], L_level=L, T_ref_livox=T_ref, segments_abs_s=np.array(segs_abs))
    with stem.with_suffix(".csv").open("w") as f:
        f.write("# robot_center の 前・左 [m]・向き [°]。基準 = 最初の止まっている区間の平均姿勢（02c は最後の区間）。"
                "床で水平にした座標（L は ICP と同じ）。stamp_s = FAST_LIO のヘッダ時刻（PC2 の時計と ~10 ms）、"
                "t_s = /dog_odom の最初の受信からの秒\n")
        f.write("stamp_s,t_s,fwd_m,left_m,yaw_deg\n")
        for s, (x, y, w) in zip(fl["stamp"], c):
            f.write(f"{s:.6f},{s - t0:.4f},{x:.5f},{y:.5f},{math.degrees(w):.4f}\n")
    return c


def static_report(fl: dict, L: np.ndarray, a: float, b: float) -> dict:
    """静止の記録: 起動直後の過渡と、その後の動き（robot_center）"""
    st = fl["stamp"]
    m = (st >= a) & (st <= b)
    Ts, ts = fl["T"][m], st[m]
    c0 = np.array([to_center(np.linalg.inv(Ts[0]) @ T, L) for T in Ts])
    warm = ts < ts[0] + WARMUP_S
    ref = mean_pose(Ts[warm][-5:]) if warm.sum() >= 5 else Ts[0]
    c = np.array([to_center(np.linalg.inv(ref) @ T, L) for T in Ts[~warm]])
    r = np.hypot(c[:, 0], c[:, 1])
    tt = ts[~warm] - ts[~warm][0]
    return {"秒": round(float(ts[-1] - ts[0]), 1),
            "起動直後 WARMUP_S の動き [mm]": round(float(np.hypot(*c0[warm][-1, :2])) * 1000, 1),
            "その後の最大 [mm]": round(float(r.max()) * 1000, 1),
            "その後の終わり [mm]": round(float(r[-1]) * 1000, 1),
            "その後の流れ [mm/分]": round(float(np.hypot(np.polyfit(tt, c[:, 0], 1)[0], np.polyfit(tt, c[:, 1], 1)[0])) * 60000, 1),
            "その後の向きの終わり [°]": round(math.degrees(float(c[-1, 2])), 3)}


# ── 1 本ぶん ───────────────────────────────────────────────────────
def r4(v) -> list[float]:
    return [round(float(x), 4) for x in v]


def run_icp(lev: list[np.ndarray], k: int, init: np.ndarray) -> tuple[np.ndarray, float, float]:
    T, fit, rmse = lsi.icp(lev[k], lev[0], init)
    return lsi.lidar_to_center(*lsi.planar(T)), fit, rmse


def process(name: str, subs: list[str], out: Path) -> dict:
    li, od = session(name, "li"), session(name, "odom")
    remote = REMOTE_DIR / f"odom_1021_{name}_remote.log"
    tr = load_odom(od)
    t0 = float(tr.recv_s[0])
    segs = still_segments(tr, remote_events(remote if remote.exists() else None, t0))
    segs_abs = [(a + t0, b + t0) for a, b in segs]
    res: dict = {"名前": name, "remote.log": remote.exists(), "区間 [s]": [[round(a, 1), round(b, 1)] for a, b in segs]}
    if not segs:
        res["注"] = "止まっている区間が無い"
        return res
    clouds = lsi.segment_clouds(li, segs_abs)
    R, h = dla.level([clouds[0][::3]])
    L = np.eye(4)
    L[:3, :3], L[2, 3] = R, h
    lev = [c @ R.T + np.array([0.0, 0.0, h]) for c in clouds]
    lr, _ = lidar_times(li)
    res["ライダーの高さ [m]"] = round(float(h), 3)
    odo = odom_rel(tr, segs)
    poses: dict[str, list] = {"odom": odo}
    for sub in subs:
        fl = fastlio_track(li, sub, lr)
        if fl is None:
            res.setdefault("FAST_LIO が無い", []).append(sub)
            continue
        res[f"{sub} の確かめ"] = fl["check"]
        seg_T, jit = zip(*[segment_pose(fl, a, b, L) for a, b in segs_abs])
        res[f"{sub} の区間内の揺れ"] = list(jit)
        if len(segs) == 1 and segs[0][1] - segs[0][0] > 0.8 * (tr.recv_s[-1] - t0):
            res[f"{sub} の静止"] = static_report(fl, L, *segs_abs[0])
        ref = seg_T[0] if seg_T[0] is not None else fl["T"][np.searchsorted(fl["stamp"], fl["stamp"][0] + WARMUP_S)]
        save_track(out, name, sub, fl, L, ref, t0, segs_abs)
        if seg_T[0] is not None:
            poses[sub] = [to_center(np.linalg.inv(seg_T[0]) @ T, L) if T is not None else np.full(3, np.nan)
                          for T in seg_T]
    if len(segs) < 2:
        return res
    icp_pose, rows = [np.zeros(3)], []
    for k in range(1, len(segs)):
        truth, fit, rmse = run_icp(lev, k, init_from_center(odo[k]))
        row = {"区間": k, "icp": r4(truth), "fitness": round(fit, 3), "rmse_cm": round(rmse * 100, 2)}
        fl_sub = next((s for s in subs if s in poses), None)
        if fl_sub is not None and np.isfinite(poses[fl_sub][k]).all():
            alt, _, _ = run_icp(lev, k, init_from_center(poses[fl_sub][k]))
            row["icp の初期値依存（odom 初期値 − FAST_LIO 初期値）"] = [round(float(np.hypot(*(truth - alt)[:2])) * 1000, 1),
                                                         round(math.degrees(float(wrap(truth[2] - alt[2]))), 3)]
        for src, ps in poses.items():
            d = ps[k] - truth
            row[src] = r4(ps[k])
            row[f"{src} − icp [cm,cm,°]"] = [round(float(d[0]) * 100, 1), round(float(d[1]) * 100, 1),
                                             round(math.degrees(float(wrap(d[2]))), 2)]
        icp_pose.append(truth)
        rows.append(row)
    res["区間ごと（区間 0 の robot_center から見た 前・左 m・向き rad）"] = rows
    res["隣どうし"] = legs(name, icp_pose, poses)
    # 正解のばらつき: 隣どうしを直接重ねた ICP と、区間 0 を経由した ICP の差（区間 0→1 は同じものなので 1→2 から）
    for leg in res["隣どうし"][1:]:
        k = leg["区間"][1]
        chained = se2_rel(icp_pose[k - 1], icp_pose[k])
        T, fit, _ = lsi.icp(lev[k], lev[k - 1], init_from_center(chained))
        direct = lsi.lidar_to_center(*lsi.planar(T))
        leg["正解の再現（直接の ICP − 区間 0 経由）[cm,°]"] = [round(float(np.hypot(*(direct - chained)[:2])) * 100, 2),
                                                round(math.degrees(wrap(direct[2] - chained[2])), 3), round(fit, 3)]
    return res


def legs(name: str, icp_pose: list[np.ndarray], poses: dict[str, list]) -> list[dict]:
    out = []
    ahead = np.array([FRAME_AHEAD_M, 0.0])
    for k in range(1, len(icp_pose)):
        dt = se2_rel(icp_pose[k - 1], icp_pose[k])
        d_icp = float(np.hypot(*dt[:2]))
        kind = "旋回" if abs(math.degrees(dt[2])) > TURN_DEG else ("横" if abs(dt[1]) > abs(dt[0]) else "前後")
        leg = {"記録": name, "区間": [k - 1, k], "種類": kind, "icp の距離 [m]": round(d_icp, 4),
               "icp の回転 [°]": round(math.degrees(dt[2]), 2)}
        q = se2_apply(dt, ahead)  # 着いた先で 1 m 前にある点（出発の座標）
        for src, ps in poses.items():
            p0, p1 = ps[k - 1], ps[k]
            if not (np.isfinite(p0).all() and np.isfinite(p1).all()):
                continue
            ds = se2_rel(p0, p1)
            e = float(np.hypot(*(ds[:2] - dt[:2])))
            carried = se2_apply(se2_inv(ds), q)
            leg[src] = {"距離 [m]": round(float(np.hypot(*ds[:2])), 4), "回転 [°]": round(math.degrees(ds[2]), 2),
                        "位置の差 [cm]": round(e * 100, 2), "向きの差 [°]": round(math.degrees(wrap(ds[2] - dt[2])), 2),
                        "距離の比": round(float(np.hypot(*ds[:2])) / d_icp, 4) if d_icp >= MIN_LEG_M else None,
                        "回転の比": round(ds[2] / dt[2], 4) if abs(math.degrees(dt[2])) > 10 else None,
                        f"{FRAME_AHEAD_M:.0f} m 先の点の差 [cm]": round(float(np.hypot(*(carried - ahead))) * 100, 2)}
        out.append(leg)
    return out


# ── まとめと表示 ───────────────────────────────────────────────────
def summarize(all_legs: list[dict], srcs: list[str]) -> dict:
    groups: dict[str, list[dict]] = {}
    for leg in all_legs:
        g = "08b" if leg["記録"] == "08b" else leg["種類"]
        groups.setdefault(g, []).append(leg)
    out = {}
    for g, ls in groups.items():
        row = {"区間の数": len(ls), "icp の距離の和 [m]": round(sum(x["icp の距離 [m]"] for x in ls), 3)}
        for src in srcs:
            have = [x for x in ls if src in x]
            if not have:
                continue
            tr = [x for x in have if x["icp の距離 [m]"] >= MIN_LEG_M]
            e = [x[src]["位置の差 [cm]"] for x in have]
            row[src] = {
                "位置の差 [cm] 平均・最大": [round(float(np.mean(e)), 2), round(float(np.max(e)), 2)],
                "1 m あたりの位置の差 [cm/m]（和÷和）": (round(sum(x[src]["位置の差 [cm]"] for x in tr)
                                                   / sum(x["icp の距離 [m]"] for x in tr), 2) if tr else None),
                "距離の比 [各区間]": [x[src]["距離の比"] for x in tr],
                "向きの差 [°] 各区間": [x[src]["向きの差 [°]"] for x in have],
                "回転の比 [各区間]": [x[src]["回転の比"] for x in have if x[src]["回転の比"] is not None],
                f"{FRAME_AHEAD_M:.0f} m 先の点の差 [cm] 平均・最大": [
                    round(float(np.mean([x[src][f"{FRAME_AHEAD_M:.0f} m 先の点の差 [cm]"] for x in have])), 2),
                    round(float(np.max([x[src][f"{FRAME_AHEAD_M:.0f} m 先の点の差 [cm]"] for x in have])), 2)],
            }
        out[g] = row
    return out


def print_result(res: dict, srcs: list[str]) -> None:
    print(f"\n== {res['名前']}  区間 {res['区間 [s]']}")
    for key, val in res.items():
        if key.endswith("の確かめ") or key.endswith("の静止") or key == "注" or key == "FAST_LIO が無い":
            print(f"  {key}: {val}")
    rows = res.get("区間ごと（区間 0 の robot_center から見た 前・左 m・向き rad）", [])
    if rows:
        print("  区間   ICP 前・左 m・向き°          " + "  ".join(f"{s:>26}" for s in srcs) + "   fit・rmse  ICP初期値依存")
    for r in rows:
        ic = r["icp"]
        line = f"  0→{r['区間']}  {ic[0]:+.3f} {ic[1]:+.3f} {math.degrees(ic[2]):+7.2f}°  "
        for s in srcs:
            if s in r:
                d = r[f"{s} − icp [cm,cm,°]"]
                line += f"  {r[s][0]:+.3f} {r[s][1]:+.3f}（差 {d[0]:+5.1f} {d[1]:+5.1f} cm {d[2]:+5.2f}°）"
            else:
                line += f"  {'—':>26}"
        line += f"   {r['fitness']:.2f}・{r['rmse_cm']:.1f}cm  {r.get('icp の初期値依存（odom 初期値 − FAST_LIO 初期値）', '—')}"
        print(line)
    for leg in res.get("隣どうし", []):
        line = f"  {leg['区間'][0]}→{leg['区間'][1]} {leg['種類']:<3} ICP {leg['icp の距離 [m]']:.3f} m {leg['icp の回転 [°]']:+7.2f}°"
        for s in srcs:
            if s in leg:
                v = leg[s]
                line += (f" | {s}: 差 {v['位置の差 [cm]']:.1f} cm {v['向きの差 [°]']:+.2f}° 比 {v['距離の比']}"
                         f" 1m先 {v['1 m 先の点の差 [cm]']:.1f} cm")
        rep = leg.get("正解の再現（直接の ICP − 区間 0 経由）[cm,°]")
        if rep:
            line += f" | 正解の再現 {rep[0]:.1f} cm {rep[1]:+.3f}°（fit {rep[2]:.2f}）"
        print(line)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--names", nargs="+", default=NAMES_DEFAULT)
    ap.add_argument("--subs", nargs="+", default=SUBS_DEFAULT, help="runs/<セッション>/ の下の FAST_LIO の出力の名前")
    ap.add_argument("--out", type=Path, default=OUT_DEFAULT)
    ap.add_argument("--fl-window", choices=["all", "tail3", "head3"], default="all", help="FAST_LIO の区間の窓")
    a = ap.parse_args()
    global FL_WINDOW
    FL_WINDOW = a.fl_window
    unknown = [s for s in a.subs if s not in EXTRINSIC_T]
    if unknown:
        raise SystemExit(f"[fl] 外部パラメータを知らない出力: {unknown}")
    a.out.mkdir(parents=True, exist_ok=True)
    srcs = ["odom"] + a.subs
    results, all_legs = [], []
    for name in a.names:
        res = process(name, a.subs, a.out)
        print_result(res, srcs)
        results.append(res)
        all_legs += res.get("隣どうし", [])
    summary = summarize(all_legs, srcs)
    print("\n== まとめ（種類ごと。08b は別）")
    for g, row in summary.items():
        print(f"  {g}: {json.dumps(row, ensure_ascii=False)}")
    (a.out / "result.json").write_text(json.dumps({"設定": {"WARMUP_S": WARMUP_S, "EDGE_S": lsi.EDGE_S, "FL_WINDOW": FL_WINDOW,
                                                            "LIVOX_XY": list(lsi.LIVOX_XY), "外部パラメータ": EXTRINSIC_T},
                                                   "記録": results, "まとめ": summary}, ensure_ascii=False, indent=1))
    print(f"[fl] 書いた: {a.out / 'result.json'}")


if __name__ == "__main__":
    main()
