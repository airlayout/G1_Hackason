"""11-11: 段の走らせ役（door_run_stock.py --staged）の 1 回を、3 つの目で並べた動画にする。OMEN の CPU（numpy・matplotlib・cv2）。

  左   ライダー: 生の点群を床で水平にし、FAST-LIO の口の姿勢で扉の座標に置いて上から見る（扉が上・把手の側が左＝機体の後ろから見た向き）。
       色は床からの高さ。扉の板・レバー・シムの脇の壁の面・pelvis・左の手のひらと指先（順運動学・過去 1 秒の跡）を重ねる
  右上 頭のカラー（網 c2 に入る 192×108）
  右下 深度（網に入る 96×54・0〜2.5 m）
時刻: 走らせ役と網は PC2 の同じ単調時計。点群は口の電文のライダーの時刻で、その時点で機体が受け取っていた最後の回を出す。

  python render_staged_views_1111.py --stock door_stock_*.npz --vision vision_*.npz --session mapping_runs/<..._li> \\
      --fastlio fastlio_node_*.npz --out out.mp4 [--from -2 --to 40]（DOOR の始めからの秒）[--urdf g1_29dof_rev_1_0.urdf]
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import sqlite3
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_frame as dfr  # noqa: E402
import g1_arm_fk as fk  # noqa: E402
import lidar_door_est as est  # noqa: E402
from crop_lidar_bag import LIDAR_TOPIC, scan_xyz  # noqa: E402

PHASES = ("WAIT", "ARMS_IN", "READY", "DOOR", "POST", "ARMS_OUT", "DAMP", "PROBE")
STAGE_NAME = {0: "段 0 測る", 1: "段 1 構えの位置へ", 2: "段 2 腕を構える／腕だけの方策", 3: "段 3 引き継ぎ"}
HALF_W, AXIS_Y, LEVER_END_Y, LEVER_X = 0.47, 0.406, 0.276, -0.075
SIM_WALL_X = -0.07
Z_RANGE = (0.15, 1.9)
YAW_ONSET_RAD_S = math.radians(30.0)
ACCUM_S = 0.45  # 点群を重ねる長さ（5 回）


def header_stamp(payload: bytes) -> float:
    """CDR（リトルエンディアン）の std_msgs/Header の時刻 [s]"""
    sec = int.from_bytes(payload[4:8], "little", signed=True)
    nsec = int.from_bytes(payload[8:12], "little", signed=False)
    return sec + 1e-9 * nsec


def load_scans(session: Path, lo: float, hi: float) -> List[Tuple[float, np.ndarray]]:
    """PC2 の時計の lo〜hi に受け取った回を (PC2 の時計の時刻, 点) で。
    ⚠️ 11-11 夕に直した: 前はライダーのヘッダの時刻（ライダーの時計・11-11 は PC2 より約 1.93 s 遅れ）で選び、その時刻で口の姿勢（PC2 の時計）を
    引いていたので、点群を約 1.9 s 前の姿勢で置いていた（段 0 の窓では REF の前の座標・T3 の再生で分かった）。
    ヘッダの時刻から、最初の回で見積もった差 d（ヘッダ − 袋の受け取りの時刻）を引いて PC2 の時計に直す"""
    out = []
    for bag in sorted((session / "raw" / "rosbag2").glob("*.db3")):
        con = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
        try:
            tid = con.execute("SELECT id FROM topics WHERE name = ?", (LIDAR_TOPIC,)).fetchone()[0]
            r0 = con.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp LIMIT 1", (tid,)).fetchone()
            d = header_stamp(r0[1]) - r0[0] / 1e9
            q = "SELECT data FROM messages WHERE topic_id = ? AND timestamp BETWEEN ? AND ? ORDER BY timestamp"
            for (payload,) in con.execute(q, (tid, int((lo - 2) * 1e9), int((hi + 2) * 1e9))):
                s = header_stamp(payload) - d  # PC2 の時計へ
                if lo <= s <= hi:
                    out.append((s, scan_xyz(payload)))
        finally:
            con.close()
    return out


def stock_arrays(path: str) -> Dict[str, np.ndarray]:
    z = dict(np.load(path, allow_pickle=True))
    pose = z["pose"].copy()
    miss = ~np.isfinite(pose).all(axis=1)
    pose[miss] = z["lidar_msg"][miss][:, 1:4]
    z["pose_f"] = pose
    return z


def events(z: Dict[str, np.ndarray]) -> Dict[str, Optional[float]]:
    t, ph, st = z["t"], z["phase"], z["staged"]
    door = np.flatnonzero(ph == PHASES.index("DOOR"))
    t_door = float(t[door[0]])
    ev: Dict[str, Optional[float]] = {"t_door_mono": t_door}
    stage = st[:, 0]
    for s in (1, 2, 3):
        k = np.flatnonzero(np.isfinite(stage) & (stage == s))
        ev[f"stage{s}"] = float(t[k[0]] - t_door) if len(k) else None
    pol = np.flatnonzero(np.isfinite(stage) & (stage == 2) & (st[:, 2] == 2))
    ev["stage2_policy"] = float(t[pol[0]] - t_door) if len(pol) else None
    post = np.flatnonzero(ph == PHASES.index("POST"))
    ev["post"] = float(t[post[0]] - t_door) if len(post) else None
    ev["yaw_onset"] = None
    if len(pol):
        k = np.flatnonzero((np.arange(len(t)) > pol[0]) & (np.abs(z["gyro"][:, 2]) > YAW_ONSET_RAD_S))
        ev["yaw_onset"] = float(t[k[0]] - t_door) if len(k) else None
    return ev


POSE_MATCH_S = 0.05  # 点群のヘッダの時刻と口の記録（FAST-LIO の出力の時刻）は数〜17 ms ずれる


def nearest_pose(rows: np.ndarray, row_t: np.ndarray, stamp: float) -> Optional[Tuple[float, float, float]]:
    """点群の時刻に一番近い口の姿勢（POSE_MATCH_S 以内）。無ければ None"""
    j = int(np.searchsorted(row_t, stamp))
    best = min((i for i in (j - 1, j) if 0 <= i < len(row_t)), key=lambda i: abs(row_t[i] - stamp), default=None)
    if best is None or abs(row_t[best] - stamp) > POSE_MATCH_S:
        return None
    return float(rows[best, 1]), float(rows[best, 2]), float(rows[best, 3])


def lidar_points_door(xyz: np.ndarray, level: est.Level, pose: Tuple[float, float, float]) -> np.ndarray:
    q = level.apply(est.valid_points(xyz))
    c, s = math.cos(pose[2]), math.sin(pose[2])
    xy = q[:, :2] @ np.array([[c, s], [-s, c]]) + np.array(pose[:2])
    return np.column_stack([xy, q[:, 2]])


def arm_points(chain: fk.ArmFK, q: np.ndarray, quat: np.ndarray, pose: np.ndarray, pelvis_z: float) -> Dict[str, np.ndarray]:
    roll, pitch = dfr.roll_pitch_from_quat_wxyz(quat)
    R = dfr.rot_z(pose[2]) @ dfr.tilt_matrix(roll, pitch)
    base = np.array([pose[0], pose[1], pelvis_z])
    pts = chain.points(q)
    return {k: base + R @ pts[k] for k in ("L指先", "L手", "L肘", "L肩", "R肩")}


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stock", required=True)
    ap.add_argument("--vision", required=True)
    ap.add_argument("--session", required=True)
    ap.add_argument("--fastlio", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--from", dest="t_from", type=float, default=-2.0)
    ap.add_argument("--to", dest="t_to", type=float, default=None)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--pelvis-z", type=float, default=0.73)
    ap.add_argument("--title", default="")
    a = ap.parse_args(argv)

    import cv2
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = "Noto Sans CJK JP"

    z = stock_arrays(a.stock)
    ev = events(z)
    t, t_door = z["t"], ev["t_door_mono"]
    t_end = a.t_to if a.t_to is not None else float(t[-1] - t_door)
    frames = t_door + np.arange(a.t_from, t_end, 1.0 / a.fps)
    lm = z["lidar_msg"]
    ok = np.isfinite(lm[:, 0])
    off = float(np.median(lm[ok, 0] - t[ok]))  # ライダーの時刻 − 単調時計（受け取りまでの遅れを含む）
    rows = np.load(a.fastlio)["rows"]
    row_t = rows[:, 0]
    scans = load_scans(Path(a.session), frames[0] + off - 1.0, frames[-1] + off)
    stamps = [s for s, _ in scans]
    print(f"[render] 回 {len(scans)}・フレーム {len(frames)}・時計の差 {off:.3f} s", flush=True)
    # 床: DOOR の始めの止まっている 1 秒の回で当て、以後は回ごとに当て直す（だめなら前の値）
    still = [x for s, x in scans if t_door + off <= s <= t_door + off + 1.0] or [x for _, x in scans[:10]]
    level = est.level_from_scans(still)
    levels: List[est.Level] = []
    for _, xyz in scans:
        nxt = est.relevel(xyz, level)
        level = nxt if nxt is not None else level
        levels.append(level)
    vz = np.load(a.vision, allow_pickle=True)
    vt, vtd = vz["t_capture"], vz["t_depth"]
    chain = fk.load_chain(a.urdf)
    st = z["staged"]
    k_pol = None if ev["stage2_policy"] is None else int(np.searchsorted(t, t_door + ev["stage2_policy"]))

    W, H = 1280, 720
    vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (W, H))
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
    trail: List[Tuple[float, np.ndarray, np.ndarray]] = []
    for n, tf in enumerate(frames):
        fig.clf()
        ax = fig.add_axes([0.03, 0.06, 0.52, 0.86])
        k = int(min(len(t) - 1, max(0, np.searchsorted(t, tf))))
        # 点群
        j = bisect.bisect_right(stamps, tf + off) - 1
        note = ""
        if j >= 0:
            # 直近 ACCUM_S 秒の回を重ねる（1 回 0.1 秒の MID-360 は疎い）。どの回も、その回の姿勢で扉の座標に置く
            Ps = []
            for i in range(j, -1, -1):
                if stamps[j] - stamps[i] > ACCUM_S:
                    break
                pz = nearest_pose(rows, row_t, stamps[i])
                if pz is not None:
                    Ps.append(lidar_points_door(scans[i][1], levels[i], pz))
            if Ps:
                P = np.concatenate(Ps)
                m = (P[:, 2] > Z_RANGE[0]) & (P[:, 2] < Z_RANGE[1])
                ax.scatter(-P[m, 1], P[m, 0], c=P[m, 2], s=3.0, cmap="viridis", vmin=Z_RANGE[0], vmax=Z_RANGE[1], linewidths=0)
            else:
                note = "（FAST-LIO の口の姿勢がまだ無い＝段 0 の REF の直後）"
        # 扉・レバー・シムの壁
        ax.fill_between([-HALF_W, HALF_W], [-0.02, -0.02], [0.02, 0.02], color="#555", alpha=0.6, lw=0)
        ax.plot([-AXIS_Y, -LEVER_END_Y], [LEVER_X, LEVER_X], color="#d62728", lw=4, solid_capstyle="round")
        ax.plot([-AXIS_Y], [LEVER_X], "o", color="#d62728", ms=6)
        for sgn in (-1, 1):
            ax.plot([sgn * HALF_W, sgn * 1.0], [SIM_WALL_X, SIM_WALL_X], "--", color="#888", lw=1.2)
            ax.plot([sgn * HALF_W, sgn * HALF_W], [-0.1, 0.1], color="#333", lw=1)
        ax.text(-0.98, SIM_WALL_X + 0.02, "シムの脇の壁の面", fontsize=8, color="#666")
        ax.text(-0.43, 0.04, "レバー", fontsize=9, color="#d62728")
        ax.text(0.1, 0.04, "扉の板（閉）", fontsize=9, color="#333")
        # 機体
        pose = z["pose_f"][k]
        if np.isfinite(pose).all():
            c, s = math.cos(pose[2]), math.sin(pose[2])
            ax.annotate("", xy=(-(pose[1] + 0.18 * s), pose[0] + 0.18 * c), xytext=(-pose[1], pose[0]),
                        arrowprops=dict(arrowstyle="->", color="#1f77b4", lw=2))
            ap_ = arm_points(chain, z["q"][k], z["quat"][k], pose, a.pelvis_z)
            ax.plot([-ap_["L肩"][1], -ap_["R肩"][1]], [ap_["L肩"][0], ap_["R肩"][0]], color="#1f77b4", lw=3)
            trail.append((tf, ap_["L手"], ap_["L指先"]))
            trail = [r for r in trail if tf - r[0] <= 1.0]
            hy = np.array([-r[1][1] for r in trail]), np.array([r[1][0] for r in trail])
            ty = np.array([-r[2][1] for r in trail]), np.array([r[2][0] for r in trail])
            ax.plot(*hy, color="#ff7f0e", lw=1.5, alpha=0.8)
            ax.plot(*ty, color="#e377c2", lw=1.0, alpha=0.8)
            ax.plot(-ap_["L肘"][1], ap_["L肘"][0], "s", color="#2ca02c", ms=5)
            ax.plot(-ap_["L手"][1], ap_["L手"][0], "o", color="#ff7f0e", ms=7)
            ax.plot(-ap_["L指先"][1], ap_["L指先"][0], "o", color="#e377c2", ms=5)
            hz = ap_["L手"][2]
        else:
            hz = float("nan")
        ax.set_xlim(-1.05, 0.75)
        ax.set_ylim(-1.45, 0.35)
        ax.set_aspect("equal")
        ax.set_xticks(np.arange(-1.0, 0.8, 0.25))
        ax.set_xticklabels([f"{-v:.2f}" for v in np.arange(-1.0, 0.8, 0.25)], fontsize=7)
        ax.tick_params(labelsize=7)
        ax.set_xlabel("扉に沿った位置 [m]（左＝把手の側）", fontsize=8)
        ax.set_ylabel("扉からの前後 [m]（上＝扉）", fontsize=8)
        ax.grid(alpha=0.25)
        ax.set_title("ライダー（上から・扉の座標・直近 0.5 秒・色は床からの高さ 0.15〜1.9 m）" + note, fontsize=9)
        # カラー・深度
        iv = int(np.searchsorted(vt, tf, side="right") - 1)
        ax2 = fig.add_axes([0.58, 0.50, 0.40, 0.40])
        if iv >= 0:
            ax2.imshow(vz["rgb"][iv])
        ax2.set_axis_off()
        ax2.set_title("頭のカラー（網に入る 192×108）", fontsize=9)
        idp = int(np.searchsorted(vtd, tf, side="right") - 1)
        ax3 = fig.add_axes([0.58, 0.05, 0.40, 0.40])
        if idp >= 0:
            im = ax3.imshow(vz["depth"][idp].astype(np.float32), cmap="turbo", vmin=0.0, vmax=2.5)
            cb = fig.colorbar(im, ax=ax3, fraction=0.03, pad=0.01)
            cb.ax.tick_params(labelsize=7)
        ax3.set_axis_off()
        ax3.set_title("深度（網に入る 96×54・0〜2.5 m・黒は抜け）", fontsize=9)
        # 見出し
        ph = PHASES[int(z["phase"][k])]
        sg = st[k, 0]
        stage = STAGE_NAME.get(int(sg), "") if np.isfinite(sg) else ""
        dyaw = ""
        if k_pol is not None and k >= k_pol and np.isfinite(z["pose_f"][k_pol]).all() and np.isfinite(pose).all():
            dyaw = f"・段 2 の始めからの体の回り {math.degrees(pose[2] - z['pose_f'][k_pol][2]):+.1f}°"
        fig.text(0.03, 0.955, f"{a.title}　DOOR から {tf - t_door:+.1f} s　{ph} {stage}{dyaw}"
                              f"・回る速さ {math.degrees(z['gyro'][k, 2]):+.0f}°/s・左手の高さ {hz:.2f} m", fontsize=10)
        fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        vw.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        if n % 50 == 0:
            print(f"[render] {n}/{len(frames)}", flush=True)
    vw.release()
    Path(a.out).with_suffix(".json").write_text(json.dumps(ev, ensure_ascii=False, indent=1))
    print(f"[render] 書いた: {a.out}・{json.dumps(ev, ensure_ascii=False)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
