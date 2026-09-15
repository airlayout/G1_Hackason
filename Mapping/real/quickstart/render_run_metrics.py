#!/usr/bin/env python3
"""**記録 1 本を「測位がどう壊れるか」の動画にする。**候補を歩行で比べるため。

    Navigation/.venv/bin/python quickstart/render_run_metrics.py \\
        Mapping/real/runs/click4_20260913 --out docs/作業ログ/video/click4.mp4 --fast 6

## なぜ要るのか

静止の 5 項目は表で足りるが、**歩行と旋回の壊れ方は表では伝わらない**。
「推定が地図の上をどう滑るか」「上限超がどの区間で出るか」は絵で見た方が早い。
09-08 に「画面録画より計測から起こした動画の方が伝わる」と決めてある。

## 何を描くか

- 左: 事前地図 ＋ **推定の軌跡**（`map -> base_link` を鎖で辿る）＋ その時刻のスキャン
      ＋ **脚 odom の軌跡**（独立な参照。開始時刻で合わせるだけで、以後は補正しない）
- 右: 見かけの速さ（`velocity_smoother` の hypot **0.361 m/s** の線つき）／
      ジャイロの回転レート／累積変位の突き合わせ

⚠️ **脚 odom は真値ではない。** 独立で、静止 60 s で 2 mm という素性の良い参照というだけ。
⚠️ **早送りは画面に出す**（出さないと「機体が速い」と読み違える）。
⚠️ 日本語の等幅フォントは無い。`Hiragino Sans` を使う（09-08 に踏んだ）。
"""
from __future__ import annotations

import argparse
import math
import sqlite3
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_traj import read_static_tf, resample_hz, to_base_link  # noqa: E402
from measure_overlay import Cdr, read_bag, read_map  # noqa: E402
from tf_chain import resolve  # noqa: E402

plt.rcParams["font.family"] = ["Hiragino Sans", "DejaVu Sans"]
SPEED_LIMIT = 0.361
FPS = 10
INK, INK2, INK3 = "#111620", "#46505f", "#6b7686"
EST, ODO, BAD, SCAN = "#0f6fc4", "#14714a", "#a8202c", "#b1500f"


def yaw_of(q):
    x, y, z, w = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def read_odom(con, tid):
    """脚 odom を (t, x, y, yaw[deg]) で返す。⚠️ child は robot_center。"""
    if "/dog_odom" not in tid:
        return None
    rows = []
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?", (tid["/dog_odom"],)):
        r = Cdr(blob)
        sec, nsec = r.i32(), r.u32()
        r.st(); r.st()
        x, y, _z = (r.f64() for _ in range(3))
        q = [r.f64() for _ in range(4)]
        rows.append([sec + nsec * 1e-9, x, y, yaw_of(q)])
    return np.array(rows)


def read_gyro(con, tid):
    """IMU の角速度の大きさを (t, |w|[deg/s]) で返す。"""
    key = "/utlidar/imu_livox_mid360"
    if key not in tid:
        return None
    rows = []
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?", (tid[key],)):
        r = Cdr(blob)
        sec, nsec = r.i32(), r.u32()
        r.st()
        [r.f64() for _ in range(4)]          # orientation
        [r.f64() for _ in range(9)]          # orientation_covariance
        wx, wy, wz = (r.f64() for _ in range(3))
        rows.append([sec + nsec * 1e-9, math.degrees(math.hypot(math.hypot(wx, wy), wz))])
    return np.array(rows)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--ref", type=Path, default=None)
    ap.add_argument("--fast", type=int, default=4, help="何倍速にするか（スキャンの間引き）")
    ap.add_argument("--mark", default=None, metavar="A:B", help="強調する区間 [s]")
    ap.add_argument("--zoom", type=float, default=0.8, help="拡大窓の半幅 [m]")
    ap.add_argument("--traj", type=Path, default=None,
                    help="推定の軌跡を別の bag から読む（再生の出力）。"
                         "⚠️ 再生の出力に LiDAR を録り直すと 1 本 800 MB になるので、"
                         "スキャンは元の記録から、軌跡だけこちらから読む")
    ap.add_argument("--label", default=None, help="見出しに出す候補の名前")
    ap.add_argument("--traj-frame", default="base_link", choices=("base_link", "livox"),
                    help="livox = 軌跡が `map -> livox_frame`（**GLIM がこれ**）。"
                         "掛け忘れるとスキャンに取付が**二重に**かかって絵だけ壊れる。"
                         "⚠️ 2026-09-15 実測: 掛け忘れると GLIM の base_link が "
                         "z +1.278 m・roll -178.8°（＝ LiDAR の取付そのもの）で入り、"
                         "重畳が 85.5%% → 50.1%% に化ける")
    ap.add_argument("--hz", type=float, default=10.0,
                    help="推定の軌跡をこのレートへ揃えてから描く（0 = そのまま）。"
                         "⚠️ 候補どうしで揃えないと見かけの速さが比べられない。"
                         "MOLA の鎖は 10 Hz、AMCL の鎖は脚 odom の 140 Hz で出る"
                         "（eval_traj.resample_hz の注記）")
    a = ap.parse_args()

    bag = a.run / "bag" if (a.run / "bag").is_dir() else a.run
    ref = a.ref or (Path(__file__).resolve().parents[1]
                    / "runs/20260906T135940_UiS_room_v3/map/nav_map_ref.yaml")
    con = sqlite3.connect(f"file:{next(bag.glob('*.db3'))}?mode=ro", uri=True)
    tid = {n: i for i, n in con.execute("SELECT id,name FROM topics")}

    if a.traj is not None:
        # ⚠️ **再生の出力は候補ごとに置き場が違う**（AMCL / FAST_LIO は `tf_bag/` の下、
        #    GLIM は直下）。直下と `bag/` だけを見る実装は**黙って StopIteration で死ぬ**。
        #    2026-09-15 に 4 本の動画を取りこぼした。再帰で探す。
        tdb = next(iter(sorted(a.traj.rglob("*.db3"))), None)
        if tdb is None:
            print(f"⛔ {a.traj} に .db3 が無い")
            return 3
        tcon = sqlite3.connect(f"file:{tdb}?mode=ro", uri=True)
        ttid = {n: i for i, n in tcon.execute("SELECT id,name FROM topics")}
        est, desc = resolve(tcon, ttid)
    else:
        est, desc = resolve(con, tid)
    if a.traj_frame == "livox":
        est = to_base_link(est, read_static_tf(con, tid))
    est = resample_hz(est, a.hz)
    if len(est) < 2:
        print(f"⛔ {desc}")
        return 3
    _tfs, scans, sensor_tf = read_bag(bag)
    odo, gyro = read_odom(con, tid), read_gyro(con, tid)
    occ, res, ox, oy = read_map(ref)[:4]
    h, w = occ.shape

    t0 = est[0, 0]
    te = est[:, 0] - t0
    xy = est[:, 1:3]
    dt = np.diff(te); ok = dt > 1e-3
    spd = np.hypot(*np.diff(xy, axis=0).T)[ok] / dt[ok]
    spd_t = te[1:][ok]

    # 脚 odom を開始姿勢で推定に合わせる（以後は補正しない＝独立な参照）
    oxy = None
    if odo is not None:
        ot = odo[:, 0] - t0
        k0 = int(np.argmin(np.abs(ot)))
        c, s = math.cos(math.radians(yaw_of(est[0, 4:8]) - odo[k0, 3])), \
               math.sin(math.radians(yaw_of(est[0, 4:8]) - odo[k0, 3]))
        d = odo[:, 1:3] - odo[k0, 1:3]
        oxy = np.stack([xy[0, 0] + c * d[:, 0] - s * d[:, 1],
                        xy[0, 1] + s * d[:, 0] + c * d[:, 1]], 1)

    mark = None
    if a.mark:
        mark = tuple(float(v) for v in a.mark.split(":"))

    cum_e = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    sel = list(range(0, len(scans), max(1, a.fast)))
    ys, xs = np.nonzero(occ)
    MX = ox + (xs + 0.5) * res
    MY = oy + (h - 1 - ys + 0.5) * res
    pad = 6.0
    xlim = (xy[:, 0].min() - pad, xy[:, 0].max() + pad)
    ylim = (xy[:, 1].min() - pad, xy[:, 1].max() + pad)

    # ⚠️ **震えは全体図では見えない。** click4 は t=66 s で経路長 8.75 m に対し正味 0.20 m——
    #    0.2 m の中に 8.75 m 分の線が固まるので、点に埋もれる。拡大窓が要る。
    ZOOM = a.zoom       # 拡大窓の半幅 [m]
    ZOOM_S = 8.0        # 拡大窓に出す直近の秒数

    fig = plt.figure(figsize=(14.4, 8.1))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.30, 1.0],
                          height_ratios=[1.0, 1.0, 1.25], hspace=0.45, wspace=0.20,
                          left=0.055, right=0.985, top=0.885, bottom=0.075)
    axm = fig.add_subplot(gs[0:2, 0])
    azi = fig.add_subplot(gs[2, 0])          # 拡大窓は独立させる（地図に重ねると読めない）
    ax1, ax2, ax3 = (fig.add_subplot(gs[i, 1]) for i in range(3))
    head = f"{a.run.name} — {a.label} と脚 odom の突き合わせ" if a.label else \
           f"{a.run.name} — 推定と脚 odom の突き合わせ"
    fig.text(0.05, 0.955, head, fontsize=18, color=INK, weight="bold", va="center")
    sub = fig.text(0.05, 0.915, "", fontsize=12, color=INK2, va="center")

    # ⚠️ **`-movflags +faststart` を必ず付ける。** 付けないと `moov` アトムが
    #    ファイル末尾に置かれ、`<video preload="none">` のブラウザが読み込めない
    #    （2026-09-15 に踏んだ。既存の動画は moov が 36 バイト目、こちらは末尾だった）。
    #    `-pix_fmt yuv420p` も同様に、付けないと再生できない環境がある。
    writer = FFMpegWriter(fps=FPS, bitrate=3600, metadata={"title": a.run.name},
                          extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
    a.out.parent.mkdir(parents=True, exist_ok=True)
    print(f"[render] {len(sel)} 枚 / {a.fast} 倍速 -> {a.out}")
    with writer.saving(fig, str(a.out), dpi=100):
        for n, i in enumerate(sel):
            ts, raw = scans[i]
            now = ts - t0
            k = int(np.argmin(np.abs(est[:, 0] - ts)))
            px, py, pyaw = est[k, 1], est[k, 2], yaw_of(est[k, 4:8])

            axm.clear()
            axm.scatter(MX, MY, s=1.6, c="#9aa5b4", marker=".", linewidths=0, zorder=1)
            axm.plot(xy[:k + 1, 0], xy[:k + 1, 1], "-", color=EST, lw=2.0,
                     label=(a.label or "推定") + "（map→base_link）", zorder=5)
            if oxy is not None:
                kk = int(np.argmin(np.abs(ot - now)))
                axm.plot(oxy[:kk + 1, 0], oxy[:kk + 1, 1], "--", color=ODO, lw=1.7,
                         label="脚 odom（独立な参照）", zorder=4)
            p = raw[np.isfinite(raw).all(1)]
            if sensor_tf is not None:
                from scipy.spatial.transform import Rotation
                st, sq = sensor_tf
                p = p @ Rotation.from_quat(sq).as_matrix().T + st
            p = p[(p[:, 2] > 1.30) & (p[:, 2] < 1.82)]
            if len(p):
                cc, ss = math.cos(math.radians(pyaw)), math.sin(math.radians(pyaw))
                axm.scatter(px + cc * p[:, 0] - ss * p[:, 1], py + ss * p[:, 0] + cc * p[:, 1],
                            s=1.1, c=SCAN, marker=".", linewidths=0, alpha=0.75,
                            label="そのときのスキャン（壁の帯）", zorder=6)
            axm.plot([px], [py], "o", color=EST, ms=9, mec="white", mew=1.6, zorder=8)
            axm.set_xlim(*xlim); axm.set_ylim(*ylim); axm.set_aspect("equal")
            axm.set_xlabel("map x [m]", fontsize=10, color=INK3)
            axm.set_ylabel("map y [m]", fontsize=10, color=INK3)
            axm.legend(loc="upper left", fontsize=10, framealpha=0.92)
            if a.fast > 1:
                axm.text(0.985, 0.965, f"{a.fast} 倍速", transform=axm.transAxes,
                         ha="right", va="top", fontsize=12, color=INK2, weight="bold")
            if mark and mark[0] <= now <= mark[1]:
                axm.text(0.985, 0.915, "純回転の区間", transform=axm.transAxes,
                         ha="right", va="top", fontsize=12, color=BAD, weight="bold")

            # 拡大窓: 直近 ZOOM_S 秒を ±ZOOM m で見る（震えはここにしか出ない）
            azi.clear()
            m0 = (te >= now - ZOOM_S) & (te <= now)
            azi.scatter(MX, MY, s=8.0, c="#c3cbd7", marker=".", linewidths=0)
            if m0.any():
                azi.plot(xy[m0, 0], xy[m0, 1], "-", color=EST, lw=1.6,
                         label=f"推定 直近 {ZOOM_S:.0f} s")
            if oxy is not None:
                mo = (ot >= now - ZOOM_S) & (ot <= now)
                if mo.any():
                    azi.plot(oxy[mo, 0], oxy[mo, 1], "--", color=ODO, lw=1.6,
                             label="脚 odom")
            azi.plot([px], [py], "o", color=EST, ms=7, mec="white", mew=1.2, zorder=9)
            azi.set_xlim(px - ZOOM, px + ZOOM); azi.set_ylim(py - ZOOM, py + ZOOM)
            azi.set_aspect("equal")
            azi.tick_params(labelsize=8, colors=INK3)
            azi.grid(alpha=0.25, lw=0.6)
            azi.legend(loc="upper left", fontsize=9, framealpha=0.92)
            azi.set_title(f"拡大 ±{ZOOM:.1f} m —— 震えはここにしか出ない",
                          fontsize=11, color=INK, loc="left", pad=4)

            for ax, (tt, vv, ylab, color) in zip(
                    (ax1, ax2, ax3),
                    ((spd_t, spd, "見かけの速さ [m/s]", EST),
                     (gyro[:, 0] - t0 if gyro is not None else None,
                      gyro[:, 1] if gyro is not None else None, "回転レート [deg/s]", INK2),
                     (None, None, "経路長（累積）[m]", None))):
                ax.clear()
                ax.set_xlim(0, te[-1])
                ax.grid(alpha=0.25, lw=0.6)
                ax.set_ylabel(ylab, fontsize=9.5, color=INK3)
                ax.tick_params(labelsize=8.5, colors=INK3)
                if mark:
                    ax.axvspan(mark[0], mark[1], color=BAD, alpha=0.10, lw=0)
                if tt is not None:
                    m = tt <= now
                    ax.plot(tt[m], vv[m], "-", color=color, lw=1.2)
                ax.axvline(now, color=INK3, lw=0.9, ls=":")
            ax1.axhline(SPEED_LIMIT, color=BAD, lw=1.2, ls="--")
            ax1.text(te[-1] * 0.995, SPEED_LIMIT, " 指令上限 0.361", ha="right", va="bottom",
                     fontsize=9, color=BAD)
            ax1.set_ylim(0, max(0.6, float(spd.max()) * 1.05))
            ax3.plot(te[:k + 1], cum_e[:k + 1], "-", color=EST, lw=1.4, label="推定")
            if oxy is not None:
                cum_o = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(oxy, axis=0).T))])
                kk = int(np.argmin(np.abs(ot - now)))
                ax3.plot(ot[:kk + 1], cum_o[:kk + 1], "--", color=ODO, lw=1.4, label="脚 odom")
            ax3.legend(fontsize=9, loc="upper left", framealpha=0.9)
            ax3.set_xlabel("経過 [s]", fontsize=9.5, color=INK3)

            j = int(np.argmin(np.abs(spd_t - now))) if len(spd_t) else 0
            over = 100.0 * float((spd[:max(j, 1)] > SPEED_LIMIT).mean()) if j else 0.0
            net_now = math.hypot(*(xy[k] - xy[0]))
            sub.set_text(f"経過 {now:6.1f} s ／ 見かけの速さ {spd[j]:.2f} m/s ／ "
                         f"上限超 {over:.1f} % ／ "
                         f"推定の経路長 {cum_e[k]:.2f} m に対し正味 {net_now:.2f} m")
            writer.grab_frame()
            if n % 50 == 0:
                print(f"   {n}/{len(sel)}", flush=True)
    plt.close(fig)
    print(f"[render] 出来た: {a.out} ({a.out.stat().st_size/1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
