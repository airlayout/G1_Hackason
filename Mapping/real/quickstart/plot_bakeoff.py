#!/usr/bin/env python3
"""段 2 の比較を図にする。**手で数字を書き写さない**（記録から毎回作り直す）。

出すもの:
  1. sigma_sweep.png  — σ を振っても M1/M2 が動かない（再生・無負荷。3 つの実効レート）
  2. load_repro.png   — **負荷をかけて初めて live の崩壊が再現する**
  3. trajectories.png — 旋回中の推定の軌跡を重ねる（真値・live・再生・負荷あり）
  4. tf_rate.png      — MOLA の /tf レートの時間変化（live の階段状の低下）

    Navigation/.venv/bin/python quickstart/plot_bakeoff.py runs/click_20260912T125919 <出力先>
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
import matplotlib                                            # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                              # noqa: E402
from jp_font import japanese_font                            # noqa: E402
from eval_traj import (evaluate, gyro_yaw, net_yaw, read_odom, to_base_link,  # noqa: E402
                       open_bag, read_imu, read_odom, read_static_tf,
                       read_tf_chain, read_tum, turn_window, slice_time)

PASS_M1_M = 0.30          # 合否 2-2
PASS_M2_PCT = 5.0         # 合否 2-3
INK, GRID = "#111620", "#dce1e9"
C_TRUTH, C_LIVE, C_OK, C_BAD = "#111620", "#c9372c", "#0f6fc4", "#e07b00"


def load_traj(spec: Path) -> np.ndarray:
    """再生の出力ディレクトリ / TUM / どちらでも読む。

    ⚠️ FAST-LIO2 は `/tf` ではなく `/Odometry_loc` を出す（このフォークの改名）。
    `/tf` だけ見て「出力ゼロ」と描かないこと。
    """
    if spec.is_dir():
        c, t = open_bag(spec)
        a = read_tf_chain(c, t)
        if len(a) < 2 and "/Odometry_loc" in t:
            a = read_odom(c, t, "/Odometry_loc")
        return a if a is not None else np.empty((0, 8))
    return read_tum(spec)


def style(ax, title="", xl="", yl=""):
    ax.set_title(title, fontsize=11, color=INK, pad=10)
    ax.set_xlabel(xl, fontsize=9); ax.set_ylabel(yl, fontsize=9)
    ax.grid(True, color=GRID, lw=.7, alpha=.9)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.tick_params(labelsize=8, colors="#46505f")


def main() -> None:
    run = Path(sys.argv[1])
    out = Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
    if japanese_font() is None:
        print("⚠️ 日本語フォントが見つからない。図が豆腐になる", file=sys.stderr)

    con, tid = open_bag(run)  # noqa: F841
    imu, odom, st = read_imu(con, tid), read_odom(con, tid), read_static_tf(con, tid)
    t_g, yaw_g = gyro_yaw(imu, Rotation.from_quat(st[1]).as_matrix())
    win = turn_window(t_g, yaw_g)
    gw = yaw_g[(t_g >= win[0]) & (t_g <= win[1])]
    truth = {"gyro_net_yaw_deg_window": math.degrees(gw[-1] - gw[0])}
    live = read_tf_chain(con, tid)

    def m(spec) -> dict:
        return evaluate(load_traj(Path(spec)) if not isinstance(spec, np.ndarray) else spec,
                        truth, win)

    # ── 図1: σ を振っても動かない（無負荷）──────────────────────────────
    rates = [("9.9 Hz（全スキャン）", "sigma_sweep"),
             ("2.9 Hz（1/3 に間引き）", "sweep_dec13"),
             ("1.8 Hz（1/5 に間引き）", "sweep_dec15")]
    sig_order = ["10.0", "2.0", "0.5", "0.1", "0.00575"]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    for label, sub in rates:
        xs, m1, m2 = [], [], []
        for s in sig_order:
            hit = sorted((run / sub).glob("angacc{}_*.tum".format(s)))
            if not hit:
                continue
            r = m(hit[0])
            xs.append(s); m1.append(r["M1_phantom_m"]); m2.append(abs(r.get("M2_yaw_ratio_pct", 0)))
        axes[0].plot(xs, m1, "o-", lw=1.8, ms=5, label=label)
        axes[1].plot(xs, m2, "o-", lw=1.8, ms=5, label=label)
    axes[0].axhline(PASS_M1_M, color=C_BAD, ls="--", lw=1.2)
    axes[0].text(0.02, PASS_M1_M, " 合格線 0.30 m", color=C_BAD, fontsize=8,
                 va="bottom", transform=axes[0].get_yaxis_transform())
    axes[1].axhline(PASS_M2_PCT, color=C_BAD, ls="--", lw=1.2)
    axes[1].text(0.02, PASS_M2_PCT, " 合格線 5%", color=C_BAD, fontsize=8,
                 va="bottom", transform=axes[1].get_yaxis_transform())
    style(axes[0], "M1 幻の並進（真値 ≈ 0）", "σ_random_walk_acc_angular [rad/s²]", "M1 [m]")
    style(axes[1], "M2 回転の追従の誤差（真値 0%）", "σ_random_walk_acc_angular [rad/s²]", "|M2| [%]")
    for a in axes:
        a.invert_xaxis(); a.legend(fontsize=8, frameon=False)
    fig.suptitle("再生（無負荷）: σ を 2 桁振っても、実効レートを 1/5 に落としても壊れない",
                 fontsize=12.5, y=1.0)
    fig.tight_layout(); fig.savefig(out / "sigma_sweep.png", dpi=160,
                                    bbox_inches="tight", facecolor="white")
    plt.close(fig)

    # ── 図2: 負荷をかけて初めて再現する ────────────────────────────────
    cases = [("実機 live\n（当日の失敗）", live, C_LIVE),
             ("再生 CLI\n無負荷", run / "loc_sweep" / "angacc10.0_ang0.1.tum", C_OK),
             ("再生 実時間+L2\n無負荷", run / "rt_base" / "tf_bag", C_OK),
             ("再生 実時間+L2\nCPU 競合 3", run / "rt_load3" / "tf_bag", C_LIVE)]
    vals = []
    for name, spec, col in cases:
        try:
            r = m(spec)
        except Exception:
            continue
        vals.append((name, r.get("M1_phantom_m", float("nan")),
                     abs(r.get("M2_yaw_ratio_pct", float("nan"))), col))
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    for ax, idx, ttl, yl, line in ((axes[0], 1, "M1 幻の並進", "M1 [m]", PASS_M1_M),
                                   (axes[1], 2, "M2 回転の追従の誤差", "|M2| [%]", PASS_M2_PCT)):
        ax.bar([v[0] for v in vals], [v[idx] for v in vals],
               color=[v[3] for v in vals], width=.62)
        for i, v in enumerate(vals):
            ax.text(i, v[idx], " {:.2f}".format(v[idx]) if idx == 1 else " {:.1f}".format(v[idx]),
                    ha="center", va="bottom", fontsize=9, color=INK)
        ax.axhline(line, color=C_BAD, ls="--", lw=1.2)
        style(ax, ttl, "", yl)
    fig.suptitle("σ でもレートでもなく、実時間の CPU 競合が再現の条件だった",
                 fontsize=12.5, y=1.0)
    fig.tight_layout(); fig.savefig(out / "load_repro.png", dpi=160,
                                    bbox_inches="tight", facecolor="white")
    plt.close(fig)

    # ── 図3: 旋回中の軌跡 ─────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(6.4, 6.0))
    ow = slice_time(odom, *win)
    # 脚 odom は自分の原点なので、旋回開始時の推定に合わせて重ねる
    lw0 = slice_time(live, *win)
    if len(ow) > 1 and len(lw0) > 1:
        yaw0 = net_yaw(lw0[:1, 4:8])[0] - net_yaw(ow[:1, 4:8])[0]
        R = np.array([[math.cos(yaw0), -math.sin(yaw0)], [math.sin(yaw0), math.cos(yaw0)]])
        p = (ow[:, 1:3] - ow[0, 1:3]) @ R.T + lw0[0, 1:3]
        ax.plot(p[:, 0], p[:, 1], color=C_TRUTH, lw=2.4, label="真値（脚 odom・198° 純回転）")
    for name, spec, col, ls in (
            ("実機 live（MOLA）", live, C_LIVE, "-"),
            ("再生 実時間+L2 無負荷", run / "rt_base" / "tf_bag", C_OK, "-"),
            ("再生 実時間+L2 CPU 競合 3", run / "rt_load3" / "tf_bag", C_BAD, "-")):
        try:
            a = slice_time(load_traj(Path(spec)) if not isinstance(spec, np.ndarray) else spec, *win)
        except Exception:
            continue
        if len(a) > 1:
            ax.plot(a[:, 1], a[:, 2], ls, color=col, lw=1.7, alpha=.95, label=name)
            ax.plot(a[0, 1], a[0, 2], "o", color=col, ms=6, mec="white", mew=1.2)
    ax.set_aspect("equal")
    style(ax, "旋回 198° の間に推定がどこへ行ったか（○ = 開始点）", "map x [m]", "map y [m]")
    ax.legend(fontsize=8.5, frameon=False, loc="best")
    fig.tight_layout(); fig.savefig(out / "trajectories.png", dpi=160,
                                    bbox_inches="tight", facecolor="white")
    plt.close(fig)

    # ── 図4: /tf のレート ────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(9.5, 3.8))
    for name, spec, col in (("実機 live", live, C_LIVE),
                            ("再生 実時間 無負荷", run / "rt_base" / "tf_bag", C_OK),
                            ("再生 実時間 CPU 競合 3", run / "rt_load3" / "tf_bag", C_BAD)):
        try:
            a = load_traj(Path(spec)) if not isinstance(spec, np.ndarray) else spec
        except Exception:
            continue
        if len(a) < 5:
            continue
        t = a[:, 0] - t_g[0]
        hz = [np.sum((t >= s) & (t < s + 5.0)) / 5.0 for s in np.arange(0, t[-1], 2.0)]
        ax.plot(np.arange(0, t[-1], 2.0), hz, lw=1.8, color=col, label=name)
    ax.axvspan(win[0] - t_g[0], win[1] - t_g[0], color="#0f6fc4", alpha=.08)
    ax.text((win[0] + win[1]) / 2 - t_g[0], ax.get_ylim()[1] * .95, "旋回区間",
            ha="center", fontsize=8.5, color="#46505f")
    style(ax, "レートは犯人ではない —— 無負荷の再生も 5 Hz 前後だが M1 = 0.28 m で正しい",
          "記録の頭からの時間 [s]", "map→base_link の頻度 [Hz]")
    ax.legend(fontsize=8.5, frameon=False)
    fig.tight_layout(); fig.savefig(out / "tf_rate.png", dpi=160,
                                    bbox_inches="tight", facecolor="white")
    plt.close(fig)
    # ── 図5: 負荷の段階 ──────────────────────────────────────────────
    ladder = [("負荷 0", "rt_base"), ("負荷 1", "rt_load1"), ("負荷 2", "rt_load2"),
              ("負荷 3", "rt_load3"), ("負荷 3\n(nice 19)", "rt_load3_nice")]
    fig, ax = plt.subplots(figsize=(8.2, 4.2))
    names, ys, cols, notes = [], [], [], []
    for label, sub in ladder:
        d = run / sub / "tf_bag"
        if not d.exists():
            continue
        r = m(d)
        names.append(label)
        if "M1_phantom_m" not in r:
            # /tf が旋回の前に止まった。棒では表せないので別に印す
            a = load_traj(d)
            ys.append(0.0); cols.append("#9aa4b2")
            notes.append("/tf が {:.0f}s で停止".format(a[-1, 0] - a[0, 0]) if len(a) > 1 else "/tf 無し")
        else:
            ok = r["M1_phantom_m"] <= PASS_M1_M
            ys.append(r["M1_phantom_m"]); cols.append(C_OK if ok else C_LIVE)
            notes.append("{:.2f} m".format(r["M1_phantom_m"]))
    ax.bar(names, ys, color=cols, width=.6)
    for i, (y, n) in enumerate(zip(ys, notes)):
        ax.text(i, y, " " + n, ha="center", va="bottom", fontsize=9, color=INK)
    ax.axhline(PASS_M1_M, color=C_BAD, ls="--", lw=1.2)
    ax.text(0.30, PASS_M1_M, "合格線 0.30 m", color=C_BAD, fontsize=8, va="bottom",
            transform=ax.get_yaxis_transform())
    ax.axhline(m(live)["M1_phantom_m"], color=C_LIVE, ls=":", lw=1.2)
    ax.text(0.99, m(live)["M1_phantom_m"], "実機 live 5.91 m ", color=C_LIVE, fontsize=8,
            va="bottom", ha="right", transform=ax.get_yaxis_transform())
    style(ax, "4 コアのうち何本を取り合うか（灰色 = 旋回の前に /tf が止まった）", "", "M1 [m]")
    fig.tight_layout(); fig.savefig(out / "load_ladder.png", dpi=160,
                                    bbox_inches="tight", facecolor="white")
    plt.close(fig)

    # ── 図6: 負荷ありでの σ 掃引（段 2b の本体）───────────────────────
    under = [("10.0\n(既定) #1", "rt_load3"), ("10.0\n(既定) #2", "rt_l3_aa10.0_r2"),
             ("0.5", "rt_l3_aa0.5_a"), ("0.1", "rt_l3_aa0.1_a"),
             ("0.00575\n(段2aの導出値)", "rt_l3_aa0.00575_a")]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    xs, m1s, m2s = [], [], []
    for label, sub in under:
        d = run / sub / "tf_bag"
        if not d.exists():
            continue
        r = m(d)
        if "M1_phantom_m" not in r:
            continue
        xs.append(label); m1s.append(r["M1_phantom_m"])
        m2s.append(abs(r.get("M2_yaw_ratio_pct", float("nan"))))
    for ax, ys, ttl, yl, line, base in (
            (axes[0], m1s, "M1 幻の並進", "M1 [m]", PASS_M1_M, m(run / "rt_base" / "tf_bag")["M1_phantom_m"]),
            (axes[1], m2s, "M2 回転の追従の誤差", "|M2| [%]", PASS_M2_PCT,
             abs(m(run / "rt_base" / "tf_bag")["M2_yaw_ratio_pct"]))):
        ax.bar(xs, ys, color=C_LIVE, width=.6)
        for i, y in enumerate(ys):
            ax.text(i, y, " {:.2f}".format(y), ha="center", va="bottom", fontsize=9, color=INK)
        ax.axhline(line, color=C_BAD, ls="--", lw=1.2)
        ax.axhline(base, color=C_OK, ls=":", lw=1.4)
        ax.text(0.5, base, "同じ σ・無負荷 {:.2f}".format(base), color=C_OK, fontsize=8,
                va="bottom", ha="center", transform=ax.get_yaxis_transform())
        style(ax, ttl, "σ_random_walk_acc_angular [rad/s²]", yl)
    fig.suptitle("再現する条件（CPU 競合 3）で σ を振っても直らない —— C1 は落第",
                 fontsize=12.5, y=1.0)
    fig.tight_layout(); fig.savefig(out / "sigma_under_load.png", dpi=160,
                                    bbox_inches="tight", facecolor="white")
    plt.close(fig)

    # ── 図7: 候補の比較（C0/C1 MOLA 対 C2 FAST-LIO2）────────────────────
    def m_frame(spec, conj=False):
        a = load_traj(Path(spec))
        if conj:
            a = to_base_link(a, st, conjugate=True)
        return evaluate(a, truth, win), a

    groups = [("MOLA-LO\n（測位のみ・事前地図）",
               [("無負荷", run / "rt_base" / "tf_bag", False),
                ("CPU 競合 3", run / "rt_load3" / "tf_bag", False)]),
              ("FAST-LIO2\n（C5 の内側・odometry）",
               [("無負荷", run / "c2_base" / "tf_bag", True),
                ("CPU 競合 3", run / "c2_load3" / "tf_bag", True)])]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4))
    labels, m1s, m2s, cols, notes = [], [], [], [], []
    for gname, items in groups:
        for cond, spec, conj in items:
            if not Path(spec).exists():
                continue
            r, a = m_frame(spec, conj)
            labels.append("{}\n{}".format(gname.split("\n")[0], cond))
            if "M1_phantom_m" not in r:
                m1s.append(0.0); m2s.append(0.0); cols.append("#9aa4b2")
                notes.append("出力 {} 件で途絶".format(len(a)))
            else:
                ok = abs(r.get("M2_yaw_ratio_pct", 99)) <= PASS_M2_PCT
                m1s.append(r["M1_phantom_m"]); m2s.append(abs(r["M2_yaw_ratio_pct"]))
                cols.append(C_OK if ok else C_LIVE)
                notes.append("")
    for ax, ys, ttl, yl, line in ((axes[0], m1s, "M1 幻の並進", "M1 [m]", PASS_M1_M),
                                  (axes[1], m2s, "M2 回転の追従の誤差", "|M2| [%]", PASS_M2_PCT)):
        ax.bar(labels, ys, color=cols, width=.6)
        for i, (y, n) in enumerate(zip(ys, notes)):
            ax.text(i, y, " " + (n if n else "{:.2f}".format(y)),
                    ha="center", va="bottom", fontsize=8.5, color=INK)
        ax.axhline(line, color=C_BAD, ls="--", lw=1.2)
        style(ax, ttl, "", yl)
    axes[0].axhline(0.328, color=C_TRUTH, ls=":", lw=1.4)
    axes[0].text(0.22, 0.328, "真値（脚 odom）0.328 m", color=C_TRUTH, fontsize=8,
                 ha="left", va="bottom", transform=axes[0].get_yaxis_transform())
    fig.suptitle("乗り換え候補も同じ競合で落ちる（灰色 = 出力が途絶した）", fontsize=12.5, y=1.0)
    fig.tight_layout(); fig.savefig(out / "candidates.png", dpi=160,
                                    bbox_inches="tight", facecolor="white")
    plt.close(fig)

    print("書き出し:", out)
    for f in sorted(out.glob("*.png")):
        print("  ", f.name, "{:.0f} KB".format(f.stat().st_size / 1024))


if __name__ == "__main__":
    main()
