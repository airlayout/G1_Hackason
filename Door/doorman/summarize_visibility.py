"""camera_visibility.py の出力を区間ごとにまとめる。"""
import json
import sys

import numpy as np

STAGES = ["0 扉へ歩く", "1 構える", "2 レバーに手", "3 レバーを下げる", "4 押して開ける", "5 通り抜ける"]


def load(cv_path, summary_path):
    c = dict(np.load(cv_path))
    per = json.load(open(summary_path))["per_episode"]
    succ = np.full(len(c["env"]), -1)
    for p in per:
        m = (c["env"] == p["env"]) & (c["step"] >= p["start"]) & (c["step"] < p["start"] + p["len"])
        succ[m] = int(p["success"])
    c["succ"] = succ
    return c


def main(pairs):
    cs = [load(a, b) for a, b in pairs]
    c = {k: np.concatenate([x[k] for x in cs]) for k in cs[0]}
    keep = c["succ"] >= 0
    c = {k: v[keep] for k, v in c.items()}
    err = np.abs(c["est"] - np.concatenate([c["pose"], c["angle"][:, None]], 1))
    print("est 単位の確認: pose", np.abs(c["pose"]).max(0).round(2), "angle", np.abs(c["angle"]).max().round(2))
    lever_vis = c["lever_vis"].any(-1)
    lever_fov = c["lever_fov"].any(-1)
    rows = []
    for s in range(6):
        for label, m in [("全体", c["stage"] == s), ("成功の回", (c["stage"] == s) & (c["succ"] == 1)),
                         ("失敗の回", (c["stage"] == s) & (c["succ"] == 0))]:
            n = int(m.sum())
            if n == 0:
                continue
            rows.append(dict(
                stage=STAGES[s], group=label, sec=n * 0.1,
                lever_fov=100 * lever_fov[m].mean(), lever_vis=100 * lever_vis[m].mean(),
                lever_occl=100 * (lever_fov[m] & ~lever_vis[m]).mean(),
                panel_cov=100 * c["panel_cov"][m].mean(), panel_pts=100 * c["panel_pts_fov"][m].mean(),
                free_edge=100 * c["free_edge"][m].mean(), hinge_jamb=100 * c["hinge_jamb"][m].mean(),
                latch_jamb=100 * c["latch_jamb"][m].mean(), opening=100 * c["opening"][m].mean(),
                heading=float(np.median(np.abs(c["cam_heading"][m]))),
                ang_lever=float(np.median(c["ang_lever"][m])), ang_open=float(np.median(c["ang_opening"][m])),
                behind=100 * c["behind_panel"][m].mean(),
                ex=100 * float(np.median(err[m, 0])), ey=100 * float(np.median(err[m, 1])),
                eyaw=float(np.degrees(np.median(err[m, 2]))), eang=float(np.degrees(np.median(err[m, 3]))),
                ex90=100 * float(np.percentile(err[m, 0], 90)), eang90=float(np.degrees(np.percentile(err[m, 3], 90)))))
    hdr = ["stage", "group", "sec", "lever_fov", "lever_vis", "lever_occl", "panel_cov", "panel_pts", "free_edge",
           "hinge_jamb", "latch_jamb", "opening", "heading", "ang_lever", "ang_open", "behind", "ex", "ey", "eyaw",
           "eang", "ex90", "eang90"]
    print("\t".join(hdr))
    for r in rows:
        print("\t".join(f"{r[k]:.1f}" if isinstance(r[k], float) else str(r[k]) for k in hdr))

    # 区間 4 の中で、写り方ごとの推定の誤差
    m4 = c["stage"] == 4
    print("\n区間 4: 扉の面が画面を占める割合ごとの誤差（中央値 x cm / y cm / 向き° / 扉の角度°, 秒）")
    for lo, hi in [(0, 0.3), (0.3, 0.6), (0.6, 0.9), (0.9, 1.01)]:
        m = m4 & (c["panel_cov"] >= lo) & (c["panel_cov"] < hi)
        if m.sum() < 20:
            continue
        print(f"  {lo:.1f}-{hi:.1f}: {100*np.median(err[m,0]):.1f} {100*np.median(err[m,1]):.1f} "
              f"{np.degrees(np.median(err[m,2])):.1f} {np.degrees(np.median(err[m,3])):.1f}  ({m.sum()*0.1:.0f} s)")
    print("区間 4: 戸先が視野に入るか")
    for v in (True, False):
        m = m4 & (c["free_edge"] == v)
        print(f"  {v}: {100*np.median(err[m,0]):.1f} {100*np.median(err[m,1]):.1f} "
              f"{np.degrees(np.median(err[m,2])):.1f} {np.degrees(np.median(err[m,3])):.1f}  ({m.sum()*0.1:.0f} s)")
    print("区間 4: 枠（どちらかの縦の線）が視野に入るか")
    for v in (True, False):
        m = m4 & ((c["hinge_jamb"] | c["latch_jamb"]) == v)
        print(f"  {v}: {100*np.median(err[m,0]):.1f} {100*np.median(err[m,1]):.1f} "
              f"{np.degrees(np.median(err[m,2])):.1f} {np.degrees(np.median(err[m,3])):.1f}  ({m.sum()*0.1:.0f} s)")
    print("区間 4: カメラの向き（扉の法線から）と誤差")
    for lo, hi in [(0, 45), (45, 90), (90, 135), (135, 181)]:
        m = m4 & (np.abs(c["cam_heading"]) >= lo) & (np.abs(c["cam_heading"]) < hi)
        if m.sum() < 20:
            continue
        print(f"  {lo}-{hi}°: {100*np.median(err[m,0]):.1f} {100*np.median(err[m,1]):.1f} "
              f"{np.degrees(np.median(err[m,2])):.1f} {np.degrees(np.median(err[m,3])):.1f}  ({m.sum()*0.1:.0f} s)")
    np.savez(sys.argv[-1], **{k: v for k, v in c.items()}, err=err)


if __name__ == "__main__":
    a = sys.argv[1:-1]
    main(list(zip(a[0::2], a[1::2])))
