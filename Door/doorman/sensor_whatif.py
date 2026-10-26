"""同じ動きの記録で、センサーの付け方を変えたら扉がどれだけ見えるかを試算する。

腕による隠れはカメラの案だけ数える（ライダーは自分の体の遮蔽を数えない＝甘めの上限）。
使い方: python sensor_whatif.py <urdf> <rec> <vision.npz> <summary.json> [<rec> <vision.npz> <summary.json> ...]
"""
import json
import sys

import numpy as np

import camera_visibility as cv

STAGES = ["0 歩く", "1 構える", "2 手を掛ける", "3 下げる", "4 押す", "5 通る"]
MID360_POS = np.array([0.0002835, 0.00003, 0.428434])
MID360_RPY = (np.pi, 0.05112069379091391, 0.0)
MID360_ELEV = (-7.0, 52.0)   # センサーの座標での上下の視野 [°]
MID360_MIN_RANGE = 0.1


def rot_y(deg):
    t = np.radians(deg)
    return np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])


def rot_z(deg):
    t = np.radians(deg)
    return np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]])


def geometry(urdf, rec, vision_path, summary_path):
    st = np.load(f"{rec}/static.npz")
    names = [str(n) for n in st["dof_names"]]
    S = cv.load_states(rec)
    V = np.load(vision_path)
    env, step = V["env"], V["step"]
    row = {int(s): i for i, s in enumerate(S["step"][:, 0])}
    ti = np.array([row[int(s)] for s in step])
    q = {n: S["dof_pos"][ti, env, k] for k, n in enumerate(names)}
    pel_R = cv.quat_xyzw_to_mat(S["root_quat_xyzw"][ti, env])
    pel_p = S["root_pos"][ti, env].astype(float)
    tor_R, tor_p = urdf.fk("torso_link", pel_R, pel_p, q)
    body = S["door_body_pos"][ti, env].astype(float)
    w = st["door_width"][env].astype(float)
    h = st["door_height"][env].astype(float)
    lr = st["door_open_lr"][env].astype(float)
    hinge = S["door_dof_pos"][ti, env, 0].astype(float)
    u = np.stack([np.abs(np.sin(hinge)), lr * np.cos(hinge), 0 * hinge], -1)
    center = body[:, 1].copy()
    center[:, 2] = h / 2
    zs = np.linspace(0.05, 0.95, 12)
    edge = center[:, None] + (0.5 * w)[:, None, None] * u[:, None] + ((zs[None] - 0.5) * h[:, None])[..., None] * np.array([0, 0, 1.0])
    latch = np.stack([np.zeros((len(w), 12)), np.broadcast_to((lr * w / 2)[:, None], (len(w), 12)), zs[None] * h[:, None]], -1)
    arms = []
    for side in ("left", "right"):
        pts = {}
        for key, link in [("sh", f"{side}_shoulder_pitch_link"), ("el", f"{side}_elbow_link"),
                          ("wr", f"{side}_wrist_roll_link"), ("pa", f"{side}_rubber_hand")]:
            Rl, pl = urdf.fk(link, pel_R, pel_p, q)
            pts[key] = pl
            if key == "pa":
                pts["tip"] = pl + Rl[:, :, 0] * 0.10
        arms.append(pts)
    per = json.load(open(summary_path))["per_episode"]
    keep = np.zeros(len(env), bool)
    for p in per:
        keep |= (env == p["env"]) & (step >= p["start"]) & (step < p["start"] + p["len"])
    return dict(tor_R=tor_R, tor_p=tor_p, lever=body[:, [2, 4]], edge=edge, latch=latch, arms=arms,
                stage=V["stage"], keep=keep)


def occluded(cam_p, L, arms):
    occ = np.zeros(len(L), bool)
    segs = [("sh", "el", "upper"), ("el", "wr", "fore"), ("wr", "pa", "hand"), ("pa", "tip", "hand")]
    for pts in arms:
        for a, b, kind in segs:
            dist, s = cv.seg_ray_dist(cam_p, L, pts[a], pts[b])
            n = np.linalg.norm(L - cam_p, axis=-1)
            occ |= (dist < cv.ARM_RADIUS[kind]) & (s * n < n - cv.OCCL_MARGIN)
    return occ


def cam_fov(pc, half_w_deg, half_h_deg, fisheye_deg=None):
    x = pc[..., 0]
    if fisheye_deg is not None:
        ang = np.degrees(np.arccos(np.clip(x / np.maximum(np.linalg.norm(pc, axis=-1), 1e-9), -1, 1)))
        return (ang <= fisheye_deg) & (np.linalg.norm(pc, axis=-1) > cv.NEAR)
    ok = x > cv.NEAR
    xs = np.where(ok, x, 1.0)
    return ok & (np.abs(pc[..., 1] / xs) <= np.tan(np.radians(half_w_deg))) & (np.abs(pc[..., 2] / xs) <= np.tan(np.radians(half_h_deg)))


def eval_camera(g, pitch, yaw=0.0, half_w=43.5, half_h=28.1, fisheye=None):
    R_off = rot_z(yaw) @ rot_y(pitch)
    cam_R = g["tor_R"] @ R_off[None]
    cam_p = g["tor_p"] + np.einsum("nij,j->ni", g["tor_R"], cv.CAM_POS)
    lev_vis = np.zeros(len(cam_p), bool)
    for k in range(2):
        pc = cv.to_cam(g["lever"][:, k:k + 1], cam_R, cam_p)[:, 0]
        lev_vis |= cam_fov(pc, half_w, half_h, fisheye) & ~occluded(cam_p, g["lever"][:, k], g["arms"])
    edge = cam_fov(cv.to_cam(g["edge"], cam_R, cam_p), half_w, half_h, fisheye).any(-1)
    latch = cam_fov(cv.to_cam(g["latch"], cam_R, cam_p), half_w, half_h, fisheye).any(-1)
    return lev_vis, edge, latch


def eval_lidar(g):
    R_off = cv.rpy_to_mat(*MID360_RPY)
    R = g["tor_R"] @ R_off[None]
    p = g["tor_p"] + np.einsum("nij,j->ni", g["tor_R"], MID360_POS)
    def vis(pts):
        pc = cv.to_cam(pts, R, p)
        rng = np.linalg.norm(pc, axis=-1)
        elev = np.degrees(np.arcsin(np.clip(pc[..., 2] / np.maximum(rng, 1e-9), -1, 1)))
        return (elev >= MID360_ELEV[0]) & (elev <= MID360_ELEV[1]) & (rng > MID360_MIN_RANGE)
    return vis(g["lever"]).any(-1), vis(g["edge"]).any(-1), vis(g["latch"]).any(-1)


def need_yaw(g, pitch=41.0):
    """戸先の中ほどを画面の中心に入れるのに要る、胴に対するカメラの左右の振り [°]"""
    mid = g["edge"][:, 6]
    cam_p = g["tor_p"] + np.einsum("nij,j->ni", g["tor_R"], cv.CAM_POS)
    d = np.einsum("nji,nj->ni", g["tor_R"], mid - cam_p)
    return np.degrees(np.arctan2(d[:, 1], d[:, 0]))


def main(urdf_path, triples):
    urdf = cv.Urdf(urdf_path)
    gs = [geometry(urdf, *t) for t in triples]
    g = {k: (np.concatenate([x[k] for x in gs]) if k != "arms" else None) for k in gs[0]}
    g["arms"] = [{kk: np.concatenate([x["arms"][s][kk] for x in gs]) for kk in gs[0]["arms"][s]} for s in range(2)]
    keep = g["keep"]
    cases = [
        ("今（俯角 41°・87°×56°）", lambda: eval_camera(g, 41)),
        ("俯角 25°", lambda: eval_camera(g, 25)),
        ("俯角 41°・左へ 30° 振って固定", lambda: eval_camera(g, 41, yaw=30)),
        ("俯角 41°・右へ 30° 振って固定", lambda: eval_camera(g, 41, yaw=-30)),
        ("広角 魚眼（光軸から 80° の円）・俯角 41°", lambda: eval_camera(g, 41, fisheye=80)),
        ("頭のライダー MID-360（体の遮蔽は数えない）", lambda: eval_lidar(g)),
    ]
    st = g["stage"]
    print("案\t区間\tレバー\t戸先\t戸当たり側の枠")
    for name, fn in cases:
        lev, edge, latch = fn()
        for s in range(6):
            m = keep & (st == s)
            print(f"{name}\t{STAGES[s]}\t{100*lev[m].mean():.0f}\t{100*edge[m].mean():.0f}\t{100*latch[m].mean():.0f}")
    ny = need_yaw(g)
    for s in (3, 4, 5):
        m = keep & (st == s)
        a = np.abs(ny[m])
        print(f"首で戸先を中心に入れるのに要る振り（区間 {STAGES[s]}）: 中央 {np.median(a):.0f}° / 90% {np.percentile(a,90):.0f}° / "
              f"±45° 以内 {100*(a<=45).mean():.0f}% / ±90° 以内 {100*(a<=90).mean():.0f}%  向き {'左' if np.median(ny[m])>0 else '右'}")
    tot = keep.sum()
    for s in range(6):
        print(f"時間の割合 {STAGES[s]}: {100*(keep & (st==s)).sum()/tot:.0f}%")


if __name__ == "__main__":
    a = sys.argv[2:]
    main(sys.argv[1], [tuple(a[i:i + 3]) for i in range(0, len(a), 3)])
