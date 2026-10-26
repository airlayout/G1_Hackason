"""頭のカメラに扉・レバー・枠が写っているかを、記録の姿勢から幾何で数える。

カメラは torso_link に固定（exp_v2_eval_1016.sh と同じ取付: pos (0.0576, 0.0525, 0.4299)、pitch P）。
Isaac の "world" 規約（x 前・y 左・z 上）。焦点 1.88・開口 3.567×2.006（87°×56°）・近い側の切り 0.1 m。

使い方: python camera_visibility.py <urdf> <records/.../v2> <vision.npz> <pitch_deg> <out.npz>
"""
import sys
import xml.etree.ElementTree as ET

import numpy as np

FOCAL = 1.88
HALF_W = 3.567 / 2 / FOCAL   # tan(半画角・横)
HALF_H = 2.006 / 2 / FOCAL   # tan(半画角・縦)
NEAR = 0.1
OCCL_MARGIN = 0.005       # 手がレバーに触れていても、手前にあれば隠れに数える
CAM_POS = np.array([0.0576, 0.0525, 0.4299])
ARM_RADIUS = {"upper": 0.05, "fore": 0.045, "hand": 0.05}
RAY_W, RAY_H = 48, 27        # 扉の面の占める割合を数える光線の数


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, q):
    """axis (3,), q (N,) -> (N,3,3)"""
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    s, c = np.sin(q)[:, None, None], np.cos(q)[:, None, None]
    return np.eye(3)[None] + s * K[None] + (1 - c) * (K @ K)[None]


def quat_xyzw_to_mat(q):
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], 1)


class Urdf:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.by_child = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz") if o is not None else "0 0 0").split()])
            rpy = [float(v) for v in (o.get("rpy") if o is not None else "0 0 0").split()]
            ax = j.find("axis")
            axis = [float(v) for v in ax.get("xyz").split()] if ax is not None else [1, 0, 0]
            self.by_child[j.find("child").get("link")] = dict(
                name=j.get("name"), parent=j.find("parent").get("link"), xyz=xyz,
                R=rpy_to_mat(*rpy), axis=axis, type=j.get("type"))

    def chain(self, link):
        if link not in self.by_child:
            raise KeyError(f"URDF にリンクが無い: {link}")
        out = []
        while link in self.by_child:
            out.append(self.by_child[link])
            link = self.by_child[link]["parent"]
        return out[::-1]

    def fk(self, link, base_R, base_p, q_by_name):
        """base (N,3,3),(N,3) -> link の姿勢 (N,3,3),(N,3)"""
        R, p = base_R.copy(), base_p.copy()
        for j in self.chain(link):
            p = p + np.einsum("nij,j->ni", R, j["xyz"])
            R = R @ j["R"][None]
            if j["type"] in ("revolute", "continuous"):
                R = R @ axis_angle(j["axis"], q_by_name[j["name"]])
        return R, p


def to_cam(points, cam_R, cam_p):
    """points (N,K,3) world -> camera frame (N,K,3)"""
    return np.einsum("nji,nkj->nki", cam_R, points - cam_p[:, None])


def in_fov(pc):
    x = pc[..., 0]
    ok = x > NEAR
    xs = np.where(ok, x, 1.0)
    return ok & (np.abs(pc[..., 1] / xs) <= HALF_W) & (np.abs(pc[..., 2] / xs) <= HALF_H)


def seg_ray_dist(C, L, A, B):
    """カメラ C から点 L への線分と、腕の線分 AB の最短距離と、そのときの線分 CL 上の割合 t。(N,3) 入力"""
    d1, d2, r = L - C, B - A, C - A
    a, e = (d1 * d1).sum(-1), (d2 * d2).sum(-1)
    b, c, f = (d1 * d2).sum(-1), (d1 * r).sum(-1), (d2 * r).sum(-1)
    den = np.maximum(a * e - b * b, 1e-12)
    s = np.clip((b * f - c * e) / den, 0, 1)
    t = np.clip((b * s + f) / np.maximum(e, 1e-12), 0, 1)
    s = np.clip((b * t - c) / np.maximum(a, 1e-12), 0, 1)
    diff = (C + s[:, None] * d1) - (A + t[:, None] * d2)
    return np.linalg.norm(diff, axis=-1), s


def load_states(rec):
    import glob
    fs = sorted(glob.glob(f"{rec}/states_*.npz"))
    keys = ["step", "root_pos", "root_quat_xyzw", "dof_pos", "door_dof_pos", "stage", "door_body_pos"]
    S = {k: np.concatenate([np.load(f)[k] for f in fs], 0) for k in keys}
    o = np.argsort(S["step"][:, 0], kind="stable")
    return {k: v[o] for k, v in S.items()}


def main(urdf_path, rec, vision_path, pitch_deg, out_path):
    urdf = Urdf(urdf_path)
    st = np.load(f"{rec}/static.npz")
    names = [str(n) for n in st["dof_names"]]
    S = load_states(rec)
    V = np.load(vision_path)
    env, step = V["env"], V["step"]
    row = {int(s): i for i, s in enumerate(S["step"][:, 0])}
    ti = np.array([row[int(s)] for s in step])
    N = len(env)

    q = {n: S["dof_pos"][ti, env, k] for k, n in enumerate(names)}
    pel_R = quat_xyzw_to_mat(S["root_quat_xyzw"][ti, env])
    pel_p = S["root_pos"][ti, env].astype(float)
    tor_R, tor_p = urdf.fk("torso_link", pel_R, pel_p, q)
    th = np.radians(pitch_deg)
    Rp = np.array([[np.cos(th), 0, np.sin(th)], [0, 1, 0], [-np.sin(th), 0, np.cos(th)]])
    cam_R = tor_R @ Rp[None]
    cam_p = tor_p + np.einsum("nij,j->ni", tor_R, CAM_POS)

    body = S["door_body_pos"][ti, env].astype(float)   # root, door_panel, door_handle, latch_link, grasp_target
    lever_pts = body[:, [2, 4]]
    lever_c = to_cam(lever_pts, cam_R, cam_p)
    lever_fov = in_fov(lever_c)

    # 腕で隠れているか（線分 カメラ→レバー と 腕のカプセル）
    occl = np.zeros((N, 2), bool)
    for side in ("left", "right"):
        pts = {}
        for key, link in [("sh", f"{side}_shoulder_pitch_link"), ("el", f"{side}_elbow_link"),
                          ("wr", f"{side}_wrist_roll_link"), ("pa", f"{side}_rubber_hand")]:
            Rl, pl = urdf.fk(link, pel_R, pel_p, q)
            pts[key] = pl
            if key == "pa":
                pts["tip"] = pl + Rl[:, :, 0] * 0.10
        segs = [("sh", "el", "upper"), ("el", "wr", "fore"), ("wr", "pa", "hand"), ("pa", "tip", "hand")]
        for k in range(2):
            L = lever_pts[:, k]
            for a, b, kind in segs:
                dist, s = seg_ray_dist(cam_p, L, pts[a], pts[b])
                near_end = s * np.linalg.norm(L - cam_p, axis=-1) < np.linalg.norm(L - cam_p, axis=-1) - OCCL_MARGIN
                occl[:, k] |= (dist < ARM_RADIUS[kind]) & near_end

    # 扉の面: 中心（door_panel）と蝶番の角度から長方形を作る
    w = st["door_width"][env].astype(float)
    h = st["door_height"][env].astype(float)
    lr = st["door_open_lr"][env].astype(float)
    hinge = S["door_dof_pos"][ti, env, 0].astype(float)
    u = np.stack([np.abs(np.sin(hinge)), lr * np.cos(hinge), 0 * hinge], -1)  # 蝶番→戸先の向き
    center = body[:, 1].copy()
    center[:, 2] = h / 2
    # カメラが扉の向こう側（押して開けた先）にいるか。シムの扉はレバーが両面にあるので隠れには数えない
    n_front = np.stack([-np.cos(hinge), lr * np.abs(np.sin(hinge)), 0 * hinge], -1)
    behind_panel = np.einsum("ni,ni->n", cam_p - center, n_front) < 0
    lever_vis = lever_fov & ~occl
    # 戸先（動く縦の辺）: 扉の角度の手がかり
    edge = center[:, None] + (0.5 * w)[:, None, None] * u[:, None] \
        + ((np.linspace(0.05, 0.95, 12)[None, :] - 0.5) * h[:, None])[..., None] * np.array([0, 0, 1.0])
    free_edge = in_fov(to_cam(edge, cam_R, cam_p)).any(-1)
    def ang_to(pt):
        d = pt - cam_p
        d = d / np.linalg.norm(d, axis=-1, keepdims=True)
        return np.degrees(np.arccos(np.clip((d * cam_R[:, :, 0]).sum(-1), -1, 1)))
    ang_lever = ang_to(lever_pts[:, 1])
    ang_opening = ang_to(np.stack([0 * w, 0 * w, 0 * w + 1.0], -1))
    corners = center[:, None] + np.stack([s * w for s in (-0.5, 0.5, 0.5, -0.5)], 1)[..., None] * u[:, None] \
        + np.stack([z * h for z in (-0.5, -0.5, 0.5, 0.5)], 1)[..., None] * np.array([0, 0, 1.0])
    cc = to_cam(corners, cam_R, cam_p)
    corners_px = np.stack([-cc[..., 1] / np.maximum(cc[..., 0], 1e-3), -cc[..., 2] / np.maximum(cc[..., 0], 1e-3)], -1)
    # 光線を飛ばして、画面のうち扉の面が占める割合を数える
    gx = (np.linspace(-1, 1, RAY_W) * HALF_W)
    gz = (np.linspace(-1, 1, RAY_H) * HALF_H)
    GX, GZ = np.meshgrid(gx, gz)
    dirs_c = np.stack([np.ones_like(GX), -GX, -GZ], -1).reshape(-1, 3)      # 画面の左右・上下
    dirs_w = np.einsum("nij,kj->nki", cam_R, dirs_c)
    nrm = np.stack([lr * np.cos(hinge) * 0 + u[:, 1], -u[:, 0], 0 * hinge], -1)   # 面の法線（水平）
    nrm = nrm / np.linalg.norm(nrm, axis=-1, keepdims=True)
    denom = np.einsum("nki,ni->nk", dirs_w, nrm)
    tpl = np.einsum("ni,ni->n", center - cam_p, nrm)[:, None] / np.where(np.abs(denom) < 1e-9, 1e-9, denom)
    hit = cam_p[:, None] + tpl[..., None] * dirs_w
    rel = hit - center[:, None]
    along = np.einsum("nki,ni->nk", rel, u)
    up = rel[..., 2]
    panel_hit = (tpl > NEAR) & (np.abs(along) <= w[:, None] / 2) & (np.abs(up) <= h[:, None] / 2)
    panel_cov = panel_hit.mean(-1)

    # 枠の縦の 2 本（閉じた面 x=0、y=±w/2）と開口の中心
    zs = np.linspace(0.05, 1.0, 12)[None, :] * h[:, None]
    def line_pts(yv):
        return np.stack([np.zeros_like(zs), np.broadcast_to(yv[:, None], zs.shape), zs], -1)
    hinge_jamb = in_fov(to_cam(line_pts(-lr * w / 2), cam_R, cam_p)).any(-1)
    latch_jamb = in_fov(to_cam(line_pts(lr * w / 2), cam_R, cam_p)).any(-1)
    opening = in_fov(to_cam(np.stack([0 * w, 0 * w, 0.0 * w + 1.0], -1)[:, None], cam_R, cam_p))[:, 0]
    # 扉の面の点のうち視野に入る割合（どれだけ扉の全体を見ているか）
    ss = np.linspace(-0.5, 0.5, 9)
    zz = np.linspace(0.05, 0.95, 9)
    SS, ZZ = np.meshgrid(ss, zz)
    pp = center[:, None] + (SS.reshape(-1)[None, :, None] * w[:, None, None]) * u[:, None] \
        + ((ZZ.reshape(-1)[None, :] - 0.5) * h[:, None])[..., None] * np.array([0, 0, 1.0])
    panel_pts_fov = in_fov(to_cam(pp, cam_R, cam_p)).mean(-1)

    fwd = cam_R[:, :, 0]
    cam_heading = np.degrees(np.arctan2(fwd[:, 1], fwd[:, 0]))    # 扉の法線 +x からの向き
    pel_heading = np.degrees(np.arctan2(pel_R[:, 1, 0], pel_R[:, 0, 0]))
    lever_px = np.stack([-lever_c[:, 0, 1] / np.maximum(lever_c[:, 0, 0], 1e-3),
                         -lever_c[:, 0, 2] / np.maximum(lever_c[:, 0, 0], 1e-3)], -1)
    np.savez(out_path, env=env, step=step, stage=V["stage"], episode=V["episode"],
             lever_fov=lever_fov, lever_occl=occl, behind_panel=behind_panel, free_edge=free_edge, ang_lever=ang_lever, ang_opening=ang_opening, corners_px=corners_px, corners_x=cc[..., 0], lever_vis=lever_vis, lever_px=lever_px,
             lever_dist=np.linalg.norm(lever_pts[:, 0] - cam_p, axis=-1),
             panel_cov=panel_cov, panel_pts_fov=panel_pts_fov, hinge_jamb=hinge_jamb,
             latch_jamb=latch_jamb, opening=opening, cam_heading=cam_heading,
             pel_heading=pel_heading, hinge_deg=np.degrees(hinge), cam_z=cam_p[:, 2],
             est=V["est"], pose=V["pose"], angle=V["angle"])
    print(out_path, N)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4]), sys.argv[5])
