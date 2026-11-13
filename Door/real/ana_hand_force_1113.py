"""11-13 机上検討: 腕の関節の力（tau_est）から、左手が扉・レバーから受けている力を逆算する。読むだけ。
   F = pinv(J^T)(g(q) − tau)。重力は MuJoCo（URDF の慣性だけ・pelvis を IMU の傾きで固定）。触れる前（2b の終わり）の残りを引く。"""
import re, sys, math, glob
import numpy as np, mujoco
sys.path.insert(0, "/Users/inouereo/git_research/physical_ai/G1_Hackason/Door/real")
import door_script as dscr, door_frame as dfr, g1_arm_fk as fk
URDF = "/Users/inouereo/git_research/physical_ai/G1_Hackason/Door/doorman6/usd_fix/urdf_src/g1_29dof_rev_1_0.urdf"
txt = open(URDF).read()
txt = re.sub(r"<visual>.*?</visual>", "", txt, flags=re.S); txt = re.sub(r"<collision>.*?</collision>", "", txt, flags=re.S)
m = mujoco.MjModel.from_xml_string(txt); d = mujoco.MjData(m)
jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in fk.JOINT_ORDER]
qadr = [m.jnt_qposadr[j] for j in jid]; dadr = [m.jnt_dofadr[j] for j in jid]
ARM = [fk.JOINT_ORDER.index(f"left_{n}_joint") for n in ("shoulder_pitch","shoulder_roll","shoulder_yaw","elbow","wrist_roll","wrist_pitch","wrist_yaw")]
hand = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_wrist_yaw_link")
assert hand > 0, [m.body(i).name for i in range(m.nbody)]
CHAIN = fk.load_chain()
S3A, S3C, S2B = (dscr.SUBS.index(s) for s in ("3a", "3c", "2b"))

def ext_force(q, quat_wxyz):
    """pelvis の座標での、手が外から受ける力 [N]（残りを引く前）と、腕 7 軸の g − tau を返すための g と J"""
    roll, pitch = dfr.roll_pitch_from_quat_wxyz(quat_wxyz)
    T = dfr.tilt_matrix(roll, pitch)                 # pelvis → 水平の座標
    m.opt.gravity[:] = T.T @ np.array([0, 0, -9.81])  # pelvis の座標での重力
    d.qpos[:] = 0; d.qvel[:] = 0
    for k in range(29): d.qpos[qadr[k]] = q[k]
    mujoco.mj_forward(m, d)
    g = np.array([d.qfrc_bias[dadr[k]] for k in ARM])
    pt = CHAIN.link_pose('left_rubber_hand', q)[0]  # 手のひらの原点（pelvis の座標＝MuJoCo の世界）
    Jp = np.zeros((3, m.nv)); mujoco.mj_jac(m, d, Jp, None, pt, hand)
    J = Jp[:, [dadr[k] for k in ARM]]
    return g, J, T

def run(path):
    z = np.load(path, allow_pickle=True)
    t, sc, q, quat, pose, ph, tau, vw = z["t"], z["script"], z["q"], z["quat"], z["pose"], z["phase"].astype(int), z["tau"], z["vel_want"]
    sub = sc[:, 0]; door = (ph == 3) & np.isfinite(sub)
    k = 0; n = len(t)
    while k < n:
        if not door[k]: k += 1; continue
        j = k
        while j < n and door[j]: j += 1
        seg = np.arange(k, j); k = j
        if not (sub[seg] == S3A).any(): continue
        b = seg[sub[seg] == S2B][-25:]                     # 触れる前の 0.5 秒: 残り（力の読みの基準のずれ）
        res0 = np.mean([ (lambda g, J, T: g - tau[i, ARM])(*ext_force(q[i], quat[i])) for i in b], axis=0) if len(b) else np.zeros(7)
        i3 = seg[sub[seg] >= S3A]; i3 = i3[sub[i3] <= S3C]
        print(f"\n== {path.split('_')[-1][:6]} DOOR {t[seg[0]]-t[0]:.0f} 秒〜 前の指令 {np.nanmax(vw[i3,1]):.2f}・腰 yaw {math.degrees(np.median(q[i3,12])):+.0f}°・基準の残り（肩 pitch）{res0[0]:+.1f} N·m")
        print("  小段  t   扉°  手が受ける力 [N]（扉の座標）前後  左右   上下  | pelvis x   向き°  手の前  手の高さ−触れた高さ cm")
        t0 = t[i3[0]]
        rows = []
        for i in i3:
            g, J, T = ext_force(q[i], quat[i])
            F_p = np.linalg.lstsq(J.T, g - tau[i, ARM] - res0, rcond=None)[0]
            F = dfr.rot_z(pose[i, 2]) @ T @ F_p
            rows.append((t[i] - t0, sub[i], sc[i, 7], *F, pose[i, 0], math.degrees(pose[i, 2]), sc[i, 10], 100 * (sc[i, 6] - 1.02)))
        rows = np.array(rows)
        for a in np.arange(0, rows[-1, 0] + 1e-6, 0.4):
            w = rows[(rows[:, 0] >= a) & (rows[:, 0] < a + 0.4)]
            if not len(w): continue
            mm = np.nanmedian(w, axis=0)
            print(f"  {dscr.SUBS[int(w[0,1])]:3s} {a:4.1f} {mm[2]:5.1f}      {mm[3]:+6.0f} {mm[4]:+6.0f} {mm[5]:+6.0f}      |  {mm[6]:+.2f}   {mm[7]:+5.0f}   {mm[8]:.2f}   {mm[9]:+6.1f}")
        c = rows[rows[:, 1] == S3C]
        if len(c): print(f"  3c の中央: 前後 {np.median(c[:,3]):+.0f} N・左右 {np.median(c[:,4]):+.0f} N・上下 {np.median(c[:,5]):+.0f} N")

for p in sys.argv[1:]: run(p)
