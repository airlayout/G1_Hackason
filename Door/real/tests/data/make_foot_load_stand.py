"""test_door_foot_load.py の作り物の立ち姿（foot_load_stand_sim.npz）を作る（Mac・mujoco。2026-10-26、段 1）。

公式の g1_29dof_rev_1_0.urdf を export_leg_model.py と同じ作りで読み、床を足して、HOLD の姿勢のまま静止させる。
胴（torso_link）に綱の上向きの力（体重の share 倍）を掛け、床が足に返す力の真値（接触の力の鉛直の和）を左右ごとに取る。
逆算の入力（関節の角度・関節に掛けたトルク・pelvis の四元数・関節の速さ）と真値を npz に書く。

  - 関節は PD（kp は実機の HOLD と同じ・kd は DoorMan の設定）を qfrc_applied で掛ける。記録するトルクはこの値
  - pelvis の水平の位置と向きは外から支える（鉛直の力は 0 なので、両足の和 = 体重 − 綱 は変わらない）。向きの目標を
    傾けると片足が床から浮く（傾いた場合の確かめ）
  - URDF のモデルは armature が 0 なので、陽的な PD がずれないよう armature 0.01（unitree_mujoco と同じ）・刻み 1 ms にする。
    どちらも静止の釣り合いには効かない
  - 足の裏の球（左右 4 個ずつ）と床だけが当たる

使い方: cd G1_Hackason && Navigation/.venv/bin/python Door/real/tests/data/make_foot_load_stand.py
"""
from __future__ import annotations

import json
import sys
from datetime import datetime
from pathlib import Path

import mujoco
import numpy as np

REAL = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REAL))
import export_leg_model as E  # noqa: E402

OUT = Path(__file__).resolve().parent / "foot_load_stand_sim.npz"
# 実機の HOLD の目標（10-16・10-23 の記録の q_sent と同じ）: 股 pitch −0.1・膝 0.3・足首 pitch −0.2、ほかは 0
Q_HOLD = np.zeros(29)
Q_HOLD[[0, 6]], Q_HOLD[[3, 9]], Q_HOLD[[4, 10]] = -0.1, 0.3, -0.2
KP = np.array([150, 150, 150, 200, 40, 40] * 2 + [250] * 3 + [100, 100, 40, 40, 20, 20, 20] * 2, float)
KD = np.array([2, 2, 2, 4, 2, 2] * 2 + [5] * 3 + [5, 5, 2, 2, 2, 2, 2] * 2, float)
# (綱が受ける割合, pelvis の向きの目標の roll [°])
CASES = ((0.0, 0.0), (0.4, 0.0), (0.85, 0.0), (0.4, 3.0))
SETTLE_S = 4.0
DT = 0.001
ARMATURE = 0.01
HOLD_K, HOLD_D = 400.0, 40.0  # pelvis を支える外力のばねと減衰（10-25 の foot_load_check.py と同じ）


def build() -> mujoco.MjModel:
    spec = E.build_spec(E.URDF_DEFAULT)
    spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05])
    m = spec.compile()
    E.check_joint_order(m)
    m.opt.timestep = DT
    m.dof_armature[6:] = ARMATURE
    feet = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in E.FOOT_BODIES]
    for g in range(m.ngeom):
        hit = m.geom_bodyid[g] in feet or m.geom_bodyid[g] == 0
        m.geom_contype[g] = m.geom_conaffinity[g] = 1 if hit else 0
    return m


def settle(m: mujoco.MjModel, share: float, roll_deg: float) -> dict:
    d = mujoco.MjData(m)
    weight = float(m.body_subtreemass[1]) * -float(m.opt.gravity[2])
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
    feet = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in E.FOOT_BODIES]
    d.qpos[3:7] = [1, 0, 0, 0]
    d.qpos[7:] = Q_HOLD
    d.qpos[2] = 0.8
    mujoco.mj_forward(m, d)
    sole = min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in range(m.ngeom) if m.geom_bodyid[g] in feet)
    d.qpos[2] -= sole - 0.001  # 足の裏を床の 1 mm 上へ
    mujoco.mj_forward(m, d)
    x0 = d.xpos[1][:2].copy()
    up = np.array([0.0, -np.sin(np.radians(roll_deg)), np.cos(np.radians(roll_deg))])  # x 軸まわりに roll 傾けた上
    for _ in range(int(round(SETTLE_S / DT))):
        d.qfrc_applied[6:] = KP * (Q_HOLD - d.qpos[7:]) - KD * d.qvel[6:]
        rot = d.xmat[1].reshape(3, 3)
        err = np.cross(rot[:, 2], up)
        w = rot @ d.qvel[3:6]  # 浮いた基底の角速度は自分の座標 → 世界へ
        torque = HOLD_K * err - HOLD_D * w
        fxy = -HOLD_K * (d.xpos[1][:2] - x0) - HOLD_D * d.qvel[0:2]
        d.xfrc_applied[1] = [fxy[0], fxy[1], 0.0, torque[0], torque[1], 0.0]
        d.xfrc_applied[torso] = [0.0, 0.0, share * weight, 0.0, 0.0, 0.0]
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    fz = np.zeros(2)
    for c in range(d.ncon):
        f6 = np.zeros(6)
        mujoco.mj_contactForce(m, d, c, f6)
        world = d.contact[c].frame.reshape(3, 3).T @ f6[:3]  # 接触の座標 → 世界（geom2 が geom1 から受ける向き）
        for side, foot in enumerate(feet):
            b1, b2 = m.geom_bodyid[d.contact[c].geom1], m.geom_bodyid[d.contact[c].geom2]
            if b2 == foot:
                fz[side] += world[2]
            elif b1 == foot:
                fz[side] -= world[2]
    tau = d.qfrc_applied[6:] + d.qfrc_passive[6:]  # 関節に掛かった力（受け身の力は 0 のはず）
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2), -1, 1)))
    return dict(q=d.qpos[7:].copy(), dq=d.qvel[6:].copy(), tau=tau, quat=d.qpos[3:7].copy(), fz_true=fz, tilt_deg=tilt,
                max_speed=float(np.abs(d.qvel).max()), weight=weight)


def main() -> int:
    m = build()
    rows = [settle(m, s, r) for s, r in CASES]
    for (s, r), row in zip(CASES, rows):
        print(f"[make_foot_load_stand] 綱 {100 * s:.0f}%・roll {r:.0f}°: 床の力 L {row['fz_true'][0]:6.1f} R {row['fz_true'][1]:6.1f} N"
              f"（体重の {100 * row['fz_true'].sum() / row['weight']:.1f}%）・傾き {row['tilt_deg']:.1f}°・速さの最大 {row['max_speed']:.1e}")
    meta = dict(made_by="Door/real/tests/data/make_foot_load_stand.py", made_at=datetime.now().isoformat(timespec="seconds"),
                mujoco=mujoco.__version__, urdf_sha256=E.make_meta(E.URDF_DEFAULT)["urdf_sha256"], settle_s=SETTLE_S, dt=DT,
                armature=ARMATURE, note="綱は torso_link に上向き。pelvis の水平の位置と向きは外力で支える（鉛直は 0）")
    np.savez(OUT, meta=np.array(json.dumps(meta, ensure_ascii=False)),
             rope_share=np.array([s for s, _ in CASES]), roll_deg=np.array([r for _, r in CASES]),
             weight=np.array(rows[0]["weight"]),
             **{k: np.stack([row[k] for row in rows]) for k in ("q", "dq", "tau", "quat", "fz_true", "tilt_deg")})
    print(f"[make_foot_load_stand] {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
