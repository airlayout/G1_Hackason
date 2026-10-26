"""door_vision_push.py の式の確かめ（作り物の動き、CPU）。使い方: python -I check_0060_push.py <door_vision_push.py>"""
import importlib.util
import math
import sys
import types

import torch


def wrap(a):
    return torch.atan2(torch.sin(a), torch.cos(a))


for name in ("gr00t", "gr00t.rl", "gr00t.rl.envs", "gr00t.rl.envs.door"):
    sys.modules.setdefault(name, types.ModuleType(name))
fake = types.ModuleType("gr00t.rl.envs.door.door_vision")
fake.wrap = wrap
sys.modules["gr00t.rl.envs.door.door_vision"] = fake
spec = importlib.util.spec_from_file_location("dvp", sys.argv[1])
dvp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dvp)

N, DT = 6, 0.02
torch.manual_seed(0)


def trajectory(T=600):
    """扉の座標での pelvis の動き: 近づく → 横を向いて扉の向こうへ"""
    x0 = torch.linspace(-1.5, -1.0, N)
    out = []
    for t in range(T):
        s = t / T
        x = x0 + 2.5 * s
        y = 0.3 + 0.1 * torch.sin(torch.full((N,), 6.0 * s))
        yaw = torch.full((N,), -2.3 * max(0.0, s - 0.4))
        out.append(torch.stack([x, y, yaw], dim=-1))
    return out


def stages(t, T=600):
    return torch.full((N,), 0 if t < 0.2 * T else (3 if t < 0.4 * T else 4), dtype=torch.long)


def run(cfg, bias=None, T=600):
    p = dvp.PushStageDoorValues(N, "cpu", DT, cfg)
    p.reset(torch.arange(N))
    traj = trajectory(T)
    errs = []
    for t in range(T):
        true = traj[t]
        est = true.clone() if bias is None else true + bias
        hinge = torch.full((N,), 0.5 * max(0.0, t / T - 0.4))
        vis = torch.ones(N, dtype=torch.bool)
        hand = torch.zeros(N, dtype=torch.bool)
        pose, ang = p.apply(stages(t, T), est, hinge * 0, true, hinge, vis, hand)
        errs.append((pose - true, ang - hinge, stages(t, T)))
    return errs


fails = 0


def check(cond, msg):
    global fails
    print(("OK  " if cond else "NG  ") + msg)
    fails += 0 if cond else 1


a = torch.tensor([[0.3, -0.2, 0.7], [1.0, 2.0, -2.0]])
check(torch.allclose(dvp.se2_compose(dvp.se2_inv(a), a), torch.zeros(2, 3), atol=1e-6), "逆と合成で原点に戻る")
b = torch.tensor([[0.1, 0.4, 0.3], [-0.5, 0.2, 1.0]])
ab = dvp.se2_compose(a, b)
check(torch.allclose(dvp.se2_compose(dvp.se2_inv(a), ab), b, atol=1e-6), "a⁻¹ ⊕ (a ⊕ b) = b")

# 既定では何もしない
p0 = dvp.PushStageDoorValues(N, "cpu", DT, {})
check(not p0.active, "設定なしでは active が False")

# E1: 区間 4 以降は本当の値（推定に 5 cm の片寄りを入れても消える）
bias = torch.tensor([0.05, 0.0, 0.0])
e = run({"truth_from_stage": 4}, bias=bias)
e4 = torch.stack([x[0] for x in e if x[2][0] >= 4])
e3 = torch.stack([x[0] for x in e if x[2][0] < 4])
check(e4.abs().max() < 1e-5, f"E1: 区間 4 以降の姿勢の誤差 0（最大 {e4.abs().max():.2e}）")
check(torch.allclose(e3[..., 0], torch.full_like(e3[..., 0], 0.05), atol=1e-5), "E1: 区間 4 より前は推定のまま")

# E2: odometry の誤差 0・推定が正確なら、運んだ値は本当の値と一致
e = run({"hold_pose_from_stage": 4})
e4 = torch.stack([x[0] for x in e if x[2][0] >= 4])
check(e4.abs().max() < 1e-4, f"E2: 誤差 0 なら運んだ値は本当の値（最大 {e4.abs().max():.2e}）")

# E2: 推定に扉の座標で一定の片寄り（前後 +3 cm）があれば、運んだ値も同じ片寄りを保つ
e = run({"hold_pose_from_stage": 4}, bias=torch.tensor([0.03, 0.0, 0.0]))
e4 = torch.stack([x[0] for x in e if x[2][0] >= 4])
check(torch.allclose(e4[..., 0], torch.full_like(e4[..., 0], 0.03), atol=1e-4) and e4[..., 1:].abs().max() < 1e-4,
      f"E2: 片寄り 3 cm はそのまま運ばれる（前後 {e4[..., 0].mean():.4f}）")

# E2: 区間 4 に入った後は推定を使わない（推定が大きく外れても運んだ値は変わらない）
p = dvp.PushStageDoorValues(N, "cpu", DT, {"hold_pose_from_stage": 4})
p.reset(torch.arange(N))
traj = trajectory()
for t in range(600):
    true = traj[t]
    est = true + (torch.tensor([0.5, -0.4, 0.3]) if t >= 300 else 0.0)
    pose, _ = p.apply(stages(t), est, torch.zeros(N), true, torch.zeros(N))
check((pose - traj[-1]).abs().max() < 1e-4, "E2: 押す間の推定の外れに引きずられない")

# E2: odometry の誤差を入れると、運ぶ距離に応じてずれが増える
e = run({"hold_pose_from_stage": 4, "odom": {"scale_sigma": 0.05, "yaw_rate_bias_deg": 0.3, "pos_rw_m": 0.02}})
d_first = torch.stack([x[0] for x in e if x[2][0] >= 4][:5]).abs().mean()
d_last = torch.stack([x[0] for x in e if x[2][0] >= 4][-5:]).abs().mean()
check(d_last > d_first, f"E2: 雑音ありはずれが溜まる（始め {d_first:.4f} → 終わり {d_last:.4f}）")

# E3: 区間 4 以降の角度はライダーの模型。片寄りは ±2° 以内、10 Hz で更新
e = run({"hold_pose_from_stage": 4, "lidar_angle_from_stage": 4, "lidar": {"angle_noise_deg": 0.0, "dropout": 0.0}})
ea = torch.stack([x[1] for x in e if x[2][0] >= 4])
check(ea[10:].abs().max() <= 2.0 * math.pi / 180 + 0.01, f"E3: 角度の誤差は片寄り＋遅れの範囲（最大 {math.degrees(ea[10:].abs().max()):.2f}°）")
ea0 = torch.stack([x[1] for x in e if x[2][0] < 4])
check(ea0.abs().max() < 1e-6, "E3: 区間 4 より前は渡した角度のまま")

print("合格" if fails == 0 else f"不合格 {fails} 件")
sys.exit(1 if fails else 0)
