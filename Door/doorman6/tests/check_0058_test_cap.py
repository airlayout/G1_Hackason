# 0058 の式を作り物の環境で確かめる（10-19 夜）。OMEN で: python -I test_cap.py <VIRAL6>/gr00t/rl/envs/door/door_open_homie.py
"""① の式を作り物の環境で確かめる"""
import ast, sys
import torch
from types import SimpleNamespace as NS

SRC = sys.argv[1]
tree = ast.parse(open(SRC).read())
names = {"_reward_push_door_force", "_lever_unlatched", "_reward_press_door_handle_down", "_reward_lever_force_over_cap"}
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and any(getattr(f, "name", "") in names for f in n.body))
ns = {"torch": torch}
for f in cls.body:
    if isinstance(f, ast.FunctionDef) and f.name in names:
        f.decorator_list = []
        exec(compile(ast.Module([f], []), SRC, "exec"), ns)

class Fake:
    def __init__(self, cfg, forward, down, lever_deg, f_peak):
        n = len(forward)
        self.config = cfg; self.num_envs = n
        self.use_left_hand = torch.zeros(n, dtype=torch.bool)
        self.left_hand_indices_tgt_ct_sensor = [0]; self.right_hand_indices_tgt_ct_sensor = [1]
        force = torch.zeros(n, 1, 2, 3)
        force[:, 0, 1, 0] = torch.tensor(forward); force[:, 0, 1, 2] = -torch.tensor(down)
        jp = torch.zeros(n, 2); jp[:, 1] = torch.tensor(lever_deg) * torch.pi / 180
        self.simulator = NS(object_to_hand_contact_forces=force, scene=NS(articulations={"door": NS(data=NS(joint_pos=NS(torch=jp)))}))
        self._f = torch.tensor(f_peak)
    def _hand_lever_force_n(self): return self._f
for k in names: setattr(Fake, k, ns[k])

cfg = {"push_door_force_max": 50, "push_door_force_latched_max": 15, "press_handle_force_peak_n": 15.0, "push_unlatched_handle_deg": 30.0}
e = Fake(cfg, forward=[40.0, 40.0, 10.0, -30.0], down=[0, 15, 25, 40], lever_deg=[10.0, 35.0, 10.0, 10.0], f_peak=[40.0, 40.0, 25.0, 55.0])
r = e._reward_push_door_force(); print("前へ押す", r.tolist()); assert r.tolist() == [15.0, 40.0, 10.0, 0.0]
r = e._reward_press_door_handle_down(); print("下へ押す", [round(x, 3) for x in r.tolist()]); assert torch.allclose(r, torch.tensor([0.0, 1.0, 1.0, 1.0 - 15 / 15]))
e2 = Fake(cfg, [0, 0], [55.0, 10.0], [0, 0], [0, 0]); r = e2._reward_press_door_handle_down(); assert torch.allclose(r, torch.tensor([1 - 30 / 15, 10 / 15]))
r = e._reward_lever_force_over_cap(); print("上限超え", r.tolist()); assert torch.allclose(r, torch.tensor([1.0, 0.0, 0.0, 2.0]))
# 設定が無いときは元の式（前へは 50 N で頭打ち・下へは 30 N まで線形）
e3 = Fake({"push_door_force_max": 50}, [40.0, 60.0], [15.0, 45.0], [10.0, 10.0], [0, 0])
assert e3._reward_push_door_force().tolist() == [40.0, 50.0] and torch.allclose(e3._reward_press_door_handle_down(), torch.tensor([0.5, 1.0]))
print("すべて合格")
