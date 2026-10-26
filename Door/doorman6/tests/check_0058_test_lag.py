# 0058 の式を作り物の環境で確かめる（10-19 夜）。OMEN で: python -I test_lag.py <VIRAL6>/gr00t/rl/envs/door/door_open_homie.py
"""② の式を作り物の環境で確かめる（door_open_homie.py から 3 つの関数を抜き出して使う）"""
import ast, json, os, sys, tempfile
import torch

SRC = sys.argv[1]
tree = ast.parse(open(SRC).read())
names = {"_lever_lagrange", "_reward_lever_touch_lagrange", "_lever_lagrange_update"}
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and any(getattr(f, "name", "") in names for f in n.body))
funcs = [f for f in cls.body if isinstance(f, ast.FunctionDef) and f.name in names]
ns = {"torch": torch, "json": json, "os": os}
for f in funcs:
    f.decorator_list = []
    exec(compile(ast.Module([f], []), SRC, "exec"), ns)


class Fake:
    def __init__(self, cfg, n=4):
        self.config = {"lever_lagrange": cfg}
        self.num_envs, self.device, self.common_step_counter = n, "cpu", 0
        self.log_dict = {}
        self.speed = torch.zeros(n); self.dist = torch.ones(n); self.age = -torch.ones(n, dtype=torch.long); self.f = torch.zeros(n)
    def _touch_state(self): return self.speed, self.dist, self.age
    def _hand_lever_force_n(self): return self.f

for k in names:
    setattr(Fake, k, ns[k])

tmp = tempfile.mkdtemp(); path = os.path.join(tmp, "lam.json")
cfg = {"lambda_init": 100.0, "rate_init": 1.0, "lr": 0.02, "target_rate": 0.1, "rate_ema": 0.002, "state_path": path, "save_every_events": 4,
       "v_touch": 0.2, "slope": 2.0, "contact_dist_m": 0.07, "zone_m": 0.30, "impact_max_n": 15.0, "impact_steps": 2}
e = Fake(cfg)

# 1) 近づく速さの罰: 距離 0.17 m で上限 0.2+2*0.1=0.4、速さ 1.0 → 超過 0.6。0.4 m 先（範囲外）は 0。触れた後（age>=0）も 0
e.dist = torch.tensor([0.17, 0.40, 0.07, 0.17]); e.speed = torch.tensor([1.0, 1.0, 0.1, 1.0]); e.age = torch.tensor([-1, -1, -1, 0])
r = e._reward_lever_touch_lagrange()
print("速さの罰", r.tolist())
assert torch.allclose(r, torch.tensor([-60.0, 0.0, 0.0, 0.0]))

# 2) 当たりの罰: 触れた 0・1 ステップ目に 45 N → 超過 (45-15)/15=2 → -200。2 ステップ目は窓の外で 0
e.speed[:] = 0; e.f = torch.tensor([45.0, 10.0, 0.0, 45.0]); e.age = torch.tensor([0, 0, -1, 2])
r = e._reward_lever_touch_lagrange(); print("当たりの罰（0 ステップ目）", r.tolist())
assert torch.allclose(r, torch.tensor([-200.0, 0.0, 0.0, 0.0]))
lam0 = e._lag["lambda"]
# 1 ステップ目で窓が閉じ、2 回ぶん（台 0: 違反・台 1: 合格）を数える
e.f = torch.tensor([20.0, 12.0, 0.0, 0.0]); e.age = torch.tensor([1, 1, -1, 3])
r = e._reward_lever_touch_lagrange()
print("1 ステップ目", r.tolist(), "λ", e._lag["lambda"], "割合", round(e._lag["rate"], 4), "回", e._lag["events"], "違反", e._lag["violations"])
assert e._lag["events"] == 2 and e._lag["violations"] == 1
keep = (1 - 0.002) ** 2; rate = 1.0 * keep + 0.5 * (1 - keep)
assert abs(e._lag["rate"] - rate) < 1e-9 and abs(e._lag["lambda"] - (lam0 + 0.02 * (rate - 0.1) * 2)) < 1e-9
# 台 0 の山は 0 ステップ目の 45 N（1 ステップ目の 20 N で上書きしない）
assert e._lag_imp_max[0] == 45.0

# 3) 合格が続くと λ は下がり、0 で止まる（下げは 1 回ごとに lr×目標＝0.002 と遅い。ここは lr を 1.0 にして確かめる）
e.config["lever_lagrange"] = dict(cfg, lr=1.0)
prev = e._lag["lambda"]
for i in range(3000):
    e.age = torch.tensor([1, 1, 1, 1]); e.f = torch.zeros(4)
    e._lag_imp_max = torch.zeros(4)
    e._reward_lever_touch_lagrange()
    # 割合（指数移動平均）は 1.0 から下がっていく。目標 0.1 を下回るまでは λ は上がり、下回った後は下がるだけ
    if e._lag["rate"] < 0.1:
        assert e._lag["lambda"] <= prev + 1e-9
    prev = e._lag["lambda"]
print("合格が 12,000 回続いた後 λ", round(e._lag["lambda"], 3), "割合", round(e._lag["rate"], 4))
assert e._lag["lambda"] == 0.0 and e._lag["rate"] < 0.01

# 4) 保存と続き
saved = json.load(open(path)); print("保存", saved)
e2 = Fake(cfg); e2._lever_lagrange()
assert e2._lag["lambda"] == saved["lambda"]
# 5) update=False（評価）は λ を動かさず書かない
cfg2 = dict(cfg, update=False, state_path=os.path.join(tmp, "x.json")); e3 = Fake(cfg2)
e3.age = torch.tensor([1, 1, 1, 1]); e3.f = torch.full((4,), 50.0); e3._lever_lagrange(); e3._lag_imp_max = torch.full((4,), 50.0)
e3._reward_lever_touch_lagrange()
assert e3._lag["lambda"] == 100.0 and not os.path.exists(cfg2["state_path"])
print("すべて合格")
