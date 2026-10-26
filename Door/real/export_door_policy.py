"""段 5: 学習済みの生徒（PPO の ckpt）と HOMIE の重みを、実機用の door_core.py が読む npz に書き出す。

torch が要る（Mac の G1_Hackason/Navigation/.venv で動く）。DoorMan（VIRAL6）の door_mirror.py を読み込んで
鏡写しの対応を「そのコードに基底ベクトルを通して」作る（手で写さない）。最後に torch の LSTM・HOMIE と
numpy の計算が同じ出力を返すかを確かめ、合わなければ書き出さずに止まる。

使い方:
  python export_door_policy.py --ckpt last.pt --config config.yaml --walk model_walk.pt --stand model_stand.pt \
      --viral <VIRAL6 の場所> --out door_policy_it45150.npz
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import pickle
import re
import sys
import types
from pathlib import Path

import numpy as np
import torch
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core  # noqa: E402

# 観測の項目の次元（obs の obs_dims を解決した値。合計は ckpt の LSTM の入力の幅と照合する）
TERM_DIMS = {
    "actions": 29,
    "arm_dof_torque": 14,
    "b_homie_commands": 7,
    "base_ang_vel": 3,
    "complete": 1,
    "delta_actions": 17,
    "dof_pos": 29,
    "dof_vel": 29,
    "door_angle_lidar": 3,
    "door_frame_loc": 9,
    "door_geometry": 7,
    "projected_gravity": 3,
}


class _Stub:
    def __init__(self, *a, **k) -> None:
        pass

    def __setstate__(self, state) -> None:
        self.__dict__["_state"] = state


class _StubUnpickler(pickle.Unpickler):
    """ckpt に入っている trl などのクラスが無くても読めるように、見つからないクラスは空のクラスで代用する"""

    def find_class(self, module: str, name: str):
        try:
            return super().find_class(module, name)
        except (ModuleNotFoundError, AttributeError):
            return type(name, (_Stub,), {})


_PICKLE = types.SimpleNamespace(Unpickler=_StubUnpickler, load=pickle.load, __name__="stub_pickle")


def load_torch(path: str) -> dict:
    return torch.load(path, map_location="cpu", weights_only=False, pickle_module=_PICKLE)


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def import_door_mirror(viral: Path):
    spec = importlib.util.spec_from_file_location("door_mirror", viral / "gr00t/rl/envs/door/door_mirror.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def linear_map(fn, dim: int):
    """y = fn(x) が y = M x + b の形であるとして M, b を基底ベクトルから求める"""
    zero = torch.zeros(1, dim)
    b = fn(zero.clone())[0].double().numpy()
    M = np.zeros((dim, dim))
    for i in range(dim):
        e = torch.zeros(1, dim)
        e[0, i] = 1.0
        M[:, i] = fn(e)[0].double().numpy() - b
    return M, b


def gains_by_substring(names: list, table: dict) -> np.ndarray:
    """isaacsim.py と同じく ".*key.*" で関節名に当て、各関節でちょうど 1 つ当たることを確かめる"""
    out = np.zeros(len(names))
    for i, n in enumerate(names):
        hits = [v for k, v in table.items() if re.fullmatch(f".*{k}.*", n)]
        if len(hits) != 1:
            raise ValueError(f"{n} のゲインが {len(hits)} 個当たった")
        out[i] = float(hits[0])
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--walk", required=True)
    ap.add_argument("--stand", required=True)
    ap.add_argument("--viral", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    cfg = yaml.safe_load(open(a.config, encoding="utf-8"))
    obs_cfg, robot, envc = cfg["obs"], cfg["robot"], cfg["env"]["config"]
    algo = cfg["algo"]["config"]

    # --- 観測の並び（名前の辞書順。legged_robot_base.py:1223） ---
    terms = sorted(obs_cfg["obs_dict"]["actor_obs"])
    layout, start = {}, 0
    for t in terms:
        layout[t] = (start, start + TERM_DIMS[t])
        start += TERM_DIMS[t]
    obs_dim = start

    ck = load_torch(a.ckpt)
    P = {k: v.double() for k, v in ck["policy_state_dict"].items()}
    if P["memory.rnn.weight_ih_l0"].shape[1] != obs_dim:
        raise ValueError(f"観測の幅 {obs_dim} が ckpt の {P['memory.rnn.weight_ih_l0'].shape[1]} と合わない")

    # --- 鏡写し（DoorMan の door_mirror.py に通す） ---
    dm = import_door_mirror(Path(a.viral))
    mirror = dm.DoorMirror(robot["dof_names"], robot["left_arm_dof_names"], robot["right_arm_dof_names"])
    M = np.zeros((obs_dim, obs_dim))
    b = np.zeros(obs_dim)
    for t, (s, e) in layout.items():
        Mt, bt = linear_map(lambda x, t=t: mirror.obs(t, x.float()), e - s)
        M[s:e, s:e] = Mt
        b[s:e] = bt
    A, bA = linear_map(lambda x: mirror.teacher_actions(x.float()), door_core.NUM_ACT)
    if np.abs(bA).max() > 0:
        raise ValueError("行動の鏡写しに定数項がある")
    # 2 回掛けると戻るかを見る。door_geometry の [5]（1 − lr）は [4]（lr）から作り直されるので、入力もその関係を満たす形にする
    x = np.random.default_rng(0).normal(size=obs_dim)
    g0 = layout["door_geometry"][0]
    x[g0 + 5] = 1.0 - x[g0 + 4]
    with np.errstate(all="ignore"):  # macOS の Accelerate の matmul は値が有限でも浮動小数の警告を出すことがある
        back = M @ (M @ x + b) + b
    if not np.isfinite(back).all() or np.abs(back - x).max() > 1e-9 or np.abs(A @ A - np.eye(door_core.NUM_ACT)).max() > 1e-12:
        raise ValueError("鏡写しを 2 回掛けても元に戻らない")

    # --- 機体の表 ---
    names = list(robot["dof_names"])
    q_default = np.array([float(robot["init_state"]["default_joint_angles"][n]) for n in names])
    kp = gains_by_substring(names, robot["control"]["stiffness"])
    kd = gains_by_substring(names, robot["control"]["damping"])

    sc = obs_cfg["obs_scales"]
    consts = {
        "scale_base_ang_vel": sc["base_ang_vel"],
        "scale_dof_vel": sc["dof_vel"],
        "scale_arm_dof_torque": sc["arm_dof_torque"],
        "homie_obs_lin_scale": 2.0,  # homie_base.py:186-201
        "homie_obs_ang_scale": 0.5,
        "homie_cmd_deadzone": 0.1,
        "homie_switch_threshold": float(algo.get("homie_switch_threshold", 0.5)),
        "homie_command_scale": envc["homie_command_scale"],
        "homie_cmd_clip": envc["clip_homie_linvel_x_threshold"],
        "homie_default_height": obs_cfg["homie_default_height"],
        "delta_action_scale": envc["delta_action_scale"],
        "delta_action_clip": envc["delta_action_clip"],
        "action_scale": robot["control"]["action_scale"],
        "action_clip": robot["control"]["action_clip_value"],
        "clip_observations": envc["normalization"]["clip_observations"],
        "rms_eps": 1e-5,
        "rms_clip": 5.0,
    }
    # 10-26: 腕の積み上げの切り方（直し B）を入れて学習した方策なら、その値を残す（door_run_real.py が無ければ知らせる）
    guard_delta = None
    if envc.get("arm_accum_guard_delta") is not None:
        guard_delta = np.broadcast_to(np.asarray(envc["arm_accum_guard_delta"], dtype=float), (14,)).copy()  # 1 つか関節ごと
        consts["arm_accum_guard_margin"] = float(envc.get("arm_accum_guard_margin", 0.05))
    thr = [envc[k] for k in ("clip_homie_linvel_x_threshold", "clip_homie_linvel_y_threshold", "clip_homie_angvel_threshold")]
    if len(set(thr)) != 1:
        raise ValueError(f"指令の頭打ちが前後・横・旋回で違う: {thr}（door_core は 1 つの値しか持たない）")

    arrays = {
        "obs_mirror_M": M,
        "obs_mirror_b": b,
        "act_mirror": A,
        "rms_mean": P["running_mean_std.running_mean"].numpy(),
        "rms_var": P["running_mean_std.running_var"].numpy(),
        "q_default": q_default,
        "kp": kp,
        "kd": kd,
        "q_lower": np.array(robot["dof_pos_lower_limit_list"], dtype=float),
        "q_upper": np.array(robot["dof_pos_upper_limit_list"], dtype=float),
        "effort_limit": np.array(robot["dof_effort_limit_list"], dtype=float),
    }
    if guard_delta is not None:
        arrays["arm_guard_delta"] = guard_delta  # door_core が既定の δ の代わりに使う
    for i in range(door_core.LSTM_LAYERS):
        for k in ("w_ih", "w_hh", "b_ih", "b_hh"):
            src = f"memory.rnn.{k.replace('w_', 'weight_').replace('b_', 'bias_')}_l{i}"
            arrays[f"lstm{i}_{k}"] = P[src].numpy()
    for j, idx in enumerate((0, 2, 4, 6)):
        arrays[f"actor{j}_w"] = P[f"actor_module.module.{idx}.weight"].numpy()
        arrays[f"actor{j}_b"] = P[f"actor_module.module.{idx}.bias"].numpy()
    homie_sd = {}
    for name, path in (("walk", a.walk), ("stand", a.stand)):
        sd = {k: v.double() for k, v in load_torch(path)["model_state_dict"].items()}
        homie_sd[name] = sd
        for j, idx in enumerate((0, 2, 4)):
            arrays[f"{name}_enc{j}_w"] = sd[f"estimator.encoder.{idx}.weight"].numpy()
            arrays[f"{name}_enc{j}_b"] = sd[f"estimator.encoder.{idx}.bias"].numpy()
        for j, idx in enumerate((0, 2, 4, 6)):
            arrays[f"{name}_act{j}_w"] = sd[f"actor.{idx}.weight"].numpy()
            arrays[f"{name}_act{j}_b"] = sd[f"actor.{idx}.bias"].numpy()

    meta = {
        "layout": layout,
        "obs_dim": obs_dim,
        "mirror_left_doors": bool(envc.get("mirror_left_doors", False)),
        "constants": consts,
        "dof_names": names,
        "source": {
            "ckpt": a.ckpt,
            "ckpt_sha256": sha256(a.ckpt),
            "walk_sha256": sha256(a.walk),
            "stand_sha256": sha256(a.stand),
            "global_step": int(getattr(ck.get("state"), "__dict__", {}).get("_state", {}).get("global_step", -1))
            if isinstance(getattr(ck.get("state"), "__dict__", {}).get("_state", None), dict)
            else -1,
        },
    }
    out = Path(a.out)
    np.savez(out, meta_json=np.array(json.dumps(meta, ensure_ascii=False)), **arrays)

    # --- torch と numpy の突き合わせ ---
    p = door_core.load_params(str(out))
    lstm = torch.nn.LSTM(obs_dim, door_core.LSTM_HIDDEN, num_layers=door_core.LSTM_LAYERS).double()
    lstm.load_state_dict({k[len("memory.rnn."):]: v for k, v in P.items() if k.startswith("memory.rnn.")})
    mlp = torch.nn.Sequential(
        torch.nn.Linear(256, 512), torch.nn.SiLU(), torch.nn.Linear(512, 256), torch.nn.SiLU(),
        torch.nn.Linear(256, 128), torch.nn.SiLU(), torch.nn.Linear(128, 17),
    ).double()
    mlp.load_state_dict({k[len("actor_module.module."):]: v for k, v in P.items() if k.startswith("actor_module.module.")})
    rng = np.random.default_rng(1)
    h = np.zeros((2, 256))
    c = np.zeros((2, 256))
    th = (torch.zeros(2, 1, 256, dtype=torch.float64), torch.zeros(2, 1, 256, dtype=torch.float64))
    worst = 0.0
    mean, var = P["running_mean_std.running_mean"], P["running_mean_std.running_var"]
    with torch.no_grad():
        for _ in range(60):
            o = rng.normal(size=obs_dim) * np.sqrt(arrays["rms_var"]) + arrays["rms_mean"]
            y_np, h, c = door_core.actor_forward(p, o, h, c)
            xt = torch.clamp((torch.from_numpy(o) - mean) / torch.sqrt(var + 1e-5), -5, 5)
            out_t, th = lstm(xt.view(1, 1, -1), th)
            y_t = mlp(out_t.view(-1)).numpy()
            worst = max(worst, float(np.abs(y_np - y_t).max()))
    homie_worst = 0.0
    for name in ("walk", "stand"):
        sd = homie_sd[name]
        enc = torch.nn.Sequential(torch.nn.Linear(516, 256), torch.nn.ELU(), torch.nn.Linear(256, 256), torch.nn.ELU(), torch.nn.Linear(256, 35)).double()
        enc.load_state_dict({k[len("estimator.encoder."):]: v for k, v in sd.items() if k.startswith("estimator.encoder.")})
        act = torch.nn.Sequential(torch.nn.Linear(121, 512), torch.nn.ELU(), torch.nn.Linear(512, 256), torch.nn.ELU(), torch.nn.Linear(256, 256), torch.nn.ELU(), torch.nn.Linear(256, 15)).double()
        act.load_state_dict({k[len("actor."):]: v for k, v in sd.items() if k.startswith("actor.")})
        net = p.homie_walk if name == "walk" else p.homie_stand
        with torch.no_grad():
            for _ in range(20):
                o = rng.normal(size=516) * 0.5
                parts = enc(torch.from_numpy(o))
                z = torch.nn.functional.normalize(parts[3:], dim=-1, p=2)
                y_t = act(torch.cat([torch.from_numpy(o[-86:]), parts[:3], z])).numpy()
                homie_worst = max(homie_worst, float(np.abs(net(o) - y_t).max()))
    print(f"[export] 観測 {obs_dim}（辞書順）: " + ", ".join(f"{k}[{s}:{e}]" for k, (s, e) in layout.items()))
    print(f"[export] torch と numpy の差の最大: 生徒 {worst:.2e}・HOMIE {homie_worst:.2e}")
    if worst > 1e-8 or homie_worst > 1e-8:
        out.unlink()
        print("[export] 差が大きいので書き出しを取り消した")
        return 1
    print(f"[export] 書き出した: {out}（鏡写し {meta['mirror_left_doors']}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
