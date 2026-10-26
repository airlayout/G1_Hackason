"""段 6 (b): 映像の生徒（door_open_homie_29dof_dagger-lstm）を、PC2 の CPU で推論の速さを測れる形に書き出す。

OMEN の doorman6 の環境（torch 2.11）で動かす。VIRAL6 の actor のクラス（VisionRecurrentActorObjPred）を設定から
そのまま作り、1 ステップ分の計算（画像の正規化 → ResNet18 → 観測の正規化 → LSTM 2 層 → MLP）を抜き出して書き出す。
抜き出した計算が actor の rollout と同じ答えを返すかを、LSTM の状態を持ち回して確かめ、合わなければ書き出さない。

  student_full.ts       画像 uint8 [1,H,W,3]・観測 [1,118]・h・c → 行動 17・h・c（制御の 1 ステップ全部）
  student_encoder.ts    画像 uint8 [1,H,W,3] → 画像の特徴 [1,128]（重い部分）
  student_recurrent.ts  観測・画像の特徴・h・c → 行動・h・c（軽い部分。画像を間引くときに毎ステップ回す方）
  student_encoder.onnx  正規化済みの画像 float [1,3,H,W] → 画像の特徴（TensorRT・onnxruntime 用）
  golden.npz            入力と actor の答え（PC2 で同じ答えが出るかの確かめ用）
  golden_image0.bin     golden の 1 枚目を正規化した float32 の生のバイト列（trtexec --loadInputs 用）
  meta.json             形・観測の並び・版・md5・確かめの結果

確かめが「ずれを見逃さない」ための作り（10-21 のレビューで足した）:
  - 観測の一部に桁違いの値を入れ、正規化の ±5 の切り捨てを実際に働かせる
  - 行動の振れ幅が許容の差（TOL）より十分大きいことを確かめる（出力が小さすぎると差 TOL の確かめが効かない）
  - 状態を持ち回さない版が golden と NOCARRY_MIN 以上「違う」ことを確かめる（持ち回しの確かめが効いている証拠）。
    乱数の重みでは記憶の効きが小さすぎたので、LSTM の重みを大きめの乱数にし、出口の全結合を HEAD_GAIN 倍にする
    （どちらも速さには関係しない。学習済みの重み（--ckpt）では何もしない）
  - 3 本の TorchScript を読み直してそれぞれ確かめ、ONNX も許容の差を超えたら止める
⚠️ 基準（golden）の画像の正規化は、シミュレータの get_rgb_image（isaacsim.py: float に → /255 → ImageNet の平均と幅）を
   読んで写したもの。学習中の画像の加工（明るさ・灰色など）は乱数で、実機では灰色を 3 つ並べて同じ正規化をする

重み: --ckpt を渡さなければ乱数（ResNet は ImageNet の重み）。速さは網の形で決まり、重みには依らない。
     --ckpt を渡すと学習済みの policy_state_dict を読む（strict）。そのときは --config にその回の最終の設定
     （Hydra が書いた config.yaml。コマンドラインの上書きが入っている）を渡す

使い方（OMEN）:
  PYTHONPATH=$HOME/work_inoue/doorman6/VIRAL6 $HOME/miniforge3/envs/doorman6/bin/python export_vision_student.py \\
      --viral $HOME/work_inoue/doorman6/VIRAL6 --out $HOME/work_inoue/stage6_va/export_rand
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from hydra.utils import instantiate
from omegaconf import OmegaConf

EXP = "gr00t/rl/config/exp/wbmanip/door_open_homie_29dof_dagger-lstm.yaml"
OBS = "gr00t/rl/config/obs/wbmanip/door_open_homie_29dof_dagger.yaml"
SIM = "gr00t/rl/config/simulator/isaacsim.yaml"
# 生徒の観測（actor_obs）の項目の次元。観測は名前の辞書順につながる（legged_robot_base.py の sorted(obs_config)）
ACTOR_OBS_DIMS = {
    "actions": 29,
    "b_homie_commands": 7,
    "base_ang_vel": 3,
    "complete": 1,
    "delta_actions": 17,
    "dof_pos_non_finger": 29,
    "dof_vel_non_finger": 29,
    "projected_gravity": 3,
}
GOLDEN_STEPS = 10
TOL = 1e-4
MIN_SIGNAL = 100 * TOL  # 行動の振れ幅（標準偏差）がこれ未満だと、差 TOL の確かめが意味を持たない
NOCARRY_MIN = 1e-2  # 状態を持ち回さない版との差の下限。PC2 側の許容（1e-3）の 10 倍あれば、PC2 でも持ち回しの誤りが見える
LSTM_INIT = 0.25  # 乱数の重みのとき LSTM の重みをこの幅の一様乱数にする（既定の ±1/√256 だと記憶の効きが 1e-3 しか出ない）
HEAD_GAIN = 10.0  # 乱数の重みのとき出口の全結合を何倍にするか（既定だと行動の振れ幅が 0.06 で、記憶の効きも 8e-3 に縮む）
BIG_OBS = 40.0  # 正規化すると ±5 を越える値（切り捨てを働かせる）


class Encoder(nn.Module):
    """uint8 の画像 → 画像の特徴。正規化はシミュレータの get_rgb_image と同じ（/255 → ImageNet の平均と幅）"""

    def __init__(self, trunk: nn.Module, mean, std) -> None:
        super().__init__()
        self.trunk = trunk
        self.register_buffer("mean", torch.tensor(mean, dtype=torch.float32))
        self.register_buffer("std", torch.tensor(std, dtype=torch.float32))

    def forward(self, img_u8: torch.Tensor) -> torch.Tensor:
        x = img_u8.to(torch.float32) / 255.0
        x = (x - self.mean) / self.std
        return self.trunk(x.permute(0, 3, 1, 2))


class Recurrent(nn.Module):
    """観測の正規化（RunningMeanStd、±5 で切る）→ [観測, 画像の特徴] → LSTM → MLP"""

    def __init__(self, obs_mean, obs_var, eps: float, rnn: nn.LSTM, head: nn.Module) -> None:
        super().__init__()
        self.register_buffer("obs_mean", obs_mean.clone())
        self.register_buffer("obs_var", obs_var.clone())
        self.eps = eps
        self.rnn = rnn
        self.head = head

    def forward(self, obs, latent, h, c):
        o = torch.clamp((obs - self.obs_mean) / torch.sqrt(self.obs_var + self.eps), -5.0, 5.0)
        out, (h2, c2) = self.rnn(torch.cat([o, latent], dim=-1).unsqueeze(0), (h, c))
        return self.head(out[0]), h2, c2


class Full(nn.Module):
    def __init__(self, enc: Encoder, rec: Recurrent) -> None:
        super().__init__()
        self.enc = enc
        self.rec = rec

    def forward(self, img_u8, obs, h, c):
        return self.rec(obs, self.enc(img_u8), h, c)


def revert_sync_batchnorm(m: nn.Module) -> nn.Module:
    """VIRAL6 は ResNet の BN を SyncBatchNorm にしている。書き出し用に同じ値の BatchNorm2d に戻す"""
    if isinstance(m, nn.SyncBatchNorm):
        bn = nn.BatchNorm2d(m.num_features, m.eps, m.momentum, m.affine, m.track_running_stats)
        bn.load_state_dict(m.state_dict())
        return bn.eval()
    for name, child in m.named_children():
        setattr(m, name, revert_sync_batchnorm(child))
    return m


def md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def n_params(m: nn.Module) -> int:
    return int(sum(p.numel() for p in m.parameters()))


def load_configs(viral: Path, config: Path | None):
    """(exp を含む設定, 観測の設定, カメラの設定)。--config があればその回の最終の設定 1 つから全部取る"""
    if config is not None:
        cfg = OmegaConf.load(config)
        return cfg, cfg.obs.obs_dict, cfg.simulator.config.cameras
    return (OmegaConf.load(viral / EXP), OmegaConf.load(viral / OBS).obs.obs_dict,
            OmegaConf.load(viral / SIM).simulator.config.cameras)


def build_actor(viral: Path, config: Path | None):
    if not OmegaConf.has_resolver("eval"):
        OmegaConf.register_new_resolver("eval", eval)
    cfg, obs_dict, cams = load_configs(viral, config)
    terms = sorted(obs_dict.actor_obs)
    if terms != sorted(ACTOR_OBS_DIMS):
        raise SystemExit(f"生徒の観測の項目が想定と違う: {terms}")
    h, w = cfg.simulator.config.cameras.camera_resolutions
    env_config = OmegaConf.create(
        {"robot": {"actions_dim": int(cfg.robot.actions_dim),
                   "algo_obs_dim_dict": {"actor_obs": sum(ACTOR_OBS_DIMS.values()), "vision_obs": h * w * 3}}}
    )
    algo = cfg.algo.config
    if not algo.actor.get("running_mean_std", False):
        raise SystemExit("actor.running_mean_std が False の構成は想定していない（Recurrent が正規化を前提にしている）")
    actor = instantiate(algo.actor, env_config=env_config, algo_config=algo,
                        module_dim_dict=OmegaConf.to_container(algo.module_dim), _recursive_=False)
    return actor, (int(h), int(w)), list(cams.image_mean), list(cams.image_std)


def split(actor, mean, std):
    trunk = revert_sync_batchnorm(actor.vision_module.module).eval()
    rms = actor.running_mean_std
    rec = Recurrent(rms.running_mean, rms.running_var, rms.epsilon, actor.memory.rnn, actor.mlp_module.module)
    return Encoder(trunk, mean, std).eval(), rec.eval()


def golden_inputs(rng, n_obs: int, h: int, w: int):
    """乱数の画像と観測。2 ステップだけ一部の観測を桁違いにして、正規化の ±5 の切り捨てを働かせる"""
    imgs = rng.integers(0, 256, size=(GOLDEN_STEPS, h, w, 3), dtype=np.uint8)
    obs = rng.normal(0, 1, size=(GOLDEN_STEPS, n_obs)).astype(np.float32)
    obs[3, ::7] = BIG_OBS
    obs[7, 1::5] = -BIG_OBS
    return imgs, obs


@torch.no_grad()
def reference(actor, imgs_u8, obs, mean, std):
    """actor の rollout（VIRAL6 のコードそのまま）の答え。画像はシミュレータと同じく正規化した NHWC で渡す"""
    actor.memory.reset()
    m, s = torch.tensor(mean), torch.tensor(std)
    acts, lats = [], []
    for t in range(len(obs)):
        img = (torch.from_numpy(imgs_u8[t : t + 1]).float() / 255.0 - m) / s
        out = actor.forward({"actor_obs": torch.from_numpy(obs[t : t + 1]).clone(), "vision_obs": img})
        acts.append(out["actions"].reshape(-1).numpy())
        lats.append(actor.vision_module(img.permute(0, 3, 1, 2)).reshape(-1).numpy())
    return np.stack(acts), np.stack(lats)


@torch.no_grad()
def run_full(full, imgs_u8, obs, layers: int, hidden: int, carry: bool = True):
    h = torch.zeros(layers, 1, hidden)
    c = torch.zeros(layers, 1, hidden)
    acts = []
    for t in range(len(obs)):
        if not carry:
            h, c = torch.zeros_like(h), torch.zeros_like(c)
        a, h, c = full(torch.from_numpy(imgs_u8[t : t + 1]), torch.from_numpy(obs[t : t + 1]), h, c)
        acts.append(a.reshape(-1).numpy())
    return np.stack(acts)


@torch.no_grad()
def run_parts(enc, rec, imgs_u8, obs, lat_ref, layers: int, hidden: int):
    """画像の部分（画像 → 特徴）と記憶の部分（golden の特徴から行動）を別々に回す"""
    h = torch.zeros(layers, 1, hidden)
    c = torch.zeros(layers, 1, hidden)
    lats, acts = [], []
    for t in range(len(obs)):
        lats.append(enc(torch.from_numpy(imgs_u8[t : t + 1])).reshape(-1).numpy())
        a, h, c = rec(torch.from_numpy(obs[t : t + 1]), torch.from_numpy(lat_ref[t : t + 1]), h, c)
        acts.append(a.reshape(-1).numpy())
    return np.stack(lats), np.stack(acts)


def require(ok: bool, msg: str) -> None:
    print(("  ✔ " if ok else "  ✘ ") + msg)
    if not ok:
        raise SystemExit("確かめが通らないので書き出さない")


def check_split(actor, full, imgs, obs, ref_act, layers, hidden) -> dict:
    """抜き出した計算の確かめ（書き出す前）。ずれを見逃さない作りになっているかも同時に確かめる"""
    rms = actor.running_mean_std
    z = (torch.from_numpy(obs) - rms.running_mean) / torch.sqrt(rms.running_var + rms.epsilon)
    n_clamped = int((z.abs() > 5.0).sum())
    sig = float(ref_act.std())
    err = float(np.abs(run_full(full, imgs, obs, layers, hidden) - ref_act).max())
    err_nocarry = float(np.abs(run_full(full, imgs, obs, layers, hidden, carry=False) - ref_act).max())
    print("抜き出した計算と actor の rollout（10 ステップ、状態を持ち回し）:")
    require(n_clamped > 0, f"正規化の ±5 の切り捨てが働く入力がある（{n_clamped} 要素）")
    require(sig >= MIN_SIGNAL, f"行動の振れ幅（標準偏差 {sig:.3e}）が許容の差の 100 倍以上")
    require(err <= TOL, f"差 {err:.2e}（相対 {err / sig:.1e}）が {TOL} 以下")
    require(err_nocarry > NOCARRY_MIN, f"状態を持ち回さないと差 {err_nocarry:.2e}（{NOCARRY_MIN} 超＝持ち回しの確かめが効いている）")
    return {"n_clamped": n_clamped, "action_std": sig, "full_vs_actor": err, "nocarry_vs_actor": err_nocarry}


def check_saved(out: Path, imgs, obs, ref_act, ref_lat, layers, hidden) -> dict:
    """書き出した 3 本の TorchScript を読み直して確かめる"""
    full = torch.jit.load(str(out / "student_full.ts"))
    enc = torch.jit.load(str(out / "student_encoder.ts"))
    rec = torch.jit.load(str(out / "student_recurrent.ts"))
    e_full = float(np.abs(run_full(full, imgs, obs, layers, hidden) - ref_act).max())
    lats, acts = run_parts(enc, rec, imgs, obs, ref_lat, layers, hidden)
    e_enc = float(np.abs(lats - ref_lat).max())
    e_rec = float(np.abs(acts - ref_act).max())
    print("書き出した TorchScript を読み直して:")
    require(e_full <= TOL, f"全部 差 {e_full:.2e}")
    require(e_enc <= TOL, f"画像の部分 差 {e_enc:.2e}")
    require(e_rec <= TOL, f"記憶の部分 差 {e_rec:.2e}")
    return {"torchscript_full": e_full, "torchscript_encoder": e_enc, "torchscript_recurrent": e_rec}


def export_onnx(enc: Encoder, imgs, ref_lat, mean, std, out: Path) -> float:
    """画像の部分を ONNX（TensorRT 8.5 が読める opset 13、入力は正規化済みの NCHW）に書き、onnxruntime で確かめる"""
    x = ((torch.from_numpy(imgs[:1]).float() / 255.0 - torch.tensor(mean)) / torch.tensor(std)).permute(0, 3, 1, 2)
    x = x.contiguous()
    path = out / "student_encoder.onnx"
    torch.onnx.export(enc.trunk, (x,), str(path), opset_version=13,
                      input_names=["image"], output_names=["latent"], dynamo=False)
    x.numpy().astype(np.float32).tofile(out / "golden_image0.bin")
    import onnxruntime as ort  # OMEN の doorman6 にある。無ければ確かめられないので止まる

    sess = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    lat = sess.run(None, {"image": x.numpy()})[0].reshape(-1)
    err = float(np.abs(lat - ref_lat[0]).max())
    print("ONNX（onnxruntime）:")
    require(err <= TOL, f"画像の特徴の差 {err:.2e}")
    return err


def obs_layout() -> list:
    """観測 118 の並び（名前の辞書順）。実機で同じ順につなぐ"""
    layout, start = [], 0
    for name in sorted(ACTOR_OBS_DIMS):
        layout.append([name, start, start + ACTOR_OBS_DIMS[name]])
        start += ACTOR_OBS_DIMS[name]
    return layout


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--viral", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--ckpt", type=Path, default=None, help="学習済みの生徒の ckpt（無ければ乱数）")
    ap.add_argument("--config", type=Path, default=None, help="その回の最終の設定（Hydra の config.yaml）。--ckpt と一緒に")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    torch.manual_seed(a.seed)
    rng = np.random.default_rng(a.seed)

    actor, (H, W), mean, std = build_actor(a.viral, a.config)
    if a.ckpt is not None:
        sd = torch.load(a.ckpt, map_location="cpu", weights_only=False)["policy_state_dict"]
        actor.load_state_dict(sd, strict=True)
    else:
        # 観測の正規化が素通し（平均 0・分散 1）だと、正規化の写し間違いを確かめられないので値を入れておく
        n_obs = actor.running_mean_std.running_mean.numel()
        actor.running_mean_std.running_mean.copy_(torch.randn(n_obs) * 0.3)
        actor.running_mean_std.running_var.copy_(torch.rand(n_obs) + 0.5)
        with torch.no_grad():
            for name, prm in actor.memory.rnn.named_parameters():
                if name.startswith("weight"):
                    nn.init.uniform_(prm, -LSTM_INIT, LSTM_INIT)
            last = [m for m in actor.mlp_module.module if isinstance(m, nn.Linear)][-1]
            last.weight.mul_(HEAD_GAIN)
            last.bias.mul_(HEAD_GAIN)
    actor = actor.cpu().eval()
    rnn = actor.memory.rnn
    layers, hidden = rnn.num_layers, rnn.hidden_size

    imgs, obs = golden_inputs(rng, actor.running_mean_std.running_mean.numel(), H, W)
    ref_act, ref_lat = reference(actor, imgs, obs, mean, std)
    enc, rec = split(actor, mean, std)
    full = Full(enc, rec).eval()
    check = check_split(actor, full, imgs, obs, ref_act, layers, hidden)

    a.out.mkdir(parents=True, exist_ok=True)
    ex_img, ex_obs = torch.from_numpy(imgs[:1]), torch.from_numpy(obs[:1])
    ex_h = torch.zeros(layers, 1, hidden)
    with torch.no_grad():
        ex_lat = enc(ex_img)
        traced = {
            "student_full.ts": torch.jit.trace(full, (ex_img, ex_obs, ex_h, ex_h.clone())),
            "student_encoder.ts": torch.jit.trace(enc, (ex_img,)),
            "student_recurrent.ts": torch.jit.trace(rec, (ex_obs, ex_lat, ex_h, ex_h.clone())),
        }
    for name, mod in traced.items():
        mod.save(str(a.out / name))
    check.update(check_saved(a.out, imgs, obs, ref_act, ref_lat, layers, hidden))
    check["onnx_latent_vs_actor"] = export_onnx(enc, imgs, ref_lat, mean, std, a.out)

    np.savez_compressed(a.out / "golden.npz", imgs_u8=imgs, obs=obs, actions=ref_act, latent=ref_lat,
                        image_mean=np.array(mean, np.float32), image_std=np.array(std, np.float32))
    meta = {
        "exp": EXP, "config": str(a.config) if a.config else None, "ckpt": str(a.ckpt) if a.ckpt else None,
        "seed": a.seed, "image_hw": [H, W], "actor_obs_dim": int(obs.shape[1]), "actor_obs_order": obs_layout(),
        "latent_dim": int(ex_lat.shape[-1]), "actions_dim": int(ref_act.shape[-1]),
        "lstm": {"layers": layers, "hidden": hidden, "input": rnn.input_size},
        "params": {"encoder": n_params(enc), "recurrent": n_params(rec)},
        "image_mean": mean, "image_std": std, "torch": torch.__version__, "check": check,
        "md5": {p.name: md5(p) for p in sorted(a.out.iterdir()) if p.suffix in (".ts", ".onnx", ".npz", ".bin")},
    }
    (a.out / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=1))
    print(f"書き出した: {a.out}（画像の部分 {meta['params']['encoder']:,} 個・記憶の部分 {meta['params']['recurrent']:,} 個）")


if __name__ == "__main__":
    main()
