"""11-05: 画像だけの生徒（VIRAL6_vis の VisionDoorStudent。DAgger か PPO の last.pt）を実機用に書き出す。

書くもの（--out-dir に --name で）:
  <name>.npz          door_core.load_params が読む表（観測は f3sc の 151 の並び・鏡写し・HOMIE・定数。生徒の欄は教師 f3sc の重み）
                      ＋ door_vision_core.load_vision が読む画像の生徒（vis_*: LSTM・MLP・扉の頭・正規化・頭の重力の鏡写し）
  <name>_encoder.npz  画像の網を numpy で回す重み（BatchNorm を畳み込みに畳んだもの。door_vision_encoder.NumpyResNetEncoder）
  <name>_encoder.onnx 同じ網の onnx（入力 [1, 3, 108, 192] の正規化した RGB → [1, 128]）
最後に 3 つを突き合わせ、合わなければ書き出しを消して止まる:
  ① torch の VisionDoorStudent と door_vision_core（扉の値 17・行動 17 を 40 歩）  ② torch の画像の網と numpy・onnxruntime

OMEN の doorman6 の環境で動かす（torch・torchvision・onnx・onnxruntime と VIRAL6_vis のコードが要る）:
  cd ~/work_inoue/doorman6/VIRAL6_vis && PYTHONPATH=.:<この台本の場所> python <この台本の場所>/export_vision_student.py \\
      --ckpt logs_rl/vis_d2/.../last.pt --viral . --out-dir <出力先> --name door_vision_d2
  教師・HOMIE・画像の網の場所は重みの隣の config.yaml（teacher_actor_path・homie_*_model_path・vision_encoder_path）から取る
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import torch
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core  # noqa: E402
import door_vision_core as dvc  # noqa: E402
import door_vision_encoder as dve  # noqa: E402
import export_door_policy as edp  # noqa: E402

FEAT_KEY, HG_KEY = "vis_feat", "vis_head_gravity"


def md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def fold_bn(conv: torch.nn.Conv2d, bn: torch.nn.BatchNorm2d):
    """畳み込み（バイアス無し）＋BatchNorm（推論）→ 畳み込み 1 つの (w, b)"""
    s = bn.weight / torch.sqrt(bn.running_var + bn.eps)
    w = conv.weight * s[:, None, None, None]
    b = bn.bias - bn.running_mean * s
    return w.detach().numpy(), b.detach().numpy()


def encoder_arrays(enc) -> tuple:
    """DoorVisionEncoder（trunk = ResNet18 の [conv1, bn1, relu, maxpool, layer1..4, avgpool]）→ (arrays, meta)"""
    t = enc.trunk
    arrays = {}
    arrays["conv1_w"], arrays["conv1_b"] = fold_bn(t[0], t[1])
    blocks, strides = [], {}
    for li in range(1, 5):
        layer = t[3 + li]
        for bi, blk in enumerate(layer):
            name = f"l{li}b{bi}"
            blocks.append(name)
            strides[name] = int(blk.conv1.stride[0])
            arrays[f"{name}_c1_w"], arrays[f"{name}_c1_b"] = fold_bn(blk.conv1, blk.bn1)
            arrays[f"{name}_c2_w"], arrays[f"{name}_c2_b"] = fold_bn(blk.conv2, blk.bn2)
            if blk.downsample is not None:
                arrays[f"{name}_down_w"], arrays[f"{name}_down_b"] = fold_bn(blk.downsample[0], blk.downsample[1])
    arrays["proj_w"] = enc.proj.weight.detach().numpy()
    arrays["proj_b"] = enc.proj.bias.detach().numpy()
    arrays["ln_w"] = enc.norm.weight.detach().numpy()
    arrays["ln_b"] = enc.norm.bias.detach().numpy()
    return arrays, {"blocks": blocks, "strides": strides, "ln_eps": float(enc.norm.eps)}


def student_module(cfg: dict, feat_dim: int, sd: dict):
    """config の観測の組から VisionDoorStudent を作り、重みを読む（突き合わせの基準）"""
    from omegaconf import OmegaConf

    from gr00t.rl.trl.modules.vision_door_student import VisionDoorStudent

    groups = {g: list(cfg["obs"]["obs_dict"][g]) for g in ("actor_obs", "teacher_obs")}
    dims = dict(edp.TERM_DIMS, **{FEAT_KEY: feat_dim, HG_KEY: 3})
    env_config = OmegaConf.create({
        "obs": {"obs_dims": dims, "obs_dict": groups},
        "robot": {"algo_obs_dim_dict": {g: sum(dims[t] for t in ts) for g, ts in groups.items()}, "actions_dim": 29},
    })
    actor = cfg["algo"]["config"]["actor"]
    mlp = actor["backbone"]["mlp_module"]
    backbone = OmegaConf.create({"mlp_module": {
        "_target_": mlp["_target_"], "process_output_dim": True,
        "module_config_dict": {"input_dim": ["actor_obs"], "output_dim": [door_core.NUM_ACT],
                               "layer_config": dict(mlp["module_config_dict"]["layer_config"])}}})
    algo_config = OmegaConf.create({"init_noise_std": 0.1})
    trk_w = sd.get("tracker.rnn.weight_hh_l0")
    stu = VisionDoorStudent(env_config, algo_config, backbone, rnn_num_layers=door_core.LSTM_LAYERS,
                            angle_age=float(actor.get("angle_age", 0.15)), init_from=None,
                            door_tracker_hidden=int(trk_w.shape[1]) if trk_w is not None else 0)
    stu.load_state_dict(sd, strict=True)
    return stu.double().eval()


def check_student(stu, p: door_core.Params, vp: dvc.VisionParams, steps: int = 40) -> tuple:
    """torch の生徒と numpy を同じ入力で回し、扉の値と行動の差の最大を返す"""
    from gr00t.rl.trl.modules.vision_door_student import _term_slices

    groups = stu._assemble
    rng = np.random.default_rng(3)
    lstm = stu.memory.rnn
    th = None
    h = np.zeros((door_core.LSTM_LAYERS, door_core.LSTM_HIDDEN))
    c = np.zeros_like(h)
    pstate = None  # 追う段の (h, c)（door_core.CoreState.policy_state と同じ持ち越し）
    worst_d, worst_a = 0.0, 0.0
    s_dim = stu.student_dim
    with torch.no_grad():
        for _ in range(steps):
            s_obs = rng.normal(size=s_dim) * 0.5
            s_obs[stu.hg_slice[0]:stu.hg_slice[1]] = rng.normal(size=3) * 0.2 + np.array([0.7, 0.0, -0.7])
            # numpy の側: 観測 151（教師の並び）を生徒の観測から組む。扉の 3 項目は乱数（差し替えられるはず）
            obs = rng.normal(size=p.obs_dim) * 3.0
            ti = 0
            for src, a, b in groups:
                if src == "s":
                    obs[ti:ti + (b - a)] = s_obs[a:b]
                ti += b - a
            feat = s_obs[stu.feat_slice[0]:stu.feat_slice[1]]
            hg = s_obs[stu.hg_slice[0]:stu.hg_slice[1]]
            act = dvc.VisionActor(vp, feat, hg)
            obs_v, pstate = act.prepare(obs, pstate)
            y_np, h, c = act.forward(obs_v, h, c)
            # torch の側（追う段があれば、その記憶は生徒の中で持ち越す）
            st = torch.from_numpy(s_obs)[None]
            d17 = stu._door17({"actor_obs": st}, st)
            x = stu._lstm_input(st, d17)
            out, th = lstm(x[None], th)
            y_t = stu.mlp_module(out[0])[0].numpy()
            worst_d = max(worst_d, float(np.abs(dvc.door_from_obs(vp, obs_v) - d17[0].numpy()).max()))
            worst_a = max(worst_a, float(np.abs(y_np - y_t).max()))
    return worst_d, worst_a


def check_encoder(enc, np_enc, onnx_path: Path, images: np.ndarray) -> tuple:
    """torch の網と numpy・onnxruntime の特徴の差の最大（images: uint8 [N, 108, 192, 3]）"""
    ort_enc = dve.OrtEncoder(str(onnx_path), threads=4)
    w_np, w_ort = 0.0, 0.0
    with torch.no_grad():
        for img in images:
            x = dve.preprocess_rgb(img)
            ref = enc.forward_normalized(torch.from_numpy(x)[None].float())[0].double().numpy()
            w_np = max(w_np, float(np.abs(np_enc(x) - ref).max()))
            w_ort = max(w_ort, float(np.abs(ort_enc(x) - ref).max()))
            ref_u8 = enc(torch.from_numpy(img)[None])[0].double().numpy()  # 前処理も網の中のものと同じか
            w_np = max(w_np, float(np.abs(np_enc(x) - ref_u8).max()))
    return w_np, w_ort


def sample_images(path: str, n: int) -> np.ndarray:
    """学習の記録（door_vision_feat.VisionDumper の shard）から画像を取る。無ければ乱数の画像"""
    if path:
        z = np.load(path)
        idx = np.linspace(0, len(z["rgb"]) - 1, n).astype(int)
        return z["rgb"][idx]
    return np.random.default_rng(0).integers(0, 256, (n, dve.IMG_H, dve.IMG_W, 3), dtype=np.uint8)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ckpt", required=True, help="画像だけの生徒の last.pt（DAgger か PPO）")
    ap.add_argument("--config", default=None, help="既定は重みの隣の config.yaml")
    ap.add_argument("--teacher", default=None, help="f3sc の last.pt（既定は config の teacher_actor_path）")
    ap.add_argument("--walk", default=None)
    ap.add_argument("--stand", default=None)
    ap.add_argument("--encoder", default=None, help="V2 の出力（既定は config の vision_encoder_path）")
    ap.add_argument("--viral", required=True, help="VIRAL6_vis の場所（door_mirror.py を読む）")
    ap.add_argument("--images", default="", help="突き合わせに使う画像の shard（学習の記録）。無ければ乱数の画像")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--name", required=True)
    a = ap.parse_args()

    ckpt = Path(a.ckpt)
    cfg = yaml.safe_load(open(a.config or ckpt.parent / "config.yaml", encoding="utf-8"))
    viral = Path(a.viral)
    teacher = a.teacher or cfg["teacher_actor_path"]
    algo = cfg["algo"]["config"]
    walk = a.walk or str(viral / algo["homie_walk_model_path"])
    stand = a.stand or str(viral / algo["homie_stand_model_path"])
    enc_path = a.encoder or cfg["vision_encoder_path"]
    out_dir = Path(a.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # --- 土台: f3sc の観測の並び（teacher_obs）・鏡写し・機体の表・HOMIE。生徒の欄は教師 f3sc の重み ---
    layout, obs_dim = edp.obs_layout(cfg["obs"], "teacher_obs")
    M, b, A, mirror = edp.mirror_tables(cfg, viral, layout, obs_dim)
    robot_arr, consts = edp.robot_tables(cfg)
    if np.abs(robot_arr["q_default"][dvc.WAIST]).max() > 0:
        raise ValueError("腰の既定の角度が 0 でない（扉の頭は V2 で生の腰の角度、方策では既定からの差を見ている）")
    tck = edp.load_torch(teacher)
    PT = {k: v.double() for k, v in tck["policy_state_dict"].items()}
    if PT["memory.rnn.weight_ih_l0"].shape[1] != obs_dim:
        raise ValueError(f"教師の観測の幅 {PT['memory.rnn.weight_ih_l0'].shape[1]} が {obs_dim} と合わない")
    homie_arr, homie_sd = edp.homie_arrays(walk, stand)

    # --- 画像の生徒 ---
    sck = edp.load_torch(str(ckpt))
    S = {k: v.double() for k, v in sck["policy_state_dict"].items()}
    if "door_head.net.0.weight" not in S:
        raise ValueError(f"{ckpt} は画像だけの生徒の重みでない")
    feat_dim = int(S["memory.rnn.weight_ih_l0"].shape[1]) - obs_dim - 3
    vis = {"vis_t_mean": S["t_mean"].numpy(), "vis_t_var": S["t_var"].numpy(),
           "vis_head_out_mean": S["door_head.out_mean"].numpy(), "vis_head_out_std": S["door_head.out_std"].numpy()}
    for i in range(door_core.LSTM_LAYERS):
        for k in ("w_ih", "w_hh", "b_ih", "b_hh"):
            vis[f"vis_lstm{i}_{k}"] = S[f"memory.rnn.{k.replace('w_', 'weight_').replace('b_', 'bias_')}_l{i}"].numpy()
    for j, idx in enumerate((0, 2, 4, 6)):
        vis[f"vis_actor{j}_w"] = S[f"mlp_module.module.{idx}.weight"].numpy()
        vis[f"vis_actor{j}_b"] = S[f"mlp_module.module.{idx}.bias"].numpy()
    head_idx = sorted({int(k.split(".")[2]) for k in S if k.startswith("door_head.net.") and k.endswith(".weight")})
    for j, idx in enumerate(head_idx):
        vis[f"vis_head{j}_w"] = S[f"door_head.net.{idx}.weight"].numpy()
        vis[f"vis_head{j}_b"] = S[f"door_head.net.{idx}.bias"].numpy()
    hg_M, hg_b = edp.linear_map(lambda x: mirror.obs(HG_KEY, x.float()), 3)
    if np.abs(hg_b).max() > 0:
        raise ValueError("頭の重力の鏡写しに定数項がある")
    vis["vis_hg_mirror_M"] = hg_M
    trk_hidden = 0
    if "tracker.rnn.weight_ih_l0" in S:  # 11-05 追う段（d3 から）
        trk_hidden = int(S["tracker.rnn.weight_hh_l0"].shape[1])
        for k in ("w_ih", "w_hh", "b_ih", "b_hh"):
            vis[f"vis_trk_{k}"] = S[f"tracker.rnn.{k.replace('w_', 'weight_').replace('b_', 'bias_')}_l0"].numpy()
        vis["vis_trk_out_w"] = S["tracker_out.weight"].numpy()
        vis["vis_trk_out_b"] = S["tracker_out.bias"].numpy()
        from gr00t.rl.envs.door.door_vision_feat import door_truth_weight

        vis["vis_door_tol"] = door_truth_weight().rsqrt().double().numpy()  # 生徒の door_tol（float32 で作って倍精度へ）と同じ値

    cams = cfg["simulator"]["config"]["cameras"]
    dr = cfg.get("domain_rand", {})
    vf = cfg["env"]["config"].get("vision_feat", {})
    actor = algo["actor"]
    stem = out_dir / a.name
    meta = {
        "layout": layout,
        "obs_dim": obs_dim,
        "mirror_left_doors": bool(cfg["env"]["config"].get("mirror_left_doors", False)),
        "constants": consts,
        "dof_names": list(cfg["robot"]["dof_names"]),
        "source": {"ckpt": str(teacher), "ckpt_sha256": edp.sha256(teacher), "walk_sha256": edp.sha256(walk),
                   "stand_sha256": edp.sha256(stand), "global_step": edp.ckpt_global_step(tck),
                   "note": "生徒の欄（lstm・actor・rms）は教師 f3sc。画像の生徒は vis_*"},
        "vision": {
            "feat_dim": feat_dim,
            "angle_age": float(actor.get("angle_age", 0.15)),
            "head_layers": len(head_idx),
            "tracker_hidden": trk_hidden,
            "student_ckpt": str(ckpt),
            "student_sha256": edp.sha256(str(ckpt)),
            "student_global_step": edp.ckpt_global_step(sck),
            "encoder_src": str(enc_path),
            "encoder_src_sha256": edp.sha256(enc_path),
            "encoder_npz": f"{a.name}_encoder.npz",
            "encoder_onnx": f"{a.name}_encoder.onnx",
            "every_n": int(vf.get("every_n", 5)),
            "camera": {
                "resolution_hw": list(cams["camera_resolutions"]),
                "focal_length": float(cams["camera_focal_length"]),
                "horizontal_aperture": float(cams["camera_horizontal_aperture"]),
                "vertical_aperture": float(cams["camera_vertical_aperture"]),
                "rot_offset_wxyz": list(cams["camera_rot_offset"]),
                "pos_offset": list(cams["camera_pos_offset"]),
                "attached_link": cams["camera_attached_link"],
                "pitch_range_deg": list(dr.get("camera_pitch_range_deg", [])),
                "hfov_deg": float(np.degrees(2 * np.arctan(cams["camera_horizontal_aperture"] / 2 / cams["camera_focal_length"]))),
                "vfov_deg": float(np.degrees(2 * np.arctan(cams["camera_vertical_aperture"] / 2 / cams["camera_focal_length"]))),
            },
        },
    }
    arrays = {"obs_mirror_M": M, "obs_mirror_b": b, "act_mirror": A, **edp.policy_arrays(PT), **robot_arr, **homie_arr, **vis}
    npz = stem.with_suffix(".npz")
    np.savez(npz, meta_json=np.array(json.dumps(meta, ensure_ascii=False)), **arrays)

    # --- 画像の網: numpy 用の npz と onnx ---
    from gr00t.rl.envs.door.door_vision_feat import load_encoder

    enc = load_encoder(enc_path, feat_dim, "cpu")
    enc_arr, enc_meta = encoder_arrays(enc)
    enc_npz = out_dir / f"{a.name}_encoder.npz"
    np.savez(enc_npz, meta_json=np.array(json.dumps(dict(enc_meta, src=str(enc_path)), ensure_ascii=False)), **enc_arr)
    onnx_path = out_dir / f"{a.name}_encoder.onnx"

    class _Norm(torch.nn.Module):
        def __init__(self, e) -> None:
            super().__init__()
            self.e = e

        def forward(self, x):
            return self.e.forward_normalized(x)

    torch.onnx.export(_Norm(enc).eval(), torch.zeros(1, 3, dve.IMG_H, dve.IMG_W), str(onnx_path),
                      input_names=["image"], output_names=["feat"], opset_version=17, dynamo=False)

    # --- 突き合わせ ---
    p = door_core.load_params(str(npz))
    vp = dvc.load_vision(str(npz), p)
    worst_t = edp.check_actor(p, PT, obs_dim)
    worst_h = edp.check_homie(p, homie_sd)
    stu = student_module(cfg, feat_dim, {k: v for k, v in sck["policy_state_dict"].items()})
    worst_d, worst_a = check_student(stu, p, vp)
    np_enc = dve.NumpyResNetEncoder(str(enc_npz))
    w_np, w_ort = check_encoder(enc, np_enc, onnx_path, sample_images(a.images, 8))
    ms_np = dve.bench(np_enc, 10)
    print(f"[export] 観測 {obs_dim}（教師の並び）・特徴 {feat_dim}・扉の頭 {len(head_idx)} 層・追う段 {trk_hidden or 'なし'}・"
          f"鏡写し {meta['mirror_left_doors']}")
    print(f"[export] カメラ {meta['vision']['camera']['hfov_deg']:.1f}°×{meta['vision']['camera']['vfov_deg']:.1f}°・"
          f"{meta['vision']['camera']['resolution_hw']}・俯角の乱択 {meta['vision']['camera']['pitch_range_deg']}")
    print(f"[export] torch と numpy の差の最大: 教師 {worst_t:.1e}・HOMIE {worst_h:.1e}・扉の値 {worst_d:.1e}・行動 {worst_a:.1e}・"
          f"画像の網 numpy {w_np:.1e}・onnx {w_ort:.1e}（numpy の網 {ms_np:.0f} ms/枚・この機械）")
    bad = worst_t > 1e-8 or worst_h > 1e-8 or worst_d > 1e-6 or worst_a > 1e-6 or w_np > 2e-3 or w_ort > 2e-3
    if bad:
        for f in (npz, enc_npz, onnx_path):
            f.unlink(missing_ok=True)
        print("[export] 差が大きいので書き出しを取り消した")
        return 1
    (out_dir / f"{a.name}_meta.json").write_text(json.dumps(
        dict(meta["vision"], md5={f.name: md5(f) for f in (npz, enc_npz, onnx_path)}), ensure_ascii=False, indent=1),
        encoding="utf-8")
    print(f"[export] 書き出した: {npz}・{enc_npz.name}・{onnx_path.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
