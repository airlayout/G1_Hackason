"""11-10 手順 L6（計画書 9.4 節）: 段 0 の扉の測りに使う目の網 c2（door_vision_pretrain.py の出力 .pt）を実機用に書き出す。

実機の走らせ役は、画像の網を ONNX（door_vision_encoder.OrtRGBDEncoder）で、扉の頭を npz（door_vision_core.load_vision の vis_head*）で回す。
c2 は「網＋扉の頭」の .pt なので、
  <name>_encoder.onnx … c2 の網（入力 rgb [1,3,108,192]・depth [1,2,54,96]・color_off [1] → feat [1,128]。export_vision_student と同じ形）
  <name>.npz          … 実機の e2b の npz の写しで、扉の頭の欄（vis_head{0,1,2}_w/b・vis_head_out_mean/std）だけ c2 に替えたもの
                        ⚠️ 生徒の欄（vis_lstm・vis_actor）は e2b のまま。特徴が変わるので生徒の方策としては使えない（段 0 の扉の測り専用）
  <name>_meta.json    … 出どころと md5
最後に実物の束（real_shard_1110.py の npz）の枚で突き合わせ、合わなければ書き出しを消して止まる:
  ① torch の網と onnxruntime の特徴の差  ② torch の「網＋頭」と、onnx の特徴＋numpy の頭（door_vision_core）の扉の値 17 の差（線 1e-4）

使い方（OMEN・VIRAL6_vis で・PYTHONPATH=.:<この台本の場所>）:
  python export_c2_1110.py --pt r2b_c2.pt --npz door_vision_e2b.npz --check real_185929_S.npz --out-dir out --name door_vision_c2
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Dict

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

# DoorHead.net の Linear（net.0・net.2・net.4。間は SiLU）と、npz の名前（make_pretrain_init_1110.py と同じ対応）
HEAD_LAYERS = (("net.0", "vis_head0"), ("net.2", "vis_head1"), ("net.4", "vis_head2"))
TOL_FEAT = 2e-3  # onnx と torch の特徴（export_vision_student と同じ線）
TOL_DOOR = 1e-4  # 扉の値 17（計画書 L6 の線）


def md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def replace_head(arrays: Dict[str, np.ndarray], head: Dict[str, np.ndarray], src: str, onnx_name: str) -> Dict[str, np.ndarray]:
    """e2b の npz の中身（arrays）の扉の頭を head（DoorHead の state_dict を numpy にしたもの）で替えた新しい dict を返す（元は変えない）"""
    out = dict(arrays)
    for tname, nname in HEAD_LAYERS:
        for suffix, key in (("weight", "w"), ("bias", "b")):
            old, new = arrays[f"{nname}_{key}"], head[f"{tname}.{suffix}"]
            if old.shape != new.shape:
                raise ValueError(f"扉の頭 {nname}_{key} の形が違う: e2b {old.shape}・c2 {new.shape}")
            out[f"{nname}_{key}"] = np.asarray(new, dtype=old.dtype)
    for nname, tname in (("vis_head_out_mean", "out_mean"), ("vis_head_out_std", "out_std")):
        if arrays[nname].shape != head[tname].shape:
            raise ValueError(f"{nname} の形が違う")
        out[nname] = np.asarray(head[tname], dtype=arrays[nname].dtype)
    meta = json.loads(str(arrays["meta_json"]))
    if "vision" not in meta:
        raise ValueError("e2b の npz に meta の vision が無い")
    vis = dict(meta["vision"], encoder_onnx=onnx_name, door_head_from=src,
               note="11-10 L6: 扉の頭と網は c2。生徒の欄（vis_lstm・vis_actor）は e2b のままで、生徒の方策には使えない（段 0 の扉の測り専用）")
    out["meta_json"] = np.array(json.dumps(dict(meta, vision=vis), ensure_ascii=False))
    return out


def main(argv=None) -> int:  # noqa: C901
    import torch

    import door_core as dc
    import door_vision_core as dvc
    import door_vision_encoder as dve
    from gr00t.rl.envs.door.door_vision_feat import DoorHead, DoorRGBDEncoder

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pt", required=True)
    ap.add_argument("--npz", required=True, help="実機の e2b の npz（扉の頭以外はこれのまま）")
    ap.add_argument("--check", required=True, help="突き合わせに使う実物の束（real_shard_1110.py の npz）")
    ap.add_argument("--n-check", type=int, default=64)
    ap.add_argument("--feat-dim", type=int, default=128)
    ap.add_argument("--out-dir", required=True, type=Path)
    ap.add_argument("--name", default="door_vision_c2")
    a = ap.parse_args(argv)
    a.out_dir.mkdir(parents=True, exist_ok=True)
    npz = a.out_dir / f"{a.name}.npz"
    onnx_path = a.out_dir / f"{a.name}_encoder.onnx"
    meta_path = a.out_dir / f"{a.name}_meta.json"
    for f in (npz, onnx_path, meta_path):
        if f.exists():
            raise SystemExit(f"[c2] 出力が既にある（上書きしない）: {f}")

    ck = torch.load(a.pt, map_location="cpu", weights_only=False)
    enc = DoorRGBDEncoder(a.feat_dim, pretrained=False).eval()
    enc.load_state_dict(ck["encoder"])
    head = DoorHead(a.feat_dim).eval()
    head.load_state_dict(ck["head"])

    class _RGBD(torch.nn.Module):
        def __init__(self, e) -> None:
            super().__init__()
            self.e = e

        def forward(self, rgb, depth, color_off):
            return self.e.forward_normalized(rgb, depth, color_off > 0.5)

    torch.onnx.export(_RGBD(enc).eval(), (torch.zeros(1, 3, dve.IMG_H, dve.IMG_W), torch.zeros(1, 2, dve.DEPTH_H, dve.DEPTH_W),
                                          torch.zeros(1)),
                      str(onnx_path), input_names=["rgb", "depth", "color_off"], output_names=["feat"], opset_version=17, dynamo=False)
    with np.load(a.npz, allow_pickle=False) as z:
        arrays = {k: z[k] for k in z.files}
    head_np = {k: v.detach().double().numpy() for k, v in head.state_dict().items()}
    np.savez(npz, **replace_head(arrays, head_np, f"{a.pt}（md5 {md5(Path(a.pt))}）", onnx_path.name))

    # 突き合わせ: 実物の枚（束の先頭から n 枚を飛び飛びに）
    with np.load(a.check) as zc:
        sel = np.linspace(0, len(zc["truth"]) - 1, min(a.n_check, len(zc["truth"]))).round().astype(int)
        rgb_u8, depth_m, aux = zc["rgb"][sel], zc["depth"][sel].astype(np.float32), zc["aux"][sel].astype(np.float64)
    p = dc.load_params(str(npz))
    vp = dvc.load_vision(str(npz), p)
    ort_enc = dve.OrtRGBDEncoder(str(onnx_path), threads=4)
    worst_f, worst_d = 0.0, 0.0
    with torch.no_grad():
        for k in range(len(sel)):
            x_rgb = dve.preprocess_rgb(rgb_u8[k])
            x_dep = dve.depth_input(depth_m[k])
            f_t = enc.forward_normalized(torch.from_numpy(x_rgb)[None], torch.from_numpy(x_dep)[None],
                                         torch.zeros(1, dtype=torch.bool)).double()
            d_t = (head(f_t.float(), torch.from_numpy(aux[k])[None].float()).double().numpy()[0])
            f_o = ort_enc(x_rgb, x_dep, False)
            d_o = vp.head(np.concatenate([f_o, aux[k]])) * vp.head_out_std + vp.head_out_mean
            worst_f = max(worst_f, float(np.abs(f_o - f_t.numpy()[0]).max()))
            worst_d = max(worst_d, float(np.abs(d_o - d_t).max()))
    print(f"[c2] 突き合わせ {len(sel)} 枚（{a.check}）: 特徴 onnx − torch 最大 {worst_f:.1e}（線 {TOL_FEAT:.0e}）・"
          f"扉の値 17（onnx＋numpy の頭 − torch）最大 {worst_d:.1e}（線 {TOL_DOOR:.0e}）", flush=True)
    if worst_f > TOL_FEAT or worst_d > TOL_DOOR:
        for f in (npz, onnx_path):
            f.unlink(missing_ok=True)
        print("[c2] 差が大きいので書き出しを取り消した", flush=True)
        return 1
    meta_path.write_text(json.dumps({"pt": a.pt, "base_npz": a.npz, "check": a.check, "n_check": int(len(sel)),
                                     "worst_feat": worst_f, "worst_door17": worst_d,
                                     "md5": {f.name: md5(f) for f in (npz, onnx_path)}}, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"[c2] 書き出した: {npz}・{onnx_path.name}・{meta_path.name}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
