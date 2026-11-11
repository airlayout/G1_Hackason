"""11-10 手順 R2 の準備: 学び直し（door_vision_pretrain.py --init）の始めの重みを、実機で使った網と頭からそろえる。

実機の e2b（door_vision_e2b.npz）の扉の頭は、DAgger の間に学び直したもので、画像の網の元（pretrain_rgbd_1106_1117.pt）の頭と違う
（11-10 に確かめた: 網は pt・ONNX・実機の記録の特徴が 1e-4 まで一致、頭の重みは最大 1.0 違う）。実物では e2b の頭の方が良い
（18:59 の左右の平均の外れ 5.1 cm 対 14.9 cm）ので、学び直しは「網は pt・頭は e2b」から始める。

出力: {"encoder": pt の網, "head": e2b の頭（DoorHead の state_dict の形）, "encoder_kind": "rgbd", "src": 出どころ}

使い方（OMEN の VIRAL6_vis で・PYTHONPATH=.）:
  python make_pretrain_init_1110.py --pt pretrain_rgbd_1106_1117.pt --npz door_vision_e2b.npz --out init_e2b.pt
"""
from __future__ import annotations

import argparse

import numpy as np
import torch

# DoorHead.net の Linear の名前（net.0・net.2・net.4。間は SiLU）と、export_vision_student.py の書き出しの名前
HEAD_LAYERS = (("net.0", "vis_head0"), ("net.2", "vis_head1"), ("net.4", "vis_head2"))


def head_state_from_npz(z, like: dict) -> dict:
    """e2b の npz の頭 → DoorHead の state_dict（形は like に合わせる。書き出しが転置なら戻す）"""
    out = {}
    for tname, nname in HEAD_LAYERS:
        for suffix, key in (("weight", "w"), ("bias", "b")):
            ref = like[f"{tname}.{suffix}"]
            arr = z[f"{nname}_{key}"]
            if arr.shape != tuple(ref.shape):
                arr = arr.T
            if arr.shape != tuple(ref.shape):
                raise SystemExit(f"[init] {nname}_{key} の形 {z[f'{nname}_{key}'].shape} が {tname}.{suffix} {tuple(ref.shape)} と合わない")
            out[f"{tname}.{suffix}"] = torch.from_numpy(np.ascontiguousarray(arr)).to(ref.dtype)
    out["out_mean"] = torch.from_numpy(z["vis_head_out_mean"]).to(like["out_mean"].dtype)
    out["out_std"] = torch.from_numpy(z["vis_head_out_std"]).to(like["out_std"].dtype)
    missing = set(like) - set(out)
    if missing:
        raise SystemExit(f"[init] 頭の項目が足りない: {sorted(missing)}")
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pt", required=True, help="画像の網の元（door_vision_pretrain.py の出力）")
    ap.add_argument("--npz", required=True, help="実機で使った書き出し（export_vision_student.py の出力）")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    ck = torch.load(a.pt, map_location="cpu", weights_only=False)
    z = np.load(a.npz, allow_pickle=False)
    head = head_state_from_npz(z, ck["head"])
    diff = max(float((head[k] - ck["head"][k]).abs().max()) for k in head)
    torch.save({"encoder": ck["encoder"], "head": head, "encoder_kind": ck.get("encoder_kind", "rgbd"),
                "src": {"encoder": a.pt, "head": a.npz}}, a.out)
    print(f"[init] {a.out}: 網は {a.pt}・頭は {a.npz}（元の頭との差の最大 {diff:.3f}）", flush=True)


if __name__ == "__main__":
    main()
