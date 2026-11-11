"""11-10 手順 L1 (b): 学び直しの網（door_vision_pretrain.py の出力 .pt）を、実物の束（real_shard_1110.py の npz）だけで当てる。

door_vision_pretrain.py --eval_only はシムの束を全部読み込む（RAM 約 7 GB）ので、s2f の学習と並べられない。
同じ台本の load_real・evaluate をそのまま呼び、実物の束だけを CPU で当てる（GPU は学習が使っているので触らない）。
出力は --eval_only と同じ形の <出力>_realval.npz（names・pred:<名前>・truth:<名前>）→ real_eval_1110.py --pred で正解と比べる。
⚠️ CPU では autocast（bfloat16）が切れる。GPU の当てとの差は 18:59 の束で確かめる（--check に前の _realval.npz を渡す）。

使い方（OMEN・VIRAL6_vis で）:
  PYTHONPATH=. python real_pred_pt_1110.py --init r2b_c2.pt --real_val real_1109_x.npz ... --out l1b_c2_realval.npz \\
      [--check r2b_c2_realval.npz] [--threads 4]
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import torch

from gr00t.rl.scripts import door_vision_pretrain as dvp


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--init", required=True)
    ap.add_argument("--real_val", nargs="+", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--feat_dim", type=int, default=128)
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--check", default=None, help="前の _realval.npz（同じ束の GPU の当て）。差の最大を出す")
    a = ap.parse_args(argv)
    torch.set_num_threads(a.threads)
    device = torch.device("cpu")
    t0 = time.time()
    enc = dvp.DoorRGBDEncoder(a.feat_dim, pretrained=False).to(device)
    head = dvp.DoorHead(a.feat_dim).to(device)
    ck = torch.load(a.init, map_location="cpu", weights_only=False)
    enc.load_state_dict(ck["encoder"])
    head.load_state_dict(ck["head"])
    tol = torch.tensor(dvp.DOOR_TRUTH_TOL, device=device)
    dump = {}
    for path in a.real_val:
        name = os.path.basename(path)
        if f"pred:{name}" in dump:
            raise SystemExit(f"[pred] 同じ名前の束が 2 つある（{name}）。real_eval_1110.py --pred-key で区別できない")
        rv = dvp.load_real([path])
        preds: list = []
        res = dvp.evaluate(enc, head, rv, np.arange(len(rv["truth"])), device, tol, a.batch, noise=False, preds=preds)
        dump[f"pred:{name}"] = np.concatenate(preds)
        dump[f"truth:{name}"] = rv["truth"]
        if len(dump[f"pred:{name}"]) != len(rv["truth"]):
            raise SystemExit(f"[pred] {name}: 当ての行 {len(dump[f'pred:{name}'])} と束の枚 {len(rv['truth'])} が違う")
        print(f"[pred] {name}: {dvp.fmt(res)}", flush=True)
        if a.check:
            with np.load(a.check) as z:
                key = f"pred:{name}"
                if key in z.files:
                    d = np.abs(z[key] - dump[key])
                    print(f"[pred]   GPU の当てとの差: 位置 最大 {100 * d[:, 1:4].max():.2f} cm・中央 {100 * np.median(d[:, 1:4]):.3f} cm", flush=True)
    names = [os.path.basename(p) for p in a.real_val]
    np.savez(a.out, names=np.array(names), **dump)
    print(f"[pred] 書いた: {a.out}（{time.time() - t0:.0f} 秒）", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
