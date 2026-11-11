"""11-10 手順 R2 の準備: 実物の記録の枚を、シムの RGB-D の束（vis_1104/ds_dep/*/frames/shard_*.npz）と同じ形の npz に書き出す。

学び直しの台本（door_vision_pretrain.py、パッチ 0103 の --real_data・--real_val）がそのまま読める形:
  rgb    (n, 108, 192, 3) uint8   網に入れたカラー（実機の記録のまま）
  depth  (n, 54, 96) float16      網に入れた深度 [m]・0 は測れない（実物なので揺れは足さない）
  aux    (n, 9) float32           [頭の重力 3, 胴の重力 3, 腰 3]（鏡写しの側。door_vision_core.door_values と同じ並び）
  truth  (n, 17) float32          扉の値の正解（鏡写しの側。real_truth_1110.py の truth17_m）
  stage  (n,) float32             −1（実物。シムの段 0〜5 とは別）
  env_id (n,) float32             0
  ほかに seq・t_capture・src（正解の出どころ）・meta_json

走らせ役が観測を作っていない枚（WAIT など）と、正解の無い枚は書かない。

使い方（OMEN の real_ana_1106/・doorman6 の Python）:
  python real_shard_1110.py --truth truth_183336_B.npz --vision <vision_*.npz> --stock <door_stock_*.npz> \
      --policy door_vision_e2b.npz --out real_183336.npz
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import door_vision_core as dvc  # noqa: E402
from real_eval_1110 import stock_rows_by_seq  # noqa: E402

REAL_STAGE = -1.0


def build_aux(vp: dvc.VisionParams, obs: np.ndarray, hg_m: np.ndarray) -> np.ndarray:
    """扉の頭の手掛かり 9（door_vision_core.door_values と同じ並び）"""
    return np.concatenate([hg_m, obs[vp.pg_slice[0]:vp.pg_slice[1]], obs[vp.waist_slice[0]:vp.waist_slice[1]]])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--truth", required=True)
    ap.add_argument("--vision", required=True)
    ap.add_argument("--stock", required=True)
    ap.add_argument("--policy", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--keep", action="append", default=[],
                    help="t_capture の範囲 t0:t1（いくつでも）。渡せばその範囲の枚だけ書く（画から正解が読めない地点を外すとき）")
    a = ap.parse_args(argv)

    zt = np.load(a.truth, allow_pickle=False)
    meta = json.loads(str(zt["meta_json"]))
    p = dc.load_params(a.policy, mirror=not meta["lever_right"])
    vp = dvc.load_vision(a.policy, p)
    zv = np.load(a.vision, allow_pickle=True)
    seq = zv["seq"]
    if not np.array_equal(seq, zt["seq"]):
        raise SystemExit("[shard] 正解と画像の記録の番号がそろっていない（別の回の正解）")
    row_of, s = stock_rows_by_seq(a.stock)
    truth = zt["truth17_m"]
    ok = np.isfinite(truth).all(1) & np.array([int(k) in row_of for k in seq])
    if a.keep:
        t_cap = zt["t_capture"]
        win = np.zeros(len(seq), dtype=bool)
        for w in a.keep:
            t0, t1 = (float(x) for x in w.split(":"))
            win |= (t_cap >= t0) & (t_cap <= t1)
        ok &= win
    keep = np.nonzero(ok)[0]
    if not len(keep):
        raise SystemExit("[shard] 書ける枚が無い")
    rows = [row_of[int(seq[j])] for j in keep]
    aux = np.stack([build_aux(vp, s["obs"][i], s["vis_hg"][i]) for i in rows]).astype(np.float32)
    rgb = zv["rgb"][keep]
    depth = zv["depth"][keep].astype(np.float16)
    if rgb.shape[1:] != (108, 192, 3) or depth.shape[1:] != (54, 96):
        raise SystemExit(f"[shard] 画の形がシムの束と違う: カラー {rgb.shape[1:]}・深度 {depth.shape[1:]}")
    n = len(keep)
    np.savez(a.out, rgb=rgb, depth=depth, aux=aux, truth=truth[keep].astype(np.float32),
             stage=np.full(n, REAL_STAGE, np.float32), env_id=np.zeros(n, np.float32), seq=seq[keep],
             t_capture=zt["t_capture"][keep], src=zt["src"][keep],
             meta_json=json.dumps({"truth": a.truth, "vision": a.vision, "stock": a.stock, "policy": a.policy,
                                   "keep": a.keep, "truth_meta": meta}, ensure_ascii=False))
    print(f"[shard] {a.out}: {n} 枚（画像 {len(seq)} 枚のうち・正解と走らせ役の観測がそろった枚）", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
