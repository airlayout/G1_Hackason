"""11-10 手順 R1: 画像の網（カラー＋深度 → 扉の値 17）を、実物の記録の全部の枚でライダーの正解（real_truth_1110.py）と比べる（計画書 8.7 節）。

網の値: 扉の頭（追う段なし＝今の 1 枚が言う値）。入力は [画像の特徴 128, 頭の重力 3, 胴の重力 3, 腰 3]。
  特徴は実機で記録したもの（vision_*.npz の feat）か、--onnx を渡せば記録の画像から作り直したもの（R2 の新しい網を比べるとき）。
  頭の重力・胴の重力・腰は走らせ役の記録（door_stock_*.npz の vis_hg・obs）から。走らせ役が観測を作っていない枚（WAIT など）は比べない。
  参考に、実機で方策に渡した値（追う段込み・vis_door）も比べる。

誤差（網 − 正解。実の向き）: 前・左 = pelvis から見た開口の中心、向き = 扉の +x 軸の向き（pelvis から見た）、扉の角度、
  寸法（幅・レバーの高さ・バックセット）。前の帯・向きの帯・回（静止の地点）ごとに中央値・平均の絶対値・片寄り・90% 点を出す。

使い方（OMEN の real_ana_1106/・doorman6 の Python）:
  python real_eval_1110.py --truth truth_183336_B.npz --vision <vision_*.npz> --stock <door_stock_*.npz> \
      --policy door_vision_e2b.npz [--onnx door_vision_e2b_encoder.onnx] --out eval.npz
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import door_vision_core as dvc  # noqa: E402
from real_truth_1110 import mirror_door17  # noqa: E402

FWD_BANDS = (0.0, 0.5, 0.8, 1.2, 1.6, 9.0)  # [m] pelvis から開口の中心まで（前）
YAW_BANDS = (-90.0, -20.0, -5.0, 5.0, 20.0, 90.0)  # [°]
GEOM = {"幅": 10, "レバーの高さ": 12, "バックセット": 13}  # 扉の値 17 の番号


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--truth", required=True)
    ap.add_argument("--vision", required=True)
    ap.add_argument("--stock", required=True)
    ap.add_argument("--policy", required=True, help="扉の頭と鏡写しを読む書き出し（door_vision_e2b.npz）")
    ap.add_argument("--onnx", default=None, help="渡せば記録の画像から特徴を作り直す（RGBD の ONNX）")
    ap.add_argument("--pred", default=None, help="学び直しの台本（パッチ 0103）が書いた 1 枚ごとの当て（<出力>_realval.npz）。渡せば網を通さずにそれを比べる")
    ap.add_argument("--pred-key", default=None, help="--pred の中の名前（real_shard の npz の名前。例 real_185929.npz）")
    ap.add_argument("--out", required=True)
    return ap.parse_args(argv)


def stock_rows_by_seq(stock_path: str) -> Tuple[Dict[int, int], Dict[str, np.ndarray]]:
    """画像の番号 → 走らせ役の行（その番号で最初に観測と頭の重力がそろった行）"""
    zs = np.load(stock_path, allow_pickle=True)
    s = {k: zs[k] for k in ("vis_seq", "vis_hg", "obs", "vis_door")}
    row_of: Dict[int, int] = {}
    for i, k in enumerate(s["vis_seq"]):
        if np.isfinite(k) and int(k) not in row_of and np.isfinite(s["vis_hg"][i]).all() and np.isfinite(s["obs"][i]).all():
            row_of[int(k)] = i
    return row_of, s


def reencode(onnx: str, vision_path: str, idx: np.ndarray) -> np.ndarray:
    """記録の画像（網に入れたカラー 192×108・深度 54×96）から特徴を作り直す"""
    import door_vision_encoder as dve

    enc = dve.OrtRGBDEncoder(onnx, 4)
    zv = np.load(vision_path, allow_pickle=True)
    rgb, depth = zv["rgb"], zv["depth"]
    out = np.zeros((len(idx), 128))
    for n, j in enumerate(idx):
        out[n] = enc(dve.preprocess_rgb(rgb[j]), dve.depth_input(depth[j].astype(np.float32)), False)
    return out


def door_yaw_deg(real17: np.ndarray) -> np.ndarray:
    """扉の +x 軸の向き（pelvis から見た水平の角度）[°]"""
    return np.degrees(np.arctan2(real17[..., 5], real17[..., 4]))


def errors(pred_real: np.ndarray, truth_real: np.ndarray) -> Dict[str, np.ndarray]:
    """網 − 正解（実の向き）。長さは cm、角度は °"""
    e = {
        "前 [cm]": 100.0 * (pred_real[:, 1] - truth_real[:, 1]),
        "左 [cm]": 100.0 * (pred_real[:, 2] - truth_real[:, 2]),
        "向き [°]": (door_yaw_deg(pred_real) - door_yaw_deg(truth_real) + 180.0) % 360.0 - 180.0,
        "扉の角度 [°]": np.degrees(pred_real[:, 0] - truth_real[:, 0]),
    }
    for name, k in GEOM.items():
        e[f"{name} [cm]"] = 100.0 * (pred_real[:, k] - truth_real[:, k])
    return e


def summarize(err: Dict[str, np.ndarray], sel: np.ndarray) -> Dict[str, List[float]]:
    """項目ごとに [片寄り（中央値）, 平均の絶対値, 90% 点の絶対値]"""
    return {k: [float(np.median(v[sel])), float(np.mean(np.abs(v[sel]))), float(np.percentile(np.abs(v[sel]), 90))]
            for k, v in err.items()}


def print_table(title: str, groups: Sequence[Tuple[str, np.ndarray]], err: Dict[str, np.ndarray]) -> Dict[str, Dict]:
    keys = list(err)
    print(f"\n[eval] {title}（片寄り／平均の絶対値／90% 点）", flush=True)
    print("  " + f"{'':<16}{'枚':>6}  " + "  ".join(f"{k:<20}" for k in keys), flush=True)
    out = {}
    for name, sel in groups:
        n = int(sel.sum())
        if n == 0:
            continue
        s = summarize(err, sel)
        out[name] = {"n": n, **s}
        cells = "  ".join(f"{s[k][0]:+6.1f}/{s[k][1]:5.1f}/{s[k][2]:5.1f}  " for k in keys)
        print(f"  {name:<16}{n:>6}  {cells}", flush=True)
    return out


def band_groups(values: np.ndarray, edges: Sequence[float], unit: str, base: np.ndarray) -> List[Tuple[str, np.ndarray]]:
    return [(f"{lo:g}〜{hi:g}{unit}", base & (values >= lo) & (values < hi)) for lo, hi in zip(edges[:-1], edges[1:])]


def main(argv=None) -> int:
    a = parse_args(argv)
    zt = np.load(a.truth, allow_pickle=False)
    meta = json.loads(str(zt["meta_json"]))
    p = dc.load_params(a.policy, mirror=not meta["lever_right"])
    vp = dvc.load_vision(a.policy, p)
    zv = np.load(a.vision, allow_pickle=True)
    seq = zv["seq"]
    if not np.array_equal(seq, zt["seq"]):
        raise SystemExit("[eval] 正解と画像の記録の番号がそろっていない（別の回の正解）")
    row_of, s = stock_rows_by_seq(a.stock)
    truth_real = zt["truth17_real"]
    have = np.array([int(k) in row_of for k in seq]) & np.isfinite(truth_real).all(1)
    idx = np.nonzero(have)[0]
    print(f"[eval] 画像 {len(seq)} 枚・正解あり {int(np.isfinite(truth_real).all(1).sum())}・走らせ役の観測もある {len(idx)} 枚", flush=True)
    head_m = np.zeros((len(idx), dvc.DOOR_DIM))
    used_m = np.array([s["vis_door"][row_of[int(seq[j])]] for j in idx])
    if a.pred:
        # real_shard_1110.py と同じ選び方（正解と走らせ役の観測がそろった枚・番号の順）なので、行はそのまま idx にそろう
        zp = np.load(a.pred)
        head_m = zp[f"pred:{a.pred_key}"].astype(float)
        if len(head_m) != len(idx) or not np.allclose(zp[f"truth:{a.pred_key}"], zt["truth17_m"][idx], atol=1e-5):
            raise SystemExit(f"[eval] --pred の行（{len(head_m)}）が正解の枚（{len(idx)}）とそろわない")
        print(f"[eval] 網の当ては {a.pred} の {a.pred_key} から", flush=True)
    else:
        feat = reencode(a.onnx, a.vision, idx) if a.onnx else zv["feat"][idx]
        if a.onnx:
            print(f"[eval] 特徴を {a.onnx} で作り直した（記録の特徴との差の最大 {np.abs(feat - zv['feat'][idx]).max():.1e}）", flush=True)
        for n, j in enumerate(idx):
            i = row_of[int(seq[j])]
            head_m[n] = dvc.door_values(vp, s["obs"][i], feat[n], s["vis_hg"][i])
    head_real = np.array([mirror_door17(p, vp, d) for d in head_m])
    used_ok = np.isfinite(used_m).all(1)
    used_real = np.array([mirror_door17(p, vp, d) if ok else np.full(dvc.DOOR_DIM, np.nan) for d, ok in zip(used_m, used_ok)])
    tr = truth_real[idx]
    err = errors(head_real, tr)
    base = np.ones(len(idx), dtype=bool)
    yaw_t = door_yaw_deg(tr)
    res = {
        "全部": print_table("扉の頭（追う段なし）・全部", [("全部", base)], err),
        "前の帯": print_table("扉の頭・前の帯ごと", band_groups(tr[:, 1], FWD_BANDS, " m", base), err),
        "向きの帯": print_table("扉の頭・扉の向きの帯ごと", band_groups(yaw_t, YAW_BANDS, "°", base), err),
    }
    spots = meta.get("spots", [])
    if spots:
        t = zt["t_capture"][idx]
        res["巻き尺の地点"] = print_table("扉の頭・巻き尺の地点ごと",
                                    [(f"地点{k}（toe {sp['toe']:.2f}）", (t >= sp["t0"]) & (t <= sp["t1"])) for k, sp in enumerate(spots)], err)
    if used_ok.any():
        err_u = errors(used_real[used_ok], tr[used_ok])
        res["方策に渡した値"] = print_table("参考: 実機で方策に渡した値（追う段込み）", [("全部", np.ones(int(used_ok.sum()), bool))], err_u)
    np.savez_compressed(a.out, idx=idx, seq=seq[idx], head17_m=head_m, head17_real=head_real, used17_real=used_real,
                        truth17_real=tr, **{f"err_{k.split()[0]}": v for k, v in err.items()},
                        summary_json=json.dumps(res, ensure_ascii=False))
    print(f"\n[eval] 書いた: {a.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
