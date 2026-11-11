"""11-10: 頭のカメラの深度が正しく処理できているかを、深度だけで決まる幾何の値で確かめる。あわせて YOLO（Open Images V7 の重み・
学習なし）を実物のカラーに掛け、今の網（ResNet18＋深度の 4 層 CNN。数字は ana_branch_1106.py で別に出す）と同じ巻き尺の地点で比べる。

確かめ（深度 54×96 を網と同じ画角 SIM_TAN_HALF で 3 次元に戻し、RANSAC で平面を当てる）:
  - 平らさ: 扉の面の当てはめの残りの二乗平均（平らな板は平らに出るはず）
  - 俯角: 縦の扉の面の向きから出した俯角と、シムの頭の重力（実機は走らせ役の頭の重力）の俯角（画角を取り違えると食い違う）
  - 直角: 扉の面と床の面のなす角（90° のはず）
  - 距離: カメラから扉の面までの距離と、扉の枠の前後（シムの正解 pos_x・実物の巻き尺）の傾き（1 のはず）
  - 切り出し: D435i の生の深度 848×480（内部の値で戻す）と、網に入れる 54×96 に切り出した後とで同じ平面が出るか

使い方（OMEN）:
  sim:   python ana_depth_yolo_1110.py sim <束の frames の置き場>...
  probe: python ana_depth_yolo_1110.py probe <depth_probe_*.npz>
  real:  python ana_depth_yolo_1110.py real --vision <vision_*.npz> --jpeg <JPEG の置き場> --stock <door_stock_*.npz> \
             --fit <傾き>:<切片> --win t0:t1:前:左 ... --yolo yolov8n-oiv7.pt:640 yolov8x-oiv7.pt:640 --out <結果の置き場>
  --fit は sim が出す「扉の枠の前後 ≈ 傾き × カメラから扉の面 + 切片」。--yolo を渡さなければ深度の確かめだけ
"""
from __future__ import annotations

import argparse
import glob
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_vision_encoder as dve  # noqa: E402

TAN_X, TAN_Y = dve.SIM_TAN_HALF
# 11-06 に測った実機のカラーの画角 70.3°×43.3°（半分の tan）。中心は画面の中央とみなす（深度との 15 mm のずれは無視）
COLOR_TAN = (float(np.tan(np.radians(70.3 / 2))), float(np.tan(np.radians(43.3 / 2))))
CAM_LEFT = 0.0525  # シムの取付のカメラの左 [m]（torso_link から。実機の頭のカメラもこの位置とみなす）
DOOR_CLS, HANDLE_CLS = 164, 165  # Open Images V7 の札（yolov8*-oiv7 の「Door」「Door handle」）
PLANE_MIN_FRAC = 0.08  # 平面とみなす当たった点の割合の下限
YOLO_CONF = 0.25  # 見つけたとみなす確からしさ


def backproject(d: np.ndarray, tan_x: float = TAN_X, tan_y: float = TAN_Y) -> np.ndarray:
    """深度 [h, w]（カメラの面までの距離 m・0 は測れない・3 m に詰めた所は捨てる）→ 点 [N, 3]（光学の座標: x 右・y 下・z 前）"""
    h, w = d.shape
    tx = ((np.arange(w) + 0.5) / w * 2.0 - 1.0) * tan_x
    ty = ((np.arange(h) + 0.5) / h * 2.0 - 1.0) * tan_y
    z = d.astype(np.float64)
    ok = (z > 0.0) & (z < dve.DEPTH_MAX - 1e-3)
    return np.stack([(tx[None, :] * z)[ok], (ty[:, None] * z)[ok], z[ok]], axis=1)


def backproject_intr(d: np.ndarray, fx: float, fy: float, cx: float, cy: float, step: int = 4) -> np.ndarray:
    """D435i の生の深度 [480, 848] を内部の値で 3 次元に戻す（step 画素ごと）"""
    v, u = np.mgrid[0 : d.shape[0] : step, 0 : d.shape[1] : step]
    z = d[::step, ::step].astype(np.float64)
    ok = np.isfinite(z) & (z > 0.0) & (z < dve.DEPTH_MAX)
    x = (u + 0.5 - cx) / fx * z
    y = (v + 0.5 - cy) / fy * z
    return np.stack([x[ok], y[ok], z[ok]], axis=1)


def fit_plane(P: np.ndarray, thr: float, rng: np.random.Generator, iters: int = 300) -> Optional[Tuple[np.ndarray, float, np.ndarray, float]]:
    """RANSAC で平面を当てる → (n, c, 当たった点の印, 残りの二乗平均 m)。n·p + c = 0・c > 0（n はカメラの側を向く）"""
    if len(P) < 50:
        return None
    idx = rng.integers(0, len(P), size=(iters, 3))
    a, b, e = P[idx[:, 0]], P[idx[:, 1]], P[idx[:, 2]]
    n = np.cross(b - a, e - a)
    nn = np.linalg.norm(n, axis=1)
    good = nn > 1e-9
    if not good.any():
        return None
    n, a = n[good] / nn[good, None], a[good]
    c = -np.sum(n * a, axis=1)
    k = int(np.argmax((np.abs(P @ n.T + c[None, :]) < thr).sum(0)))
    inl = np.abs(P @ n[k] + c[k]) < thr
    Q = P[inl]
    m = Q.mean(0)
    n1 = np.linalg.svd(Q - m, full_matrices=False)[2][2]
    c1 = -float(n1 @ m)
    if c1 < 0:
        n1, c1 = -n1, -c1
    inl = np.abs(P @ n1 + c1) < thr
    rms = float(np.sqrt(np.mean((P[inl] @ n1 + c1) ** 2)))
    return n1, c1, inl, rms


def door_floor(P: np.ndarray, thr: float, rng: np.random.Generator) -> Dict[str, float]:
    """点から扉の面（縦・n の y が正）と床の面（n の y が負）を順に取り、幾何の値を返す（取れなければ nan）"""
    out = {k: float("nan") for k in ("door_dist", "door_rms", "door_frac", "pitch_door", "yaw_door",
                                      "cam_h", "floor_rms", "pitch_floor", "perp")}
    n0, rest = len(P), P
    nd = nf = None
    for _ in range(3):
        r = fit_plane(rest, thr, rng)
        if r is None or r[2].sum() < PLANE_MIN_FRAC * n0:
            break
        n, c, inl, rms = r
        if nd is None and n[1] > 0.2:
            nd = n
            out.update(door_dist=c, door_rms=rms, door_frac=float(inl.sum() / n0),
                       pitch_door=float(np.degrees(np.arctan2(n[1], -n[2]))),
                       yaw_door=float(np.degrees(np.arctan2(-n[0], np.hypot(n[1], n[2])))))
        elif nf is None and n[1] < -0.2:
            nf = n
            out.update(cam_h=c, floor_rms=rms, pitch_floor=float(np.degrees(np.arctan2(-n[2], -n[1]))))
        rest = rest[~inl]
        if nd is not None and nf is not None:
            break
    if nd is not None and nf is not None:
        out["perp"] = float(np.degrees(np.arccos(min(1.0, abs(float(nd @ nf))))))
    return out


def gravity_pitch(g: np.ndarray) -> np.ndarray:
    """重力の向き [..., 3]（体の座標: x 前・y 左・z 上）→ 俯角 [度]（前が下がると正）"""
    return np.degrees(np.arctan2(g[..., 0], -g[..., 2]))


def q(xs, p=50) -> float:
    xs = np.asarray(xs, dtype=float)
    xs = xs[np.isfinite(xs)]
    return float(np.percentile(xs, p)) if len(xs) else float("nan")


def cmd_sim(a: argparse.Namespace) -> int:
    fs = sorted(f for d in a.dirs for f in glob.glob(f"{d}/shard_*.npz"))
    rng = np.random.default_rng(0)
    rows: List[Dict[str, float]] = []
    for f in fs:
        z = np.load(f)
        tr, st, ep = z["truth"], z["stage"], z["ep_step"]
        # 扉が閉じている回の始め（段 0〜1・扉 1° 未満）。回の始めの 2 歩は前の回の画が残るので外す
        sel = np.nonzero((st <= 1) & (np.abs(tr[:, 0]) < np.radians(1.0)) & (ep >= 2))[0][:: a.every]
        for j in sel:
            g = door_floor(backproject(z["depth"][j]), 0.01, rng)
            g.update(pos_x=float(tr[j, 1]), pos_y=float(tr[j, 2]), pitch_set=float(z["pitch_deg"][j]),
                     pitch_head=float(gravity_pitch(z["aux"][j, 0:3])), pitch_torso=float(gravity_pitch(z["aux"][j, 3:6])))
            rows.append(g)
    print(f"[sim] {len(fs)} 束の欠片・扉が閉じた枚 {len(rows)}", flush=True)
    R = {k: np.array([r[k] for r in rows]) for k in rows[0]}
    ok = np.isfinite(R["door_dist"]) & (np.abs(R["yaw_door"]) < 10.0)
    A = np.stack([R["door_dist"][ok], np.ones(ok.sum())], 1)
    (slope, icpt), *_ = np.linalg.lstsq(A, R["pos_x"][ok], rcond=None)
    resid = R["pos_x"][ok] - (slope * R["door_dist"][ok] + icpt)
    for name in ("pitch_set", "pitch_head", "pitch_torso"):
        dd = R["pitch_door"] - R[name]
        print(f"[sim] 扉の面の俯角 − {name}: 中央 {q(dd):+.2f}°・5〜95% {q(dd, 5):+.2f}〜{q(dd, 95):+.2f}°", flush=True)
    print(f"[sim] 床の面の俯角 − 頭の重力の俯角: 中央 {q(R['pitch_floor'] - R['pitch_head']):+.2f}°", flush=True)
    print(f"[sim] 扉と床のなす角: 中央 {q(R['perp']):.2f}°・5〜95% {q(R['perp'], 5):.2f}〜{q(R['perp'], 95):.2f}°"
          f"（床が取れた枚 {np.isfinite(R['perp']).mean():.0%}）", flush=True)
    print(f"[sim] 扉の面の平らさ（残りの二乗平均）: 中央 {1000 * q(R['door_rms']):.1f} mm・扉の面が取れた枚 {np.isfinite(R['door_dist']).mean():.0%}", flush=True)
    print(f"[sim] カメラの高さ（床の面まで）: 中央 {q(R['cam_h']):.3f} m・5〜95% {q(R['cam_h'], 5):.3f}〜{q(R['cam_h'], 95):.3f}", flush=True)
    print(f"[sim] 扉の枠の前後 pos_x ≈ {slope:.3f} × カメラから扉の面 + {icpt:+.3f} m（扉の向き 10° 以内 {ok.sum()} 枚・"
          f"残りの中央 {100 * q(np.abs(resid)):.1f} cm・90% 点 {100 * q(np.abs(resid), 90):.1f} cm・"
          f"距離の幅 {q(R['door_dist'][ok], 5):.2f}〜{q(R['door_dist'][ok], 95):.2f} m）", flush=True)
    if a.out:
        np.savez(a.out, **R)
    return 0


def cmd_probe(a: argparse.Namespace) -> int:
    z = np.load(a.npz, allow_pickle=True)
    fx, fy, cx, cy = (float(v) for v in z["intrinsics"])
    rng = np.random.default_rng(0)
    print(f"[probe] 生の深度 {z['depth'].shape}・内部の値 fx {fx:.1f} fy {fy:.1f} cx {cx:.1f} cy {cy:.1f}"
          f"（画角 {2 * np.degrees(np.arctan(424 / fx)):.1f}°×{2 * np.degrees(np.arctan(240 / fy)):.1f}°）", flush=True)
    for i, raw in enumerate(z["depth"].astype(np.float32)):
        g_raw = door_floor(backproject_intr(raw, fx, fy, cx, cy), 0.02, rng)
        g_cut = door_floor(backproject(dve.depth_from_intrinsics(raw, fx, fy, cx, cy)), 0.02, rng)
        if i % 4 == 0 or i == len(z["depth"]) - 1:
            print(f"[probe] {i:2d} 生: 扉まで {g_raw['door_dist']:.3f} m・俯角 {g_raw['pitch_door']:.1f}°・向き {g_raw['yaw_door']:+.1f}°"
                  f"・床の俯角 {g_raw['pitch_floor']:.1f}° ｜ 切り出し後: 扉まで {g_cut['door_dist']:.3f} m・俯角 {g_cut['pitch_door']:.1f}°"
                  f"・向き {g_cut['yaw_door']:+.1f}°・床の俯角 {g_cut['pitch_floor']:.1f}°・平らさ {1000 * g_cut['door_rms']:.1f} mm", flush=True)
    return 0


def load_yolo(specs: List[str]):
    import torch
    from ultralytics import YOLO

    torch.set_num_threads(4)
    return [(s, YOLO(s.split(":")[0]), int(s.split(":")[1])) for s in specs]


def yolo_door(model, imgsz: int, img: np.ndarray) -> Dict[str, object]:
    """1 枚に YOLO を掛け、扉と把手の一番確かな箱を返す（箱は 1920×1080 の画素）"""
    t = time.perf_counter()
    r = model.predict(img, imgsz=imgsz, conf=0.05, classes=[DOOR_CLS, HANDLE_CLS], device="cpu", verbose=False)[0]
    ms = 1000 * (time.perf_counter() - t)
    out: Dict[str, object] = {"ms": ms}
    cls, conf, xyxy = r.boxes.cls.numpy().astype(int), r.boxes.conf.numpy(), r.boxes.xyxy.numpy()
    for name, k in (("door", DOOR_CLS), ("handle", HANDLE_CLS)):
        m = np.nonzero(cls == k)[0]
        if len(m):
            b = m[int(np.argmax(conf[m]))]
            out[name] = (float(conf[b]), [float(v) for v in xyxy[b]])
    return out


def box_geometry(box: List[float], d: np.ndarray, img_wh: Tuple[int, int], fit: Tuple[float, float],
                 rng: np.random.Generator) -> Dict[str, float]:
    """扉の箱の中の深度で扉の面を当て、箱の中心の向きの光線と交わる点から 前・左 を出す（実の向き・pelvis から）"""
    W, H = img_wh
    h, w = d.shape
    c0, c1 = int(np.floor(box[0] / W * w)), int(np.ceil(box[2] / W * w))
    r0, r1 = int(np.floor(box[1] / H * h)), int(np.ceil(box[3] / H * h))
    sub = np.zeros_like(d)
    sub[r0:r1, c0:c1] = d[r0:r1, c0:c1]
    r = fit_plane(backproject(sub), 0.02, rng)
    if r is None:
        return {"fwd": float("nan"), "left": float("nan"), "clip": float("nan")}
    n, c, _, _ = r
    u, v = 0.5 * (box[0] + box[2]), 0.5 * (box[1] + box[3])
    ray = np.array([(u / W * 2 - 1) * COLOR_TAN[0], (v / H * 2 - 1) * COLOR_TAN[1], 1.0])
    p = ray * (-c / float(n @ ray))
    clip = float(box[0] < 0.01 * W or box[2] > 0.99 * W)  # 箱が左右の縁に当たっている（扉の全部は写っていない）
    return {"fwd": fit[0] * c + fit[1], "left": float(-p[0] + CAM_LEFT), "clip": clip}


def cmd_real(a: argparse.Namespace) -> int:
    import cv2

    zv = np.load(a.vision, allow_pickle=True)
    v = {k: zv[k] for k in ("depth", "seq", "t_capture")}
    zs = np.load(a.stock, allow_pickle=True)
    s = {k: zs[k] for k in ("vis_seq", "vis_hg")}
    hg_of = {int(k): s["vis_hg"][i] for i, k in enumerate(s["vis_seq"]) if np.isfinite(k) and np.isfinite(s["vis_hg"][i]).all()}
    fit = tuple(float(x) for x in a.fit.split(":"))
    jpegs = sorted(Path(a.jpeg).glob("*.jpg"))
    jt = {int(p.stem.split("_")[0]): float(p.stem.split("_")[1]) for p in jpegs}
    bad = [j for j, t in jt.items() if abs(v["t_capture"][j] - t) > 1e-3]
    print(f"[real] 深度 {len(v['depth'])} 枚・JPEG {len(jpegs)} 枚（番号と撮った時刻の食い違い {len(bad)}）・前 ≈ {fit[0]:.3f} × 扉の面 + {fit[1]:+.3f}", flush=True)
    rng = np.random.default_rng(0)
    models = load_yolo(a.yolo) if a.yolo else []
    out_dir = Path(a.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    summary: Dict[str, object] = {"fit": fit, "windows": []}
    for w in a.win:
        t0, t1, fwd, left = (float(x) for x in w.split(":"))
        ks = [j for j, t in enumerate(v["t_capture"]) if t0 <= t <= t1][:: a.every]
        G = [door_floor(backproject(v["depth"][j].astype(np.float32)), 0.02, rng) for j in ks]
        hp = [float(gravity_pitch(hg_of[int(v["seq"][j])])) for j in ks if int(v["seq"][j]) in hg_of]
        col = lambda k: [g[k] for g in G]  # noqa: E731
        f_depth = [fit[0] * g["door_dist"] + fit[1] for g in G]
        win = {"win": w, "tape_fwd": fwd, "tape_left": left, "n": len(ks),
               "depth_fwd": q(f_depth), "depth_fwd_sd": float(np.nanstd(f_depth)), "door_dist": q(col("door_dist")),
               "pitch_door": q(col("pitch_door")), "pitch_floor": q(col("pitch_floor")), "pitch_hg": q(hp),
               "perp": q(col("perp")), "floor_found": float(np.isfinite(col("perp")).mean()),
               "door_rms_mm": 1000 * q(col("door_rms")), "cam_h": q(col("cam_h")), "yaw_door": q(col("yaw_door"))}
        print(f"\n[real] 窓 t {t0:.0f}〜{t1:.0f}（{len(ks)} 枚）巻き尺 前 {fwd:.3f} 左 {left:+.3f}", flush=True)
        print(f"  深度の幾何: 前 {win['depth_fwd']:.3f}±{win['depth_fwd_sd']:.3f}（差 {100 * (win['depth_fwd'] - fwd):+.1f} cm）・"
              f"扉の面まで {win['door_dist']:.3f} m・平らさ {win['door_rms_mm']:.1f} mm・扉の向き {win['yaw_door']:+.1f}°", flush=True)
        print(f"  俯角: 扉の面から {win['pitch_door']:.1f}°・床の面から {win['pitch_floor']:.1f}°・頭の重力 {win['pitch_hg']:.1f}°・"
              f"扉と床 {win['perp']:.1f}°（床が取れた枚 {win['floor_found']:.0%}）・カメラの高さ {win['cam_h']:.3f} m", flush=True)
        win["yolo"] = {}
        sel = [j for j in sorted(jt) if t0 <= jt[j] <= t1]
        for spec, model, imgsz in models:
            res = []
            for j in sel:
                img = cv2.imread(str(next(p for p in jpegs if int(p.stem.split("_")[0]) == j)))
                y = yolo_door(model, imgsz, img)
                g = box_geometry(y["door"][1], v["depth"][j].astype(np.float32), (img.shape[1], img.shape[0]), fit, rng) if "door" in y else {}
                res.append({"j": j, **{k: y[k] for k in y}, **g})
                if len(res) <= 2:
                    vis = img.copy()
                    for name, colr in (("door", (0, 200, 255)), ("handle", (255, 80, 80))):
                        if name in y:
                            b = [int(x) for x in y[name][1]]
                            cv2.rectangle(vis, (b[0], b[1]), (b[2], b[3]), colr, 4)
                            cv2.putText(vis, f"{name} {y[name][0]:.2f}", (b[0] + 8, b[1] + 40), cv2.FONT_HERSHEY_SIMPLEX, 1.4, colr, 3)
                    cv2.imwrite(str(out_dir / f"{Path(spec.split(':')[0]).stem}_{imgsz}_t{int(t0)}_{j:06d}.jpg"), vis)
            dr = [r for r in res if "door" in r and r["door"][0] >= YOLO_CONF]
            hr = [r for r in res if "handle" in r and r["handle"][0] >= YOLO_CONF]
            fw, lf = [r["fwd"] for r in dr], [r["left"] for r in dr]
            yw = {"n": len(res), "door_rate": len(dr) / max(1, len(res)), "handle_rate": len(hr) / max(1, len(res)),
                  "door_conf": q([r["door"][0] for r in res if "door" in r]), "fwd": q(fw), "left": q(lf),
                  "left_sd": float(np.nanstd(lf)) if lf else float("nan"),
                  "clip": float(np.mean([r["clip"] for r in dr])) if dr else float("nan"), "ms": q([r["ms"] for r in res])}
            win["yolo"][spec] = yw
            print(f"  YOLO {spec}: 扉を見つけた {yw['door_rate']:.0%}・把手 {yw['handle_rate']:.0%}（{len(res)} 枚・確からしさの中央 {yw['door_conf']:.2f}）・"
                  f"前 {yw['fwd']:.3f}（差 {100 * (yw['fwd'] - fwd):+.1f} cm）・左 {yw['left']:+.3f}±{yw['left_sd']:.3f}"
                  f"（差 {100 * (yw['left'] - left):+.1f} cm）・箱が縁に当たる {yw['clip']:.0%}・1 枚 {yw['ms']:.0f} ms（CPU 4 糸）", flush=True)
        summary["windows"].append(win)
    if a.all_frames and models:
        # 録った JPEG の全部で、扉の面までの距離ごとに扉と把手を見つけた割合を出す
        bins = [0.0, 0.5, 1.0, 1.5, 2.0, 3.0]
        rate = {}
        for spec, model, imgsz in models:
            hits: Dict[int, List[Tuple[bool, bool]]] = {i: [] for i in range(len(bins) - 1)}
            for p in jpegs:
                j = int(p.stem.split("_")[0])
                dd = door_floor(backproject(v["depth"][j].astype(np.float32)), 0.02, rng)["door_dist"]
                if not np.isfinite(dd):
                    continue
                y = yolo_door(model, imgsz, cv2.imread(str(p)))
                b = int(np.clip(np.searchsorted(bins, dd) - 1, 0, len(bins) - 2))
                hits[b].append(("door" in y and y["door"][0] >= YOLO_CONF, "handle" in y and y["handle"][0] >= YOLO_CONF))
            rate[spec] = {f"{bins[i]:.1f}-{bins[i + 1]:.1f}": (len(h), float(np.mean([x[0] for x in h])) if h else float("nan"),
                                                             float(np.mean([x[1] for x in h])) if h else float("nan"))
                          for i, h in hits.items()}
            print(f"\n[real] 全部の JPEG・{spec}（扉の面までの距離ごとに 枚・扉・把手）: {rate[spec]}", flush=True)
        summary["all_frames"] = rate
    (out_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    print(f"\n[real] 書いた: {out_dir / 'summary.json'}", flush=True)
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("sim")
    p.add_argument("dirs", nargs="+")
    p.add_argument("--every", type=int, default=7)
    p.add_argument("--out", default=None)
    p = sub.add_parser("probe")
    p.add_argument("npz")
    p = sub.add_parser("real")
    p.add_argument("--vision", required=True)
    p.add_argument("--jpeg", required=True)
    p.add_argument("--stock", required=True)
    p.add_argument("--fit", required=True)
    p.add_argument("--win", action="append", required=True)
    p.add_argument("--every", type=int, default=3)
    p.add_argument("--yolo", nargs="*", default=[])
    p.add_argument("--all-frames", action="store_true")
    p.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    return {"sim": cmd_sim, "probe": cmd_probe, "real": cmd_real}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
