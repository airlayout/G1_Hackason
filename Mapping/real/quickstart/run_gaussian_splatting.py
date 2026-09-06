#!/usr/bin/env python3
"""生の LiDAR スキャンだけで 3D Gaussian Splatting を回し、地図を掃除する。

## OctoMap と何を揃えてあるか

`run_octomap.py` とまったく同じ入力を食う。

* 学習に使う観測 … `<session>/benchmark/pcd/*.pcd`（姿勢つき 6,305 枚）
* 掃除する対象   … `<session>/map/map_raw.pcd`（669,239 点）

出力も同じ形にしてある。`map_raw.pcd` の点を**座標そのままで残すか消すか**決めて
書き出すので、`eval_removal.py` と `compare_maps.py` にそのまま渡せる。

## 何が違うか

OctoMap は八分木のセルに「空 −0.176 / 占有 +0.847」の票を入れて閾値で切る。
こちらは同じ「可視性」の証拠を、**連続量の勾配**として受ける。
`map_raw.pcd` の 1 点につきガウシアンを 1 個置き（近傍の主成分分析で
壁の上では平たい円盤になる）、6,305 枚の実測距離に対して微分可能に描画して、
位置・姿勢・大きさ・不透明度を最適化する。

追従者の居た場所は数秒後に光線が通り抜けるので、そこのガウシアンを残したままだと
描画距離が手前に寄って損失が増える。損失を下げる向きは α を下げる向きなので、
不透明度が落ちて刈られる。机は光線が天板で止まるため下げる圧力がかからない。

**色は一切使わない。** 3DGS の球面調和（SH）係数は持たず、形と不透明度だけを持つ。

⚠️ ただしこれは「使えない」からではなく「この記録に無い」からである（2026-09-05 訂正）。
生の `/utlidar/cloud_livox_mid360` には `intensity`（float32・0〜155）が入っており、
同じ 10cm ボクセルを 5 枚以上から見たときのばらつき（中央値 7.2）は
場所による違い（標準偏差 28.8）の 1/4 で、1 チャンネルの「色」として使える水準だった。
`write_pcd()` が x/y/z しか書かないので取りこぼしていた。

## 「消える」以外にもう一つの逃げ道がある

3DGS は不透明度を下げるだけでなく、**ガウシアンを動かして辻褄を合わせる**ことも
できる。追従者のガウシアンが背後の壁まで移動すると、不透明度は高いまま残る。
これは OctoMap には無い挙動なので、元の位置からの移動量も見て、
`--max-drift` より動いたものは「その場所には無かった」として消す。
`--freeze-means` を付けると位置を固定でき、不透明度の証拠だけを見た比較ができる。

## 使い方

    ../../Navigation/.venv/bin/python quickstart/run_gaussian_splatting.py \\
        runs/20260904T183457_UiS_room_v2 --iterations 12000

出力は `<session>/map/map_3dgs.pcd`（掃除後）、`map_3dgs_removed.pcd`（消した点）、
`<session>/gs/gaussians.ply`（3DGS のビューアで開ける形式）、
`<session>/benchmark/gs_report.txt`。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from g1_mapping.lidar_gs import (  # noqa: E402
    RenderConfig,
    init_from_points,
    query_density,
    render_rays,
    to_parameters,
    write_splat_ply,
)
from g1_mapping.pcd_io import read_pcd  # noqa: E402
from g1_mapping.scan_intensity import (  # noqa: E402
    INTENSITY_SCALE_HINT,
    IntensityScanReader,
    intensity_for_map,
)
from g1_mapping.rebuild import write_pcd  # noqa: E402

from gsplat.strategy.ops import remove as gsplat_remove  # noqa: E402

# 3DGS 本家（INRIA）の既定に合わせた学習率。means だけはシーンの広さで割り増しする
LR_MEANS = 1.6e-4
LR_SCALES = 5e-3
LR_QUATS = 1e-3
LR_OPACITIES = 5e-2
LR_COLORS = 2.5e-3      # 本家 3DGS の SH 0 次と同じ

DEFAULT_PRUNE_OPACITY = 0.05   # これを下回った不透明度は「証拠が無い」とみなす
DEFAULT_MAX_DRIFT = 0.15       # 元の位置からこれ以上動いたら、そこには無かった扱い[m]
# 1 スキャンの対応付けは 2〜3 GB を一時的に確保する。MPS のキャッシュに溜まると
# 断片化して上限（16GB機で 20.1GB）に当たるので、定期的に返す。実測で
# 毎反復だと 448ms、10 反復に 1 回だと 350ms 程度に収まる。
EMPTY_CACHE_EVERY = 10


def release_cache(device: str) -> None:
    if device == "mps":
        torch.mps.empty_cache()
    elif device == "cuda":
        torch.cuda.empty_cache()


def pick_device(requested: str) -> str:
    if requested != "auto":
        return requested
    if torch.backends.mps.is_available():
        return "mps"
    if torch.cuda.is_available():
        return "cuda"
    return "cpu"


def load_frames(pcd_dir: Path, stride: int, limit: int) -> "list[Path]":
    files = sorted(pcd_dir.glob("*.pcd"))
    if not files:
        raise SystemExit(f"PCD がありません: {pcd_dir}。先に export_benchmark_data.py を回すこと")
    chosen = files[::stride]
    if limit:
        chosen = chosen[:limit]
    return chosen


def read_frame(path: Path, device: str, max_range: float
               ) -> "tuple[torch.Tensor, torch.Tensor, torch.Tensor]":
    """姿勢つき PCD から (センサ位置, 光線方向, 実測距離) を作る。

    書き出し済みの点は world 座標なので、VIEWPOINT の並進を引けば光線になる。
    回転は要らない（レイ方式なので画像平面が無い）。
    """
    data = read_pcd(path)
    origin = torch.tensor(data.origin, dtype=torch.float32, device=device)
    points = torch.from_numpy(data.points.astype(np.float32)).to(device)
    offsets = points - origin
    ranges = offsets.norm(dim=-1)
    keep = ranges > 0.1
    if max_range > 0:
        keep &= ranges <= max_range
    return origin, offsets[keep], ranges[keep]


def trajectory_radius(poses_path: Path) -> float:
    """センサ位置の重心からの最大距離。位置の学習率のスケールに使う。"""
    translations = np.loadtxt(poses_path)[:, 1:4]
    return float(np.linalg.norm(translations - translations.mean(axis=0), axis=1).max())


def read_intensity_frame(reader: IntensityScanReader, index: int, device: str,
                        max_range: float
                        ) -> "tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]":
    """生トピックから (センサ位置, 光線, 実測距離, 反射強度) を作る。"""
    scan = reader.read(index)
    origin = torch.tensor(scan.origin, dtype=torch.float32, device=device)
    points = torch.from_numpy(scan.points.astype(np.float32)).to(device)
    offsets = points - origin
    ranges = offsets.norm(dim=-1)
    intensity = torch.from_numpy(
        (scan.intensity / INTENSITY_SCALE_HINT).astype(np.float32)).to(device)
    keep = ranges > 0.1
    if max_range > 0:
        keep &= ranges <= max_range
    return origin, offsets[keep], ranges[keep], intensity[keep].clamp(0.0, 1.0)


def build_optimizers(params: "dict[str, torch.nn.Parameter]", scene_scale: float
                     ) -> "dict[str, torch.optim.Adam]":
    """パラメータごとに Adam を 1 本ずつ持つ。

    gsplat の `strategy.ops` は「1 optimizer につき param_group が 1 つ」を前提に
    内部状態を差し替えるので、まとめて 1 本にしてはいけない。
    """
    rates = {
        "means": LR_MEANS * scene_scale,
        "scales": LR_SCALES,
        "quats": LR_QUATS,
        "opacities": LR_OPACITIES,
        "colors": LR_COLORS,
    }
    return {
        name: torch.optim.Adam([{"params": [param], "lr": rates[name], "name": name}],
                               eps=1e-15)
        for name, param in params.items()
    }


def evaluate_frames(params, frames, device, config, max_range, sample) -> "dict[str, float]":
    """描画した距離が実測とどれだけ合っているか。学習の前後で比べる。"""
    absolute, squared, accumulation, total = 0.0, 0.0, 0.0, 0
    with torch.no_grad():
        for path in sample:
            origin, offsets, ranges = read_frame(path, device, max_range)
            if len(ranges) == 0:
                continue
            depth, acc, _ = render_rays(params, origin, offsets, config)
            predicted = depth / acc.clamp_min(0.1)
            error = (predicted - ranges).abs()
            absolute += float(error.sum())
            squared += float((error ** 2).sum())
            accumulation += float(acc.sum())
            total += len(ranges)
            release_cache(device)   # 1 枚ごとに返さないと 20 枚で 19 GB まで積み上がる
    if total == 0:
        return {"mae": float("nan"), "rmse": float("nan"), "acc": float("nan")}
    return {"mae": absolute / total, "rmse": (squared / total) ** 0.5,
            "acc": accumulation / total}


def main() -> None:
    parser = argparse.ArgumentParser(
        description="生 LiDAR だけで 3DGS を回して地図から動的な点を消す")
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--map", type=Path, default=None,
                        help="掃除する地図（既定 <session>/map/map_raw.pcd）")
    parser.add_argument("--output", default="map_3dgs.pcd", help="map/ 配下の出力名")
    parser.add_argument("--tag", default="3dgs", help="ply とレポートに付ける名前")
    parser.add_argument("--iterations", type=int, default=12000)
    parser.add_argument("--stride", type=int, default=1, help="何枚に1枚使うか")
    parser.add_argument("--limit", type=int, default=0, help="使う枚数の上限（0で全部）")
    parser.add_argument("--max-range", type=float, default=-1.0,
                        help="学習に使う光線の長さの上限[m]（-1で制限なし）")
    parser.add_argument("--rays-per-scan", type=int, default=0,
                        help="1 枚あたり使う光線数（0で全部）")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--init-opacity", type=float, default=0.3)
    parser.add_argument("--prune-opacity", type=float, default=DEFAULT_PRUNE_OPACITY)
    parser.add_argument("--prune-every", type=int, default=1000)
    parser.add_argument("--prune-start", type=int, default=2000)
    parser.add_argument("--max-drift", type=float, default=DEFAULT_MAX_DRIFT,
                        help="元の位置からこれ以上動いた点は消す[m]（0で無効）")
    parser.add_argument("--freeze-means", action="store_true",
                        help="位置を固定して、不透明度の証拠だけで判定する")
    parser.add_argument("--lambda-acc", type=float, default=0.1,
                        help="実測がある光線は不透明になってほしい、の重み")
    parser.add_argument("--azimuth-bins", type=int, default=720)
    parser.add_argument("--sigma", type=float, default=2.0,
                        help="footprint を何σまで広げるか（2.0 で 3σ 相当と中央値 0.3 cm 差）")
    parser.add_argument("--elevation-bins", type=int, default=180)
    parser.add_argument("--max-per-ray", type=int, default=48)
    parser.add_argument("--lambda-free", type=float, default=0.0,
                        help="自由空間の罰則の重み。実測より手前にあるガウシアンの α を "
                             "透過率で重み付けせずに直接下げる（OctoMap の「空」の票に相当）")
    parser.add_argument("--free-margin", type=float, default=0.10,
                        help="実測距離のこれだけ手前までは罰しない[m]（測定誤差の逃げ）")
    parser.add_argument("--free-max-range", type=float, default=3.0,
                        help="自由空間の罰則を効かせる距離の上限[m]。"
                             "OctoMap が 3 m に絞らないと天井が消えたのと同じ理由")
    parser.add_argument("--use-intensity", action="store_true",
                        help="生トピックの反射強度を 1 チャンネルの「色」として学習に入れる")
    parser.add_argument("--lambda-color", type=float, default=1.0,
                        help="反射強度の損失の重み（--use-intensity のときだけ効く）")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    generator = np.random.default_rng(args.seed)
    device = pick_device(args.device)
    session = args.session_dir
    map_path = args.map or session / "map" / "map_raw.pcd"
    if not map_path.exists():
        raise SystemExit(f"地図が見つかりません: {map_path}")

    config = RenderConfig(n_azimuth=args.azimuth_bins, n_elevation=args.elevation_bins,
                          max_per_ray=args.max_per_ray, sigma=args.sigma)

    target = read_pcd(map_path)
    points = target.points
    # 位置の学習率の基準は「点群の広がり」ではなく「センサが動いた範囲」。本家 3DGS の
    # getNerfppNorm がカメラ中心から測るのと同じ。この記録は部屋 23.5x32m に対して
    # 歩いたのは 3.4x7.7m なので、点群で測ると 38m になり学習率が 10 倍過大になる。
    scene_scale = trajectory_radius(session / "benchmark" / "poses.txt")
    print(f"device={device}  地図={map_path.name}（{len(points):,} 点）  "
          f"軌跡半径={scene_scale:.2f} m  means lr={LR_MEANS*scene_scale:.2e}")

    began = time.time()
    measured = None
    if args.use_intensity:
        measured, unseen = intensity_for_map(session, points)
        print(f"反射強度を地図に割り当てた: 実測から取れた点 {100*(1-unseen.mean()):.1f} %  "
              f"中央値 {np.median(measured):.0f}（0〜255 で正規化して色にする）")
    arrays = init_from_points(points, init_opacity=args.init_opacity,
                              intensities=measured)
    params = to_parameters(arrays, device)
    if args.freeze_means:
        params["means"].requires_grad_(False)
    optimizers = build_optimizers(
        {k: v for k, v in params.items() if v.requires_grad}, scene_scale)
    # 位置の学習率は本家 3DGS と同じく指数減衰させる（gsplat の simple_trainer と同じ式）。
    # 一定のままだと終盤までガウシアンが動き続け、「消す」より「動かして辻褄を合わせる」
    # 逃げ道に流れてしまう。
    schedulers = ([torch.optim.lr_scheduler.ExponentialLR(
        optimizers["means"], gamma=0.01 ** (1.0 / max(args.iterations, 1)))]
        if "means" in optimizers else [])
    print(f"初期化おわり {time.time()-began:.0f} 秒  "
          f"スケール中央値={np.exp(arrays['scales']).mean(axis=1).mean()*100:.1f} cm")

    intensity_reader = IntensityScanReader(session) if args.use_intensity else None
    frames = load_frames(session / "benchmark" / "pcd", args.stride, args.limit)
    held_out = [frames[i] for i in np.linspace(0, len(frames) - 1, 20).astype(int)]
    print(f"学習に使うスキャン: {len(frames)} 枚  評価用に {len(held_out)} 枚を毎回見る")

    # 元の位置を控えておく。刈り込みで並びが変わっても追えるよう state に載せる
    origins_xyz = params["means"].detach().clone()
    state = {"orig_index": torch.arange(len(points), device=device),
             "orig_xyz": origins_xyz}

    before = evaluate_frames(params, frames, device, config, args.max_range, held_out)
    release_cache(device)
    print(f"学習前の描画誤差: MAE={before['mae']*100:.1f} cm  "
          f"RMSE={before['rmse']*100:.1f} cm  平均acc={before['acc']:.3f}\n")

    began = time.time()
    running = 0.0
    for step in range(1, args.iterations + 1):
        truth = None
        if intensity_reader is not None:
            index = int(generator.integers(len(intensity_reader)))
            origin, offsets, ranges, truth = read_intensity_frame(
                intensity_reader, index, device, args.max_range)
        else:
            path = frames[generator.integers(len(frames))]
            origin, offsets, ranges = read_frame(path, device, args.max_range)
        if len(ranges) == 0:
            continue
        if args.rays_per_scan and len(ranges) > args.rays_per_scan:
            pick = torch.randperm(len(ranges), device=device)[:args.rays_per_scan]
            offsets, ranges = offsets[pick], ranges[pick]
            if truth is not None:
                truth = truth[pick]

        free = ranges if args.lambda_free > 0 else None
        rendered = render_rays(params, origin, offsets, config,
                               with_color=truth is not None, free_space=free,
                               free_space_margin=args.free_margin,
                               free_space_max_range=args.free_max_range)
        depth, acc = rendered[0], rendered[1]
        predicted = depth / acc.clamp_min(0.1)
        loss = F.smooth_l1_loss(predicted, ranges, beta=0.05)
        loss = loss + args.lambda_acc * (1.0 - acc).abs().mean()
        if truth is not None:
            loss = loss + args.lambda_color * (
                rendered[3] / acc.clamp_min(0.1) - truth).abs().mean()
        if free is not None:
            loss = loss + args.lambda_free * rendered[-1].mean()

        for optimizer in optimizers.values():
            optimizer.zero_grad(set_to_none=True)
        loss.backward()
        for optimizer in optimizers.values():
            optimizer.step()
        for scheduler in schedulers:
            scheduler.step()
        running += float(loss.detach())
        if step % EMPTY_CACHE_EVERY == 0:
            release_cache(device)

        if step >= args.prune_start and step % args.prune_every == 0:
            with torch.no_grad():
                faint = torch.sigmoid(params["opacities"]) < args.prune_opacity
            if bool(faint.any()):
                gsplat_remove(params=params, optimizers=optimizers, state=state, mask=faint)
            print(f"  step {step}: {int(faint.sum()):,} 個を刈った → "
                  f"残り {len(params['means']):,}", flush=True)

        if step % 500 == 0:
            elapsed = time.time() - began
            with torch.no_grad():
                opacity_now = torch.sigmoid(params["opacities"])
                faint_now = float((opacity_now < args.prune_opacity).float().mean())
            print(f"    不透明度 中央値={float(opacity_now.median()):.3f} "
                  f"p10={float(opacity_now.quantile(0.1)):.3f} "
                  f"閾値未満={faint_now*100:.1f}%", flush=True)
            print(f"  step {step}/{args.iterations}  loss={running/500:.4f}  "
                  f"{step/elapsed:.1f} it/s  残り {(args.iterations-step)/(step/elapsed)/60:.1f} 分",
                  flush=True)
            running = 0.0

    train_seconds = time.time() - began
    release_cache(device)
    after = evaluate_frames(params, frames, device, config, args.max_range, held_out)
    release_cache(device)
    print(f"\n学習おわり {train_seconds/60:.1f} 分")
    print(f"学習後の描画誤差: MAE={after['mae']*100:.1f} cm  "
          f"RMSE={after['rmse']*100:.1f} cm  平均acc={after['acc']:.3f}")

    # ── 地図の各点を残すか消すか決める ──────────────────────────
    # OctoMap は八分木に点を問い合わせて占有/空を返す（`getLabels()`）。
    # こちらの対応物は「学習後のガウシアン場が、その位置をまだ支えているか」である。
    # 一番よく効いているガウシアンの α を密度とみなし、刈り込みと同じ閾値で切る。
    # 位置で判定するので、ガウシアンが逃げた跡地はちゃんと空になるし、
    # 逃げた先が実在の面なら別のガウシアンが支えていて残る。
    query_points = torch.from_numpy(points.astype(np.float32)).to(device)
    density = query_density(params, query_points)
    keep = (density >= args.prune_opacity).cpu().numpy()
    release_cache(device)

    # 内訳（不透明度だけ／移動量だけで見た場合との比較用）
    opacity = torch.sigmoid(params["opacities"])
    bright = opacity >= args.prune_opacity
    drift = (params["means"].detach() - state["orig_xyz"]).norm(dim=-1)
    stayed = torch.ones_like(bright) if args.max_drift <= 0 else drift <= args.max_drift
    survived = torch.zeros(len(points), dtype=torch.bool, device=device)
    survived[state["orig_index"][bright & stayed]] = True
    by_index = survived.cpu().numpy()
    removed = ~keep
    kept_points = points[keep]

    output_dir = session / "map"
    write_pcd(output_dir / args.output, [tuple(p) for p in kept_points])
    write_pcd(output_dir / args.output.replace(".pcd", "_removed.pcd"),
              [tuple(p) for p in points[removed]])

    splat_dir = session / "gs"
    splat_dir.mkdir(parents=True, exist_ok=True)
    write_splat_ply(splat_dir / f"gaussians_{args.tag}.ply", params)

    floor_z = estimate_floor(points)
    lines = [
        f"入力: {map_path}（{len(points):,} 点）",
        f"学習に使ったスキャン: {len(frames)} 枚（stride={args.stride}）  "
        f"反復 {args.iterations}  device={device}  {train_seconds/60:.1f} 分",
        f"自由空間の罰則: 重み {args.lambda_free}  余裕 {args.free_margin} m  "
        f"上限 {args.free_max_range} m" if args.lambda_free > 0 else "自由空間の罰則: 未使用",
        f"反射強度: {'色として学習に使用' if args.use_intensity else '未使用'}"
        f"{f'（重み {args.lambda_color}）' if args.use_intensity else ''}",
        f"設定: max_range={args.max_range}  init_opacity={args.init_opacity}  "
        f"prune_opacity={args.prune_opacity}  max_drift={args.max_drift}  "
        f"freeze_means={args.freeze_means}  lambda_acc={args.lambda_acc}",
        f"ビン {args.azimuth_bins}x{args.elevation_bins}  K={args.max_per_ray}  σ={args.sigma}",
        "",
        f"描画誤差（held-out {len(held_out)} 枚）: "
        f"学習前 MAE={before['mae']*100:.1f} cm / RMSE={before['rmse']*100:.1f} cm → "
        f"学習後 MAE={after['mae']*100:.1f} cm / RMSE={after['rmse']*100:.1f} cm",
        f"平均 acc: {before['acc']:.3f} → {after['acc']:.3f}",
        "",
        f"生き残ったガウシアン: {len(params['means']):,} / {len(points):,}",
        f"  不透明度が閾値以上: {int(bright.sum()):,}",
        f"  かつ移動量 {args.max_drift} m 以内: {int((bright & stayed).sum()):,}",
        f"判定は「学習後の場の密度 >= {args.prune_opacity}」。"
        f"不透明度と移動量だけで決めた場合は {int(by_index.sum()):,} 点が残る"
        f"（差 {int(keep.sum())-int(by_index.sum()):+,d}）",
        f"  移動量 中央値={float(drift.median())*100:.1f} cm  "
        f"p90={float(drift.quantile(0.9))*100:.1f} cm  最大={float(drift.max())*100:.1f} cm",
        f"残した点: {int(keep.sum()):,}（{100*keep.mean():.1f}%）  "
        f"消した点: {int(removed.sum()):,}（{100*removed.mean():.1f}%）",
        "",
        *report_by_height(points, removed, floor_z),
    ]
    report = session / "benchmark" / f"gs_report_{args.tag}.txt"
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n" + "\n".join(lines))
    print(f"\n書き出し: {output_dir/args.output} / {splat_dir}/gaussians_{args.tag}.ply / {report}")

    (splat_dir / f"summary_{args.tag}.json").write_text(json.dumps({
        "kept": int(keep.sum()), "removed": int(removed.sum()),
        "kept_by_index": int(by_index.sum()),
        "gaussians": len(params["means"]),
        "before": before, "after": after,
        "drift_median_cm": float(drift.median()) * 100,
        "train_minutes": train_seconds / 60,
    }, indent=2), encoding="utf-8")


def estimate_floor(points: np.ndarray) -> float:
    """run_octomap.py と同じ床の推定（高さ帯の表をそろえるため）。"""
    z = points[:, 2]
    low, high = np.percentile(z, [1.0, 50.0])
    lower = z[(z >= low) & (z <= high)]
    if len(lower) == 0:
        return float(z.min())
    counts, edges = np.histogram(lower, bins=100)
    return float(edges[int(counts.argmax())] + (edges[1] - edges[0]) / 2)


def report_by_height(points: np.ndarray, removed: np.ndarray, floor_z: float) -> "list[str]":
    lines = [f"床の高さ z={floor_z:.3f} m を基準にした高さ帯ごとの除去率",
             f"  {'高さ[m]':>12s} {'元の点数':>10s} {'消した点数':>10s} {'除去率':>8s}"]
    heights = points[:, 2] - floor_z
    edges = [-0.2, 0.25, 0.45, 0.65, 0.85, 1.05, 1.25, 1.5, 1.8, 2.2, 3.5]
    for low, high in zip(edges[:-1], edges[1:]):
        band = (heights >= low) & (heights < high)
        total = int(band.sum())
        if total == 0:
            continue
        gone = int((band & removed).sum())
        lines.append(f"  {low:5.2f}〜{high:5.2f} {total:10,d} {gone:10,d} {100*gone/total:7.1f}%")
    return lines


if __name__ == "__main__":
    main()
