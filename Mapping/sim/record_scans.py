#!/usr/bin/env python
"""[エントリポイント] MuJoCo 上の G1 を部屋の中で歩かせ、LiDAR スキャンを収録する。

計測（このスクリプト）と地図化（`Mapping/run_slam.py`）を分けてある。理由は 2 つ:

1. SLAM のパラメータを触るたびにシミュレーションを回し直すのは重すぎる。
   一度収録した `scans.npz` に対して地図化だけ何度でもやり直せるようにしたい。
2. 実機でも「歩いて計測する」と「持ち帰って地図にする」は別作業になる。
   同じ `scans.npz` の形式に落としておけば、地図化側は sim / real 共通で使える。

収録するのは各フレームの
  - LiDAR センサー座標系の点群（SLAM への入力）
  - LiDAR のワールド姿勢の**真値**（SLAM は使わない。誤差評価のためだけに保存する）

このスクリプトはシミュレーション専用。`SimpleWalk/sim/release_band_and_walk_forward.py`
と同じく MuJoCo の内部オブジェクト（非公開 API）に触っているため、実機では動かない。

使い方:
    python Mapping/sim/record_scans.py
    python Mapping/sim/record_scans.py --out Mapping/data/scans.npz --scan-hz 10
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from Mapping.common.lidar_spec import LidarSpec  # noqa: E402
from Mapping.sim.mujoco_lidar import MujocoLidar  # noqa: E402
from Mapping.sim.scene import SceneOverride, build_scene  # noqa: E402

# (ラベル, remote 入力, 継続秒数)。remote.ly=前進, remote.lx=横移動, remote.rx=旋回。
# 符号の対応（ly->vx, lx->-vy, rx->-yaw_rate）は
# SimpleWalk/sim/verify_g1_sim_command.py で検証済み。
# 部屋は x:-5〜5 / y:-4〜4 で、G1 は原点で +x を向いてスポーンする。
# 壁と障害物を別角度から見られるよう、部屋の中央付近で長方形を 1 周する。
#
# 継続秒数は実測から決めている（真値の軌跡を読んで計測した値）:
#   remote.ly=0.5 -> 前進 約 0.29 m/s
#   remote.rx=-0.5 -> 左旋回 約 6.6 deg/s（かなり遅い。90 度回るのに 14 秒かかる）
# 歩行ポリシーには障害物回避が無いので、壁や箱に当たらない大きさに収めること。
DEFAULT_TOUR: list[tuple[str, dict[str, float], float]] = [
    ("settle", {}, 2.0),
    ("forward", {"remote.ly": 0.5}, 8.0),
    ("turn_left", {"remote.rx": -0.5}, 14.0),
    ("forward", {"remote.ly": 0.5}, 6.0),
    ("turn_left", {"remote.rx": -0.5}, 14.0),
    ("forward", {"remote.ly": 0.5}, 8.0),
    ("turn_left", {"remote.rx": -0.5}, 14.0),
    ("forward", {"remote.ly": 0.5}, 6.0),
    ("turn_left", {"remote.rx": -0.5}, 14.0),
]


def get_inner_sim(robot):  # type: ignore[no-untyped-def]
    """MuJoCo の生データ（mj_model / mj_data）と elastic_band を持つ実体を取り出す。

    `UnitreeG1.sim_env` は gym ラッパーで、実体はさらに 1 段ネストした
    `.simulator.sim_env` にある。非公開 API なので HF Hub 側の更新で壊れうる
    （`SimpleWalk/sim/release_band_and_walk_forward.py` と同じ辿り方）。
    """
    sim = getattr(robot.sim_env, "simulator", None)
    return getattr(sim, "sim_env", None) if sim is not None else None


def main() -> None:
    parser = argparse.ArgumentParser(description="MuJoCo 上の G1 で LiDAR スキャンを収録する")
    parser.add_argument("--out", default="Mapping/data/scans.npz", help="出力先の npz")
    parser.add_argument("--scan-hz", type=float, default=10.0, help="スキャンを撮る頻度[Hz]")
    parser.add_argument("--control-hz", type=float, default=50.0, help="コマンド送信の頻度[Hz]")
    parser.add_argument(
        "--work-dir",
        default="",
        help="シーン XML の生成先。既定は出力先と同じディレクトリの scene/",
    )
    args = parser.parse_args()

    out_path = Path(args.out)
    work_dir = Path(args.work_dir) if args.work_dir else out_path.parent / "scene"

    # robot.connect() より前にシーンを差し替える。既定シーンは無限平面と G1 だけで
    # 壁も障害物も無く、LiDAR を回しても床しか返らない（scene.py の説明を参照）。
    scene_path = build_scene(work_dir)
    override = SceneOverride(scene_path)
    override.install()
    print(f"[scene] 部屋シーンを生成: {scene_path}", flush=True)

    from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
    from lerobot.robots.unitree_g1.g1_utils import default_remote_input

    cfg = UnitreeG1Config(is_simulation=True, controller="GrootLocomotionController")
    robot = UnitreeG1(cfg)
    print("[sim] 接続中（初回はポリシーの取得で時間がかかる）...", flush=True)
    robot.connect()
    time.sleep(1.0)

    if override.hit_count == 0:
        print("[error] シーンの差し替えが効いていない。既定の無限平面で走っている。", flush=True)
        robot.disconnect()
        os._exit(1)

    inner = get_inner_sim(robot)
    if inner is None or not hasattr(inner, "elastic_band"):
        print("[error] MuJoCo の内部階層が想定と違う（HF Hub 側の更新か）。中止する。", flush=True)
        robot.disconnect()
        os._exit(1)

    # 既定で骨盤を z=1m 付近に吊る elastic band が有効になっており、
    # 解除しないと足が接地せず、そもそも歩かない（G1_HuggingFace/README.md 参照）。
    inner.elastic_band.enable = False
    print("[sim] elastic band を解除した", flush=True)

    spec = LidarSpec()
    lidar = MujocoLidar(inner.mj_model, spec)
    print(f"[lidar] {spec.n_channels}ch x {spec.n_azimuth}az = {spec.n_rays} rays/frame", flush=True)

    scan_points: list[np.ndarray] = []
    scan_poses: list[np.ndarray] = []
    scan_times: list[float] = []

    control_dt = 1.0 / args.control_hz
    scan_interval = 1.0 / args.scan_hz
    action = default_remote_input()
    t_start = time.perf_counter()
    next_scan = t_start

    for label, overrides, duration in DEFAULT_TOUR:
        print(f"[tour] {label} ({duration:.0f}s) {overrides}", flush=True)
        command = default_remote_input()
        command.update(overrides)
        t_end = time.perf_counter() + duration
        while time.perf_counter() < t_end:
            loop_start = time.perf_counter()
            action.update(command)
            robot.send_action(dict(action))

            if loop_start >= next_scan:
                points, pose = lidar.scan(inner.mj_model, inner.mj_data)
                scan_points.append(points.astype(np.float32))
                scan_poses.append(pose.astype(np.float64))
                scan_times.append(loop_start - t_start)
                next_scan += scan_interval

            elapsed = time.perf_counter() - loop_start
            if elapsed < control_dt:
                time.sleep(control_dt - elapsed)

    counts = np.array([len(p) for p in scan_points], dtype=np.int64)
    if counts.sum() == 0:
        print("[error] 点が 1 つも取れていない。LiDAR の取り付け位置を確認すること。", flush=True)
        robot.disconnect()
        os._exit(1)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        out_path,
        # 全フレームの点を 1 本に連結し、フレーム境界を counts で持つ
        # （フレームごとに点数が違うので 3 次元配列にはできない）。
        points=np.concatenate(scan_points, axis=0),
        counts=counts,
        # 真値。SLAM には渡さず、run_slam.py の誤差評価だけに使う。
        gt_poses=np.stack(scan_poses, axis=0),
        times=np.array(scan_times, dtype=np.float64),
    )

    gt_xy = np.stack([p[:3, 3] for p in scan_poses])[:, :2]
    print("", flush=True)
    print(f"[result] {len(scan_points)} フレーム / {counts.sum()} 点 を保存: {out_path}", flush=True)
    print(f"[result] 1 フレームあたりの点数: 平均 {counts.mean():.0f} / 最小 {counts.min()}", flush=True)
    print(
        f"[result] 実際に歩いた範囲(真値): "
        f"x [{gt_xy[:, 0].min():.2f}, {gt_xy[:, 0].max():.2f}] "
        f"y [{gt_xy[:, 1].min():.2f}, {gt_xy[:, 1].max():.2f}]",
        flush=True,
    )
    print(f"[next] python Mapping/run_slam.py --scans {out_path}", flush=True)

    try:
        robot.disconnect()
    except Exception as e:  # noqa: BLE001
        print(f"[warn] disconnect() でエラー（無視する）: {e}", flush=True)
    sys.stdout.flush()
    # disconnect() 後も非デーモンの subscribe スレッドが残ってプロセスが終わらない
    # ことがあるため、後片付けが済んだここで確実に落とす（SimpleWalk と同じ）。
    os._exit(0)


if __name__ == "__main__":
    main()
