"""LidarPatternCfg が channels を反映しているかを確認する（Isaac Sim 不要）。

probe_lidar3d_cost.py の実測で「層数を 64 倍にしても時間が変わらない」という
物理的にありえない結果が出たため、パターン生成そのものを切り分ける。

実行方法:
    cd /home/spacedata/isaac_dev/G1/SimEnv3D
    source env.sh
    python3 src/probe_pattern_check.py
"""

from __future__ import annotations

import torch

from isaaclab.sensors.ray_caster.patterns import patterns, patterns_cfg

MID360_VERTICAL_FOV: tuple[float, float] = (-7.0, 52.0)


def check(channels: int, horizontal_beams: int) -> int:
    """指定条件で生成されるレイ本数を返す。"""
    cfg = patterns_cfg.LidarPatternCfg(
        channels=channels,
        vertical_fov_range=MID360_VERTICAL_FOV,
        horizontal_fov_range=(-180.0, 180.0),
        horizontal_res=360.0 / horizontal_beams,
    )
    starts, dirs = patterns.lidar_pattern(cfg, "cpu")

    # 一意な z 成分の数 = 実際の層数（層ごとに仰角が違うため）
    unique_z = torch.unique(torch.round(dirs[:, 2] * 1e6) / 1e6).numel()
    print(
        f"  {channels:>3} 層 x {horizontal_beams:>4} = 期待 {channels * horizontal_beams:>6} 本 "
        f"-> 実際 {dirs.shape[0]:>6} 本, 一意な仰角 {unique_z:>3} 個"
    )
    return dirs.shape[0]


def main() -> None:
    print("[Check] LidarPatternCfg が生成するレイ本数")
    for channels in (1, 4, 8, 16, 32, 64):
        for beams in (90, 360):
            check(channels, beams)


main()
