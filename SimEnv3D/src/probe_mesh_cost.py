"""レイキャストのコストがメッシュ数に支配されることを確認する。

背景:
    3D LiDAR のコスト実測で、同じ 11520 ビームが
      平地（メッシュ 1 個）      : 2.2 ms
      Warehouse（メッシュ 3473） : 255.8 ms
    と 100 倍以上違った。ビーム数（360 〜 11520）を 32 倍にしても時間は
    変わらなかったため、コストはビーム数ではなくメッシュ数に支配される疑いが強い。

    これを確かめるため、raycast 対象に渡すメッシュ数を段階的に増やして測る。
    結果によって 3D 化の設計方針（メッシュを絞るか、別方式にするか）が決まる。

実行方法:
    cd <このリポジトリ>/SimEnv3D
    source env.sh
    "$ISAAC_SIM/python.sh" src/probe_mesh_cost.py --viz none --num-meshes 200
"""

from __future__ import annotations

import argparse
import time

# --- Isaac Sim の起動は他の import より先に行う必要がある ---
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="メッシュ数とレイキャストコストの関係")
parser.add_argument(
    "--num-meshes",
    type=int,
    required=True,
    help="raycast 対象に渡すメッシュ数（0 なら /World/Warehouse を丸ごと 1 パスで渡す）",
)
parser.add_argument("--channels", type=int, default=32, help="垂直方向の層数")
parser.add_argument("--beams", type=int, default=360, help="水平方向のビーム数")
parser.add_argument("--warmup", type=int, default=3, help="計測前に捨てる試行回数")
parser.add_argument("--trials", type=int, default=10, help="計測回数（中央値を採る）")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- Isaac Sim 起動後に import する ---
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from isaaclab.sensors import (  # noqa: E402
    MultiMeshRayCaster,
    MultiMeshRayCasterCfg,
    patterns,
)
from pxr import Usd, UsdGeom  # noqa: E402

WAREHOUSE_USD: str = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
MID360_VERTICAL_FOV: tuple[float, float] = (-7.0, 52.0)
MAX_RANGE: float = 30.0


def collect_mesh_paths(limit: int) -> list[str]:
    """Warehouse 内の Mesh prim パスを集める。

    Args:
        limit: 集める上限。0 以下なら全件。
    """
    import isaacsim.core.utils.stage as stage_utils

    stage = stage_utils.get_current_stage()
    paths: list[str] = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Warehouse")):
        if prim.IsA(UsdGeom.Mesh):
            paths.append(str(prim.GetPath()))
            if 0 < limit <= len(paths):
                break
    return paths


def sync() -> None:
    """GPU の処理完了を待つ。"""
    if torch.cuda.is_available():
        torch.cuda.synchronize()


def main() -> None:
    """指定したメッシュ数でのレイキャストコストを測る。"""
    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device=args.device)
    sim = sim_utils.SimulationContext(sim_cfg)

    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("[Probe] アセットサーバーに接続できません。")
    add_reference_to_stage(
        usd_path=assets_root + WAREHOUSE_USD, prim_path="/World/Warehouse"
    )
    light = sim_utils.DomeLightCfg(intensity=1500.0, color=(0.9, 0.9, 0.9))
    light.func("/World/DomeLight", light)

    if args.num_meshes == 0:
        mesh_paths = ["/World/Warehouse"]
        label = "/World/Warehouse を 1 パスで指定"
    else:
        mesh_paths = collect_mesh_paths(args.num_meshes)
        label = f"個別 Mesh を {len(mesh_paths)} 個指定"

    pattern = patterns.LidarPatternCfg(
        channels=args.channels,
        vertical_fov_range=MID360_VERTICAL_FOV,
        horizontal_fov_range=(-180.0, 180.0),
        horizontal_res=360.0 / args.beams,
    )
    # ロボットは置かず、固定位置のセンサだけで測る（姿勢の影響を排除する）
    cfg = MultiMeshRayCasterCfg(
        prim_path="/World/DomeLight",
        offset=MultiMeshRayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 1.1)),
        ray_alignment="base",
        pattern_cfg=pattern,
        mesh_prim_paths=mesh_paths,
        max_distance=MAX_RANGE,
        debug_vis=False,
    )

    build_start = time.perf_counter()
    sensor = MultiMeshRayCaster(cfg)
    sim.reset()
    build_ms = (time.perf_counter() - build_start) * 1000.0

    dt = sim.get_physics_dt()
    samples: list[float] = []
    for i in range(args.warmup + args.trials):
        sync()
        start = time.perf_counter()
        sensor.update(dt, force_recompute=True)
        sync()
        if i >= args.warmup:
            samples.append((time.perf_counter() - start) * 1000.0)

    samples.sort()
    median = samples[len(samples) // 2]
    num_beams = sensor.data.ray_hits_w[0].shape[0]

    print()
    print("=" * 70)
    print(f"[結果] {label}")
    print(f"  ビーム数    : {num_beams}")
    print(f"  構築+reset  : {build_ms:.0f} ms")
    print(f"  1 スキャン  : 中央値 {median:.1f} ms")
    print("=" * 70)


main()

simulation_app.close()
