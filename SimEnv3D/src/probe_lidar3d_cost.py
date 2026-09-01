"""3D LiDAR のレイキャストコストを実測する（ステップ 1: コストマップ）。

目的:
    2D LiDAR（360 ビーム 1 層）は 1 スキャン 74 ms かかっており、10Hz 配信で
    ぎりぎりだった。3D 化で層数を増やしたときコストがどう伸びるかを実測し、
    採用できる層数・水平解像度を決める。

重要（最初の実装での失敗）:
    当初は 24 条件ぶんのセンサを 1 プロセス内に全部構築し、順に測っていた。
    しかし MultiMeshRayCaster はシーンに登録された全センサがまとめて更新される
    ため、1 条件を測っているつもりで毎回 24 センサ分のコストを払っていた。
    その結果「層数を 64 倍にしても時間が変わらない」という物理的にありえない
    数字が出た（層数ではなく水平解像度だけが効いているように見えた）。

    センサは 1 プロセスに 1 つだけ構築すること。条件を変えるにはプロセスを
    作り直す。--channels / --beams で 1 条件だけ測り、グリッド全体は
    run_cost_map.sh が条件ごとにプロセスを起こして回す。

実行方法（単一条件）:
    cd <このリポジトリ>/SimEnv3D
    source env.sh
    "$ISAAC_SIM/python.sh" src/probe_lidar3d_cost.py --viz none --channels 16 --beams 360

実行方法（グリッド全体）:
    bash run_cost_map.sh

注意:
    run_g1_twin.py は import してはいけない（モジュール直下で AppLauncher を
    起動するため、アプリが二重起動して無言で落ちる）。そのため本スクリプトは
    独自に AppLauncher を起動する。
"""

from __future__ import annotations

import argparse
import math
import time

# --- Isaac Sim の起動は他の import より先に行う必要がある ---
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="3D LiDAR のレイキャストコスト実測")
parser.add_argument("--channels", type=int, required=True, help="垂直方向の層数")
parser.add_argument(
    "--beams", type=int, required=True, help="水平方向のビーム数（360 度を等分）"
)
parser.add_argument(
    "--flat",
    action="store_true",
    help="Warehouse を使わず平地で実行する（コストの下限を見る用）",
)
parser.add_argument(
    "--warmup", type=int, default=3, help="計測前に捨てる試行回数（初回は構築を含む）"
)
parser.add_argument(
    "--trials", type=int, default=15, help="各条件での計測回数（中央値を採る）"
)
parser.add_argument(
    "--settle-steps",
    type=int,
    default=60,
    help="G1 を接地させ姿勢を安定させるためのステップ数",
)
parser.add_argument(
    "--tilt", type=float, default=20.0, help="センサの前傾角 [deg]"
)
parser.add_argument(
    "--tsv",
    type=str,
    default="",
    help="結果を 1 行の TSV として追記するファイル（グリッド集計用）",
)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- Isaac Sim 起動後に import する ---
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sensors import (  # noqa: E402
    MultiMeshRayCaster,
    MultiMeshRayCasterCfg,
    patterns,
)
from isaaclab_assets.robots.unitree import G1_CFG  # noqa: E402

# Warehouse シーンの相対パス（runner.py と同じ）
WAREHOUSE_USD: str = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
SPAWN_HEIGHT: float = 0.80

# Livox Mid-360 の垂直 FOV [deg]。実機 G1 の標準構成に合わせる。
MID360_VERTICAL_FOV: tuple[float, float] = (-7.0, 52.0)

MAX_RANGE: float = 30.0
MIN_RANGE: float = 0.3
# torso_link は地上 0.753 m。センサを地上 1.1 m に置く。
LIDAR_OFFSET_Z: float = 1.1 - 0.753


def tilt_quat_wxyz(tilt_deg: float) -> tuple[float, float, float, float]:
    """前傾（pitch 方向の回転）を表すクォータニオンを返す。

    OffsetCfg.rot は IsaacLab の規約で (w, x, y, z) 順。ROS の (x,y,z,w) とは
    異なるので注意する（過去にこの取り違えで /odom が壊れた）。

    Args:
        tilt_deg: 前傾角 [deg]。正の値でセンサが下を向く。
    """
    half = math.radians(tilt_deg) / 2.0
    return (math.cos(half), 0.0, math.sin(half), 0.0)


def collect_mesh_paths(root: str) -> list[str]:
    """root 配下の Mesh prim パスを個別に列挙する。

    これが性能上の決定的なポイント。親パス（"/World/Warehouse"）を 1 つ渡すと
    MultiMeshRayCaster は毎スキャンで配下を走査し直すらしく、同じメッシュ・
    同じビーム数でも 467 ms かかる。Mesh を個別に列挙して渡すと 3.2 ms になる
    （146 倍の差、実測）。ビーム数やメッシュ数ではなくこの指定方法が支配的だった。
    """
    import isaacsim.core.utils.stage as stage_utils
    from pxr import Usd, UsdGeom

    stage = stage_utils.get_current_stage()
    paths: list[str] = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.IsA(UsdGeom.Mesh):
            paths.append(str(prim.GetPath()))
    return paths


def build_scene(use_warehouse: bool) -> Articulation:
    """Warehouse（または平地）と G1 を配置する。runner.build_scene と同等。"""
    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError(
            "[Probe] アセットサーバーに接続できません。ネットワーク接続を確認してください。"
        )

    if use_warehouse:
        warehouse_path = assets_root + WAREHOUSE_USD
        add_reference_to_stage(usd_path=warehouse_path, prim_path="/World/Warehouse")
        print(f"[OK] Warehouse シーンを読み込みました")
    else:
        ground = sim_utils.GroundPlaneCfg()
        ground.func("/World/GroundPlane", ground)
        print("[OK] 平地を生成しました")

    light = sim_utils.DomeLightCfg(intensity=1500.0, color=(0.9, 0.9, 0.9))
    light.func("/World/DomeLight", light)

    robot_cfg = G1_CFG.replace(prim_path="/World/G1")
    robot_cfg.init_state = robot_cfg.init_state.replace(pos=(0.0, 0.0, SPAWN_HEIGHT))
    robot = Articulation(robot_cfg)
    print(f"[OK] G1 を配置しました")
    return robot


def make_sensor(
    channels: int, horizontal_beams: int, mesh_paths: list[str], tilt_deg: float
) -> MultiMeshRayCaster:
    """指定した層数・水平ビーム数の 3D LiDAR を構築する。

    1 プロセスに 1 つだけ構築すること（複数作ると全部まとめて更新され、
    1 条件の計測に他条件のコストが混入する）。
    """
    pattern = patterns.LidarPatternCfg(
        channels=channels,
        vertical_fov_range=MID360_VERTICAL_FOV,
        horizontal_fov_range=(-180.0, 180.0),
        horizontal_res=360.0 / horizontal_beams,
    )

    # 前傾させる。3D 点群は各点が 3 次元座標を持つため、2D と違い
    # ray_alignment="base" でも傾きで壊れない（実機の搭載姿勢にも近い）。
    cfg = MultiMeshRayCasterCfg(
        prim_path="/World/G1/torso_link",
        offset=MultiMeshRayCasterCfg.OffsetCfg(
            pos=(0.0, 0.0, LIDAR_OFFSET_Z),
            rot=tilt_quat_wxyz(tilt_deg),
        ),
        ray_alignment="base",
        pattern_cfg=pattern,
        mesh_prim_paths=mesh_paths,
        max_distance=MAX_RANGE,
        debug_vis=False,
    )
    return MultiMeshRayCaster(cfg)


def sync() -> None:
    """GPU の処理完了を待つ（非同期実行で計測がずれるのを防ぐ）。"""
    if torch.cuda.is_available():
        torch.cuda.synchronize()


def measure(
    sensor: MultiMeshRayCaster, dt: float, trials: int, warmup: int
) -> tuple[float, float, float, float, int]:
    """1 スキャンの所要時間を測る。

    Returns:
        (中央値 [ms], 最小 [ms], レイキャストのみ [ms], 当たり率 [0-1], 総ビーム数)
    """
    totals: list[float] = []
    casts: list[float] = []
    hit_ratio = 0.0
    num_beams = 0

    for i in range(warmup + trials):
        sync()
        start = time.perf_counter()

        # force_recompute=True は本番と同じ条件（毎回レイキャストし直す）
        sensor.update(dt, force_recompute=True)
        sync()
        after_cast = time.perf_counter()

        # 距離への変換（lidar.py の read_scan と同じ処理）
        hits_w = sensor.data.ray_hits_w[0]
        origin = sensor.data.pos_w[0]
        finite = torch.isfinite(hits_w).all(dim=-1)
        distances = torch.full(
            (hits_w.shape[0],), float("inf"), device=hits_w.device, dtype=torch.float32
        )
        if finite.any():
            distances[finite] = torch.linalg.norm(
                hits_w[finite] - origin.unsqueeze(0), dim=-1
            )
        distances[distances < MIN_RANGE] = 0.0

        sync()
        end = time.perf_counter()

        if i >= warmup:
            totals.append((end - start) * 1000.0)
            casts.append((after_cast - start) * 1000.0)
            hit_ratio = float(finite.float().mean().item())
            num_beams = hits_w.shape[0]

    totals.sort()
    casts.sort()
    return (
        totals[len(totals) // 2],
        totals[0],
        casts[len(casts) // 2],
        hit_ratio,
        num_beams,
    )


def main() -> None:
    """1 条件のコストを実測して出力する。"""
    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device=args.device)
    sim = sim_utils.SimulationContext(sim_cfg)

    use_warehouse = not args.flat
    robot = build_scene(use_warehouse)

    # Mesh を個別に列挙して渡す（親パスを渡すと 146 倍遅くなる。実測済み）
    if use_warehouse:
        mesh_paths = collect_mesh_paths("/World/Warehouse")
        print(f"[OK] raycast 対象の Mesh を {len(mesh_paths)} 個列挙しました")
    else:
        mesh_paths = ["/World/GroundPlane"]

    # センサは sim.reset() より前に構築する必要がある（登録が reset 時に行われる）
    build_start = time.perf_counter()
    sensor = make_sensor(args.channels, args.beams, mesh_paths, args.tilt)
    build_ms = (time.perf_counter() - build_start) * 1000.0

    sim.reset()

    # G1 を接地させ姿勢を安定させる（宙に浮いた状態では当たり率が変わる）
    dt = sim.get_physics_dt()
    for _ in range(args.settle_steps):
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)

    median, minimum, cast_ms, hit_ratio, num_beams = measure(
        sensor, dt, args.trials, args.warmup
    )

    print()
    print("=" * 70)
    print(f"[結果] {args.channels} 層 x {args.beams} 水平 = {num_beams} ビーム")
    print(f"  シーン        : {'Warehouse' if use_warehouse else '平地'}")
    print(f"  垂直 FOV      : {MID360_VERTICAL_FOV[0]} 〜 {MID360_VERTICAL_FOV[1]} deg")
    print(f"  前傾角        : {args.tilt} deg")
    print(f"  センサ構築    : {build_ms:.0f} ms")
    print(f"  1 スキャン    : 中央値 {median:.1f} ms / 最小 {minimum:.1f} ms")
    print(f"   うちレイ演算 : {cast_ms:.1f} ms")
    print(f"  当たり率      : {hit_ratio * 100:.1f}%")
    print(f"  10Hz (100ms)  : {'OK' if median < 100.0 else 'NG'}")
    print(f"  50Hz  (20ms)  : {'OK' if median < 20.0 else 'NG'}")
    print("=" * 70)

    if args.tsv:
        # グリッド集計用に 1 行追記する
        with open(args.tsv, "a") as f:
            f.write(
                f"{args.channels}\t{args.beams}\t{num_beams}\t{median:.2f}\t"
                f"{minimum:.2f}\t{cast_ms:.2f}\t{hit_ratio * 100:.1f}\n"
            )
        print(f"[Probe] TSV へ追記しました: {args.tsv}")


main()

# finally で閉じない（例外が隠れて「エラー無く終了した」ように見えるため）
simulation_app.close()
