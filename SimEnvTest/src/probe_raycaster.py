"""IsaacLab の MultiMeshRayCaster で 2D LiDAR を作れるか検証する調査用スクリプト。

PhysX LiDAR (RangeSensorSchema) は Isaac Sim 6.0 のこのビルドでは動作しない
（visibility 属性が無く prim 作成に失敗する / debug_draw プラグインが
undefined symbol で読めない）ため、代替として RayCaster を検証する。

確認したいこと:
    1. Warehouse の全 Mesh (3473 個) を raycast 対象にできるか
    2. その場合の初期化時間とメモリが現実的か
    3. LidarPatternCfg で 2D スキャン (channels=1, 360度) が得られるか
    4. 得られる距離が正しいか（既知の位置の壁で確認）

実行方法:
    source env.sh && "$ISAAC_SIM/python.sh" src/probe_raycaster.py
"""

from __future__ import annotations

import argparse
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="RayCaster 版 LiDAR の検証")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args(["--viz", "none"])

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- Isaac Sim 起動後にのみ import 可能 ---
import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sensors import MultiMeshRayCasterCfg, patterns  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_assets.robots.unitree import G1_CFG  # noqa: E402

from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402

WAREHOUSE_USD: str = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
# LiDAR の設置高さ（G1 の胴体上部〜頭部。地上 1.0〜1.2 m の想定）
LIDAR_HEIGHT: float = 1.1
SPAWN_HEIGHT: float = 0.74


def main() -> None:
    """Warehouse に G1 と RayCaster LiDAR を置いてスキャンを取得する。"""
    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device=args.device)
    sim = SimulationContext(sim_cfg)

    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("[NG] アセットサーバーに接続できません。")

    add_reference_to_stage(
        usd_path=assets_root + WAREHOUSE_USD, prim_path="/World/Warehouse"
    )
    print("[OK] Warehouse を読み込みました")

    light = sim_utils.DomeLightCfg(intensity=1500.0)
    light.func("/World/DomeLight", light)

    robot_cfg = G1_CFG.replace(prim_path="/World/G1")
    robot_cfg.init_state = robot_cfg.init_state.replace(pos=(0.0, 0.0, SPAWN_HEIGHT))
    robot = Articulation(robot_cfg)
    print("[OK] G1 を配置しました")

    # 2D LiDAR パターン: 1 層 (channels=1)、水平 360 度を 1 度刻み
    lidar_pattern = patterns.LidarPatternCfg(
        channels=1,
        vertical_fov_range=(0.0, 0.0),
        horizontal_fov_range=(-180.0, 180.0),
        horizontal_res=1.0,
    )

    # 胴体 (torso_link) に取り付ける。offset で LiDAR の高さまで持ち上げる。
    # G1 の胴体原点は腰付近なので、そこからの相対で 1.1 m 付近を狙う。
    lidar_cfg = MultiMeshRayCasterCfg(
        prim_path="/World/G1/torso_link",
        offset=MultiMeshRayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.0)),
        ray_alignment="base",
        pattern_cfg=lidar_pattern,
        # Warehouse 配下の全 Mesh を対象にする
        mesh_prim_paths=["/World/Warehouse"],
        max_distance=30.0,
        debug_vis=False,
    )

    print("[INFO] RayCaster を構築します（Warehouse の Mesh 走査に時間がかかります）...")
    build_start = time.time()
    from isaaclab.sensors import MultiMeshRayCaster

    lidar = MultiMeshRayCaster(lidar_cfg)
    build_elapsed = time.time() - build_start
    print(f"[INFO] RayCaster の生成: {build_elapsed:.1f} 秒")

    reset_start = time.time()
    sim.reset()
    reset_elapsed = time.time() - reset_start
    print(f"[INFO] sim.reset(): {reset_elapsed:.1f} 秒")
    print("[OK] シミュレーションを初期化しました")

    # 数ステップ回してスキャンを取得
    for step in range(10):
        sim.step(render=False)
        robot.update(0.005)
        lidar.update(0.005, force_recompute=True)

    # ray_hits_w はワールド座標の当たり点。センサ原点からの距離に変換する。
    hits_w = lidar.data.ray_hits_w  # (N, B, 3)
    sensor_pos = lidar.data.pos_w  # (N, 3)
    print(f"\n[INFO] ray_hits_w: shape={tuple(hits_w.shape)} dtype={hits_w.dtype}")
    print(f"[INFO] sensor_pos: {sensor_pos[0].cpu().numpy().round(3)}")

    # 取り付け位置の確認: torso_link の実際の高さと、ロボット原点との関係を出す。
    # offset をいくつにすれば地上 1.1 m になるかを決めるため。
    import warp as wp

    root_pos = wp.to_torch(robot.data.root_pos_w)[0]
    print(f"[INFO] G1 root_pos_w: {root_pos.cpu().numpy().round(3)}")
    body_names = robot.data.body_names
    if "torso_link" in body_names:
        torso_idx = body_names.index("torso_link")
        torso_pos = wp.to_torch(robot.data.body_pos_w)[0, torso_idx]
        print(f"[INFO] torso_link 位置: {torso_pos.cpu().numpy().round(3)}")
        print(
            f"[INFO] LiDAR は torso_link から "
            f"{(sensor_pos[0][2] - torso_pos[2]).item():+.3f} m の位置"
        )
        print(
            f"[HINT] 地上 {LIDAR_HEIGHT} m にするには offset.z を "
            f"{LIDAR_HEIGHT - torso_pos[2].item():+.3f} にする"
        )
    else:
        print(f"[WARN] torso_link が見つかりません。body 一覧: {body_names[:10]}")

    hits = hits_w[0]  # (B, 3)
    origin = sensor_pos[0]  # (3,)
    # 無限遠は inf で返る場合があるので有限のものだけ距離を出す
    finite_mask = torch.isfinite(hits).all(dim=-1)
    distances = torch.full((hits.shape[0],), float("inf"), device=hits.device)
    distances[finite_mask] = torch.linalg.norm(
        hits[finite_mask] - origin.unsqueeze(0), dim=-1
    )

    d = distances.cpu().numpy()
    valid = np.isfinite(d)
    print(f"\n[INFO] ビーム数: {d.size}（期待値: 360）")
    print(f"[INFO] 有効ビーム数: {int(valid.sum())}")
    if valid.any():
        print(
            f"[INFO] 距離 min={d[valid].min():.3f} max={d[valid].max():.3f} "
            f"mean={d[valid].mean():.3f}"
        )
        print(f"[INFO] 先頭 12 個: {np.round(d[:12], 2)}")
        print(f"[{'OK' if valid.sum() > d.size * 0.5 else 'WARN'}] "
              f"有効ビームの割合: {valid.sum() / d.size:.1%}")
    else:
        print("[NG] 有効なビームがありません")

    print("\n[OK] 調査が完了しました")


if __name__ == "__main__":
    main()
    simulation_app.close()
