"""PhysX LiDAR の挙動を確認する調査用スクリプト。

RotatingLidarPhysX クラスは Isaac Sim 6.0 の PhysxManager と非互換
（SimulationManager._get_backend_utils が無い）のため、
RangeSensorCreateLidar コマンドで prim を直接作り、
lidar_sensor_interface から読み出す方式を検証する。

LaserScan へ変換するために必要な以下を実測する:
    - linear_depth 配列の形状と値域
    - 1 スキャン分のビーム数が水平解像度と一致するか
    - 何もない方向（無限遠）がどんな値で返るか

平地に壁を 1 枚立てて、既知の距離が正しく測れるかを確認する。

実行方法:
    source env.sh && "$ISAAC_SIM/python.sh" src/probe_lidar.py
"""

from __future__ import annotations

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="PhysX LiDAR の挙動確認")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args(["--viz", "none"])

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- Isaac Sim 起動後にのみ import 可能 ---
import numpy as np  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402

# LiDAR の水平解像度 [deg]。360 / 1.0 = 360 ビームになるはず
H_RES: float = 1.0
# 壁を置く距離 [m]（この値が測れるかを確認する）
WALL_DISTANCE: float = 3.0
# LiDAR の設置高さ [m]
LIDAR_HEIGHT: float = 1.1
# LiDAR prim のパス
LIDAR_PATH: str = "/World/Lidar"


def main() -> None:
    """LiDAR を 1 基置いて、取得できるデータの形を確認する。"""
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.sensors.physx")

    import omni.kit.commands
    from isaacsim.sensors.physx import _range_sensor
    from pxr import Gf

    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device=args.device)
    sim = SimulationContext(sim_cfg)

    # 平地
    ground = sim_utils.GroundPlaneCfg()
    ground.func("/World/GroundPlane", ground)

    # 既知の距離に壁を置く。x = +WALL_DISTANCE の位置に薄い箱を立てる。
    wall = sim_utils.CuboidCfg(
        size=(0.2, 6.0, 3.0),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),
    )
    wall.func("/World/Wall", wall, translation=(WALL_DISTANCE, 0.0, 1.5))
    print(f"[INFO] 壁を x={WALL_DISTANCE} m に配置しました")

    # LiDAR prim を作る。
    # vertical_fov=0 / vertical_resolution=1 で 1 層だけの 2D LiDAR にする。
    # rotation_rate=0.0 は「1 物理ステップで全周を取得する」モード。
    result, _ = omni.kit.commands.execute(
        "RangeSensorCreateLidar",
        path=LIDAR_PATH,
        parent=None,
        translation=Gf.Vec3d(0.0, 0.0, LIDAR_HEIGHT),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        min_range=0.1,
        max_range=30.0,
        draw_points=False,
        draw_lines=False,
        horizontal_fov=360.0,
        vertical_fov=0.0,
        horizontal_resolution=H_RES,
        vertical_resolution=1.0,
        rotation_rate=0.0,
        high_lod=False,
        yaw_offset=0.0,
        enable_semantics=False,
    )
    print(f"[{'OK' if result else 'NG'}] LiDAR prim を作成しました: {LIDAR_PATH}")

    # prim が実際にステージ上に存在するか、型は何かを確認する
    from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid

    print(f"[INFO] prim 有効: {is_prim_path_valid(LIDAR_PATH)}")
    prim = get_prim_at_path(LIDAR_PATH)
    if prim and prim.IsValid():
        print(f"[INFO] prim 型: {prim.GetTypeName()}")
        print(f"[INFO] prim の属性: {[a.GetName() for a in prim.GetAttributes()][:12]}")

    lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
    print(f"[INFO] is_lidar_sensor: {lidar_interface.is_lidar_sensor(LIDAR_PATH)}")

    sim.reset()
    print("[OK] シミュレーションを初期化しました")
    print(f"[INFO] reset 後の is_lidar_sensor: {lidar_interface.is_lidar_sensor(LIDAR_PATH)}")

    # 物理タイムラインが動いているかを確認する（止まっていると LiDAR は更新されない）
    import omni.timeline

    timeline = omni.timeline.get_timeline_interface()
    print(f"[INFO] timeline is_playing: {timeline.is_playing()}")
    if not timeline.is_playing():
        timeline.play()
        print("[INFO] timeline を play しました")

    # 数ステップ回してデータが溜まるのを待つ
    for step in range(20):
        # LiDAR の更新はレンダリング／アプリ更新に紐づく可能性があるため
        # render=True で回し、app の update も明示的に呼ぶ
        sim.step(render=True)
        simulation_app.update()

        if step < 3 or step == 19:
            depth = np.asarray(lidar_interface.get_linear_depth_data(LIDAR_PATH))
            azimuth = np.asarray(lidar_interface.get_azimuth_data(LIDAR_PATH))
            print(f"\n--- step {step} ---")
            print(f"    linear_depth: shape={depth.shape} dtype={depth.dtype}")
            print(f"    azimuth:      shape={azimuth.shape} dtype={azimuth.dtype}")
            if depth.size:
                flat = depth.reshape(-1)
                print(
                    f"        min={flat.min():.3f} max={flat.max():.3f} "
                    f"mean={flat.mean():.3f}"
                )
                print(f"        先頭 8 個: {np.round(flat[:8], 3)}")
            if azimuth.size:
                az = azimuth.reshape(-1)
                print(
                    f"        azimuth min={az.min():.3f} max={az.max():.3f} "
                    f"（rad か deg かの判別用）"
                )

    # 最終確認
    depth = np.asarray(lidar_interface.get_linear_depth_data(LIDAR_PATH)).reshape(-1)
    azimuth = np.asarray(lidar_interface.get_azimuth_data(LIDAR_PATH)).reshape(-1)

    print(f"\n[INFO] 最終ビーム数: {depth.size}（期待値: {int(360 / H_RES)}）")

    if depth.size:
        # 壁までの距離が測れているか
        near_wall = depth[np.isclose(depth, WALL_DISTANCE, atol=0.3)]
        print(
            f"[{'OK' if near_wall.size else 'NG'}] 壁 ({WALL_DISTANCE} m) 付近を指す"
            f"ビーム数: {near_wall.size}"
        )

        # 何もない方向がどう返るかを調べる（LaserScan の range_max 判定に使う）
        far = depth[depth > 25.0]
        print(f"[INFO] 25 m 超のビーム数: {far.size}")
        if far.size:
            print(f"    その値の例: {np.unique(np.round(far, 3))[:5]}")

        # azimuth と depth の対応（壁の方向が azimuth のどこに来るか）
        if azimuth.size == depth.size:
            wall_idx = int(np.argmin(depth))
            print(
                f"[INFO] 最短ビーム: index={wall_idx} depth={depth[wall_idx]:.3f} "
                f"azimuth={azimuth[wall_idx]:.4f}"
            )
        else:
            print(
                f"[WARN] azimuth と depth の要素数が不一致: "
                f"{azimuth.size} vs {depth.size}"
            )
    else:
        print("[NG] linear_depth が空です")

    print("\n[OK] 調査が完了しました")


if __name__ == "__main__":
    main()
    simulation_app.close()
