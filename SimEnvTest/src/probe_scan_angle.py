"""LaserScan の角度規約が正しいかを検証する調査用スクリプト。

地図が放射状に壊れる原因の切り分け用。

既知の位置に壁を 1 枚だけ置き、「その壁が LaserScan のどの index に
現れるか」を調べる。ranges[i] の角度は angle_min + i * angle_increment
であり、この対応がずれていると SLAM はスキャンを回転させて重ねてしまい、
放射状の縞模様になる。

実行方法:
    source env.sh && "$ISAAC_SIM/python.sh" src/probe_scan_angle.py
"""

from __future__ import annotations

import argparse
import math

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="LaserScan の角度規約の検証")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args(["--viz", "none"])

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- Isaac Sim 起動後にのみ import 可能 ---
import numpy as np  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_assets.robots.unitree import G1_CFG  # noqa: E402

import sys  # noqa: E402

sys.path.insert(0, "/home/spacedata/isaac_dev/G1/SimEnvTest/src")
from g1_twin.lidar import G1Lidar  # noqa: E402

SPAWN_HEIGHT: float = 0.74
# 壁を置く距離 [m]
WALL_DIST: float = 4.0


def main() -> None:
    """ロボットの正面・左・右に壁を置き、スキャン上の位置を確認する。"""
    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device=args.device)
    sim = SimulationContext(sim_cfg)

    ground = sim_utils.GroundPlaneCfg()
    ground.func("/World/GroundPlane", ground)
    light = sim_utils.DomeLightCfg(intensity=1500.0)
    light.func("/World/DomeLight", light)

    # 壁を 3 枚置く。ロボットは原点で yaw=0（+X 向き）とする。
    #   前方 (+X): 4 m   -> scan では 0 度に出るはず
    #   左   (+Y): 6 m   -> scan では +90 度に出るはず
    #   右   (-Y): 8 m   -> scan では -90 度に出るはず
    # 距離を変えてあるので、どれがどれか区別できる。
    walls = {
        "前方(+X) 4m": ((WALL_DIST, 0.0), (0.2, 8.0, 3.0)),
        "左(+Y) 6m": ((0.0, 6.0), (8.0, 0.2, 3.0)),
        "右(-Y) 8m": ((0.0, -8.0), (8.0, 0.2, 3.0)),
    }
    for i, (name, ((wx, wy), size)) in enumerate(walls.items()):
        cfg = sim_utils.CuboidCfg(
            size=size,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),
        )
        cfg.func(f"/World/Wall{i}", cfg, translation=(wx, wy, 1.5))
        print(f"[INFO] 壁を配置: {name} at ({wx}, {wy})")

    robot_cfg = G1_CFG.replace(prim_path="/World/G1")
    robot_cfg.init_state = robot_cfg.init_state.replace(
        pos=(0.0, 0.0, SPAWN_HEIGHT), rot=(1.0, 0.0, 0.0, 0.0)
    )
    robot = Articulation(robot_cfg)

    # 壁も raycast の対象に含める
    lidar = G1Lidar(
        robot_prim_path="/World/G1",
        mesh_prim_paths=["/World/Wall0", "/World/Wall1", "/World/Wall2"],
    )

    sim.reset()
    robot.reset()
    for _ in range(10):
        sim.step(render=False)
        robot.update(0.005)
    lidar.update(0.02)

    scan = lidar.read_scan()
    ranges = np.array(scan.ranges)

    print(f"\n[INFO] angle_min={math.degrees(scan.angle_min):+.1f} deg")
    print(f"[INFO] angle_max={math.degrees(scan.angle_max):+.1f} deg")
    print(f"[INFO] angle_increment={math.degrees(scan.angle_increment):.3f} deg")
    print(f"[INFO] ビーム数={len(ranges)}")

    # 各壁が実際にどの角度に現れたかを調べる
    print("\n--- 各壁がスキャン上のどこに現れたか ---")
    for label, expected_deg, expected_dist in (
        ("前方(+X)", 0.0, 4.0),
        ("左(+Y)", 90.0, 6.0),
        ("右(-Y)", -90.0, 8.0),
    ):
        # その距離に近いビームを探す
        near = np.where(np.abs(ranges - expected_dist) < 0.5)[0]
        if near.size == 0:
            print(f"[NG] {label}: 距離 {expected_dist} m のビームが見つからない")
            continue
        # 見つかったビームの角度（中央値で代表させる）
        angles = np.degrees(scan.angle_min + near * scan.angle_increment)
        # -180..180 の範囲で中央値を取る（角度の巻き付きを避けるため
        # 期待値の周りに寄せてから計算する）
        shifted = (angles - expected_deg + 180.0) % 360.0 - 180.0
        actual_deg = expected_deg + float(np.median(shifted))
        error = abs(((actual_deg - expected_deg + 180.0) % 360.0) - 180.0)
        status = "OK" if error < 5.0 else "NG"
        print(
            f"[{status}] {label}: 期待 {expected_deg:+.0f} 度 -> "
            f"実際 {actual_deg:+.1f} 度 (誤差 {error:.1f} 度, {near.size} ビーム)"
        )

    # 最短ビームの位置も確認（最も近い壁は前方 4 m のはず）
    finite = np.isfinite(ranges)
    if finite.any():
        idx = int(np.nanargmin(np.where(finite, ranges, np.inf)))
        deg = math.degrees(scan.angle_min + idx * scan.angle_increment)
        print(
            f"\n[INFO] 最短ビーム: index={idx} 距離={ranges[idx]:.2f} m "
            f"角度={deg:+.1f} 度（前方 4 m なので 0 度付近が正しい）"
        )

    print("\n[OK] 調査が完了しました")


if __name__ == "__main__":
    main()
    simulation_app.close()
