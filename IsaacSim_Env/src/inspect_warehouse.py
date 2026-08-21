"""Warehouse シーンの prim 構造を調べる調査用スクリプト。

LiDAR（MultiMeshRayCaster）の raycast 対象を正しく指定するために、
Warehouse USD に含まれる Mesh prim のパス構造を把握する。

実行方法:
    bash inspect_warehouse.sh
"""

from __future__ import annotations

import argparse

from isaaclab.app import AppLauncher

# 既存スクリプトと同じ argparse 方式で起動する（dict 渡しは版によって差があるため）
parser = argparse.ArgumentParser(description="Warehouse の prim 構造調査")
AppLauncher.add_app_launcher_args(parser)
# 調査目的なので GUI は不要
args = parser.parse_args(["--viz", "none"])

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- Isaac Sim 起動後にのみ import 可能 ---
from collections import Counter  # noqa: E402

from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402
import isaacsim.core.utils.stage as stage_utils  # noqa: E402

WAREHOUSE_USD: str = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"


def main() -> None:
    """Warehouse を読み込み、Mesh prim の構造を出力する。"""
    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("[NG] アセットサーバーに接続できません。")

    usd_path = assets_root + WAREHOUSE_USD
    print(f"[INFO] 読み込み: {usd_path}")
    add_reference_to_stage(usd_path=usd_path, prim_path="/World/Warehouse")

    stage: Usd.Stage = stage_utils.get_current_stage()

    mesh_paths: list[str] = []
    type_counter: Counter[str] = Counter()

    for prim in stage.Traverse():
        type_counter[prim.GetTypeName()] += 1
        if prim.IsA(UsdGeom.Mesh):
            mesh_paths.append(str(prim.GetPath()))

    print(f"\n[INFO] prim 総数: {sum(type_counter.values())}")
    print("[INFO] 型別の内訳（上位 15）:")
    for type_name, count in type_counter.most_common(15):
        print(f"    {type_name or '(型なし)'}: {count}")

    print(f"\n[INFO] Mesh prim 数: {len(mesh_paths)}")

    # 階層の深さごとの代表パスを出して、正規表現の当て方を判断する
    print("\n[INFO] Mesh prim パスの例（先頭 30 件）:")
    for path in mesh_paths[:30]:
        print(f"    {path}")

    # 第 3 階層（/World/Warehouse/<ここ>）ごとの Mesh 数を集計する。
    # raycast 対象の正規表現をどの階層に当てるかの判断材料になる。
    group_counter: Counter[str] = Counter()
    for path in mesh_paths:
        parts = path.strip("/").split("/")
        # /World/Warehouse/<group>/... の <group> を取る
        group = parts[2] if len(parts) > 2 else "(直下)"
        group_counter[group] += 1

    print("\n[INFO] /World/Warehouse/<group> ごとの Mesh 数（上位 20）:")
    for group, count in group_counter.most_common(20):
        print(f"    {group}: {count}")

    # 深さの分布（正規表現のワイルドカード段数を決めるため）
    depth_counter: Counter[int] = Counter(
        len(path.strip("/").split("/")) for path in mesh_paths
    )
    print("\n[INFO] Mesh パスの階層の深さ分布:")
    for depth in sorted(depth_counter):
        print(f"    深さ {depth}: {depth_counter[depth]} 個")

    print("\n[OK] 調査が完了しました")


if __name__ == "__main__":
    main()
    simulation_app.close()
