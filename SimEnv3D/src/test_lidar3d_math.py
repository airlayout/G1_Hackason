"""lidar3d.py の座標変換とクォータニオンの単体テスト（Isaac Sim 不要）。

クォータニオンの要素順は過去に取り違えて実害を出している箇所なので
（(w,x,y,z) と (x,y,z,w) の混同で /odom が壊れた）、ここだけは
Isaac Sim を起動せずに検証できるようにしておく。

実行方法:
    cd <このリポジトリ>/SimEnv3D
    source env.sh
    python3 src/test_lidar3d_math.py
"""

from __future__ import annotations

import math
import sys

import torch

# lidar.py の import（isaaclab 依存）を避けるため、必要な関数だけ直接読み込む。
# lidar3d.py はモジュール先頭で .lidar を import するが、その中身は
# 関数内 import なので、パッケージとして読めば副作用は無い。
sys.path.insert(0, "src")
from g1_twin.lidar3d import (  # noqa: E402
    VERTICAL_FOV_DEG,
    _rotate_by_quat,
    _rotate_by_quat_inverse,
    tilt_quat_wxyz,
)

PASS = "[OK]"
FAIL = "[NG]"
failures = 0


def check(label: str, actual: torch.Tensor, expected: list[float], tol: float = 1e-5) -> None:
    """ベクトルが期待値と一致するか確認する。"""
    global failures
    exp = torch.tensor(expected, dtype=torch.float32)
    diff = float(torch.linalg.norm(actual.flatten() - exp))
    if diff < tol:
        print(f"  {PASS} {label}: {[round(v, 4) for v in actual.flatten().tolist()]}")
    else:
        failures += 1
        print(
            f"  {FAIL} {label}: 実際 {[round(v, 4) for v in actual.flatten().tolist()]} "
            f"!= 期待 {expected} (差 {diff:.6f})"
        )


def test_tilt_quat_identity() -> None:
    """前傾 0 度は単位クォータニオンになる。"""
    print("[Test] 前傾 0 度 = 単位クォータニオン")
    q = tilt_quat_wxyz(0.0)
    check("quat(w,x,y,z)", torch.tensor(q), [1.0, 0.0, 0.0, 0.0])


def test_tilt_quat_order() -> None:
    """前傾は y 軸まわりの回転で、(w,x,y,z) 順に入っている。"""
    print("[Test] 前傾 90 度の要素順")
    q = torch.tensor(tilt_quat_wxyz(90.0))
    h = math.cos(math.radians(45.0))
    # w=cos45, y=sin45、x と z は 0
    check("quat(w,x,y,z)", q, [h, 0.0, h, 0.0])


def test_tilt_points_down() -> None:
    """前傾させると前方ベクトルが下を向く（正の tilt で -z 方向へ）。"""
    print("[Test] 前傾 20 度で前方ベクトルが下を向く")
    q = torch.tensor(tilt_quat_wxyz(20.0))
    forward = torch.tensor([[1.0, 0.0, 0.0]])
    rotated = _rotate_by_quat(forward, q)
    # 20 度下向き: x=cos20, z=-sin20
    check(
        "回転後の前方",
        rotated,
        [math.cos(math.radians(20.0)), 0.0, -math.sin(math.radians(20.0))],
    )
    z = float(rotated[0, 2])
    if z < 0:
        print(f"  {PASS} z 成分が負（下向き）: {z:.4f}")
    else:
        globals()["failures"] += 1
        print(f"  {FAIL} z 成分が負でない: {z:.4f}")


def test_rotate_inverse_roundtrip() -> None:
    """回転 -> 逆回転で元に戻る。"""
    print("[Test] 回転と逆回転の往復")
    q = torch.tensor(tilt_quat_wxyz(35.0))
    points = torch.tensor(
        [[1.0, 2.0, 3.0], [-4.0, 0.5, 2.0], [0.0, 0.0, 1.0]], dtype=torch.float32
    )
    roundtrip = _rotate_by_quat_inverse(_rotate_by_quat(points, q), q)
    diff = float(torch.linalg.norm(roundtrip - points))
    if diff < 1e-5:
        print(f"  {PASS} 往復で一致（差 {diff:.8f}）")
    else:
        globals()["failures"] += 1
        print(f"  {FAIL} 往復で一致しない（差 {diff:.6f}）")


def test_rotation_preserves_length() -> None:
    """回転は長さを変えない。"""
    print("[Test] 回転が長さを保つ")
    q = torch.tensor(tilt_quat_wxyz(20.0))
    points = torch.tensor([[3.0, 4.0, 12.0]], dtype=torch.float32)
    before = float(torch.linalg.norm(points))
    after = float(torch.linalg.norm(_rotate_by_quat(points, q)))
    if abs(before - after) < 1e-5:
        print(f"  {PASS} 長さ保存: {before:.4f} -> {after:.4f}")
    else:
        globals()["failures"] += 1
        print(f"  {FAIL} 長さが変わった: {before:.4f} -> {after:.4f}")


def test_empty_input() -> None:
    """空の点群でも落ちない（当たりが 0 本のとき）。"""
    print("[Test] 空の点群")
    q = torch.tensor(tilt_quat_wxyz(20.0))
    empty = torch.zeros((0, 3), dtype=torch.float32)
    out = _rotate_by_quat_inverse(empty, q)
    if out.shape == (0, 3):
        print(f"  {PASS} 形状を保って返る: {tuple(out.shape)}")
    else:
        globals()["failures"] += 1
        print(f"  {FAIL} 形状が壊れた: {tuple(out.shape)}")


def test_floor_visibility() -> None:
    """前傾 20 度で床が見える距離を確認する（仕様の根拠）。"""
    print("[Test] 前傾による床の可視距離")
    height = 1.1
    for tilt in (0.0, 20.0):
        down_deg = VERTICAL_FOV_DEG[0] - tilt
        if down_deg < 0:
            dist = height / math.tan(math.radians(-down_deg))
            print(f"  [INFO] 前傾 {tilt:>4.0f} 度: 下向き {down_deg:+.0f} 度 -> {dist:.1f} m 先の床")
        else:
            print(f"  [INFO] 前傾 {tilt:>4.0f} 度: 床は見えない")

    # 前傾なしは 8.9 m 先、20 度で 2.2 m 先になるはず
    d0 = height / math.tan(math.radians(-(VERTICAL_FOV_DEG[0] - 0.0)))
    d20 = height / math.tan(math.radians(-(VERTICAL_FOV_DEG[0] - 20.0)))
    if d0 > 8.0 and d20 < 2.5:
        print(f"  {PASS} 前傾で床が近づく: {d0:.1f} m -> {d20:.1f} m")
    else:
        globals()["failures"] += 1
        print(f"  {FAIL} 想定と違う: {d0:.1f} m -> {d20:.1f} m")


def main() -> None:
    print("=" * 60)
    print("lidar3d.py 座標変換の単体テスト")
    print("=" * 60)
    test_tilt_quat_identity()
    test_tilt_quat_order()
    test_tilt_points_down()
    test_rotate_inverse_roundtrip()
    test_rotation_preserves_length()
    test_empty_input()
    test_floor_visibility()
    print("=" * 60)
    if failures:
        print(f"{FAIL} {failures} 件が失敗しました")
        sys.exit(1)
    print(f"{PASS} すべて成功しました")


main()
