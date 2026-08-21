"""LaserScan の値の意味づけを検証する回帰テスト。

地図が壊れた原因のうち最も見つけにくかったのが、近すぎる測距を inf に
していたこと。LaserScan の inf は「最大距離まで何も無い」を意味するため、
近くの壁が「30 m の空白」に化けて地図が塗り潰されていた。

Isaac Sim を起動せずに ScanData の規約だけを検証する。

実行方法:
    python3 src/test_scan_semantics.py
"""

from __future__ import annotations

import math
import pathlib
import sys

# このファイルの位置から src/ を求める（リポジトリの置き場所に依存しない）
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import torch  # noqa: E402

from g1_twin import lidar as lidar_module  # noqa: E402


class FakeSensorData:
    """MultiMeshRayCaster.data を模した最小限のオブジェクト。"""

    def __init__(self, hits: torch.Tensor, pos: torch.Tensor) -> None:
        self.ray_hits_w = hits.unsqueeze(0)
        self.pos_w = pos.unsqueeze(0)


class FakeSensor:
    """MultiMeshRayCaster を模した最小限のオブジェクト。"""

    def __init__(self, hits: torch.Tensor, pos: torch.Tensor) -> None:
        self.data = FakeSensorData(hits, pos)


def make_lidar(distances: list[float]) -> lidar_module.G1Lidar:
    """指定した距離を返す偽の LiDAR を作る。

    G1Lidar.__init__ は Isaac Sim を必要とするため、インスタンスだけ作って
    センサと角度情報を差し替える。

    Args:
        distances: 各ビームの距離 [m]。inf を渡すと「当たらなかった」を表す。
    """
    origin = torch.zeros(3)
    hits = torch.zeros((len(distances), 3))
    for i, d in enumerate(distances):
        if math.isinf(d):
            hits[i] = torch.tensor([float("inf")] * 3)
        else:
            # +X 方向に距離 d の点を置く
            hits[i] = torch.tensor([d, 0.0, 0.0])

    obj = object.__new__(lidar_module.G1Lidar)
    obj._sensor = FakeSensor(hits, origin)
    obj._num_beams = len(distances)
    obj._angle_increment = math.radians(1.0)
    obj._angle_min = math.radians(-180.0)
    obj._angle_max = obj._angle_min + obj._angle_increment * (len(distances) - 1)
    return obj


def test_near_range_is_zero_not_inf() -> bool:
    """近すぎる測距は 0.0 になる（inf にしてはいけない）。

    inf にすると SLAM が「最大距離まで空き」と解釈し、地図が壊れる。
    """
    # MIN_RANGE (0.3) 未満の値を混ぜる
    lidar = make_lidar([0.1, 0.25, 2.0, 5.0])
    scan = lidar.read_scan()

    near = [scan.ranges[0], scan.ranges[1]]
    ok = all(r == 0.0 for r in near)
    print(
        f"[{'OK' if ok else 'NG'}] 近すぎる測距 (0.1, 0.25 m) -> "
        f"{near}（0.0 が正しい。inf は地図を壊す）"
    )
    return ok


def test_normal_range_preserved() -> bool:
    """通常の距離はそのまま保たれる。"""
    lidar = make_lidar([2.0, 5.0, 29.0])
    scan = lidar.read_scan()
    expected = [2.0, 5.0, 29.0]
    ok = all(abs(a - b) < 0.01 for a, b in zip(scan.ranges, expected))
    print(f"[{'OK' if ok else 'NG'}] 通常の距離: {[round(r, 2) for r in scan.ranges]}")
    return ok


def test_no_hit_is_inf() -> bool:
    """何にも当たらないビームは inf のままにする。

    これは「最大距離まで何も無い」という正しい意味なので inf でよい。
    """
    lidar = make_lidar([float("inf"), 3.0])
    scan = lidar.read_scan()
    ok = math.isinf(scan.ranges[0]) and abs(scan.ranges[1] - 3.0) < 0.01
    print(
        f"[{'OK' if ok else 'NG'}] 当たらないビーム -> {scan.ranges[0]}"
        f"（inf が正しい）"
    )
    return ok


def test_angles_cover_full_circle() -> bool:
    """角度が全周をちょうど 1 周する。"""
    lidar = make_lidar([1.0] * 360)
    scan = lidar.read_scan()
    span = math.degrees(scan.angle_max - scan.angle_min)
    increment = math.degrees(scan.angle_increment)
    ok = abs(span - 359.0) < 0.01 and abs(increment - 1.0) < 0.01
    print(
        f"[{'OK' if ok else 'NG'}] 角度: {math.degrees(scan.angle_min):+.0f} 〜 "
        f"{math.degrees(scan.angle_max):+.0f} 度、刻み {increment:.2f} 度"
    )
    return ok


def main() -> None:
    """全テストを実行する。"""
    results = [
        test_near_range_is_zero_not_inf(),
        test_normal_range_preserved(),
        test_no_hit_is_inf(),
        test_angles_cover_full_circle(),
    ]
    passed = sum(results)
    print(f"\n{passed}/{len(results)} 件が成功しました")
    sys.exit(0 if passed == len(results) else 1)


if __name__ == "__main__":
    main()
