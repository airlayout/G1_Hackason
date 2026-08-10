"""自動巡回ロジックの単体テスト。

Isaac Sim を起動せずに巡回の判断だけを検証する。実測で遭遇した
「壁に鼻先を突きつけて動けなくなる」状況を再現し、そこから脱出できる
ことを確かめる。

実行方法:
    python3 src/test_patrol.py
"""

from __future__ import annotations

import math
import pathlib
import sys

# このファイルの位置から src/ を求める（リポジトリの置き場所に依存しない）
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from g1_twin.lidar import ScanData  # noqa: E402
from g1_twin.patrol import AutoPatrol  # noqa: E402

NUM_BEAMS: int = 360


def make_scan(distance_by_angle: dict[int, float], default: float = 30.0) -> ScanData:
    """指定した方位の距離を持つスキャンを作る。

    Args:
        distance_by_angle: {中心角[deg]: 距離[m]}。その角度 ±15 度に適用する。
        default: 指定の無い方向の距離 [m]

    Returns:
        テスト用の ScanData
    """
    angle_min = math.radians(-180.0)
    increment = math.radians(1.0)
    ranges = [default] * NUM_BEAMS
    for index in range(NUM_BEAMS):
        angle_deg = math.degrees(angle_min + index * increment)
        for center, distance in distance_by_angle.items():
            delta = (angle_deg - center + 180.0) % 360.0 - 180.0
            if abs(delta) <= 15.0:
                ranges[index] = min(ranges[index], distance)
    return ScanData(
        ranges=ranges,
        angle_min=angle_min,
        angle_max=angle_min + increment * (NUM_BEAMS - 1),
        angle_increment=increment,
        range_min=0.3,
        range_max=30.0,
    )


def test_open_space_goes_forward() -> bool:
    """開けた場所では前進する。"""
    patrol = AutoPatrol(seed=0)
    command = patrol.step(make_scan({}))
    ok = command.vx > 0.0 and command.yaw_rate == 0.0
    print(f"[{'OK' if ok else 'NG'}] 開けた場所: vx={command.vx:.2f} "
          f"yaw={command.yaw_rate:.2f}")
    return ok


def test_wall_ahead_turns() -> bool:
    """正面が壁なら旋回する。"""
    patrol = AutoPatrol(seed=0)
    command = patrol.step(make_scan({0: 0.5}))
    ok = command.vx == 0.0 and abs(command.yaw_rate) > 0.0
    print(f"[{'OK' if ok else 'NG'}] 正面が壁: vx={command.vx:.2f} "
          f"yaw={command.yaw_rate:.2f}")
    return ok


def test_escapes_dead_end() -> bool:
    """袋小路（前方 0.5 m / 後方 30 m）から脱出できる。

    実測で遭遇した状況をそのまま再現する。前方 ±60 度が 0.45〜0.54 m、
    左右 90 度が 1.2 m、後方が開いている。以前の実装ではここで
    左右に揺れ続けて永久に脱出できなかった。
    """
    patrol = AutoPatrol(seed=0)
    scan = make_scan(
        {0: 0.45, 30: 0.46, -30: 0.46, 60: 0.54, -60: 0.54, 90: 1.18, -90: 1.25}
    )

    # 旋回しながら、ロボットの向きが変わるのを模擬する。
    # 旋回すると相対的に見える景色も回るので、スキャンを回転させる。
    heading_deg = 0.0
    forward_count = 0
    for step in range(1500):
        # 現在の向きに応じてスキャンを回転させる
        rotated = {
            (angle - int(heading_deg)) % 360 - 360
            if (angle - int(heading_deg)) % 360 > 180
            else (angle - int(heading_deg)) % 360: dist
            for angle, dist in {
                0: 0.45, 30: 0.46, -30: 0.46, 60: 0.54,
                -60: 0.54, 90: 1.18, -90: 1.25,
            }.items()
        }
        command = patrol.step(make_scan(rotated))
        heading_deg += math.degrees(command.yaw_rate * 0.02)
        if command.vx > 0.0:
            forward_count += 1
            if forward_count > 10:
                print(
                    f"[OK] 袋小路から脱出: {step} step で前進を開始 "
                    f"(向き {heading_deg:+.0f} 度)"
                )
                return True

    print(f"[NG] 袋小路から脱出できない（1500 step 経過、向き {heading_deg:+.0f} 度）")
    return False


def test_turn_completes() -> bool:
    """旋回は目標角度まで回りきる（途中で反転しない）。"""
    patrol = AutoPatrol(seed=0)
    # 左 150 度が最も開けている状況
    scan = make_scan({0: 0.5, 150: 25.0})
    signs = set()
    for _ in range(200):
        command = patrol.step(scan)
        if command.yaw_rate != 0.0:
            signs.add(1 if command.yaw_rate > 0 else -1)
    ok = len(signs) == 1
    print(f"[{'OK' if ok else 'NG'}] 旋回方向が一定: 使われた向き={signs}")
    return ok


def test_detects_confinement() -> bool:
    """同じ場所に留まり続けたら閉じ込めとして検知する。

    実測では全スキャンの 77% が半径 5 cm の一点から取られていた。
    その場旋回を繰り返している間は「足踏み」判定が働かないため、
    位置が変わらないこと自体を別途監視する必要がある。
    """
    patrol = AutoPatrol(seed=0)
    scan = make_scan({0: 0.5, 150: 25.0})

    # 同じ位置を通知し続ける（＝その場で回っている状況）
    detected_at = None
    for step in range(1500):
        patrol.notify_position(10.0, 20.0)
        patrol.step(scan)
        if patrol.stats.confined_recoveries > 0:
            detected_at = step
            break

    ok = detected_at is not None
    print(
        f"[{'OK' if ok else 'NG'}] 閉じ込め検知: "
        f"{f'{detected_at} step で検知' if ok else '1500 step 経過しても検知せず'}"
    )
    return ok


def test_moving_does_not_trigger_confinement() -> bool:
    """移動しているときは閉じ込めと誤判定しない。"""
    patrol = AutoPatrol(seed=0)
    scan = make_scan({})

    for step in range(1500):
        # 少しずつ移動する（1 周期 0.0076 m 相当）
        patrol.notify_position(step * 0.0076, 0.0)
        patrol.step(scan)

    ok = patrol.stats.confined_recoveries == 0
    print(
        f"[{'OK' if ok else 'NG'}] 移動中は誤検知しない: "
        f"閉じ込め検知 {patrol.stats.confined_recoveries} 回"
    )
    return ok


def main() -> None:
    """全テストを実行する。"""
    results = [
        test_open_space_goes_forward(),
        test_wall_ahead_turns(),
        test_turn_completes(),
        test_escapes_dead_end(),
        test_detects_confinement(),
        test_moving_does_not_trigger_confinement(),
    ]
    passed = sum(results)
    print(f"\n{passed}/{len(results)} 件が成功しました")
    sys.exit(0 if passed == len(results) else 1)


if __name__ == "__main__":
    main()
