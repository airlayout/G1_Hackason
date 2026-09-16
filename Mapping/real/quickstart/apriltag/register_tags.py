#!/usr/bin/env python3
"""貼ったタグが地図のどこに在るかを決める（段 4）。

## 何を計算しているか

    T_map_tag = T_map_base ・ T_base_camera ・ T_camera_tag

- `T_map_base` … **測位が出したロボットの姿勢**。`/tf` の `map -> base_link`
- `T_base_camera` … `measure_extrinsics.py` が測ったカメラの取付
- `T_camera_tag` … この場で検出したタグの姿勢

## ⚠️ 測位アルゴリズムには依存しない

`map -> base_link` を **`/tf` から取る**ので、MOLA でも AMCL でも
FAST-LIO でも同じ道具が使える。鎖の形は候補ごとに違うが（MOLA は直接、
AMCL は `map->odom->base_link`、FAST-LIO は静的 2 本を挟む）、
**tf2 が勝手に辿る**ので呼ぶ側は気にしなくてよい。
`record_registration.sh` が `log_tf_pose.py --target map --source base_link` で録る。

## 静止して録る

ロボットを**止めて**録るので、画像と姿勢を 1 枚ずつ対応づける必要がない。
姿勢は窓の中央値を使う。**止まっていないと嘘になる**ので、
このスクリプトは姿勢の散らばりを見て、動いていたら止まる。

## 2 地点から録ると 3 つ分かる

同じタグを別の場所から見て、**同じ地図座標に落ちるか**を見る。落ちなければ
測位か取付が狂っている。さらに `--solve-camera-yaw` を付けると、
**床タグでは測れなかったカメラのヨー**を「2 地点の答えが一致する角度」として出せる。

## 使い方

    # 機体側（測位を動かしたまま、止まった状態で）
    bash record_registration.sh /tmp/reg_A 15
    # 別の場所へ移動して
    bash record_registration.sh /tmp/reg_B 15

    # 解く
    python3 register_tags.py --session /tmp/reg_A --session /tmp/reg_B \\
        --extrinsics /tmp/extrinsics.json --tag-mm 160 \\
        --solve-camera-yaw --out /tmp/tag_registry.json
"""

from __future__ import annotations

import argparse
import itertools
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tag_detect
from measure_extrinsics import base_to_optical

# 静止の判定。これを超えて動いていたら「止まっていない」として断る。
STILL_POSITION_M = 0.05
STILL_YAW_DEG = 2.0


def quaternion_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    """(x, y, z, w) から 3x3 の回転行列。"""

    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """3x3 の回転行列から (x, y, z, w)。"""

    trace = rotation[0, 0] + rotation[1, 1] + rotation[2, 2]
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
    quaternion = np.array([x, y, z, w])
    return quaternion / np.linalg.norm(quaternion)


def yaw_of(rotation: np.ndarray) -> float:
    """地図平面での向き [deg]。"""

    return float(np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0])))


def transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def read_tum(path: Path) -> list:
    """`log_tf_pose.py` が書く TUM 形式（時刻 tx ty tz qx qy qz qw）を読む。"""

    poses = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) < 8:
            continue
        values = [float(v) for v in fields[:8]]
        poses.append((values[0], np.array(values[1:4]), np.array(values[4:8])))
    return poses


def average_pose(poses: list) -> tuple:
    """静止している窓の代表姿勢と、その散らばりを返す。"""

    positions = np.array([p for _, p, _ in poses])
    quaternions = np.array([q for _, _, q in poses])
    # 符号を揃えてから平均する（q と -q は同じ回転）
    reference = quaternions[0]
    aligned = np.array([q if q @ reference >= 0 else -q for q in quaternions])
    mean_quaternion = aligned.mean(axis=0)
    mean_quaternion /= np.linalg.norm(mean_quaternion)
    center = np.median(positions, axis=0)
    spread = float(np.linalg.norm(positions - center, axis=1).max())
    yaws = [yaw_of(quaternion_to_matrix(q)) for q in aligned]
    yaw_spread = float(np.ptp(np.unwrap(np.radians(yaws)) * 180.0 / np.pi))
    return center, mean_quaternion, spread, yaw_spread


def camera_transform(extrinsics: dict, yaw_offset_deg: float = None) -> np.ndarray:
    """`base_link -> camera_optical` の 4x4。"""

    yaw = extrinsics.get("yaw_deg", 0.0) if yaw_offset_deg is None else yaw_offset_deg
    rotation = base_to_optical(extrinsics["pitch_down_deg"], extrinsics["roll_deg"])
    spin = np.array([[np.cos(np.radians(yaw)), -np.sin(np.radians(yaw)), 0.0],
                     [np.sin(np.radians(yaw)), np.cos(np.radians(yaw)), 0.0],
                     [0.0, 0.0, 1.0]])
    translation = np.array([extrinsics.get("forward_m", 0.0),
                            extrinsics.get("lateral_m", 0.0),
                            extrinsics["height_m"]])
    return transform(spin @ rotation, translation)


class Session:
    """1 地点ぶんの記録（画像 + そのときの map->base_link）。"""

    def __init__(self, directory: Path, detector, tag_m: float,
                 camera_matrix: np.ndarray, distortion: np.ndarray,
                 pattern: str) -> None:
        self.directory = directory
        self.label = directory.name
        pose_path = directory / "pose.txt"
        if not pose_path.exists():
            raise SystemExit(f"⛔ {pose_path} が無い。record_registration.sh で録る")
        poses = read_tum(pose_path)
        if len(poses) < 5:
            raise SystemExit(f"⛔ {pose_path} の姿勢が {len(poses)} 件しかない。"
                             "測位が動いていたか確認する")
        self.position, self.quaternion, self.spread, self.yaw_spread = average_pose(poses)
        self.pose_count = len(poses)
        self.map_from_base = transform(quaternion_to_matrix(self.quaternion), self.position)

        self.observations: dict = {}
        images = sorted(directory.glob(pattern))
        for path in images:
            image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if image is None:
                continue
            for found in tag_detect.detect(detector, image):
                try:
                    pose = tag_detect.solve_pose(found, tag_m, camera_matrix, distortion)
                except RuntimeError:
                    continue
                if not pose.trustworthy:
                    continue
                self.observations.setdefault(found.tag_id, []).append(pose.matrix())
        self.image_count = len(images)

    def is_still(self) -> bool:
        return self.spread <= STILL_POSITION_M and self.yaw_spread <= STILL_YAW_DEG

    def tags_in_map(self, base_from_camera: np.ndarray) -> dict:
        """このセッションから見た、各タグの地図座標での 4x4。"""

        result = {}
        for tag_id, matrices in self.observations.items():
            stacked = [self.map_from_base @ base_from_camera @ m for m in matrices]
            positions = np.array([m[:3, 3] for m in stacked])
            quaternions = np.array([matrix_to_quaternion(m[:3, :3]) for m in stacked])
            reference = quaternions[0]
            aligned = np.array([q if q @ reference >= 0 else -q for q in quaternions])
            mean_quaternion = aligned.mean(axis=0)
            mean_quaternion /= np.linalg.norm(mean_quaternion)
            center = np.median(positions, axis=0)
            scatter = float(np.linalg.norm(positions - center, axis=1).std())
            result[tag_id] = {
                "position": center,
                "quaternion": mean_quaternion,
                "scatter_m": scatter,
                "observations": len(matrices),
            }
        return result


def cached_tag_positions(sessions: list) -> list:
    """地点ごと・タグごとに、カメラ座標での中心位置をまとめておく。

    取付（ヨー・前後・左右）を探索するとき、毎回 60 枚を解き直すのは無駄なので
    先に潰しておく。探索は行列 2 個の掛け算だけになる。
    """

    cache = []
    for session in sessions:
        table = {}
        for tag_id, matrices in session.observations.items():
            table[tag_id] = np.median(np.array([m[:3, 3] for m in matrices]), axis=0)
        cache.append(table)
    return cache


def solve_camera_mounting(sessions: list, extrinsics: dict, yaw_range: float,
                          offset_range: float) -> tuple:
    """地点間の答えが一致する（ヨー, 前後, 左右）を探す。

    ⚠️ **180 度ふり返った 2 地点があると、前後オフセットの誤差はちょうど 2 倍で出る。**
    だから 180 度の対は、前後オフセットを測るのに最も good な配置である。
    逆に向きがほとんど同じ 2 地点だと、この探索は何も決められない。
    """

    cache = cached_tag_positions(sessions)
    shared = set(cache[0])
    for table in cache[1:]:
        shared &= set(table)
    if not shared:
        return None
    base_rotation = base_to_optical(extrinsics["pitch_down_deg"], extrinsics["roll_deg"])
    map_from_base = [s.map_from_base for s in sessions]

    def cost(yaw_deg: float, forward: float, lateral: float) -> float:
        angle = np.radians(yaw_deg)
        spin = np.array([[np.cos(angle), -np.sin(angle), 0.0],
                         [np.sin(angle), np.cos(angle), 0.0], [0.0, 0.0, 1.0]])
        rotation = spin @ base_rotation
        translation = np.array([forward, lateral, extrinsics["height_m"]])
        worst = []
        for tag_id in shared:
            points = []
            for table, transform_map in zip(cache, map_from_base):
                local = rotation @ table[tag_id] + translation
                points.append(transform_map[:3, :3] @ local + transform_map[:3, 3])
            points = np.array(points)
            worst.append(np.linalg.norm(points - points.mean(axis=0), axis=1).max())
        return float(np.mean(worst))

    samples = []
    best = None
    for yaw in np.arange(-yaw_range, yaw_range + 0.5, 1.0):
        for forward in np.arange(-offset_range, offset_range + 0.01, 0.02):
            for lateral in np.arange(-offset_range, offset_range + 0.01, 0.02):
                value = cost(float(yaw), float(forward), float(lateral))
                samples.append((float(yaw), float(forward), float(lateral), value))
                if best is None or value < best[3]:
                    best = (float(yaw), float(forward), float(lateral), value)
    # 粗い格子の周りを細かく詰める
    yaw0, forward0, lateral0, _ = best
    for yaw in np.arange(yaw0 - 1.0, yaw0 + 1.01, 0.1):
        for forward in np.arange(forward0 - 0.02, forward0 + 0.021, 0.004):
            for lateral in np.arange(lateral0 - 0.02, lateral0 + 0.021, 0.004):
                value = cost(float(yaw), float(forward), float(lateral))
                if value < best[3]:
                    best = (float(yaw), float(forward), float(lateral), value)

    # ⚠️ **解が谷になっていないか見る。** 2 地点しかないと、取付の誤差と測位の誤差が
    # 分離できず、まるで違う取付が同じ食い違いを出す（2026-09-16 実測: 探索範囲を
    # 変えるとヨーが 7.6 → 20.5 deg に動くのに食い違いは 36 mm のまま動かなかった）。
    # 「最良から 10 % 以内」の解がどれだけ広がっているかを返す。
    limit = best[3] * 1.10 + 0.002
    near = np.array([[y, f, l] for y, f, l, v in samples if v <= limit])
    spread = (float(np.ptp(near[:, 0])), float(np.ptp(near[:, 1])), float(np.ptp(near[:, 2]))) \
        if len(near) > 1 else (0.0, 0.0, 0.0)
    return best + (spread, len(near))


def disagreement(sessions: list, base_from_camera: np.ndarray) -> tuple:
    """複数地点の答えがどれだけ食い違うか [m]。共通のタグだけを見る。"""

    per_session = [s.tags_in_map(base_from_camera) for s in sessions]
    shared = set(per_session[0])
    for table in per_session[1:]:
        shared &= set(table)
    if not shared:
        return float("nan"), {}
    details = {}
    for tag_id in sorted(shared):
        points = np.array([table[tag_id]["position"] for table in per_session])
        center = points.mean(axis=0)
        details[tag_id] = float(np.linalg.norm(points - center, axis=1).max())
    return float(np.mean(list(details.values()))), details


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--session", action="append", required=True,
                        help="record_registration.sh が作った置き場。複数回渡せる")
    parser.add_argument("--extrinsics", required=True,
                        help="measure_extrinsics.py --out が書いた JSON")
    parser.add_argument("--intrinsics", default=None)
    parser.add_argument("--hfov-deg", type=float, default=87.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--tag-mm", type=float, default=160.0)
    parser.add_argument("--family", default="36h11", choices=sorted(tag_detect.FAMILIES))
    parser.add_argument("--pattern", default="raw_*.png")
    parser.add_argument("--solve-camera-yaw", action="store_true",
                        help="2 地点以上の答えが一致するカメラのヨーを探す")
    parser.add_argument("--yaw-range", type=float, default=25.0, help="探す幅 [deg]")
    parser.add_argument("--solve-camera-mounting", action="store_true",
                        help="ヨーに加えて前後・左右のオフセットも探す。"
                             "⚠️ 180 度ふり返った 2 地点があると前後が特によく決まる")
    parser.add_argument("--offset-range", type=float, default=0.30,
                        help="前後・左右を探す幅 [m]")
    parser.add_argument("--known-gap", nargs=3, metavar=("ID_A", "ID_B", "METERS"),
                        help="巻尺で測ったタグ間距離。地図の上でも同じになるかを見る")
    parser.add_argument("--out", default=None)
    parser.add_argument("--force", action="store_true", help="静止していなくても続ける")
    arguments = parser.parse_args()

    extrinsics = json.loads(Path(arguments.extrinsics).read_text())
    if arguments.intrinsics:
        camera_matrix, distortion, _ = tag_detect.load_intrinsics(arguments.intrinsics)
    else:
        camera_matrix = tag_detect.default_camera_matrix(
            arguments.width, arguments.height, arguments.hfov_deg)
        distortion = np.zeros((1, 5))
        print(f"⚠️ 内部パラメータは未校正（画角 {arguments.hfov_deg:.0f}deg の仮置き）")

    detector = tag_detect.make_detector(arguments.family)
    sessions = [Session(Path(d), detector, arguments.tag_mm / 1000.0,
                        camera_matrix, distortion, arguments.pattern)
                for d in arguments.session]

    print("\n=== 記録 ===")
    stopped = True
    for session in sessions:
        mark = "✅" if session.is_still() else "⛔"
        print(f"{mark} {session.label}: 画像 {session.image_count} 枚 / 姿勢 {session.pose_count} 件 / "
              f"タグ {sorted(session.observations)}")
        print(f"   ロボット (x {session.position[0]:+.3f}, y {session.position[1]:+.3f}, "
              f"z {session.position[2]:+.3f}) 向き {yaw_of(quaternion_to_matrix(session.quaternion)):+.1f} deg")
        print(f"   静止の確認: 位置の振れ {session.spread * 1000:.0f} mm "
              f"/ 向きの振れ {session.yaw_spread:.2f} deg")
        if not session.is_still():
            stopped = False
            print(f"   ⛔ 止まっていない（位置 {STILL_POSITION_M * 1000:.0f} mm / "
                  f"向き {STILL_YAW_DEG:.0f} deg を超えた）。測位が暴れているか、機体が動いている")
    if not stopped and not arguments.force:
        print("\n⛔ 静止していない記録がある。録り直すか --force で続ける")
        return 1

    yaw_used = extrinsics.get("yaw_deg", 0.0)
    if arguments.solve_camera_mounting:
        if len(sessions) < 2:
            print("\n⚠️ --solve-camera-mounting は 2 地点以上ないと解けない。飛ばす")
        else:
            before, _ = disagreement(sessions, camera_transform(extrinsics, 0.0))
            found = solve_camera_mounting(sessions, extrinsics, arguments.yaw_range,
                                          arguments.offset_range)
            if found is not None:
                yaw, forward, lateral, value, solution_spread, near_count = found
                headings = [yaw_of(quaternion_to_matrix(s.quaternion)) for s in sessions]
                heading_spread = max(headings) - min(headings)
                print("\n=== カメラの取付を解く（ヨー ＋ 前後 ＋ 左右）===")
                print(f"地点の向きの差: {heading_spread:.1f} deg "
                      f"{'（180 度に近い。前後がよく決まる）' if abs(abs(heading_spread) - 180) < 40 else '（差が小さい。前後は決まりにくい）'}")
                print(f"すべて 0 のとき 食い違い {before * 1000:.0f} mm")
                print(f"→ ヨー {yaw:+.2f} deg / 前方 {forward:+.3f} m / 左右 {lateral:+.3f} m "
                      f"で **{value * 1000:.0f} mm**")
                print(f"最良から 10 % 以内の解が {near_count} 個: "
                      f"ヨー幅 {solution_spread[0]:.1f} deg / "
                      f"前方幅 {solution_spread[1] * 100:.1f} cm / "
                      f"左右幅 {solution_spread[2] * 100:.1f} cm")
                gain = before - value
                loose = (solution_spread[0] > 4.0 or solution_spread[1] > 0.06
                         or solution_spread[2] > 0.06)
                if gain < 0.5 * before:
                    # 取付を自由にしても食い違いが半分も減らない。
                    # ＝ 残っているのは取付の誤差ではない。
                    print(f"⛔ **取付を動かしても {gain * 1000:.0f} mm しか減らない"
                          f"（{before * 1000:.0f} → {value * 1000:.0f} mm）。**")
                    print("   残りは取付ではなく**測位**の誤差である。")
                    print("   ⇒ 取付は 0 のまま（物理的に妥当な値）にして、")
                    print("     この食い違いを登録の不確かさとして受け入れる方がよい。")
                    print("     数字を合わせに行くと、測位の誤差を取付に焼き込むことになる。")
                elif loose:
                    unique_yaws = len({round(yaw_of(quaternion_to_matrix(x.quaternion)) / 30.0)
                                       for x in sessions})
                    print("⛔ **解が一意に決まっていない。** 同じ食い違いを出す取付が広く並んでいる。")
                    print(f"   いまの地点の向きは {unique_yaws} 通り。"
                          "**これまでと 90 度ずれた向きの地点を足す**と分かれる。")
                    print("   いまの値は採用しないこと（タグの座標が 10 cm 規模で動く）。")
                else:
                    print("✅ 解は一意に決まっている")
                extrinsics = dict(extrinsics)
                extrinsics["forward_m"] = forward
                extrinsics["lateral_m"] = lateral
                yaw_used = yaw
    elif arguments.solve_camera_yaw:
        if len(sessions) < 2:
            print("\n⚠️ --solve-camera-yaw は 2 地点以上ないと解けない。飛ばす")
        else:
            best = None
            for yaw in np.arange(-arguments.yaw_range, arguments.yaw_range + 0.25, 0.25):
                score, _ = disagreement(sessions, camera_transform(extrinsics, float(yaw)))
                if not np.isnan(score) and (best is None or score < best[1]):
                    best = (float(yaw), score)
            if best is not None:
                baseline, _ = disagreement(sessions, camera_transform(extrinsics, 0.0))
                print(f"\n=== カメラのヨーを解く ===")
                print(f"ヨー 0 deg のとき 地点間の食い違い {baseline * 1000:.0f} mm")
                print(f"ヨー {best[0]:+.2f} deg にすると **{best[1] * 1000:.0f} mm** まで下がる")
                if abs(best[0]) >= arguments.yaw_range - 0.3:
                    print("⚠️ 探索範囲の端に張り付いた。--yaw-range を広げるか、"
                          "食い違いの原因はヨー以外（測位・前後オフセット）")
                yaw_used = best[0]

    base_from_camera = camera_transform(extrinsics, yaw_used)
    print(f"\n=== タグの地図座標（カメラのヨー {yaw_used:+.2f} deg を使用）===")
    merged: dict = {}
    for session in sessions:
        for tag_id, entry in session.tags_in_map(base_from_camera).items():
            merged.setdefault(tag_id, []).append((session.label, entry))

    print("ID    x        y        z       向き      観測   ばらつき   地点")
    registry = {}
    for tag_id in sorted(merged):
        entries = [e for _, e in merged[tag_id]]
        positions = np.array([e["position"] for e in entries])
        weights = np.array([e["observations"] for e in entries], dtype=float)
        position = (positions * weights[:, None]).sum(axis=0) / weights.sum()
        quaternions = np.array([e["quaternion"] for e in entries])
        reference = quaternions[0]
        aligned = np.array([q if q @ reference >= 0 else -q for q in quaternions])
        quaternion = aligned.mean(axis=0)
        quaternion /= np.linalg.norm(quaternion)
        scatter = float(np.mean([e["scatter_m"] for e in entries]))
        total = int(weights.sum())
        print("%-4d %+7.3f  %+7.3f  %+7.3f  %+7.1f   %4d   %5.0f mm  %s" % (
            tag_id, position[0], position[1], position[2],
            yaw_of(quaternion_to_matrix(quaternion)), total, scatter * 1000,
            ",".join(label for label, _ in merged[tag_id])))
        registry[str(tag_id)] = {
            "position": position.tolist(),
            "quaternion": quaternion.tolist(),
            "observations": total,
            "scatter_m": scatter,
            "sessions": [label for label, _ in merged[tag_id]],
        }

    if len(sessions) >= 2:
        score, details = disagreement(sessions, base_from_camera)
        if details:
            print(f"\n=== 地点間の食い違い（測位と取付が正しいかの本番の検査）===")
            for tag_id, value in details.items():
                mark = "✅" if value < 0.10 else ("⚠️" if value < 0.30 else "⛔")
                print(f"  {mark} ID{tag_id}: {value * 1000:.0f} mm")
            print(f"  平均 {score * 1000:.0f} mm")
            print("  読み方: 100 mm 未満なら使える。300 mm を超えるなら、"
                  "測位が地点によって外れているか、取付のヨー・前後オフセットが効いている")

    if len(registry) >= 2:
        print("\n=== 地図の上でのタグ間距離 ===")
        for left, right in itertools.combinations(sorted(registry, key=int), 2):
            distance = float(np.linalg.norm(
                np.array(registry[left]["position"]) - np.array(registry[right]["position"])))
            line = f"  ID{left} — ID{right}: {distance:.3f} m"
            if arguments.known_gap and {left, right} == {arguments.known_gap[0],
                                                          arguments.known_gap[1]}:
                truth = float(arguments.known_gap[2])
                error = 100.0 * (distance / truth - 1.0)
                line += f"   巻尺 {truth:.3f} m に対し {error:+.1f} %"
            print(line)
        print("  ⚠️ この距離は剛体変換で保たれるので、**測位の正しさは測れない**。"
              "検出と寸法の検算にしかならない")

    if arguments.out:
        payload = {
            "frame_id": "map",
            "tag_family": arguments.family,
            "tag_size_m": arguments.tag_mm / 1000.0,
            "camera_yaw_deg": yaw_used,
            "extrinsics": extrinsics,
            "sessions": [{"label": s.label, "images": s.image_count,
                          "poses": s.pose_count,
                          "robot_position": s.position.tolist(),
                          "robot_yaw_deg": yaw_of(quaternion_to_matrix(s.quaternion)),
                          "robot_quaternion": s.quaternion.tolist(),
                          # 地点ごとの推定も残す。図にすると食い違いの向きが見える
                          "tags": {str(tag_id): entry["position"].tolist()
                                   for tag_id, entry
                                   in s.tags_in_map(base_from_camera).items()}}
                         for s in sessions],
            "tags": registry,
        }
        Path(arguments.out).write_text(json.dumps(payload, indent=2, ensure_ascii=False))
        print(f"\n書いた: {arguments.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
