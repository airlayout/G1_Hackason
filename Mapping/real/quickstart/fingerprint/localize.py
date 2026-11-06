#!/usr/bin/env python3
"""LiDAR の指紋（UpDown-SC）で初期姿勢を推定し、PNG に描く。

    Navigation/.venv/bin/python localize.py \\
        --query fp_query_1.npz --db db_session --map-pcd map_octomap_r4_s5_floor0.pcd \\
        --nav-map nav_map_run.yaml --updown-bin <updown-sc>/build/updown-sc \\
        --out result_1

## 流れ

1. 問い合わせ（`grab_scan.py` の .npz）を **base_link の向き・センサ原点**に直す
   （取付値 `LIVOX_*` を使う。台帳の姿勢も base_link の向きで書いてあるので、
   当たった向きがそのまま機体の向きになる）
2. `scan_context_rebuild` で台帳と問い合わせの指紋を作り、
   `scan_context_cross_sequence_evaluator` で上位の候補を引く
3. **候補ごとに事前地図へ ICP で詰める。** 指紋は候補を出すだけで、採否は ICP が決める
4. 1 位と 2 位が**別の場所**で、fitness の差が小さければ「自信なし」と言う（黙って選ばない）

## 出力

- `<out>/result.png` … 地図に候補・採用した姿勢・スキャンを重ねた図
- `<out>/result.json` … 採用した base_link の (x, y, yaw) と、候補ごとの ICP の成績
- `<out>/scan.yaml` … `global_localize.py` にそのまま渡せる 2D スキャン（突き合わせ用）
"""
import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from fingerprint.build_db import load_nav_map, up_rotation, write_pcd  # noqa: E402

# run_fastlio_loc_live.sh の既定値（base_link -> livox_frame、2026-09-16 に床基準へ直した値）
LIVOX_XYZ = np.array([-0.112579, -0.057151, 1.209672])
LIVOX_RPY_DEG = np.array([-179.401, -1.9437, 0.0321])
BODY_HIT_M = 0.8
MAX_RANGE_M = 30.0
TOP_K = 10               # ICP で詰める候補の数
SAME_PLACE_M = 1.0       # これより近い候補は同じ場所とみなす
CONFIDENT_MARGIN = 0.05  # 1 位と「別の場所の最良」の fitness の差がこれ未満なら自信なし
SCAN_BAND_M = (1.30, 1.82)  # live/up.sh の cloud_to_scan と同じ高さの帯（床基準）


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def yaw_matrix(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def query_points(npz_path: Path) -> tuple:
    """問い合わせを base_link の向き・センサ原点の座標に直す。IMU との食い違い[deg]も返す。"""
    data = np.load(npz_path)
    raw = data["points"]
    raw = raw[np.all(np.abs(raw) < 100.0, axis=1)]
    ranges = np.linalg.norm(raw, axis=1)
    raw = raw[(ranges > BODY_HIT_M) & (ranges < MAX_RANGE_M)]
    mount = rpy_matrix(*np.radians(LIVOX_RPY_DEG))
    # IMU の上向きを base_link へ回すと +z になるはず（機体が水平なら）
    imu_up_in_base = mount @ (data["accel"] / np.linalg.norm(data["accel"]))
    tilt_deg = math.degrees(math.acos(np.clip(imu_up_in_base[2], -1, 1)))
    # 取付で回したうえで、IMU で残りの傾きを除く（向きの基準は取付が決める）
    level = up_rotation(imu_up_in_base)
    with np.errstate(all="ignore"):   # macOS の Accelerate が出す見かけの警告を黙らせる
        leveled = (raw @ mount.T) @ level.T
    if not np.isfinite(leveled).all():
        raise SystemExit("[fp] 問い合わせの点に非有限値が混じった")
    return leveled, tilt_deg


def write_query_session(points: np.ndarray, session: Path, sensor_z: float) -> None:
    (session / "key_point_frame").mkdir(parents=True, exist_ok=True)
    write_pcd(session / "key_point_frame" / "0.pcd", points)
    (session / "optimized_poses_tum.txt").write_text("0.0 0 0 %.6f 0 0 0 1\n" % sensor_z)
    (session / "scan_context_gravity.csv").write_text("index,stamp,up_x,up_y,up_z\n0,0.0,0,0,1\n")


def run(command: list) -> None:
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        raise SystemExit("[fp] 失敗: %s\n%s%s" % (" ".join(map(str, command)),
                                                   result.stdout[-2000:], result.stderr[-2000:]))


def fingerprint_candidates(args: argparse.Namespace, query_session: Path) -> list:
    tools = args.updown_bin
    params = args.db / "runtime_params.yaml"
    db_scd = args.db / "db.scd"
    if not db_scd.exists():
        run([tools / "scan_context_rebuild", args.db, db_scd, "gravity", params])
        # 指紋ができたら合成スキャンは要らない（残すとディスクを食う）
        shutil.rmtree(args.db / "key_point_frame", ignore_errors=True)
    query_scd = args.out / "query.scd"
    run([tools / "scan_context_rebuild", query_session, query_scd, "gravity", params])
    out_csv = args.out / "candidates.csv"
    run([tools / "scan_context_cross_sequence_evaluator", db_scd, query_scd, out_csv,
         "100", "0.3", "0.7", "2"])
    with out_csv.open() as handle:
        rows = list(csv.DictReader(handle))
    return rows


def icp_refine(query: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud,
               x: float, y: float, z: float, yaw: float) -> dict:
    init = np.eye(4)
    init[:3, :3] = yaw_matrix(yaw)
    init[:3, 3] = (x, y, z)
    estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    transform = init
    for distance in (1.0, 0.4, 0.15):
        result = o3d.pipelines.registration.registration_icp(
            query, target, distance, transform, estimation,
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40))
        transform = result.transformation
    final = o3d.pipelines.registration.evaluate_registration(query, target, 0.15, transform)
    rotation = transform[:3, :3]
    return {
        "transform": transform,
        "fitness": float(final.fitness),
        "rmse": float(final.inlier_rmse),
        "yaw": math.atan2(rotation[1, 0], rotation[0, 0]),
        "tilt_deg": math.degrees(math.acos(np.clip(rotation[2, 2], -1, 1))),
        "dz": float(transform[2, 3] - z),
    }


def to_base(transform: np.ndarray) -> tuple:
    """センサ原点（base_link の向き）の姿勢 -> base_link の (x, y, yaw)。"""
    yaw = math.atan2(transform[1, 0], transform[0, 0])
    offset = yaw_matrix(yaw) @ np.array([LIVOX_XYZ[0], LIVOX_XYZ[1], 0.0])
    return float(transform[0, 3] - offset[0]), float(transform[1, 3] - offset[1]), yaw


def write_scan_yaml(points: np.ndarray, sensor_height: float, path: Path) -> None:
    """床基準の高さの帯を切って 0.5 度刻みの LaserScan 風 YAML にする。"""
    z = points[:, 2] + sensor_height
    band = points[(z > SCAN_BAND_M[0]) & (z < SCAN_BAND_M[1])]
    # base_link 原点から見た距離にする（LiDAR は base_link から少しずれている）
    xy = band[:, :2] + LIVOX_XYZ[:2]
    angles = np.arctan2(xy[:, 1], xy[:, 0])
    ranges = np.linalg.norm(xy, axis=1)
    increment = math.radians(0.5)
    bins = np.floor((angles + math.pi) / increment).astype(int)
    out = [None] * int(round(2 * math.pi / increment))
    for index in np.unique(bins):
        out[index % len(out)] = float(ranges[bins == index].min())
    lines = ["header:", "  frame_id: base_link", "angle_min: %.9f" % -math.pi,
             "angle_increment: %.9f" % increment, "range_min: 0.3", "range_max: 30.0",
             "ranges:"] + ["- %s" % (".inf" if r is None else "%.4f" % r) for r in out]
    path.write_text("\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--query", required=True, type=Path)
    parser.add_argument("--db", required=True, type=Path)
    parser.add_argument("--map-pcd", required=True, type=Path)
    parser.add_argument("--nav-map", required=True, type=Path)
    parser.add_argument("--updown-bin", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--truth", default=None, help="比べる姿勢 'X Y YAW_DEG'（任意）")
    parser.add_argument("--truth-label", default="比較")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    meta = json.loads((args.db / "build_meta.json").read_text())
    points, tilt_deg = query_points(args.query)
    print("[fp] 問い合わせ %d 点 / 取付と IMU の食い違い %.2f 度" % (len(points), tilt_deg))
    query_session = args.out / "query_session"
    write_query_session(points, query_session, meta["sensor_z"])
    write_scan_yaml(points, meta["sensor_height_m"], args.out / "scan.yaml")

    rows = fingerprint_candidates(args, query_session)
    print("[fp] 指紋の候補 %d 件" % len(rows))

    target = o3d.io.read_point_cloud(str(args.map_pcd)).voxel_down_sample(0.1)
    target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30))
    query = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)).voxel_down_sample(0.1)

    results = []
    for row in rows[:TOP_K]:
        cx, cy = float(row["candidate_x"]), float(row["candidate_y"])
        base_yaw = float(row["candidate_yaw"])
        shift = float(row["yaw_shift_rad"])
        # 向きのずれの符号の約束は実装依存なので両方試し、ICP の良い方を取る
        tries = [icp_refine(query, target, cx, cy, meta["sensor_z"], base_yaw + sign * shift)
                 for sign in (+1, -1)]
        best = max(tries, key=lambda item: item["fitness"])
        bx, by, byaw = to_base(best["transform"])
        results.append({
            "rank": int(row["candidate_rank"]), "db_index": int(row["candidate_index"]),
            "db_x": cx, "db_y": cy, "descriptor_distance": float(row["distance"]),
            "base_x": bx, "base_y": by, "base_yaw_deg": math.degrees(byaw),
            "fitness": best["fitness"], "rmse": best["rmse"],
            "tilt_deg": best["tilt_deg"], "dz": best["dz"],
        })
        print("  #%2d 台帳(%6.2f,%6.2f) 距離 %.3f -> base (%6.2f,%6.2f,%7.1f°) fitness %.3f rmse %.3f"
              % (results[-1]["rank"], cx, cy, results[-1]["descriptor_distance"], bx, by,
                 math.degrees(byaw), best["fitness"], best["rmse"]))

    verdict = judge(results)
    summary = {"verdict": verdict, "tilt_vs_imu_deg": tilt_deg, "candidates": results}
    (args.out / "result.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    print("[fp] 判定: %s" % verdict["message"])
    truth = tuple(float(v) for v in args.truth.split()) if args.truth else None
    from fingerprint.draw import draw_result
    db_xy = np.loadtxt(args.db / "optimized_poses_tum.txt", usecols=(1, 2), ndmin=2)[::meta["yaws"]]
    draw_result(args.nav_map, meta, db_xy, points, results, verdict, truth, args.truth_label,
                args.out / "result.png")
    print("[fp] 図 -> %s" % (args.out / "result.png"))
    return 0


def judge(results: list) -> dict:
    ranked = sorted(results, key=lambda item: item["fitness"], reverse=True)
    best = ranked[0]
    rivals = [r for r in ranked[1:]
              if math.hypot(r["base_x"] - best["base_x"], r["base_y"] - best["base_y"]) > SAME_PLACE_M]
    margin = best["fitness"] - rivals[0]["fitness"] if rivals else 1.0
    confident = margin >= CONFIDENT_MARGIN and best["tilt_deg"] < 3.0 and abs(best["dz"]) < 0.2
    message = ("採用: base (%.2f, %.2f, %.1f°) fitness %.3f / 別の場所の次点との差 %.3f"
               % (best["base_x"], best["base_y"], best["base_yaw_deg"], best["fitness"], margin))
    if not confident:
        message = "自信なし（" + message + "）"
    return {"confident": confident, "best": best, "margin": margin,
            "rival": rivals[0] if rivals else None, "message": message}


if __name__ == "__main__":
    raise SystemExit(main())
