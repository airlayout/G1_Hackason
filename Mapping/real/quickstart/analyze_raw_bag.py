#!/usr/bin/env python3
"""生 LiDAR + IMU の db3 を、歩かせずに取れる範囲で解析する。

目的は3つ。

1. **IMU が FAST-LIO2 に食わせられる素性かを確かめる**
   レート・欠測・単位（g か m/s²）・静止バイアス・ノイズ σ を出す。
   FAST-LIO2 の `gyr_cov` / `acc_cov` / `b_gyr_cov` / `b_acc_cov` は
   ここで測った値からしか決められない。

2. **センサ座標系での追従者と自己遮蔽の位置を同定する**
   `2026-09-04-g1-mapping-drift-and-follower.md` 第8.2節のマスク値。
   追従者はセンサ座標系でほぼ定位置なので、姿勢推定なしに落とせる。
   `blind` を何mにすれば追従者が落ちるかもここで決まる。

3. **LiDAR と IMU の時刻が揃っているかを確かめる**
   ずれていると FAST-LIO2 の de-skew が効かない。

歩行を一切必要としない。静止記録があれば全部出る。

    ./quickstart/analyze_raw_bag.py runs/<session_id>
"""
from __future__ import annotations

import argparse
import math
import sqlite3
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import numpy as np  # noqa: E402

from g1_mapping.rebuild import _CdrReader, parse_pointcloud2  # noqa: E402

IMU_TOPIC = "/utlidar/imu_livox_mid360"
RAW_POINTS_TOPIC = "/utlidar/cloud_livox_mid360"

GRAVITY = 9.80665
# |a| がこの範囲なら g 単位、GRAVITY 近傍なら m/s²。中間なら判定不能として報せる。
G_UNIT_BAND = (0.90, 1.10)
MS2_UNIT_BAND = (9.0, 10.5)


def read_bag(session_dir: Path) -> tuple[Path, dict[str, int]]:
    """db3 を1つ選び、トピック名 → topic_id を返す。"""
    candidates = sorted(session_dir.glob("raw/rosbag2/*.db3"))
    if not candidates:
        raise SystemExit(f"db3 が見つかりません: {session_dir}/raw/rosbag2/")
    bag = candidates[0]
    with sqlite3.connect(f"file:{bag}?mode=ro", uri=True) as connection:
        rows = connection.execute("SELECT id, name, type FROM topics").fetchall()
    print(f"bag     : {bag.name}  ({bag.stat().st_size / 1e6:.1f} MB)")
    print("topics  :")
    ids = {}
    with sqlite3.connect(f"file:{bag}?mode=ro", uri=True) as connection:
        for topic_id, name, ros_type in rows:
            count = connection.execute(
                "SELECT COUNT(*) FROM messages WHERE topic_id=?", (topic_id,)
            ).fetchone()[0]
            print(f"          {name:<34} {ros_type:<28} {count:>7} 件")
            ids[name] = topic_id
    return bag, ids


def parse_imu(payload: bytes) -> tuple[float, tuple[float, float, float],
                                       tuple[float, float, float], str]:
    """sensor_msgs/Imu の CDR から (stamp秒, gyro, accel, frame_id) を取り出す。

    フィールド順は ROS 2 の定義通り。covariance(9要素)を読み飛ばさないと
    次のフィールドの位置がずれるので、**要素数を必ず数えて進める**。
    """
    reader = _CdrReader(payload)
    sec = reader.int32()
    nanosec = reader.uint32()
    frame_id = reader.string()
    # ここから先はすべて float64。CDR の 8 バイト境界に揃える。
    body = payload[4:]
    position = reader.position
    if position % 8:
        position += 8 - position % 8
    values = struct.unpack_from("<37d", body, position)
    #  0-3 orientation(x,y,z,w) / 4-12 orientation_cov
    # 13-15 angular_velocity   / 16-24 angular_velocity_cov
    # 25-27 linear_acceleration/ 28-36 linear_acceleration_cov
    stamp = sec + nanosec * 1e-9
    return stamp, values[13:16], values[25:28], frame_id


def _quietest_window(stamps: np.ndarray, gyro: np.ndarray, rate: float,
                     window_s: float = 5.0) -> tuple[np.ndarray, tuple[float, float]]:
    """角速度の変動が最も小さい連続窓を返す。

    静止しているつもりの記録でも、機体を触られた区間が混じる。角速度は
    並進より姿勢変化に鋭敏なので、これを静止判定の指標にする。
    """
    elapsed = stamps - stamps[0]
    total = elapsed[-1]
    if total <= window_s:
        return np.ones(len(stamps), dtype=bool), (0.0, float(total))
    best_mask, best_span, best_score = None, None, None
    for start in np.arange(0.0, total - window_s, window_s / 5):
        mask = (elapsed >= start) & (elapsed < start + window_s)
        if mask.sum() < rate:  # 1秒分に満たない窓は評価しない
            continue
        score = float(np.linalg.norm(gyro[mask].std(axis=0)))
        if best_score is None or score < best_score:
            best_mask, best_span, best_score = mask, (start, start + window_s), score
    if best_mask is None:
        return np.ones(len(stamps), dtype=bool), (0.0, float(total))
    return best_mask, best_span


def analyse_imu(bag: Path, topic_id: int) -> dict:
    with sqlite3.connect(f"file:{bag}?mode=ro", uri=True) as connection:
        rows = connection.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topic_id,),
        ).fetchall()
    if not rows:
        raise SystemExit("IMU のメッセージが 0 件です")

    recv_ns = np.array([r[0] for r in rows], dtype=np.int64)
    stamps, gyros, accels = [], [], []
    frame_id = ""
    for _, blob in rows:
        stamp, gyro, accel, frame_id = parse_imu(bytes(blob))
        stamps.append(stamp)
        gyros.append(gyro)
        accels.append(accel)
    stamps = np.array(stamps)
    gyro = np.array(gyros)
    accel = np.array(accels)

    print()
    print("=" * 72)
    print("IMU  {}  ({} 件, frame_id={!r})".format(IMU_TOPIC, len(stamps), frame_id))
    print("=" * 72)

    # --- レートと欠測 ---------------------------------------------------
    dt = np.diff(stamps)
    span = stamps[-1] - stamps[0]
    rate = (len(stamps) - 1) / span if span > 0 else 0.0
    nominal = 1.0 / rate if rate else float("nan")
    gaps = dt[dt > nominal * 2.5]
    print(f"レート  : {rate:.2f} Hz  （記録 {span:.1f} 秒）")
    print(f"間隔    : 中央 {np.median(dt) * 1e3:.3f} ms / p99 {np.percentile(dt, 99) * 1e3:.3f} ms"
          f" / 最大 {dt.max() * 1e3:.3f} ms")
    print(f"欠測    : {len(gaps)} 箇所（間隔が公称の2.5倍を超えたもの）"
          + (f"  最大 {gaps.max() * 1e3:.1f} ms" if len(gaps) else ""))

    # --- 単位 -----------------------------------------------------------
    norms = np.linalg.norm(accel, axis=1)
    mean_norm = float(norms.mean())
    if G_UNIT_BAND[0] <= mean_norm <= G_UNIT_BAND[1]:
        unit, scale = "g（重力単位）", GRAVITY
    elif MS2_UNIT_BAND[0] <= mean_norm <= MS2_UNIT_BAND[1]:
        unit, scale = "m/s²", 1.0
    else:
        unit, scale = f"不明（|a|={mean_norm:.3f}）", float("nan")
    print(f"加速度  : |a| 平均 {mean_norm:.4f} → 単位は **{unit}**")
    if scale == GRAVITY:
        print("          FAST-LIO2 は初期静止区間の平均で正規化するので g 単位でも動く")
        print("          （`IMU_Processing` が mean_acc から G_m_s2 へスケールする）。")
        print("          ただし**開始時に静止していること**が前提になる。")

    # --- 静止区間の自動選択 ---------------------------------------------
    # 記録の全区間が静止しているとは限らない。2026-09-04 の 20 秒スモークテストでは
    # 途中 5 秒間だけ機体が回されており、全区間で σ を取ると yaw のノイズを
    # **150 倍**に見積もってしまった（0.0012 → 0.183 rad/s）。
    # 誰かが触った区間を混ぜると FAST-LIO2 の共分散がまるごと狂うので、
    # 「最も静かな連続区間」を探してそこだけで統計を取る。
    quiet, quiet_span = _quietest_window(stamps, gyro, rate)
    moved = float(np.linalg.norm(gyro.std(axis=0)) / max(
        np.linalg.norm(gyro[quiet].std(axis=0)), 1e-12))
    print()
    print(f"静止区間 : t = {quiet_span[0]:.1f} 〜 {quiet_span[1]:.1f} 秒"
          f"（{quiet.sum()} サンプル）を統計に使う")
    if moved > 3.0:
        print(f"  ⚠️ 全区間の σ は静止区間の **{moved:.0f} 倍**。"
              "記録中に機体が動かされている。")
        print("     全区間で共分散を取ると大幅な過大評価になる。")

    gyro_mean = gyro[quiet].mean(axis=0)
    gyro_std = gyro[quiet].std(axis=0)
    accel_std_raw = accel[quiet].std(axis=0)
    accel_std = accel_std_raw * (scale if scale == scale else 1.0)
    print()
    print("静止区間の統計")
    print(f"  gyro 平均 = [{gyro_mean[0]:+.6f} {gyro_mean[1]:+.6f} {gyro_mean[2]:+.6f}] rad/s"
          f"  ← バイアス（|b|={np.linalg.norm(gyro_mean) * 1e3:.3f} mrad/s）")
    print(f"  gyro σ    = [{gyro_std[0]:.6f} {gyro_std[1]:.6f} {gyro_std[2]:.6f}] rad/s")
    print(f"  accel σ   = [{accel_std[0]:.5f} {accel_std[1]:.5f} {accel_std[2]:.5f}] m/s²"
          f"  （生値 σ = {accel_std_raw.max():.6f} {unit.split('（')[0]}）")

    # 連続時間のノイズ密度 σ_c = σ_d / sqrt(f)。FAST-LIO2 の共分散はこちらの系。
    gyr_density = float(gyro_std.max() / math.sqrt(rate))
    acc_density = float(accel_std.max() / math.sqrt(rate))
    gyr_cov = float((gyro_std ** 2).max())
    acc_cov = float((accel_std ** 2).max())
    print()
    print("FAST-LIO2 の設定に使う値")
    print(f"  1サンプルあたりの分散  gyr {gyr_cov:.4g} (rad/s)²"
          f" / acc {acc_cov:.4g} (m/s²)²")
    print(f"  連続時間ノイズ密度     gyr {gyr_density:.4g} rad/s/√Hz"
          f" / acc {acc_density:.4g} m/s²/√Hz")
    print("  ⚠️ FAST-LIO2 既定の gyr_cov/acc_cov=0.1 は実測より桁で大きい"
          "（意図的に緩い）。")
    print("     まず既定で回し、収束しない場合にここの実測値を根拠に絞ること。")

    # --- 重力方向からセンサ取付角 ---------------------------------------
    accel_mean = accel[quiet].mean(axis=0)
    unit_g = accel_mean / np.linalg.norm(accel_mean)
    # LiDAR の Z 軸が鉛直からどれだけ傾いているか（重力は -Z 方向を向くのが理想）
    tilt = math.degrees(math.acos(min(1.0, abs(unit_g[2]))))
    roll = math.degrees(math.atan2(unit_g[1], -unit_g[2] if unit_g[2] < 0 else unit_g[2]))
    pitch = math.degrees(math.atan2(-unit_g[0], math.hypot(unit_g[1], unit_g[2])))
    print()
    print(f"重力方向 : [{unit_g[0]:+.4f} {unit_g[1]:+.4f} {unit_g[2]:+.4f}]（正規化）")
    print(f"  LiDAR の Z 軸と鉛直の角度 = **{tilt:.2f}°**"
          f"（roll {roll:+.2f}° / pitch {pitch:+.2f}°）")

    # --- 受信時刻とセンサ時刻のずれ -------------------------------------
    recv = recv_ns / 1e9
    offset = recv - stamps
    print()
    print(f"受信-送信: 中央 {np.median(offset) * 1e3:+.1f} ms"
          f" / 幅 {(np.percentile(offset, 95) - np.percentile(offset, 5)) * 1e3:.1f} ms")

    return {"stamps": stamps, "gyro": gyro, "accel": accel, "rate": rate,
            "gyr_cov": gyr_cov, "acc_cov": acc_cov, "tilt": tilt}


def analyse_points(bag: Path, topic_id: int, max_scans: int) -> dict:
    with sqlite3.connect(f"file:{bag}?mode=ro", uri=True) as connection:
        rows = connection.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topic_id,),
        ).fetchall()
    if not rows:
        raise SystemExit("生 LiDAR のメッセージが 0 件です")

    layout = parse_pointcloud2(bytes(rows[0][1]))
    print()
    print("=" * 72)
    print("生 LiDAR  {}  ({} スキャン)".format(RAW_POINTS_TOPIC, len(rows)))
    print("=" * 72)
    print(f"frame_id: {layout.frame_id}   point_step={layout.point_step}"
          f"   1スキャン {layout.point_count} 点")

    stamps = []
    step = max(1, len(rows) // max_scans)
    chunks = []
    for _, blob in rows[::step]:
        payload = bytes(blob)
        item = parse_pointcloud2(payload)
        reader = _CdrReader(payload)
        reader.int32()
        reader.uint32()
        reader.string()
        raw = np.frombuffer(payload, dtype=np.uint8,
                            count=item.data_length, offset=item.data_start)
        stride = item.point_step
        block = raw.reshape(-1, stride)
        xyz = np.stack([
            block[:, o:o + 4].copy().view(np.float32).ravel()
            for o in (item.x_offset, item.y_offset, item.z_offset)
        ], axis=1)
        chunks.append(xyz[np.isfinite(xyz).all(axis=1)])
    for _, blob in rows:
        payload = bytes(blob)
        reader = _CdrReader(payload)
        sec = reader.int32()
        nanosec = reader.uint32()
        stamps.append(sec + nanosec * 1e-9)
    stamps = np.array(stamps)
    points = np.concatenate(chunks, axis=0)

    dt = np.diff(stamps)
    span = stamps[-1] - stamps[0]
    print(f"レート  : {(len(stamps) - 1) / span:.2f} Hz  （記録 {span:.1f} 秒）")
    print(f"間隔    : 中央 {np.median(dt) * 1e3:.1f} ms / 最大 {dt.max() * 1e3:.1f} ms")
    print(f"解析対象: {len(chunks)} スキャン / {len(points):,} 点（{step} スキャンおきに間引き）")

    # --- 距離分布 -------------------------------------------------------
    rng = np.linalg.norm(points, axis=1)
    nonzero = rng > 1e-3
    points, rng = points[nonzero], rng[nonzero]
    print()
    print("距離分布（センサ原点から）")
    for threshold in (0.5, 1.0, 1.5, 2.0, 2.2, 3.0, 5.0):
        share = float((rng < threshold).mean()) * 100
        print(f"  < {threshold:>4.1f} m : {share:6.2f}% の点")
    print(f"  中央 {np.median(rng):.2f} m / p95 {np.percentile(rng, 95):.2f} m"
          f" / 最大 {rng.max():.1f} m")

    # --- 方位ごとの分布（自己遮蔽と追従者を見る） ------------------------
    azimuth = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
    elevation = np.degrees(np.arcsin(np.clip(points[:, 2] / rng, -1, 1)))
    print()
    print("仰角     : p1 {:+.1f}° / 中央 {:+.1f}° / p99 {:+.1f}°".format(
        *np.percentile(elevation, [1, 50, 99])))

    print()
    print("方位ごとの近傍点（センサ座標系。0°=前方 / +90°=左 / ±180°=後方）")
    print(f"{'方位':>12}  {'点数':>9} {'全体比':>7}  {'最小距離':>8} {'p5':>6} {'中央':>6}"
          f"  {'<2.2m の割合':>12}")
    print("-" * 76)
    edges = np.arange(-180, 181, 30)
    for low, high in zip(edges[:-1], edges[1:]):
        mask = (azimuth >= low) & (azimuth < high)
        if not mask.any():
            print(f"{low:+4.0f}〜{high:+4.0f}°  {'0':>9} {'0.00%':>7}  {'-':>8}"
                  f"{'-':>7}{'-':>7}  {'-':>12}   ← 点が無い")
            continue
        sub = rng[mask]
        near = float((sub < 2.2).mean()) * 100
        print(f"{low:+4.0f}〜{high:+4.0f}°  {mask.sum():>9,} {mask.mean() * 100:6.2f}%"
              f"  {sub.min():8.2f} {np.percentile(sub, 5):6.2f} {np.median(sub):6.2f}"
              f"  {near:11.2f}%")

    return {"points": points, "rng": rng, "azimuth": azimuth,
            "elevation": elevation, "stamps": stamps}


def report_sync(imu: dict, cloud: dict) -> None:
    print()
    print("=" * 72)
    print("LiDAR と IMU の時刻同期")
    print("=" * 72)
    imu_stamps, cloud_stamps = imu["stamps"], cloud["stamps"]
    print(f"IMU  : {imu_stamps[0]:.3f} 〜 {imu_stamps[-1]:.3f}")
    print(f"LiDAR: {cloud_stamps[0]:.3f} 〜 {cloud_stamps[-1]:.3f}")
    print(f"開始のずれ : {(cloud_stamps[0] - imu_stamps[0]) * 1e3:+.1f} ms")
    print(f"終了のずれ : {(cloud_stamps[-1] - imu_stamps[-1]) * 1e3:+.1f} ms")
    # 各スキャンの直近 IMU との差。同じ時計なら数 ms に収まる。
    #
    # 購読を張る順序の都合で、記録の先頭と末尾には片方しか無い区間ができる。
    # そこを混ぜると「最大 92ms ずれている」ように見えてしまうので、
    # **両方が揃っている区間だけ**で評価する。
    covered = (cloud_stamps >= imu_stamps[0]) & (cloud_stamps <= imu_stamps[-1])
    dropped = int((~covered).sum())
    inside = cloud_stamps[covered]
    if len(inside) == 0:
        print("→ ⚠️ IMU と LiDAR の時間帯が重なっていない")
        return
    index = np.searchsorted(imu_stamps, inside).clip(1, len(imu_stamps) - 1)
    nearest = np.minimum(np.abs(imu_stamps[index] - inside),
                         np.abs(imu_stamps[index - 1] - inside))
    print(f"評価対象   : {len(inside)}/{len(cloud_stamps)} スキャン"
          + (f"（IMU の範囲外 {dropped} スキャンを除外）" if dropped else ""))
    print(f"各スキャン直近の IMU との差 : 中央 {np.median(nearest) * 1e3:.2f} ms"
          f" / p99 {np.percentile(nearest, 99) * 1e3:.2f} ms"
          f" / 最大 {nearest.max() * 1e3:.2f} ms")
    # IMU は 200Hz なので、同じ時計なら最悪でも半周期 2.5ms に収まる。
    half_period = 0.5 / max((len(imu_stamps) - 1) / (imu_stamps[-1] - imu_stamps[0]), 1e-9)
    if nearest.max() <= half_period * 1.5:
        print(f"→ **同じ時計**（IMU 半周期 {half_period * 1e3:.2f} ms 以内）。"
              "FAST-LIO2 の de-skew はそのまま効く")
    else:
        print(f"→ ⚠️ IMU 半周期 {half_period * 1e3:.2f} ms を超えるずれがある。"
              "欠測か time_sync_en の検討が要る")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", type=Path, help="runs/<session_id>")
    parser.add_argument("--max-scans", type=int, default=200,
                        help="点群解析に使うスキャン数の上限（既定 200）")
    args = parser.parse_args()

    session = args.session if args.session.is_dir() else Path("runs") / args.session
    bag, ids = read_bag(session)

    imu = analyse_imu(bag, ids[IMU_TOPIC]) if IMU_TOPIC in ids else None
    cloud = (analyse_points(bag, ids[RAW_POINTS_TOPIC], args.max_scans)
             if RAW_POINTS_TOPIC in ids else None)
    if imu and cloud:
        report_sync(imu, cloud)
    if not imu:
        print(f"\n⚠️ {IMU_TOPIC} がこの bag に入っていません。FAST-LIO2 は回せません")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
