#!/usr/bin/env python3
"""測位を疑ったら**最初にこれを回す**。床平面から幾何を測り、設定値を算出する。

## なぜ固定値を持たないか

センサ→床の距離は**姿勢依存**（G1 は二足なので立ち方で胴体高さが変わる）。
さらに MOLA の絶対 z は歩行中に振れる（2026-09-07 実測で 590 秒中 **0.149 m**）。
だから定数を焼き込まず、**その都度測って設定値を出す**のがこのスクリプトの役目。

## 使い方

    # 実機（15 秒で足りる。590 秒の全長は要らない）
    # 記録トピックは quickstart/record_topics.txt が定義する（直書きしない）
    TOPICS="$(awk '/^[[:space:]]*#/{next} $2=="sensor"||$2=="ros"{printf "%s ", $1}' quickstart/record_topics.txt)"
    ros2 bag record -o calib_check ${TOPICS}
    Navigation/.venv/bin/python quickstart/check_calibration.py runs/calib_check

    # 記録＋MOLA-LO の軌跡（/tf が無い記録はこちら）
    ... check_calibration.py runs/<S>/raw/rosbag2 --traj runs/<S>/mola_floor0/traj.txt

    # 地図 PCD の床も一緒に見る
    ... --pcd runs/<S>/map/map_octomap_r4_s5.pcd

## `--base-link`: **base_link が床に対して水平か**だけを見る

この道具の中でもこれだけは **測位も地図も /tf も使わない**。
生 LiDAR に固定変換 `base_link -> livox_frame` を掛けて床平面を当てるだけなので、
測位の良し悪しから完全に独立した審判になる。

    python3 quickstart/check_calibration.py runs/<bag> --base-link

⚠️ **タグでは判定できない（循環する）。** `measure_extrinsics.py` は
「base_link は床と水平」を前提に床タグから測るので、機体の傾きがカメラ取付へ
吸収されて**構造的に 0 に出る**。2026-09-16 と 2026-09-17 に 2 回、これで誤診した。
疑ったらこちらを回すこと。

### この判定が正しいことの検算（2026-09-18）

同じ記録（`runs/_ov/bag`・2026-09-16 の補正後）に新旧の取付を当てて、
`base-link-was-tilted-59deg-floor-fix` に残っている値を再現できることを確かめた。

| 当てた取付 | このモードの出力 | 記録にある値 |
|---|---|---|
| 補正後 `-0.112579 -0.057151 1.209672` / `-179.401 -1.9437 0.0321` | **0.35 度** / 床 -0.001 m | 0.046 度 / +0.00002 m |
| 補正前 `0 0 1.228` / `177.93 3.32 0` | **5.24 度** | 5.90 度 |

床が base_link の原点に乗る（-0.001 m）ことも、新旧の差も再現する。

観測した値: 2026-09-16 の補正後 0.046 度 / 2026-09-17 朝（歩けた）pitch +0.62 度 /
同日午後の立位ロック +6.74 度 / R1+X 後 +4.30 度。
**同じ通常運転でもセッションごとに数度動く**ので、固定値にしてはいけない。

## 出るもの

  * **傾き**        map 系の床が水平か。2 deg 未満なら向きのcalibrationは正しい
  * **センサ→床**   姿勢に依らない不変量。取付が健全かの判定に使う
  * **床の高さ**    0 でなければ map の z 原点が床でない（向きとは別問題）
  * **振れ幅**      時間窓に分けて測ったドリフト。**設定の余裕はここから決める**
  * **推奨値**      MOLA_INITIAL_Z と min_obstacle_height をそのまま印字する

2026-09-07 の実測: 傾き 0.80 deg / センサ→床 1.228 m（2 地点で mm 一致）/
床の振れ幅 0.149 m（歩行 590 秒）。
"""
import argparse
import math
import sqlite3
import sys
import warnings
from pathlib import Path

import numpy as np

warnings.filterwarnings("ignore", message=".*encountered in matmul", category=RuntimeWarning)
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parent))
import measure_overlay as mo

SCAN_STEP = 20        # 3 GB の bag でも落ちないよう間引く（OOM で 1 度殺された）
POINT_STEP = 6
RANGE_MIN, RANGE_MAX = 1.0, 5.0
BAND = 0.25           # 最密 z 帯から床候補として拾う厚み [m]
MARGIN = 0.10         # 床の最大値に足す余裕 [m]


def floor_of(P: np.ndarray):
    """最密 z 帯を床とみなし、高さ(中央値)と傾きを返す。"""
    hist, ed = np.histogram(P[:, 2], bins=np.arange(-4, 4.01, 0.05))
    zc = ed[hist.argmax()]
    F = P[np.abs(P[:, 2] - zc) < BAND]
    if len(F) < 500:
        return None
    cen = F.mean(0)
    _, _, vt = np.linalg.svd(F - cen, full_matrices=False)
    n = vt[-1] / np.linalg.norm(vt[-1])
    if n[2] < 0:
        n = -n
    tilt = math.degrees(math.acos(min(1.0, abs(n[2]))))
    return float(np.median(F[:, 2])), tilt, len(F)


def base_link_level(a) -> int:
    """**base_link が床に対して水平か**を、生 LiDAR だけで測る。

    p_base = R(rpy) · p_livox + xyz  を当てて床平面を当てるだけ。
    法線が base_link の +z からどれだけ傾いているかが答え。
    """
    xyz = np.array([float(v) for v in a.livox_xyz.split()])
    rpy = np.array([float(v) for v in a.livox_rpy_deg.split()])
    # ROS の rpy 規約（R = Rz·Ry·Rx）。⚠️ open3d_loc の Euler2Matrix3d は Rx·Ry·Rz で
    # **別物**（run_fastlio_loc_live.sh の要点 2）。ここは静的 TF と同じ ROS 規約。
    R = Rotation.from_euler("xyz", rpy, degrees=True).as_matrix()

    _, scans, _ = mo.read_bag(a.bag)
    if not scans:
        print("生 LiDAR が 1 枚も無い（/utlidar/cloud_livox_mid360）", file=sys.stderr)
        return 1
    used = scans[:: max(1, a.scan_step)]
    P = np.vstack([s[1] for s in used])
    P = P[np.isfinite(P).all(1)]
    rng = np.linalg.norm(P, axis=1)
    # ⚠️ 至近は機体自身が写る。遠方は点が疎で平面フィットが暴れる
    P = P[(rng > RANGE_MIN) & (rng < RANGE_MAX)]
    B = P @ R.T + xyz

    got = floor_of(B)
    if got is None:
        print("床とみなせる帯に点が足りない（500 点未満）", file=sys.stderr)
        return 1
    z, tilt, cnt = got
    print(f"スキャン {len(used)} 枚 / 床候補 {cnt} 点（{RANGE_MIN}〜{RANGE_MAX} m）")
    print(f"base_link 系の床の高さ  {z:+.3f} m   （0 に近いほどよい）")
    print(f"base_link の傾き        {tilt:.2f} deg （床の法線と +z の角度）")
    print()
    if tilt < 1.0:
        print("✅ 水平。base_link の取付は今のままでよい")
    elif tilt < 3.0:
        print("⚠️ 1〜3 度。preflight の pitch ±3 度は通るが、詰めるなら測り直す")
    else:
        print("⛔ **3 度を超えている。** 次の順で切り分ける:")
        print("   1. 機体が通常運転（FSM 500/501）か。立位ロックだと数度前傾する")
        print("   2. それでも傾くなら base_link -> livox_frame を測り直す")
        print("      （2026-09-16 と同じ手順。水平軸まわりの最小回転を足す）")
        print("   ⚠️ タグ（measure_extrinsics.py）で確かめようとしないこと。循環する")
    return 0


def poses_from_tf(con, tid) -> np.ndarray:
    out = []
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?", (tid["/tf"],)):
        r = mo.Cdr(blob)
        for _ in range(r.u32()):
            sec, nsec = r.i32(), r.u32()
            parent, child = r.st(), r.st()
            tr = [r.f64() for _ in range(3)]
            q = [r.f64() for _ in range(4)]
            if parent == "map" and child == "base_link":
                out.append([sec + nsec * 1e-9, *tr, *q])
    return np.array(sorted(out))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--traj", type=Path, help="TUM 軌跡。指定すると bag の /tf の代わりに使う")
    ap.add_argument("--pcd", type=Path, help="地図 PCD。床の高さを併せて測る")
    ap.add_argument("--windows", type=int, default=8, help="ドリフトを見る時間窓の数")
    ap.add_argument("--scan-step", type=int, default=SCAN_STEP)
    ap.add_argument("--base-link", action="store_true",
                    help="base_link が床に対し水平かだけを見る（測位も /tf も使わない）")
    # 既定は run_fastlio_loc_live.sh / run_mola_live.sh と**同じ値**にしてある。
    # ⚠️ あちらを変えたらこちらも変えること（値は 7 ファイルに散っている）
    ap.add_argument("--livox-xyz", default="-0.112579 -0.057151 1.209672")
    ap.add_argument("--livox-rpy-deg", default="-179.401 -1.9437 0.0321")
    a = ap.parse_args()

    if a.base_link:
        return base_link_level(a)

    if a.traj:
        T = np.loadtxt(a.traj)
    else:
        db = next(a.bag.glob("*.db3"))
        c0 = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        t0 = {n: i for i, n in c0.execute("SELECT id,name FROM topics")}
        if "/tf" not in t0:
            print("bag に /tf が無い。--traj で軌跡を渡すこと", file=sys.stderr)
            return 1
        T = poses_from_tf(c0, t0)
    if len(T) < 2:
        print("map->base_link の姿勢が足りない", file=sys.stderr)
        return 1
    tt = T[:, 0]
    slerp = Slerp(tt, Rotation.from_quat(T[:, 4:8]))
    print(f"姿勢 {len(T)} 件 / {tt[-1]-tt[0]:.1f} 秒")

    db = next(a.bag.glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    tid = {n: i for i, n in con.execute("SELECT id,name FROM topics")}
    edges = np.linspace(tt[0], tt[-1], a.windows + 1)
    win = [[] for _ in range(a.windows)]
    sensor, bz = [], []
    n = 0
    for (blob,) in con.execute(
        "SELECT data FROM messages WHERE topic_id=?", (tid["/utlidar/cloud_livox_mid360"],)
    ):
        r = mo.Cdr(blob)
        sec, nsec = r.i32(), r.u32()
        fid = r.st()
        r.u32(); r.u32()
        for _ in range(r.u32()):
            r.st(); r.u32(); r.u8(); r.u32()
        r.u8()
        ps = r.u32(); r.u32(); dl = r.u32()
        if fid != "livox_frame" or ps != mo.LIVOX_POINT_STEP:
            continue                                   # 記録に混ざる SLAM 点群を弾く
        n += 1
        if n % a.scan_step:
            continue
        ts = sec + nsec * 1e-9
        if not (tt[0] <= ts <= tt[-1]):
            continue
        raw = np.frombuffer(blob, dtype=np.uint8, count=dl, offset=r.a)
        xyz = raw.reshape(-1, mo.LIVOX_POINT_STEP)[:, :12].copy().view(np.float32).reshape(-1, 3)
        d = np.linalg.norm(xyz, axis=1)
        xyz = xyz[(d > RANGE_MIN) & (d < RANGE_MAX)][::POINT_STEP]
        if not len(xyz):
            continue
        sensor.append(xyz.astype(np.float32))
        R = slerp(ts).as_matrix()
        tr = np.array([np.interp(ts, tt, T[:, 1 + i]) for i in range(3)])
        w = min(int(np.searchsorted(edges, ts, "right")) - 1, a.windows - 1)
        win[w].append((xyz @ R.T + tr).astype(np.float32))
        bz.append(float(np.interp(ts, tt, T[:, 3])))
    if not sensor or not any(win):
        print("使えるスキャンが無かった", file=sys.stderr)
        return 1

    s = floor_of(np.vstack(sensor))
    print(f"\n=== センサ系（その姿勢での取付傾き） ===")
    print(f"  床 {s[0]:+.3f} m / 傾き {s[1]:.2f} deg / inlier {s[2]:,}")

    print(f"\n=== map 系を {a.windows} 分割（★ドリフトを見る） ===")
    print(f"{'窓':>4} {'秒':>12} {'床の高さ':>10} {'傾き':>8} {'点数':>9}")
    print("-" * 48)
    hs, tilts = [], []
    for i, b in enumerate(win):
        if not b:
            continue
        f = floor_of(np.vstack(b))
        if f is None:
            continue
        hs.append(f[0]); tilts.append(f[1])
        print(f"{i+1:>4} {edges[i]-tt[0]:>7.0f}-{edges[i+1]-tt[0]:>4.0f}"
              f" {f[0]:>+10.3f} {f[1]:>7.2f}° {f[2]:>9,}")
    hs = np.array(hs); tilts = np.array(tilts)
    print("-" * 48)
    span = float(hs.max() - hs.min())
    print(f"  床の高さ 中央 {np.median(hs):+.3f} / σ {hs.std():.3f}"
          f" / 範囲 {hs.min():+.3f}..{hs.max():+.3f} → **振れ幅 {span:.3f} m**")
    print(f"  傾き     中央 {np.median(tilts):.2f} deg / 最大 {tilts.max():.2f} deg")

    height = float(np.mean(bz)) - float(np.median(hs))
    print(f"\n  センサ→床（不変量） {height:.3f} m")

    if a.pcd:
        import open3d as o3d
        P = np.asarray(o3d.io.read_point_cloud(str(a.pcd)).points)
        f = floor_of(P)
        print(f"\n=== 地図 PCD {a.pcd.name} ===")
        print(f"  床 {f[0]:+.3f} m / 傾き {f[1]:.2f} deg / inlier {f[2]:,}")
        print(f"  ライブの床との差 {f[0]-float(np.median(hs)):+.3f} m"
              f"（0 でなければ PCD と地図の datum が違う）")

    print("\n" + "=" * 62)
    print("判定")
    print("=" * 62)
    ok_tilt = tilts.max() < 2.0
    print(f"  向きのcalibration : {'OK' if ok_tilt else '⚠️ NG'}"
          f"  （map 系の床の傾き 最大 {tilts.max():.2f} deg < 2.0）")
    print(f"  z 原点            : {'OK' if abs(np.median(hs)) < 0.15 else '⚠️ 床が z=0 でない'}"
          f"  （床の中央 {np.median(hs):+.3f} m）")
    print("\n算出した設定値（固定値を焼かず、毎回ここから取ること）")
    if abs(np.median(hs)) >= 0.15:
        print(f"  MOLA_INITIAL_Z = {height:.3f}"
              f"   ← 地図を作り直して床を z=0 にする"
              f"（run_mola_lo.sh <S> --only-first-n 600 で足りる）")
    else:
        print(f"  MOLA_INITIAL_Z = {height:.3f}   （現地図は既に床 z≈0。作り直し不要）")
    rec = math.ceil((hs.max() + MARGIN) * 100) / 100
    print(f"  min_obstacle_height >= {rec:.2f}"
          f"   ← 床の最大 {hs.max():+.3f} に {MARGIN*100:.0f}cm の余裕。")
    print(f"     **これを下回ると床が間欠的に障害物として撃たれ、機体が囲まれて"
          f"経路が引けなくなる**")
    return 0


if __name__ == "__main__":
    sys.exit(main())
