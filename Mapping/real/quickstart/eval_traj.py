#!/usr/bin/env python3
"""**測位アルゴリズムの比較の入り口。** 軌跡 1 本 + 記録 1 本 -> M1〜M5。

## なぜ「入り口を 1 つ」なのか

候補ごとに違う測り方をすると、数字は出るが**比較にならない**。
09-10 に「重畳の評価点数が 6,021 対 1,959,631」で桁の違う測り方を並べた前例があり、
09-12 の blend 掃引は「同じ条件が 6 倍ばらついて」結論が出なかった。

だから **入力を TUM 形式の軌跡テキスト 1 本に固定**する。候補（MOLA / FAST-LIO2 /
open3d_loc / KISS-ICP …）ごとに出力形式は違うが、**ここへ落とす変換だけ書けばよい**。

## 真値（モーキャプは無い。記録の中の拘束を使う）

| 真値 | 根拠 |
|---|---|
| **回転** = ジャイロの積分 | 頭と胴体の IMU が \|w\| で 0.999 の比。脚 odom とも 221° で +0.3% |
| **純回転の並進 = 0** | `RETURN=1` は逆回しで戻るので開始姿勢 = 終了姿勢 |
| **静止 = 動いていない** | 脚 odom で 60.9 s の端から端が 2 mm / 13 mm |

⚠️ **歩行中の並進には独立した真値が無い。** 歩行では M3（見かけの速さ）と
脚 odom との粗い食い違いしか見られない。**歩行の順位付けはこの入り口では出さない。**

⚠️ **ICP 品質 / fitness は指標にしない。** 09-12 の実測で `blend > 0` は品質を
0.83 -> 0.93 に水増しし、**間違った場所の品質のほうが平常より高く出た**（0.95〜1.00 対 0.851）。
正直な指標は M1 と M2 だけである。

## 使い方

    # 記録に入っている MOLA の /tf をそのまま測る（= C0 基準線）
    Navigation/.venv/bin/python quickstart/eval_traj.py runs/click_20260912T125919

    # 候補の軌跡（TUM）を測る
    ... eval_traj.py runs/click_20260912T125919 --traj "C1 sigma=0.5:/path/out.tum"

    # 複数の記録・複数の候補を 1 つの表に
    ... eval_traj.py runs/spin_*T13* --json out.json

TUM 形式: `timestamp tx ty tz qx qy qz qw`（`#` で始まる行は註釈）。
⚠️ **軌跡は `base_link`（水平・床面）で渡すこと。** IMU 系（`livox_frame`）のまま渡すと
MID-360 が上下逆さま（roll 177.93°）なので、落ちずに数字だけ悪くなる。
`--traj-frame livox` を付ければこちらで `base_link` へ落とす。
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import sqlite3
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from measure_overlay import Cdr  # noqa: E402

# velocity_smoother の並進の上限（hypot）。M3 はこれを超えた割合
SPEED_LIMIT_MPS = 0.361
# 回転していると見なす角速度。静止中の震えより十分上、遅い旋回 0.16 rad/s より下
TURN_RATE_RAD_S = 0.05
# M4 の窓（eval_guard_speed.py と同じ。1 サンプルの尖りで鳴らせないため中央値）
STILL_WINDOW_S = 0.5
# 見かけの速さを出すときに無視する dt。MOLA の /tf は 10 Hz 前後
MIN_DT_S = 0.01


# ---------------------------------------------------------------- 記録を読む

def open_bag(run_dir: Path) -> tuple[sqlite3.Connection, dict]:
    cands = sorted(glob.glob(str(run_dir / "**" / "*.db3"), recursive=True))
    if not cands:
        raise SystemExit("db3 が見つからない: {}".format(run_dir))
    con = sqlite3.connect("file:{}?mode=ro".format(cands[0]), uri=True)
    return con, {n: i for i, n in con.execute("SELECT id,name FROM topics")}


def read_imu(con, tid, topic="/utlidar/imu_livox_mid360") -> np.ndarray | None:
    """[t, wx, wy, wz] を返す（IMU = livox_frame 系）。"""
    if topic not in tid:
        return None
    rows = []
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?", (tid[topic],)):
        r = Cdr(blob)
        sec, nsec = r.i32(), r.u32(); r.st()
        [r.f64() for _ in range(4)]                    # orientation
        [r.f64() for _ in range(9)]                    # orientation_covariance
        w = [r.f64() for _ in range(3)]                # angular_velocity
        rows.append([sec + nsec * 1e-9] + w)
    return np.array(sorted(rows)) if rows else None


def read_odom(con, tid, topic="/dog_odom") -> np.ndarray | None:
    """[t, x, y, z, qx, qy, qz, qw] を返す（純正の脚 odometry）。"""
    if topic not in tid:
        return None
    rows = []
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?", (tid[topic],)):
        r = Cdr(blob)
        sec, nsec = r.i32(), r.u32(); r.st(); r.st()
        p = [r.f64() for _ in range(3)]; q = [r.f64() for _ in range(4)]
        rows.append([sec + nsec * 1e-9] + p + q)
    return np.array(sorted(rows)) if rows else None


def read_tf_chain(con, tid, parent="map", child="base_link") -> np.ndarray:
    """記録に入っている推定姿勢（= その回に動いていた測位）を軌跡として取り出す。"""
    if "/tf" not in tid:
        return np.empty((0, 8))
    rows = []
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?", (tid["/tf"],)):
        r = Cdr(blob)
        for _ in range(r.u32()):
            sec, nsec = r.i32(), r.u32(); par, ch = r.st(), r.st()
            tr = [r.f64() for _ in range(3)]; q = [r.f64() for _ in range(4)]
            if par == parent and ch == child:
                rows.append([sec + nsec * 1e-9] + tr + q)
    return np.array(sorted(rows)) if rows else np.empty((0, 8))


def read_static_tf(con, tid, parent="base_link", child="livox_frame"):
    """(translation, quaternion) を返す。⚠️ (R, t) ではない（09-12 に読み違えた）。"""
    if "/tf_static" not in tid:
        return None
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=?", (tid["/tf_static"],)):
        r = Cdr(blob)
        for _ in range(r.u32()):
            r.i32(); r.u32(); par, ch = r.st(), r.st()
            tr = [r.f64() for _ in range(3)]; q = [r.f64() for _ in range(4)]
            if par == parent and ch == child:
                return np.array(tr), np.array(q)
    return None


def read_tum(path: Path) -> np.ndarray:
    a = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        v = line.split()
        if len(v) < 8:
            continue
        a.append([float(x) for x in v[:8]])
    if not a:
        raise SystemExit("TUM が空: {}".format(path))
    return np.array(sorted(a))


# ---------------------------------------------------------------- 真値


def gyro_yaw(imu: np.ndarray, R_bl_imu: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """ジャイロを `base_link` へ回して積分し、(t, 累積 yaw[rad]) を返す。

    ⚠️ **w_z をそのまま積分しない。** MID-360 は上下逆さまなので符号が反転する。
    さらに roll/pitch が乗った姿勢では z 軸が鉛直と一致しないので、
    **回転を丸ごと積分してから yaw を取り出す**。
    """
    t = imu[:, 0]
    w_imu = imu[:, 1:4]
    w_bl = w_imu @ R_bl_imu.T                       # livox 系 -> base_link 系
    dt = np.diff(t, prepend=t[0])
    rot = Rotation.identity()
    out = np.empty(len(t))
    prev = Rotation.identity()
    for i in range(len(t)):
        if i:
            rot = rot * Rotation.from_rotvec(w_bl[i] * dt[i])
        out[i] = rot.as_euler("xyz")[2]
        prev = rot
    del prev
    return t, np.unwrap(out)


def net_yaw(q: np.ndarray) -> np.ndarray:
    return np.unwrap(Rotation.from_quat(q).as_euler("xyz")[:, 2])


# ---------------------------------------------------------------- 指標


def worst_window_median(t: np.ndarray, speed: np.ndarray, window_s: float) -> float:
    """窓 window_s 秒の中央値の最大。eval_guard_speed.py と同じ作り。"""
    worst = 0.0
    for i in range(len(t)):
        w = speed[(t >= t[i]) & (t < t[i] + window_s)]
        if len(w) >= 3:
            worst = max(worst, float(np.median(w)))
    return worst


def apparent_speed(traj: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """連続する推定の差分から見かけの速さ [m/s] を出す。(中点の時刻, 速さ)。"""
    dt = np.diff(traj[:, 0])
    ok = dt > MIN_DT_S
    d = np.hypot(np.diff(traj[:, 1]), np.diff(traj[:, 2]))
    return (traj[:-1, 0][ok] + dt[ok] / 2), (d[ok] / dt[ok])


def turn_window(t_gyro: np.ndarray, yaw_gyro: np.ndarray) -> tuple[float, float]:
    """回転している区間を返す。角速度がしきい値を超えた最初と最後。"""
    rate = np.abs(np.gradient(yaw_gyro, t_gyro))
    idx = np.flatnonzero(rate > TURN_RATE_RAD_S)
    if len(idx) < 2:
        return float(t_gyro[0]), float(t_gyro[-1])
    return float(t_gyro[idx[0]]), float(t_gyro[idx[-1]])


def slice_time(a: np.ndarray, t0: float, t1: float) -> np.ndarray:
    return a[(a[:, 0] >= t0) & (a[:, 0] <= t1)]


def evaluate(traj: np.ndarray, truth: dict, window: tuple[float, float]) -> dict:
    """軌跡 1 本 -> M1〜M4。M5 は歩行記録で M3 を見るので同じ関数で足りる。"""
    t0, t1 = window
    seg = slice_time(traj, t0, t1)
    out = {"samples": int(len(traj)), "samples_in_window": int(len(seg))}
    if len(seg) < 2:
        out["note"] = "窓の中に推定が 2 点未満"
        return out

    # M1 幻の並進: 純回転中の端から端の変位。真値 ~ 0
    out["M1_phantom_m"] = float(math.hypot(seg[-1, 1] - seg[0, 1], seg[-1, 2] - seg[0, 2]))
    out["max_stray_m"] = float(np.hypot(seg[:, 1] - seg[0, 1], seg[:, 2] - seg[0, 2]).max())

    # M2 回転の追従: 推定の正味 dyaw / ジャイロの正味 dyaw - 1
    est = net_yaw(seg[:, 4:8])
    out["est_net_yaw_deg"] = float(math.degrees(est[-1] - est[0]))
    gt = truth.get("gyro_net_yaw_deg_window")
    if gt is not None and abs(gt) > 1.0:
        out["M2_yaw_ratio_pct"] = float((out["est_net_yaw_deg"] / gt - 1.0) * 100.0)

    # M3 見かけの速さの上限超（窓の中）
    ts, sp = apparent_speed(seg)
    if len(sp):
        out["M3_over_limit_pct"] = float((sp > SPEED_LIMIT_MPS).mean() * 100.0)
        out["max_apparent_mps"] = float(sp.max())

    # M4 静止の震え（記録全体の 0.5 s 窓の中央値の最大）
    ta, sa = apparent_speed(traj)
    if len(sa):
        out["M4_still_jitter_mps"] = worst_window_median(ta, sa, STILL_WINDOW_S)
        out["M3_over_limit_pct_full"] = float((sa > SPEED_LIMIT_MPS).mean() * 100.0)
    return out


# ---------------------------------------------------------------- 本体


def analyze_run(run_dir: Path, traj_specs: list[tuple[str, Path, str]],
                window_arg: str) -> dict:
    con, tid = open_bag(run_dir)
    imu = read_imu(con, tid)
    odom = read_odom(con, tid)
    st = read_static_tf(con, tid)
    if st is None:
        raise SystemExit("{}: /tf_static に base_link->livox_frame が無い".format(run_dir.name))
    R_bl_imu = Rotation.from_quat(st[1]).as_matrix()

    res = {"run": run_dir.name, "truth": {}, "traj": {}}
    if imu is None or len(imu) < 10:
        raise SystemExit("{}: IMU が記録に無い。回転の真値が作れない".format(run_dir.name))

    t_g, yaw_g = gyro_yaw(imu, R_bl_imu)
    if window_arg == "full":
        win = (float(t_g[0]), float(t_g[-1]))
    elif window_arg == "auto":
        win = turn_window(t_g, yaw_g)
    else:
        a, b = window_arg.split(":")
        win = (t_g[0] + float(a), t_g[0] + float(b))

    gw = yaw_g[(t_g >= win[0]) & (t_g <= win[1])]
    res["truth"] = {
        "imu_hz": float(len(imu) / (imu[-1, 0] - imu[0, 0])),
        "duration_s": float(t_g[-1] - t_g[0]),
        "window_s": [round(float(win[0] - t_g[0]), 2), round(float(win[1] - t_g[0]), 2)],
        "gyro_net_yaw_deg": float(math.degrees(yaw_g[-1] - yaw_g[0])),
        "gyro_net_yaw_deg_window": float(math.degrees(gw[-1] - gw[0])) if len(gw) > 1 else None,
        "gyro_total_yaw_deg": float(math.degrees(np.abs(np.diff(yaw_g)).sum())),
    }
    if odom is not None and len(odom) > 2:
        ow = slice_time(odom, *win)
        oy = net_yaw(ow[:, 4:8]) if len(ow) > 1 else np.zeros(2)
        res["truth"].update({
            "odom_hz": float(len(odom) / (odom[-1, 0] - odom[0, 0])),
            "odom_net_disp_m": float(math.hypot(ow[-1, 1] - ow[0, 1], ow[-1, 2] - ow[0, 2]))
            if len(ow) > 1 else None,
            "odom_net_yaw_deg": float(math.degrees(oy[-1] - oy[0])) if len(ow) > 1 else None,
        })

    # 既定の軌跡: 記録に入っている /tf（= その回に動いていた測位。C0 の基準線）
    if not traj_specs:
        tf = read_tf_chain(con, tid)
        if len(tf) > 1:
            traj_specs = [("C0 bag内 /tf", None, "base_link")]
            res["traj"]["C0 bag内 /tf"] = evaluate(tf, res["truth"], win)
        else:
            res["traj"]["(bag に /tf が無い)"] = {"note": "推定が記録されていない"}
        return res

    for name, path, frame in traj_specs:
        if path is None:
            traj = read_tf_chain(con, tid)
        elif path.is_dir():
            # 再生で録り直した /tf（`replay_realtime.sh` の出力）。sim time なので
            # 時刻は元の記録と揃っている
            c2, t2 = open_bag(path)
            traj = read_tf_chain(c2, t2)
            if len(traj) < 2:
                # FAST-LIO2 は事前地図を読まないので /tf ではなく /Odometry を出す
                # （camera_init -> body）。M1 も M2 も相対量なのでそのまま測れる
                traj = (read_odom(c2, t2, "/Odometry_loc")
                        if "/Odometry_loc" in t2 else read_odom(c2, t2, "/Odometry"))
                if traj is None:
                    traj = np.empty((0, 8))
        else:
            traj = read_tum(path)
        if frame in ("livox", "livox_odom"):
            traj = to_base_link(traj, st, conjugate=(frame == "livox_odom"))
        res["traj"][name] = evaluate(traj, res["truth"], win)
    return res


def to_base_link(traj: np.ndarray, st, conjugate: bool = False) -> np.ndarray:
    """`livox_frame` の軌跡を `base_link` へ落とす。

    ⚠️ 09-09 にこれを掛け忘れて「測位がずれた」と読み違えた。落ちずに数字だけ下がる。

    **2 通りある。取り違えると yaw の符号が反転する**（2026-09-12 に踏んだ）:

    | 軌跡が何か | 基準の系 | 要る変換 |
    |---|---|---|
    | `map -> livox`（MOLA が地図の中で出す） | `map`（既に水平） | `T ∘ A⁻¹` |
    | **odometry**（FAST-LIO2 の `camera_init -> body`） | **開始時のセンサ姿勢**（＝逆さま） | **`A ∘ T ∘ A⁻¹`** |

    後者は基準の系も逆さまなので、**両側から**掛けないと直らない。
    片側だけだと M2 が **−200%**（＝ 符号が反転して倍になった）という形で出る。
    """
    t_bl_lv, q_bl_lv = st
    R = Rotation.from_quat(q_bl_lv)
    out = traj.copy()
    Rm = Rotation.from_quat(traj[:, 4:8])
    # map->livox を map->base_link にする: T_m_bl = T_m_lv * inv(T_bl_lv)
    if conjugate:
        # T' = A ∘ T ∘ A⁻¹  （A = base_link -> livox_frame）
        Rb = R * Rm * R.inv()
        pos = R.apply(traj[:, 1:4] + Rm.apply(-R.inv().apply(t_bl_lv))) + t_bl_lv
        out[:, 1:4] = pos
        out[:, 4:8] = Rb.as_quat()
        return out
    Rb = Rm * R.inv()
    out[:, 1:4] = traj[:, 1:4] - Rb.apply(t_bl_lv)
    out[:, 4:8] = Rb.as_quat()
    return out


FMT = "  {:<26} {:>9} {:>9} {:>9} {:>9} {:>9}"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--traj", action="append", default=[],
                    help='"名前:path.tum"（TUM）または "名前:再生の出力ディレクトリ"'
                         "（中の bag の /tf を読む）。省略すると記録の /tf（C0）を測る")
    ap.add_argument("--traj-frame", default="base_link",
                    choices=("base_link", "livox", "livox_odom"),
                    help="livox = map->livox の軌跡 / livox_odom = 基準も逆さまな odometry")
    ap.add_argument("--window", default="auto",
                    help="auto（回転区間）/ full / t0:t1（記録の頭からの秒）")
    ap.add_argument("--json", type=Path)
    args = ap.parse_args()

    specs = []
    for s in args.traj:
        name, _, path = s.rpartition(":")
        specs.append((name or Path(path).stem, Path(path), args.traj_frame))

    all_res = []
    for run in args.runs:
        r = analyze_run(run, specs, args.window)
        all_res.append(r)
        tr = r["truth"]
        print("\n=== {} ===".format(r["run"]))
        print("  長さ {:.1f} s / IMU {:.1f} Hz{} / 窓 {} s".format(
            tr["duration_s"], tr["imu_hz"],
            " / 脚odom {:.0f} Hz".format(tr["odom_hz"]) if "odom_hz" in tr else "",
            tr["window_s"]))
        print("  真値: ジャイロ 正味 {:+.1f}° (窓 {:+.1f}°) / 総回転 {:.0f}°{}".format(
            tr["gyro_net_yaw_deg"], tr["gyro_net_yaw_deg_window"] or float("nan"),
            tr["gyro_total_yaw_deg"],
            "  脚odom 正味 {:+.1f}° / 変位 {:.3f} m".format(
                tr["odom_net_yaw_deg"], tr["odom_net_disp_m"])
            if tr.get("odom_net_yaw_deg") is not None else ""))
        print(FMT.format("候補", "M1[m]", "M2[%]", "M3[%]", "M4[m/s]", "最大[m/s]"))
        for name, m in r["traj"].items():
            if "M1_phantom_m" not in m:
                print("  {:<26} {}".format(name, m.get("note", "?")))
                continue
            print(FMT.format(
                name[:26],
                "{:.3f}".format(m["M1_phantom_m"]),
                "{:+.1f}".format(m["M2_yaw_ratio_pct"]) if "M2_yaw_ratio_pct" in m else "-",
                "{:.1f}".format(m.get("M3_over_limit_pct", float("nan"))),
                "{:.3f}".format(m.get("M4_still_jitter_mps", float("nan"))),
                "{:.3f}".format(m.get("max_apparent_mps", float("nan")))))

    if args.json:
        args.json.write_text(json.dumps(all_res, ensure_ascii=False, indent=2))
        print("\n書き出し: {}".format(args.json))


if __name__ == "__main__":
    main()
