#!/usr/bin/env python3
"""手本の記録を 1 本ごとに点検する（2026-10-27）。PC2（環境 tv）でも Mac（Navigation/.venv）でも動く。

1 本録るたびに流し、録り損ないをその場で見つける（10-26〜27 に、歩きが記録に入らない・時刻が無い・
センサーを録り忘れる、を続けて踏んだ）。見るもの:
  1. 遠隔操作の記録（data.json）: コマ数・1 コマごとの時刻（time_ns）・周期の乱れ・歩きの指令・腕
     ＋左右の腕の動きの合計・速さの最大・推定トルクの最大（関節つき）・上限（URDF）への張り付き
  2. センサーの記録（rec_take.sh）: その 1 本の時間にライダー・/dog_odom・赤外線の画像が入っているか
  3. 扉の角度: 記録を始める直前の 4 秒（開始位置で止まっている間・扉は閉じている）で扉と地図を作り、
     記録の間を追う（Door/real/lidar_door_est.py）。扉の角度の最大と、どの段階まで行ったか（S0〜S4）

    python3 teleop_episode_check.py --task door_room4_1027            # いちばん新しい 1 本
    python3 teleop_episode_check.py --task door_room4_1027 --episode 3
    python3 teleop_episode_check.py --episode-dir <episode_0003> --runs <mapping_runs>   # 場所を直接

結果は課題のフォルダの episode_checks.csv に 1 行ずつ足す（記録表の下書き）。
録り直しが要るもの（✗）が 1 つでもあれば終了コード 1。
段階: S1 扉角 2° 超（ラッチが外れた）／S2 30° 以上／S3 70° 以上 85° 以下で止めた／S4 頭が扉の座標で x ≥ 1.5 m
（扉の座標: 原点＝開口の中心の床・+x＝押して入る向き。頭は pelvis より約 5 cm 前なので S4 は目安）
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sqlite3
import sys
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

HERE = Path(__file__).resolve().parent
JST = timezone(timedelta(hours=9))
LIDAR_TOPIC = "/utlidar/cloud_livox_mid360"
ODOM_TOPIC = "/dog_odom"
REF_S = 4.0                   # 記録を始める直前の、扉と地図を作るのに使う秒
REF_MAX_SCANS = 25            # そのうち新しい方から（door_lidar_node.STILL_SCANS と同じ 2.5 秒ぶん）
DOOR_THICKNESS_M = 0.04       # door_lidar_node の既定と同じ
STAGE_ANGLES = (2.0, 30.0, 70.0, 85.0)
PASS_X_M = 1.5
# 合格の線（下回ったら ✗ ＝録り直し）
MIN_FRAMES = 30
MIN_LIDAR_HZ, MIN_ODOM_HZ, MIN_IR_FPS = 8.0, 500.0, 5.0
WARN_DT_P95_MS = 50.0         # 30 Hz（33 ms）の周期の 95% がこれを超えたら注意
ARM_JOINTS = ("肩 pitch", "肩 roll", "肩 yaw", "肘", "手首 roll", "手首 pitch", "手首 yaw")
ARM_TAU_LIMIT = np.array([25.0, 25.0, 25.0, 25.0, 25.0, 5.0, 5.0])  # G1 29DoF の URDF の上限 [N·m]
NEAR_LIMIT = 0.8              # 上限のこの割合を超えたら「張り付き」と数える
WARN_LR_RATIO = 2.0           # 左右のトルクの最大がこの倍率以上違ったら注意


@dataclass
class Report:
    lines: List[str] = field(default_factory=list)
    bad: int = 0
    row: dict = field(default_factory=dict)

    def ok(self, msg: str) -> None:
        self.lines.append(f"  ✓ {msg}")

    def warn(self, msg: str) -> None:
        self.lines.append(f"  ⚠️ {msg}")

    def ng(self, msg: str) -> None:
        self.lines.append(f"  ✗ {msg}")
        self.bad += 1


# ---------------------------------------------------------------- 遠隔操作の記録
def load_frames(json_path: Path) -> List[dict]:
    """data.json の data を返す。途中で止まって閉じていない記録も、最後のコマまで読む"""
    text = json_path.read_text(encoding="utf-8")
    try:
        return json.loads(text)["data"]
    except json.JSONDecodeError:
        body = text.rstrip().rstrip(",")
        for tail in ("\n]\n}", "}\n]\n}"):
            try:
                return json.loads(body + tail)["data"]
            except json.JSONDecodeError:
                continue
        cut = body.rfind("\n}")                       # 書きかけの最後のコマを捨てて閉じる
        return json.loads(body[:cut + 2] + "\n]\n}")["data"]


def check_frames(frames: Sequence[dict], rep: Report) -> Optional[Tuple[int, int]]:
    """コマ・時刻・周期・歩き・腕を見て、記録の時間 (t0_ns, t1_ns) を返す（時刻が無ければ None）"""
    n = len(frames)
    rep.row["frames"] = n
    (rep.ok if n >= MIN_FRAMES else rep.ng)(f"コマ数 {n}（{MIN_FRAMES} 以上）")
    t = [f.get("time_ns") for f in frames]
    if not t or any(v is None for v in t):
        rep.ng("1 コマごとの時刻（time_ns）が無い → 修正 record-timestamp が当たっていない")
        return None
    t = np.array(t, dtype=np.int64)
    dt_ms = np.diff(t) / 1e6
    dur = (t[-1] - t[0]) / 1e9
    rep.row.update(start_jst=datetime.fromtimestamp(t[0] / 1e9, JST).strftime("%Y-%m-%d %H:%M:%S"),
                   seconds=round(dur, 1))
    if len(dt_ms) and (dt_ms <= 0).any():
        rep.ng("時刻が進まない／戻るコマがある")
    if len(dt_ms):
        med, p95, mx = np.median(dt_ms), np.percentile(dt_ms, 95), dt_ms.max()
        rep.row.update(dt_median_ms=round(med, 1), dt_p95_ms=round(p95, 1), dt_max_ms=round(mx, 1))
        (rep.ok if p95 <= WARN_DT_P95_MS else rep.warn)(
            f"周期 中央 {med:.1f} ms・95% {p95:.1f} ms・最大 {mx:.1f} ms（{dur:.1f} 秒）")

    body = [((f.get("actions") or {}).get("body") or {}).get("qpos") for f in frames]
    if any(b is None or len(b) != 3 for b in body):
        rep.ng("歩きの指令が記録に無い（3 つの値でないコマがある）→ 修正 record-loco-cmd が当たっていない")
    else:
        v = np.array(body, dtype=float)
        moving = np.abs(v).max(1) > 1e-6
        rep.row.update(walk_frac=round(float(moving.mean()), 3), vx_max=round(float(v[:, 0].max()), 2),
                       vx_min=round(float(v[:, 0].min()), 2), vyaw_absmax=round(float(np.abs(v[:, 2]).max()), 2))
        rep.ok(f"歩きの指令あり（歩いたコマ {moving.mean() * 100:.0f}%・前後 {v[:, 0].min():+.2f}〜{v[:, 0].max():+.2f} m/s・"
               f"横 {v[:, 1].min():+.2f}〜{v[:, 1].max():+.2f}・旋回 {v[:, 2].min():+.2f}〜{v[:, 2].max():+.2f} rad/s）")
        if not moving.any():
            rep.warn("歩きの指令が全部 0（歩かずに終わった回）")

    arms = [(((f.get("actions") or {}).get(k) or {}).get("qpos") or []) for f in frames for k in ("left_arm", "right_arm")]
    (rep.ok if all(len(a) == 7 for a in arms) else rep.ng)("腕の目標が左右 7 関節ずつ入っている")
    check_arm_effort(frames, rep)
    return int(t[0]), int(t[-1])


def check_arm_effort(frames: Sequence[dict], rep: Report) -> None:
    """左右の腕の動きの合計・速さの最大・推定トルクの最大（関節つき）・上限への張り付きを比べる"""
    res = {}
    for side, key in (("左", "left_arm"), ("右", "right_arm")):
        st = [((f.get("states") or {}).get(key) or {}) for f in frames]
        q, dq, tau = ([s.get(k) or [] for s in st] for k in ("qpos", "qvel", "torque"))
        if any(len(x) != 7 for x in q):
            rep.ng(f"{side}腕の角度の状態が記録に無い")
            return
        if any(len(x) != 7 for x in dq) or any(len(x) != 7 for x in tau):
            rep.ng("腕の速さ・トルクが記録に無い → 修正 record-arm-effort が当たっていない")
            return
        q, dq, tau = np.array(q, float), np.abs(np.array(dq, float)), np.abs(np.array(tau, float))
        near = (tau / ARM_TAU_LIMIT >= NEAR_LIMIT).any(1).mean()
        res[side] = {"travel": float(np.abs(np.diff(q, axis=0)).sum()), "dq": float(dq.max()),
                     "dq_j": ARM_JOINTS[int(dq.max(0).argmax())], "tau": float(tau.max()),
                     "tau_j": ARM_JOINTS[int(tau.max(0).argmax())], "near": float(near)}
    L, R = res["左"], res["右"]
    pre = {"左": "L", "右": "R"}
    for side, r in res.items():
        rep.row.update({f"{pre[side]}_travel_rad": round(r["travel"], 2), f"{pre[side]}_dq_max": round(r["dq"], 2),
                        f"{pre[side]}_tau_max": round(r["tau"], 1), f"{pre[side]}_tau_joint": r["tau_j"],
                        f"{pre[side]}_near_limit_frac": round(r["near"], 3)})
    for side, r in res.items():
        rep.ok(f"{side}腕: 動きの合計 {r['travel']:.2f} rad・速さの最大 {r['dq']:.2f} rad/s（{r['dq_j']}）・"
               f"トルクの最大 {r['tau']:.1f} N·m（{r['tau_j']}）")
    lo, hi = sorted((L["tau"], R["tau"]))
    if lo > 0.5 and hi / lo >= WARN_LR_RATIO:
        big = "左" if L["tau"] > R["tau"] else "右"
        rep.warn(f"トルクの最大が左右で {hi / lo:.1f} 倍違う（{big}が大きい。押した手の側なら普通）")
    for side, r in res.items():
        if r["near"] > 0:
            rep.warn(f"{side}腕が上限の {NEAR_LIMIT * 100:.0f}% を超えた時間 {r['near'] * 100:.0f}%（関節が張り付いている）")


# ---------------------------------------------------------------- センサーの記録
def db3_files(session: Path) -> List[Path]:
    return sorted(session.glob("raw/**/*.db3")) or sorted(session.glob("**/*.db3"))


def db3_span(session: Path) -> Optional[Tuple[int, int]]:
    spans = []
    for f in db3_files(session):
        con = sqlite3.connect(f"file:{f}?mode=ro", uri=True)
        try:
            spans.append(con.execute("SELECT MIN(timestamp), MAX(timestamp) FROM messages").fetchone())
        finally:
            con.close()
    spans = [s for s in spans if s[0] is not None]
    return (min(s[0] for s in spans), max(s[1] for s in spans)) if spans else None


def db3_rows(session: Path, topic: str, t0: int, t1: int, payload: bool):
    for f in db3_files(session):
        con = sqlite3.connect(f"file:{f}?mode=ro", uri=True)
        try:
            row = con.execute("SELECT id FROM topics WHERE name = ?", (topic,)).fetchone()
            if row is None:
                continue
            col = "timestamp, data" if payload else "timestamp"
            yield from con.execute(f"SELECT {col} FROM messages WHERE topic_id = ? AND timestamp BETWEEN ? AND ? "
                                   "ORDER BY timestamp", (row[0], t0, t1))
        finally:
            con.close()


def find_session(runs: Path, suffix: str, t0: int, t1: int, name: Optional[str]) -> Optional[Path]:
    """名前の末尾が _li／_odom／_ir で、その 1 本の時間を含む記録（名前を渡せばそれに絞る）"""
    cands = sorted(p for p in runs.glob(f"*{suffix}") if p.is_dir() and (name is None or f"_{name}{suffix}" in p.name))
    for p in reversed(cands):
        if suffix == "_ir":
            idx = p / "index.npz"
            if idx.exists():
                tn = np.load(idx)["t_ns"]
                if len(tn) and tn[0] <= t0 and tn[-1] >= t1 - 2_000_000_000:
                    return p
            continue
        span = db3_span(p)
        if span and span[0] <= t0 - int(REF_S * 1e9) and span[1] >= t1 - 2_000_000_000:
            return p
    return None


def check_sensors(runs: Path, t0: int, t1: int, name: Optional[str], rep: Report) -> Optional[Path]:
    dur = max((t1 - t0) / 1e9, 1e-3)
    li = find_session(runs, "_li", t0, t1, name)
    if li is None:
        rep.ng(f"この時間を含むライダーの記録が {runs} に無い（sensors を先に起こしたか・記録を始める 4 秒前から録っているか）")
    else:
        hz = sum(1 for _ in db3_rows(li, LIDAR_TOPIC, t0, t1, False)) / dur
        rep.row["lidar_hz"] = round(hz, 1)
        (rep.ok if hz >= MIN_LIDAR_HZ else rep.ng)(f"ライダー {hz:.1f} 回/秒（{li.name}）")
    od = find_session(runs, "_odom", t0, t1, name)
    if od is None:
        rep.ng("この時間を含む /dog_odom の記録が無い")
    else:
        hz = sum(1 for _ in db3_rows(od, ODOM_TOPIC, t0, t1, False)) / dur
        rep.row["odom_hz"] = round(hz, 1)
        (rep.ok if hz >= MIN_ODOM_HZ else rep.ng)(f"/dog_odom {hz:.0f} 回/秒")
    ir = find_session(runs, "_ir", t0, t1, name)
    if ir is None:
        rep.warn("この時間を含む赤外線の画像の記録が無い（rec_take.sh の 3 つ目に ir を付けたか）")
    else:
        tn = np.load(ir / "index.npz")["t_ns"]
        fps = int(((tn >= t0) & (tn <= t1)).sum()) / dur
        rep.row["ir_fps"] = round(fps, 1)
        (rep.ok if fps >= MIN_IR_FPS else rep.ng)(f"赤外線の画像 {fps:.1f} 枚/秒（{ir.name}）")
    return li


# ---------------------------------------------------------------- 扉の角度
def load_door_modules(door_real: Path):
    sys.path.insert(0, str(door_real))
    import door_lidar_node as node  # noqa: E402
    import lidar_door_est as est  # noqa: E402
    return node, est


def make_decoder(node) -> Callable[[bytes], np.ndarray]:
    """記録の CDR → ライダーの座標の点 (n, 3)。PC2 は unitree_sdk2py、Mac は g1_mapping（crop_lidar_bag）で"""
    try:
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
        return lambda payload: node.cloud_xyz(PointCloud2_.deserialize(payload))
    except Exception:  # noqa: BLE001  Mac の環境に SDK が無いとき
        from crop_lidar_bag import scan_xyz
        return scan_xyz


def stage_of(max_angle: float, max_x: float) -> str:
    s = "S0"
    if math.isfinite(max_angle):
        if max_angle > STAGE_ANGLES[0]:
            s = "S1"
        if max_angle >= STAGE_ANGLES[1]:
            s = "S2"
        if STAGE_ANGLES[2] <= max_angle <= STAGE_ANGLES[3]:
            s = "S3"
    if s == "S3" and math.isfinite(max_x) and max_x >= PASS_X_M:
        s = "S4"
    return s


def track_door(est, ref_scans: Sequence[np.ndarray], scans: Sequence[Tuple[float, np.ndarray]],
               lever_side: int = +1, thickness: float = DOOR_THICKNESS_M) -> dict:
    """止まっている間の点群で扉と地図を作り、記録の間を追う。結果を辞書で（記録が無くても試せるよう点群を直接受ける）"""
    tracker = est.DoorTracker.from_still(list(ref_scans), thickness, lever_side)
    angles, xs, acc, lost = [], [], 0, False
    for t, xyz in scans:
        e = tracker.step(t, xyz)
        acc += int(e.accepted)
        lost = lost or e.lost
        angles.append(e.angle_deg)
        if e.accepted:
            xs.append(e.lidar.x)
    a = np.array(angles, dtype=float)
    fin = np.isfinite(a)
    max_angle = float(a[fin].max()) if fin.any() else float("nan")
    max_x = float(max(xs)) if xs else float("nan")
    return {"start_x": tracker.door.lidar_in_door.x, "start_y": tracker.door.lidar_in_door.y,
            "width": tracker.door.width, "max_angle": max_angle, "max_x": max_x,
            "accepted_frac": acc / max(len(scans), 1), "lost": lost, "stage": stage_of(max_angle, max_x),
            "angle_measured_frac": float(fin.mean()) if len(a) else 0.0}


def check_door(li: Path, t0: int, t1: int, door_real: Path, lever_side: int, rep: Report) -> None:
    try:
        node, est = load_door_modules(door_real)
        decode = make_decoder(node)
    except Exception as e:  # noqa: BLE001
        rep.ng(f"扉の推定器を読み込めない（{door_real}）: {type(e).__name__}: {e}")
        return
    delay = getattr(node, "SCAN_DELAY_S", 0.055)
    try:
        ref = [decode(p) for _, p in db3_rows(li, LIDAR_TOPIC, t0 - int(REF_S * 1e9), t0, True)][-REF_MAX_SCANS:]
    except Exception as e:  # noqa: BLE001  ⚠️ PC2 の unitree_sdk2py での読み出しは 10-27 時点で未確認
        rep.ng(f"ライダーの記録を点群に直せない: {type(e).__name__}: {e}")
        return
    if len(ref) < 10:
        rep.ng(f"記録を始める前のライダーが {len(ref)} 回しか無い（扉と地図を作れない。sensors を 4 秒以上前から）")
        return
    scans = [(ns / 1e9 - delay, decode(p)) for ns, p in db3_rows(li, LIDAR_TOPIC, t0, t1, True)]
    try:
        r = track_door(est, ref, scans, lever_side)
    except Exception as e:  # noqa: BLE001  扉が写っていない・開いていた など
        rep.ng(f"扉が測れない（開始位置で扉が写っていない・閉じていない？）: {type(e).__name__}: {e}")
        return
    rep.row.update(door_width=round(r["width"], 3), start_x=round(r["start_x"], 2), start_y=round(r["start_y"], 2),
                   max_angle_deg=round(r["max_angle"], 1) if math.isfinite(r["max_angle"]) else "",
                   max_x=round(r["max_x"], 2) if math.isfinite(r["max_x"]) else "",
                   track_frac=round(r["accepted_frac"], 2), stage=r["stage"])
    rep.ok(f"扉と地図を作れた（幅 {r['width']:.3f} m・開始位置 x {r['start_x']:+.2f} y {r['start_y']:+.2f} m）")
    ang = f"{r['max_angle']:.1f}°" if math.isfinite(r["max_angle"]) else "測れない"
    (rep.ok if r["angle_measured_frac"] > 0.5 else rep.warn)(
        f"扉の角度の最大 {ang}（測れた回 {r['angle_measured_frac'] * 100:.0f}%）・頭の x の最大 {r['max_x']:+.2f} m → 段階 {r['stage']}")
    if r["lost"] or r["accepted_frac"] < 0.8:
        rep.warn(f"重ね直しが続かなかった回がある（受け入れ {r['accepted_frac'] * 100:.0f}%）。通り抜けた後は見失いやすい")
    if math.isfinite(r["max_angle"]) and r["max_angle"] > STAGE_ANGLES[3]:
        rep.warn(f"85° を超えて押した（{r['max_angle']:.1f}°。約 90° で開いたまま止まる）")


# ---------------------------------------------------------------- 入口
def pick_episode(data_dir: Path, task: Optional[str], episode: Optional[int], episode_dir: Optional[Path]) -> Path:
    if episode_dir:
        return episode_dir
    if not task:
        raise SystemExit("[点検] --task か --episode-dir を渡す")
    eps = sorted((data_dir / task).glob("episode_*"))
    if not eps:
        raise SystemExit(f"[点検] {data_dir / task} に記録が無い")
    if episode is None:
        return eps[-1]
    want = data_dir / task / f"episode_{episode:04d}"
    if not want.exists():
        raise SystemExit(f"[点検] {want} が無い")
    return want


def append_csv(path: Path, row: dict) -> None:
    keys = ["episode", "start_jst", "seconds", "frames", "dt_median_ms", "dt_p95_ms", "dt_max_ms", "walk_frac",
            "vx_min", "vx_max", "vyaw_absmax", "L_travel_rad", "R_travel_rad", "L_dq_max", "R_dq_max",
            "L_tau_max", "L_tau_joint", "R_tau_max", "R_tau_joint", "L_near_limit_frac", "R_near_limit_frac", "lidar_hz", "odom_hz", "ir_fps", "door_width", "start_x", "start_y",
            "max_angle_deg", "max_x", "track_frac", "stage", "problems"]
    new = not path.exists()
    with path.open("a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        if new:
            w.writeheader()
        w.writerow(row)


def default_door_real() -> Path:
    for p in (Path.home() / "door_real", HERE.parents[2] / "Door" / "real"):
        if (p / "lidar_door_est.py").exists():
            return p
    return Path.home() / "door_real"


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="手本の記録を 1 本ごとに点検する")
    ap.add_argument("--task", help="課題名（~/teleop_data/<課題名>/）")
    ap.add_argument("--episode", type=int, help="番号（既定はいちばん新しい 1 本）")
    ap.add_argument("--episode-dir", type=Path, help="1 本のフォルダを直接")
    ap.add_argument("--data-dir", type=Path, default=Path.home() / "teleop_data")
    ap.add_argument("--runs", type=Path, default=Path.home() / "mapping_runs", help="センサーの記録の置き場")
    ap.add_argument("--sensors", help="センサーの記録の名前（sensors に渡した名前。既定は時間で探す）")
    ap.add_argument("--door-real", type=Path, default=None, help="扉の推定器（Door/real）の場所")
    ap.add_argument("--lever-right", action="store_true", help="把手が右の扉（UIS Room 4 は左なので付けない）")
    ap.add_argument("--no-door", action="store_true", help="扉の角度を計算しない（速く見たいとき）")
    ap.add_argument("--no-csv", action="store_true")
    a = ap.parse_args(argv)

    ep = pick_episode(a.data_dir, a.task, a.episode, a.episode_dir)
    rep = Report(row={"episode": ep.name})
    print(f"[点検] {ep}")
    try:
        frames = load_frames(ep / "data.json")
    except Exception as e:  # noqa: BLE001
        print(f"  ✗ data.json を読めない: {type(e).__name__}: {e}")
        return 1
    span = check_frames(frames, rep)
    if span:
        t0, t1 = span
        li = check_sensors(a.runs, t0, t1, a.sensors, rep)
        if li is not None and not a.no_door:
            check_door(li, t0, t1, a.door_real or default_door_real(), -1 if a.lever_right else +1, rep)
    rep.row["problems"] = rep.bad
    print("\n".join(rep.lines))
    print(f"[点検] {'✓ 録り直し不要' if rep.bad == 0 else f'✗ 録り直しが要る所 {rep.bad} 件'}")
    if not a.no_csv:
        out = ep.parent / "episode_checks.csv"
        append_csv(out, rep.row)
        print(f"[点検] 記録表の下書きに 1 行足した: {out}")
    return 0 if rep.bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
