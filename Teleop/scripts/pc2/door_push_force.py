#!/usr/bin/env python3
"""手本の記録 1 本から、手が扉の板を押した力と、前へ歩けの区間ごとの押し方を出す（2026-10-30）。環境 tv（pinocchio）。

10-29 の 1 本目を OMEN で解析した door_push_force.py・fwd_force.py（~/door_teleop_data/20261029/analysis/code/）を、
1 本ごとに回せる形にした。関門の試し（純正の脚で扉を押して前へ出られるか）で、前へ出ない理由を分けるのに使う:
  (a) 押す力が扉を止めておく力（Room 4 で約 11〜13 N）を超えているのに、前へ進まない → 純正の歩行が押し返されて出ない
  (b) 手先が骨盤に対して縮み、押す力も上がらない → 腕が押し負けている
  (c) 機体が中央から寄って枠に当たる → check-ep の開始位置（start_y）と動画で見る

計算: 静止の釣り合い tau_meas = g(q) − Jᵀ F_ext（F_ext は環境が手に掛けた力）から F_ext を最小二乗で出し、扉が受けた力は −F_ext。
重力の向きは pelvis の IMU の姿勢（赤外線の記録の index.npz の quat）で模型に入れる。手から扉への向きと位置は、
check-ep が 1 本のフォルダに残す door_track.npz（扉の座標での頭の位置と向き）でつなぐ。当たる点は rubber_hand の先 10 cm。
⚠️ 左腕のトルクは腕を下ろした静止でも 18.5 N と出た（10-29。トルクの読みか模型のずれ）。左手の値は当てにしない。

    python door_push_force.py --episode-dir ~/teleop_data/door_gate_1031/episode_0003 --runs ~/mapping_runs
先に check-ep を流しておくこと（door_track.npz が要る）。結果は 1 本のフォルダの push_force.npz にも残す。
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import teleop_episode_check as C  # noqa: E402

SDK_JOINTS = ["left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
              "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
              "waist_yaw", "waist_roll", "waist_pitch"] + [f"{s}_{n}" for s in ("left", "right") for n in
              ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")]
ARM_IDX = {"left": list(range(15, 22)), "right": list(range(22, 29))}
WAIST_YAW = 12
CONTACT_TIP_M = 0.10          # rubber_hand の先のこの距離を当たる点にする
ON_PANEL_OFF_M = 0.12         # 板からこの距離より近く
ON_PANEL_EDGE_M = 0.05        # 蝶番からこの距離より先、板の幅＋この距離より手前
HOLD_FORCE_N = (11.0, 13.0)   # Room 4 のクローザーに対して扉を止めておくだけで要る力（10-29 の推定、蝶番から約 0.8 m を押すとき）
REF_S = 4.0                   # 記録を始める前の、立って止まっている間（骨盤の傾きの基準）
URDF_NAME = "g1_29dof_rev_1_0.urdf"   # シミュレータと同じ版。PC2 には deploy で ~/teleop_tools/ に置く


def default_urdf() -> Path:
    """台本と同じ置き場（PC2）か、リポジトリの Door/doorman6/usd_fix/urdf_src/"""
    cands = [HERE / URDF_NAME] + [p / "Door" / "doorman6" / "usd_fix" / "urdf_src" / URDF_NAME for p in HERE.parents[1:3]]
    return next((p for p in cands if p.exists()), cands[0])


@dataclass(frozen=True)
class PanelContact:
    from_hinge_m: float       # 板の向きに、蝶番から
    off_panel_m: float        # 板の面から（押す側が負）
    push_n: float             # 板を奥へ押す向きの力
    on_panel: bool            # 板の上にあって、板を奥へ押している
    moment_nm: float          # 扉を開ける向きに回す力（板の上のときだけ）


def panel_contact(hand_xy: np.ndarray, force_xy: np.ndarray, angle_deg: float, width: float,
                  lever_side: int = +1) -> PanelContact:
    """扉の座標（原点＝開口の中心の床・+x＝押して入る向き）での手の位置と、扉が受けた力から、板の上で押しているかを出す。
    lever_side +1 は把手が左（蝶番は右＝−y。UIS Room 4）"""
    th = math.radians(angle_deg) if math.isfinite(angle_deg) else 0.0
    hinge = np.array([0.0, -lever_side * width / 2])
    u = np.array([math.sin(th), lever_side * math.cos(th)])      # 蝶番から把手の側への板の向き
    n = np.array([math.cos(th), -lever_side * math.sin(th)])     # 板を奥へ押す向き
    r, d, fn = float((hand_xy - hinge) @ u), float((hand_xy - hinge) @ n), float(force_xy @ n)
    on = abs(d) < ON_PANEL_OFF_M and ON_PANEL_EDGE_M < r < width + ON_PANEL_EDGE_M and fn > 0
    return PanelContact(r, d, fn, on, r * fn if on else 0.0)


def pitch_deg(quat_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = quat_wxyz.T
    return np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0)))


def segment_rows(s: Dict[str, np.ndarray], tail_s: float = C.ODOM_TAIL_S) -> List[dict]:
    """前へ歩けの区間（check-ep と同じ数え方）ごとに、右手の押し方・手先の縮み・前へ進んだ距離・扉の角度の最大"""
    rows = []
    t = s["t"]
    for a, b in C.command_segments(t, s["cmd_vx"]):
        on = s["on_right"][a:b]
        hx = s["hand_x_right"][a:b]
        rows.append({
            "t0": float(t[a]), "t1": float(t[b]), "cmd_vx": float(s["cmd_vx"][a:b].mean()),
            "on_right_frac": float(on.mean()),
            "push_right_median": float(np.median(s["push_right"][a:b][on])) if on.any() else float("nan"),
            "hand_retract_m": float(hx[0] - hx[-1]),
            "body_fwd_m": float(np.interp(t[b] + tail_s, t, s["body_fwd"]) - np.interp(t[a], t, s["body_fwd"])),
            "angle_max": float(np.nanmax(s["angle"][a:b])) if np.isfinite(s["angle"][a:b]).any() else float("nan"),
        })
    return rows


class ArmModel:
    """pinocchio の模型（土台は pelvis に固定）。腕のトルクから手が受けた力を出す"""

    def __init__(self, urdf: Path):
        import pinocchio as pin
        self.pin = pin
        model = pin.buildModelFromUrdf(str(urdf))
        self.qidx = [model.joints[model.getJointId(n + "_joint")].idx_q for n in SDK_JOINTS]
        self.vidx = [model.joints[model.getJointId(n + "_joint")].idx_v for n in SDK_JOINTS]
        self.contact = {}
        for side in ("left", "right"):
            f = model.frames[model.getFrameId(f"{side}_rubber_hand")]
            tip = f.placement * pin.SE3(np.eye(3), np.array([CONTACT_TIP_M, 0.0, 0.0]))
            self.contact[side] = model.addFrame(pin.Frame(f"{side}_contact", f.parentJoint, model.getFrameId(f"{side}_rubber_hand"),
                                                          tip, pin.FrameType.OP_FRAME))
        self.lidar = model.getFrameId("mid360_link")
        self.model, self.data = model, model.createData()

    def solve(self, q29: np.ndarray, quat_wxyz: np.ndarray, tau14: np.ndarray) -> dict:
        """手が扉に掛けた力（世界の向き）・頭から見た当たる点（世界の向き）・骨盤の座標の当たる点・pelvis の yaw"""
        pin, model, data = self.pin, self.model, self.data
        q = pin.neutral(model)
        q[self.qidx] = q29
        rot = pin.Quaternion(*quat_wxyz).toRotationMatrix()
        model.gravity.linear = rot.T @ np.array([0.0, 0.0, -9.81])
        g = pin.computeGeneralizedGravity(model, data, q)
        pin.computeJointJacobians(model, data, q)
        pin.updateFramePlacements(model, data)
        lidar_w = rot @ data.oMf[self.lidar].translation
        out = {"yaw_pelvis": math.atan2(rot[1, 0], rot[0, 0])}
        for side, sl in (("left", slice(0, 7)), ("right", slice(7, 14))):
            v = [self.vidx[i] for i in ARM_IDX[side]]
            tau_ext = g[v] - tau14[sl]
            jac = pin.getFrameJacobian(model, data, self.contact[side], pin.LOCAL_WORLD_ALIGNED)[:3][:, v]
            f_ext = np.linalg.lstsq(jac.T, tau_ext, rcond=None)[0]
            p = data.oMf[self.contact[side]].translation
            out[side] = {"force_w": rot @ (-f_ext), "from_lidar_w": rot @ p - lidar_w, "pelvis": p.copy(),
                         "resid": float(np.linalg.norm(jac.T @ f_ext - tau_ext))}
        return out


def nearest(ts: np.ndarray, t: float) -> int:
    return int(np.clip(np.searchsorted(ts, t), 0, len(ts) - 1))


def analyze(frames: Sequence[dict], ir: dict, track: dict, model: ArmModel, odom: Optional[tuple],
            lever_side: int) -> Dict[str, np.ndarray]:
    """赤外線の記録の 1 コマ（10 Hz）ごとに、左右の手の力と板への当たり方・指令・前へ進んだ距離を並べる"""
    ft = np.array([f["time_ns"] for f in frames], dtype=np.int64)
    t0, t1 = int(ft[0]), int(ft[-1])
    tau = np.array([f["states"]["left_arm"]["torque"] + f["states"]["right_arm"]["torque"] for f in frames], dtype=float)
    cmd = np.array([f["actions"]["body"]["qpos"] for f in frames], dtype=float)
    keep = (ir["t_ns"] >= t0) & (ir["t_ns"] <= t1)
    it, iq, iquat = ir["t_ns"][keep], ir["q"][keep], ir["quat"][keep]
    tr_t = track["t"]
    raw = []
    for tn, q29, quat in zip(it, iq, iquat):
        j, m = nearest(ft, tn), nearest(tr_t, tn / 1e9)
        r = model.solve(q29, quat, tau[j])
        r.update(t=(tn - t0) / 1e9, j=j, m=m, psi=float(track["yaw"][m] - (r["yaw_pelvis"] + q29[WAIST_YAW])))
        raw.append(r)
    acc = np.array([bool(track["accepted"][r["m"]]) for r in raw])
    psi = np.unwrap(np.array([r["psi"] for r in raw]))
    psi0 = float(np.median(psi[acc])) if acc.any() else float(np.median(psi))
    c, s_ = math.cos(psi0), math.sin(psi0)
    rot = np.array([[c, -s_], [s_, c]])
    width = float(track["width"])
    out = {k: [] for k in ("t", "cmd_vx", "angle", "body_fwd")}
    for side in ("left", "right"):
        for k in ("on", "push", "force", "moment", "hand_x", "resid"):
            out[f"{k}_{side}"] = []
    for r in raw:
        m = r["m"]
        ang = float(track["angle"][m]) if track["accepted"][m] else float("nan")
        head = np.array([track["x"][m], track["y"][m]])
        out["t"].append(r["t"])
        out["cmd_vx"].append(cmd[r["j"], 0])
        out["angle"].append(ang)
        for side in ("left", "right"):
            hs = r[side]
            pc = panel_contact(head + rot @ hs["from_lidar_w"][:2], rot @ hs["force_w"][:2], ang, width, lever_side)
            for k, v in (("on", pc.on_panel), ("push", pc.push_n), ("force", float(np.linalg.norm(hs["force_w"]))),
                         ("moment", pc.moment_nm), ("hand_x", hs["pelvis"][0]), ("resid", hs["resid"])):
                out[f"{k}_{side}"].append(v)
    res = {k: np.array(v, dtype=bool if k.startswith("on_") else float) for k, v in out.items()}
    res["body_fwd"] = (np.array([C.body_displacement(odom[0], odom[1], odom[2], 0.0, x)[0] for x in res["t"]])
                       if odom is not None else np.full(len(res["t"]), np.nan))
    ref = (ir["t_ns"] >= t0 - int(REF_S * 1e9)) & (ir["t_ns"] < t0)       # 記録の前に開始位置で立っていた間
    base = float(np.median(pitch_deg(ir["quat"][ref]))) if ref.any() else float("nan")
    res["pitch_rel"] = pitch_deg(iquat) - base
    res["psi0"] = np.array(psi0)
    res["psi_spread_deg"] = np.degrees(np.percentile(psi[acc], [16, 84]) - psi0) if acc.any() else np.array([np.nan, np.nan])
    return res


def load_inputs(ep: Path, runs: Path, sensors: Optional[str]):
    frames = C.load_frames(ep / "data.json")
    t0, t1 = int(frames[0]["time_ns"]), int(frames[-1]["time_ns"])
    track_path = ep / "door_track.npz"
    if not track_path.exists():
        raise SystemExit(f"[押す力] {track_path} が無い。先に check-ep（teleop_episode_check.py）を流す")
    track = dict(np.load(track_path))
    ir_dir = C.find_session(runs, "_ir", t0, t1, sensors)
    if ir_dir is None:
        raise SystemExit(f"[押す力] この 1 本の時間を含む赤外線の記録（index.npz に腕の角度と pelvis の姿勢）が {runs} に無い")
    ir_all = dict(np.load(ir_dir / "index.npz"))
    od_dir = C.find_session(runs, "_odom", t0, t1, sensors)
    return frames, track, ir_all, od_dir


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="手本の記録 1 本から、手が扉を押した力と前へ歩けの区間ごとの押し方を出す")
    ap.add_argument("--episode-dir", type=Path, required=True)
    ap.add_argument("--runs", type=Path, default=Path.home() / "mapping_runs")
    ap.add_argument("--sensors", help="センサーの記録の名前（既定は時間で探す）")
    ap.add_argument("--urdf", type=Path, default=None, help="rubber_hand と mid360_link のある G1 29DoF の URDF（既定は rev_1_0）")
    ap.add_argument("--door-real", type=Path, default=None, help="/dog_odom の読み方（Door/real）の場所")
    a = ap.parse_args(argv)

    frames, track, ir_all, od_dir = load_inputs(a.episode_dir, a.runs, a.sensors)
    t0, t1 = int(frames[0]["time_ns"]), int(frames[-1]["time_ns"])
    odom = None
    if od_dir is not None:
        ot, oxy, oyaw = C.read_odom(od_dir, t0, t1 + int(C.ODOM_TAIL_S * 1e9), C.load_odom_parser(a.door_real or C.default_door_real()))
        odom = ((ot - t0) / 1e9, oxy, oyaw)
    res = analyze(frames, ir_all, track, ArmModel(a.urdf or default_urdf()), odom, int(track.get("lever_side", 1)))
    np.savez(a.episode_dir / "push_force.npz", **res)
    report(res)
    return 0


def report(res: Dict[str, np.ndarray]) -> None:
    lo, hi = res["psi_spread_deg"]
    print(f"[押す力] 世界→扉の回転 {math.degrees(float(res['psi0'])):+.1f}°・広がり（16〜84%）{lo:+.1f}〜{hi:+.1f}°（±3° 程度ならつなぎ方は正しい）")
    pitch = res["pitch_rel"]
    fwd = np.zeros(len(res["t"]), dtype=bool)
    for a, b in C.command_segments(res["t"], res["cmd_vx"]):
        fwd[a:b] = True
    for name, sel in (("前へ歩けの間", fwd), ("それ以外", ~fwd)):
        if not sel.any():
            continue
        on = res["on_right"] & sel
        push = f"中央 {np.median(res['push_right'][on]):.1f} N（上位 10% {np.percentile(res['push_right'][on], 90):.1f} N）" if on.any() else "—"
        print(f"[押す力] {name}（{sel.sum()} コマ）: 右手が板の上 {res['on_right'][sel].mean() * 100:.0f}%・板を押す力 {push}・"
              f"骨盤の前後の傾き（記録の前に立っていた時との差）中央 {np.nanmedian(pitch[sel]):+.2f}°")
    print(f"[押す力] 扉を止めておくだけで要る力の目安 {HOLD_FORCE_N[0]:.0f}〜{HOLD_FORCE_N[1]:.0f} N（蝶番から約 0.8 m）")
    rows = segment_rows(res)
    if rows:
        print("[押す力] 前へ歩けの区間ごと（前へ進んだ距離は区間の終わりの 1 秒後まで。手先の縮みは骨盤から見た右手の前後）:")
    for r in rows:
        push = f"{r['push_right_median']:5.1f} N" if math.isfinite(r["push_right_median"]) else "   —   "
        ang = f"{r['angle_max']:4.0f}°" if math.isfinite(r["angle_max"]) else "  —"
        print(f"  {r['t0']:6.1f}–{r['t1']:6.1f} 秒  指令 {r['cmd_vx']:.2f} m/s  前へ {r['body_fwd_m']:+.2f} m  右手 板の上 {r['on_right_frac'] * 100:3.0f}%・"
              f"押す力 {push}  手先の縮み {r['hand_retract_m'] * 100:+5.1f} cm  扉の最大 {ang}")


if __name__ == "__main__":
    sys.exit(main())
