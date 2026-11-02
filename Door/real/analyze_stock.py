"""11-02: 純正の脚の口（door_run_stock.py）の記録（logs/door_stock_<日時>.npz）を読み、機体の日（計画の手順 7〜11）の結果をまとめる。

出すもの:
  1. 段ごとの長さ・傾き・制御の周期（最大・99%）
  2. 扉: 測れた扉の角度の最大と最後・DOOR の間に pelvis が動いた距離・通り抜けたか（扉の先 1.5 m）
  3. 歩き: 向きごとに「指令に対して進んだ割合」（指令の積分と /dog_odom の動きを機体の座標で比べる）・SetVelocity の応答時間と失敗。
     ⚠️ /dog_odom は距離を横 7〜10%・前後 4〜6% 短く出す（10-21）。割合が 0.9 前後なら指令どおり。10-29 の遠隔操作は扉の前で前 12%
  4. 腕: arm_sdk で送った目標と実際の角度の差（DOOR の 95%・最大）・腕のトルクの最大
  5. 観測（--policy を渡したとき）: 生徒に入れた観測を学習の正規化の統計（rms_mean・rms_var）で割った値 |z| の 99% を項目ごとに。
     脚と腰（dof_pos・dof_vel・actions の先頭 15）を別に出す。純正の歩き方が学習の HOMIE の範囲から外れていないかを見る
     （脚と腰を隠すと成功は 0/256 だった＝ここが外れると方策は見慣れない入力で動く）
  6. 押す試し（PROBE）ごとの前への指令・進んだ距離・割合・傾きの最大
  7. 立ったまま止めた理由（記録の meta の reason）

使い方（Mac）: Navigation/.venv/bin/python Door/real/analyze_stock.py <door_stock_*.npz> [--policy door_policy_*.npz]
"""
from __future__ import annotations

import argparse
import ast
import sys
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_run_stock as drs  # noqa: E402

ARMS = slice(15, 29)
LEGS = 15  # 脚 12＋腰 3（観測の dof_pos・dof_vel・actions の先頭）
WALK_MIN = 0.05  # [m/s・rad/s] これより小さい指令の周期は割合に数えない


def _has(z, key: str) -> bool:
    return key in (z.files if hasattr(z, "files") else z)


def _tilt_deg(quat: np.ndarray) -> np.ndarray:
    return np.degrees(np.arccos(np.clip(1 - 2 * (quat[:, 1] ** 2 + quat[:, 2] ** 2), -1, 1)))


def _wrap(a: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(a), np.cos(a))


def follow_ratios(t: np.ndarray, want: np.ndarray, odom: np.ndarray, mask: np.ndarray) -> Dict[str, dict]:
    """向きごとの「指令に対して進んだ割合」。want は [歩くか, vx, vy, wz]、odom は世界の (x, y, yaw)。

    周期 k から k+1 の odometry の動きを k の向きで機体の座標に直し、その周期の指令の向きごとに足す。指令の積分で割る
    """
    out: Dict[str, dict] = {}
    k = np.where(mask[:-1] & np.isfinite(odom[:-1]).all(1) & np.isfinite(odom[1:]).all(1))[0]
    if len(k) == 0:
        return out
    dt = t[k + 1] - t[k]
    d = odom[k + 1, :2] - odom[k, :2]
    c, s = np.cos(odom[k, 2]), np.sin(odom[k, 2])
    moved = np.c_[c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], _wrap(odom[k + 1, 2] - odom[k, 2])]
    cmd = want[k, 1:4] * (want[k, 0:1] > 0.5)
    for name, axis, sign in (("前", 0, 1), ("後ろ", 0, -1), ("左", 1, 1), ("右", 1, -1), ("左旋回", 2, 1), ("右旋回", 2, -1)):
        m = sign * cmd[:, axis] > WALK_MIN
        if not m.any():
            continue
        asked = float(np.sum(sign * cmd[m, axis] * dt[m]))
        got = float(np.sum(sign * moved[m, axis]))
        out[name] = {"指令の積分": asked, "進んだ": got, "割合": got / asked if asked > 0 else float("nan"), "秒": float(dt[m].sum())}
    return out


def obs_zscores(obs: np.ndarray, policy) -> Dict[str, dict]:
    """観測を学習の正規化の統計で割った |z| の 99% と、正規化の頭打ち（rms_clip）を越えた割合を項目ごとに。脚と腰は別の項目にする"""
    x = obs[np.isfinite(obs).all(1)]
    if len(x) == 0:
        return {}
    zabs = np.abs((x - policy.rms_mean) / np.sqrt(policy.rms_var + policy.c["rms_eps"]))
    clip = policy.c["rms_clip"]
    out = {}
    for name, (a, b) in sorted(policy.layout.items(), key=lambda kv: kv[1][0]):
        parts = [(name, a, b)]
        if name in ("dof_pos", "dof_vel", "actions"):
            parts = [(f"{name}（脚と腰）", a, a + LEGS), (f"{name}（腕）", a + LEGS, b)]
        for label, lo, hi in parts:
            zz = zabs[:, lo:hi]
            worst = int(np.argmax(np.percentile(zz, 99, axis=0)))
            out[label] = {"z99": float(np.percentile(zz, 99)), "頭打ち超え": float((zz > clip).mean()), "最悪の次元": lo - a + worst}
    return out


def _segments(mask: np.ndarray) -> List[tuple]:
    """真の続く区間 [(始め, 終わりの次)]"""
    edges = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
    return list(zip(edges[0::2], edges[1::2]))


def summarize(z, policy=None) -> dict:
    """z は np.load の結果か、同じ名前の配列の辞書。policy は door_core.load_params の値（観測の比べに使う）"""
    t = z["t"] - z["t"][0]
    ph = z["phase"].astype(int)
    tilt = _tilt_deg(z["quat"])
    out: dict = {"段": {}}
    for i, name in enumerate(drs.PHASES):
        m = ph == i
        if m.any():
            per = z["period"][m] * 1000
            out["段"][name] = {"秒": float(m.sum() * drs.DT), "傾き最大°": float(np.nanmax(tilt[m])),
                              "周期最大ms": float(np.nanmax(per)), "周期99%ms": float(np.nanpercentile(per, 99))}
    door = ph == drs.PHASES.index("DOOR")
    probe = ph == drs.PHASES.index("PROBE")
    ang = z["door_angle_obs"]
    ok = ang[:, 1] > 0.5
    if ok.any():
        out["扉"] = {"最大°": float(np.degrees(ang[ok, 0].max())), "最後°": float(np.degrees(ang[ok, 0][-1])), "測れた割合": float(ok.mean())}
    pose = z["pose"]
    if door.any() and np.isfinite(pose[door]).all(1).any():
        pd = pose[door][np.isfinite(pose[door]).all(1)]
        out.setdefault("扉", {}).update({"DOOR の始めの姿勢": pd[0].round(3).tolist(), "DOOR の終わりの姿勢": pd[-1].round(3).tolist(),
                                         "動いた距離 m": float(np.linalg.norm(pd[-1, :2] - pd[0, :2])),
                                         "通り抜けた": bool((pd[:, 0] > 1.5).any())})
    out["歩き"] = follow_ratios(t, z["vel_want"], z["odom"], door)
    sent = np.r_[False, np.diff(z["vel_sent"]) > 0]
    if sent.any():
        rpc = z["vel_rpc_ms"][sent]
        code = z["vel_code"][sent]
        out["RPC"] = {"回数": int(sent.sum()), "中央ms": float(np.median(rpc)), "95%ms": float(np.percentile(rpc, 95)),
                      "最大ms": float(rpc.max()), "失敗": int(np.sum(np.isfinite(code) & (code != 0)))}
    if door.any():
        err = np.abs(z["q"][door][:, ARMS] - z["arm_sent"][door])
        err = err[np.isfinite(err).all(1)]
        if len(err):
            out["腕"] = {"差95%rad": float(np.percentile(err, 95)), "差最大rad": float(err.max()),
                        "差最大の関節": 15 + int(np.argmax(err.max(0))), "トルク最大Nm": float(np.abs(z["tau"][door][:, ARMS]).max())}
    if policy is not None and door.any():
        out["観測"] = obs_zscores(z["obs"][door], policy)
    out["押す試し"] = []
    for a, b in _segments(probe):
        m = np.zeros(len(ph), bool)
        m[a:b] = True
        fr = follow_ratios(t, z["vel_want"], z["odom"], m).get("前", {})
        out["押す試し"].append({"始め秒": float(t[a]), "前の指令 m/s": float(np.nanmax(z["vel_want"][a:b, 1])),
                              "進んだ m": fr.get("進んだ", float("nan")), "割合": fr.get("割合", float("nan")),
                              "傾き最大°": float(np.nanmax(tilt[a:b])), "腕のトルク最大Nm": float(np.abs(z["tau"][a:b, ARMS]).max())})
    if _has(z, "meta"):
        try:
            out["止めた理由"] = ast.literal_eval(str(z["meta"])).get("reason", "")
        except (ValueError, SyntaxError):
            out["止めた理由"] = "（meta を読めない）"
    return out


def print_report(r: dict, z_warn: float = 4.0) -> None:
    print("== 段")
    for name, v in r["段"].items():
        print(f"  {name:8s} {v['秒']:6.1f} 秒  傾き最大 {v['傾き最大°']:4.1f}°  周期 最大 {v['周期最大ms']:5.1f}・99% {v['周期99%ms']:5.1f} ms")
    if "扉" in r:
        print("== 扉  " + "  ".join(f"{k} {v}" for k, v in r["扉"].items()))
    print("== 歩き（指令に対して進んだ割合。/dog_odom は 5〜10% 短く出す）")
    for k, v in r["歩き"].items():
        print(f"  {k:4s} 指令 {v['指令の積分']:+.2f} → 進んだ {v['進んだ']:+.2f}（{v['割合']:.0%}・{v['秒']:.1f} 秒）")
    if "RPC" in r:
        print("== SetVelocity  " + "  ".join(f"{k} {v:.1f}" if isinstance(v, float) else f"{k} {v}" for k, v in r["RPC"].items()))
    if "腕" in r:
        print("== 腕  " + "  ".join(f"{k} {v:.3f}" if isinstance(v, float) else f"{k} {v}" for k, v in r["腕"].items()))
    if r.get("観測"):
        print(f"== 観測（学習の統計で割った |z| の 99%。{z_warn} を越えたら ⚠️）")
        for k, v in r["観測"].items():
            mark = "⚠️ " if v["z99"] > z_warn else "   "
            print(f"  {mark}{k:22s} z99 {v['z99']:6.2f}  頭打ち超え {v['頭打ち超え']:.1%}  最悪の次元 {v['最悪の次元']}")
    for p in r["押す試し"]:
        print("== 押す試し  " + "  ".join(f"{k} {v:.2f}" if isinstance(v, float) else f"{k} {v}" for k, v in p.items()))
    if r.get("止めた理由"):
        print(f"== 止めた理由  {r['止めた理由']}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="door_run_stock.py の記録（door_stock_<日時>.npz）")
    ap.add_argument("--policy", help="実機で使った重みの npz（観測を学習の統計と比べる）")
    a = ap.parse_args(argv)
    policy: Optional[object] = None
    if a.policy:
        import door_core as dc

        policy = dc.load_params(a.policy)
    with np.load(a.log, allow_pickle=False) as z:
        print_report(summarize(z, policy))
    return 0


if __name__ == "__main__":
    sys.exit(main())
