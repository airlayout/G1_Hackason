"""11-12 夜（作業ログ 2026-11-12 の 6 章 ㋐㋑・7 章の 1）: 台本の定数を機体へ配る前の門番。numpy だけ。

台本（door_script.ScriptDoor ＋ 走らせ役のつなぎ door_run_staged.ScriptArm）を、行き先どおりに動く腕で机の上で走らせ、
構えの位置（真ん中と許容の 4 隅）から 2b（構える）と 2c（下ろして触れる）の間を見る:

  ① 台本そのものが止まらない（手が進まない・可動域の端・時間切れ）
  ② 2b の間、腕（上腕・前腕・手・手の先）と 板・枠・壁 のすき間が fail_m 以上（warn_m 未満は注意）
     棒・付け根には下や横から触れない（すき間 fail_bar_m 以上）
  ③ 2c で手が当てる点の高さに下りた時、手の先の線分が棒の上（付け根から 1 cm〜棒の先）に掛かっている
  ④ 腕の関節が可動域の端から margin_rad 以上内側（warn_margin_rad 未満は注意）

すき間の線は 11-12 の記録で決めた（--history で答え合わせ）: 手の先は計画より実機で約 2 cm 前へ出る（10 回目の計画 1.7 cm →
実機 −0.4 cm で「少しかすった」）。だから計画で 2 cm 無ければ注意、1 cm 無ければ落とす。

  python script_gate.py script_params.json --door 0.94,2.05,0.99,0.064 --pelvis-z 0.73   # 通れば <定数>.ok を書く
  python script_gate.py script_params.json --door ... --check                            # .ok が今の定数と合うかだけ
  python script_gate.py --history                                                        # 11-12 の各回の定数で答え合わせ

<定数>.ok には、定数ファイルと寸法のファイルの sha256・DOOR・pelvis の高さを書く。走らせ役（--script-gate on）は
d のたびにこれを確かめ、合わなければ d を受け付けない（門番を通っていない定数で機体を動かさない）。
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import json
import math
import sys
import time
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

import door_clearance as dcl
import door_frame as dfr
import door_run_staged as drst
import door_script as dscr
import g1_arm_fk as fk
import g1_arm_ik as aik

HOME7 = np.array([0.29, 0.2, -0.02, 0.99, 0.15, 0.02, -0.02])  # 11-11・12 の記録の、純正が腕を下ろしている角度
FAIL_M, WARN_M = 0.01, 0.02          # 2b の 板・枠・壁 とのすき間（計画）
FAIL_BAR_M = 0.01                    # 2b で棒・付け根に下や横から触れない（板と同じく、実機の約 2 cm の前ずれを見込む）
LIMIT_NEAR = 0.05                    # 逆運動学が切る所（URDF の端から LIMIT_MARGIN 内）までこれ以内なら「端に付いている」[rad]
LIMIT_FAIL_S = 3.0                   # 1 つの関節が端に付いている長さがこれを越えたら落とす（実機: 5 回目 手首 yaw 4 秒で止まった・10 回目 手首 roll 1.3 秒は通った）
LIMIT_PAIR_S = 0.5                   # 2 つ以上の関節が同時に端に付いている長さがこれを越えたら落とす
# 11-12 の機体: 5 回目は肘・手首 pitch・手首 yaw が 2b の間に 1.6〜3.3 秒端に付いて手が止まった。
# 10 回目は手首 roll だけが 1.3 秒付いたが通った（作業ログ 6 章 ㋑）
BAR_ON_GAP_M = 0.02                  # 2c で手の先と棒のすき間がこれ以下なら「棒の上に掛かっている」
BAR_ON_MIN = 0.01                    # 付け根から少なくともこれだけ先（付け根の際は回す腕が短く下がらない・11-11）
SIM_S = 25.0
DT = 0.02
WALL_OBS = ("板", "枠", "壁")
HOOK_FAIL_M = 0.02                   # 掛け終えた時の手のひらと掛けた形の差の線（行き先どおりに動く腕で）
HOOK_DIR_FAIL_DEG, HOOK_DIR_WARN_DEG = 10.0, 5.0  # 掛け終えた時の手の向きの差の線
BAR_OBS = ("棒", "付け根")


@dataclass
class PoseResult:
    name: str
    ok: bool = True
    warn: bool = False
    lines: List[str] = field(default_factory=list)
    min_gap: Dict[str, Tuple[float, str]] = field(default_factory=dict)
    margin: float = math.inf
    limit_s: Dict[str, float] = field(default_factory=dict)
    pair_s: float = 0.0
    bar: Tuple[float, float] = (math.nan, math.nan)

    def bad(self, msg: str) -> None:
        self.ok = False
        self.lines.append("× " + msg)

    def note(self, msg: str) -> None:
        self.warn = True
        self.lines.append("△ " + msg)


def stance_poses(st) -> List[Tuple[str, dfr.PlanarPose]]:
    """構えの位置の真ん中と、許容の 4 隅（前後・横・向きを組で振る）"""
    t = st.target
    out = [("真ん中", t)]
    for sx, sy, sa, name in ((1, 1, 1, "前・左・左回り"), (1, -1, -1, "前・右・右回り"), (-1, 1, -1, "後ろ・左・右回り"),
                             (-1, -1, 1, "後ろ・右・左回り")):
        out.append((name, dfr.PlanarPose(t.x + sx * st.tol_x, t.y + sy * st.tol_y, t.yaw + sa * st.tol_yaw)))
    return out


def run_pose(p: dscr.ScriptParams, g: dcl.DoorGeom, pose: dfr.PlanarPose, pelvis_z: float, chain: fk.ArmFK, ik: aik.ArmIK,
             name: str, shift: float = 0.0, hook: bool = False) -> PoseResult:
    """行き先どおりに動く腕で、2c の触れた所（当てる点の高さ）まで台本を走らせる。
    hook（11-13 夜・p.hook_deg > 0）: 2d の押し下げと手首を下げて掛ける所まで走らせ、掛けた形に届くかと板・枠・壁とのすき間を見る
    （門番の棒は閉じたまま動かないので、2c・2d の棒とのすき間は見ない。押し下げで棒が下がっても板からの距離は変わらない）"""
    res = PoseResult(name)
    p = replace(p, touch_by_height=True, stop_after="2d" if hook else "2c", retries=0)  # 門番は「当てる点の高さに下りた時」で見る
    sd = dscr.ScriptDoor(p, g.width, g.backset, g.lever_height, g.lever_left)
    sd.shift = shift  # 空振りの押し直しの横のずらし（0 でなければ、棒に乗るかは見ず、壁・板とのすき間と可動域だけ見る）
    arm = drst.ScriptArm(sd, ik, drst.make_palm_fn(chain, pelvis_z, g.lever_left), pelvis_z)
    q = np.zeros(29)
    q[12] = sd.twist_rad()  # 11-13 半身で押す: 腰 yaw を回した姿勢で見る（回り切った後の形）
    q[14] = math.radians(p.lean_deg) if dscr.SUBS.index(p.lean_from) <= dscr.SUBS.index("2c") else 0.0  # 11-13 前へ傾けた姿勢で見る
    q[ik.idx] = HOME7
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    tau = np.zeros(29)
    obs = dcl.obstacles(g)
    res.min_gap = {k: (math.inf, "") for k in obs}
    out = None
    with contextlib.redirect_stdout(io.StringIO()):
        for k in range(int(SIM_S / DT)):
            out, q7 = arm.step(k * DT, q, quat, pose, tau)
            if q7 is not None:
                q[ik.idx] = q7
            if out.sub in ("2b", "2c") or (hook and out.sub == "2d"):
                # 逆運動学が切る所（URDF の端から LIMIT_MARGIN 内側）までの余裕。0 は逆運動学が端に張り付いている
                m = np.minimum(q[ik.idx] - ik.lo, ik.hi - q[ik.idx])
                res.margin = min(res.margin, float(m.min()))
                near = [aik.ARM_JOINTS[j] for j in np.where(m < LIMIT_NEAR)[0]]
                for j in near:
                    res.limit_s[j] = res.limit_s.get(j, 0.0) + DT
                if len(near) >= 2:
                    res.pair_s += DT
                if k % 2 == 0 or out.stop:
                    segs = dcl.arm_segments(chain, q, pose, quat, pelvis_z, g, "left" if g.lever_left else "right")
                    gaps = dcl.clearances(segs, g, obs)
                    for o, (d, part) in gaps.items():
                        if out.sub in ("2c", "2d") and o in BAR_OBS:
                            continue  # 2c は棒へ下りていく所・2d は棒を押し下げている所
                        if d < res.min_gap[o][0]:
                            res.min_gap[o] = (d, f"{part}・{out.sub}")
            if out.stop:
                break
    if out is None or not out.stop:
        res.bad(f"{SIM_S:.0f} 秒で 2c の終わりに着かない（最後の小段 {out.sub if out else '?'}）")
        return res
    if "止める所" not in out.stop:
        res.bad(f"台本が止まった: {out.stop}")
        return res
    segs = dcl.arm_segments(chain, q, pose, quat, pelvis_z, g, "left" if g.lever_left else "right")
    res.bar = dcl.bar_contact(segs, g)
    gap, at = res.bar
    if hook:
        _hook_end(res, sd, arm, ik, q, pose, quat)
    elif shift == 0.0 and not (gap <= BAR_ON_GAP_M and BAR_ON_MIN <= at <= g.bar_len):
        res.bad(f"当てる点の高さに下りた時、手の先が棒の上に無い（すき間 {100 * gap:.1f} cm・付け根から {100 * at:.1f} cm）")
    for o in WALL_OBS:
        d, where = res.min_gap[o]
        if d < FAIL_M:
            res.bad(f"{o}とのすき間 {100 * d:.1f} cm（{where}・線 {100 * FAIL_M:.0f} cm）")
        elif d < WARN_M:
            res.note(f"{o}とのすき間 {100 * d:.1f} cm（{where}・実機では計画より約 2 cm 前へ出る）")
    for o in BAR_OBS if shift == 0.0 else ():
        d, where = res.min_gap[o]
        if d < FAIL_BAR_M:
            res.bad(f"構える間に{o}に触れる（すき間 {100 * d:.1f} cm・{where}）")
        elif d < WARN_M:
            res.note(f"構える間の{o}とのすき間 {100 * d:.1f} cm（{where}・1 回目は棒の下に手の先が掛かった）")
    long = {j: s for j, s in res.limit_s.items() if s > LIMIT_FAIL_S}
    held = "・".join(f"{j} {s:.1f} 秒" for j, s in res.limit_s.items())
    if long or res.pair_s > LIMIT_PAIR_S:
        res.bad(f"腕が可動域の端に付く（{held}・2 つ以上同時に {res.pair_s:.1f} 秒）")
    elif res.limit_s:
        res.note(f"腕の関節が一時、可動域の端に付く（{held}）")
    return res


def _hook_end(res: PoseResult, sd: dscr.ScriptDoor, arm: drst.ScriptArm, ik: aik.ArmIK, q: np.ndarray,
              pose: dfr.PlanarPose, quat: np.ndarray) -> None:
    """掛け終えた時: 手のひらが掛けた形（＋押さえの先行）に着き、手の向きが上向きになっているか"""
    if sd.hook_palm0 is None:
        res.bad("2d で手首を下げ始めなかった")
        return
    geo, d = sd.hook_geometry(sd.hook_palm0, 1.0)
    R, T = arm._frame(pose, quat)
    palm = T + R @ fk.palm_in_pelvis(ik.chain, q, ik.side)
    err = float(np.linalg.norm(palm - (geo + np.array([0.0, 0.0, sd.hook_lead]))))
    u, _ = ik.direction(q, q[ik.idx])
    ang = math.degrees(math.acos(float(np.clip((R @ u) @ d, -1.0, 1.0))))
    res.lines.append(f"  掛けた形: 手のひらの差 {100 * err:.1f} cm・手の向きの差 {ang:.1f}°（手の先 {sd.p.hook_deg:.0f}° 上）")
    if err > HOOK_FAIL_M:
        res.bad(f"掛けた形に手のひらが届かない（差 {100 * err:.1f} cm・線 {100 * HOOK_FAIL_M:.0f} cm）")
    if ang > HOOK_DIR_FAIL_DEG:
        res.bad(f"手首を下げ切れない（手の向きの差 {ang:.1f}°・線 {HOOK_DIR_FAIL_DEG:.0f}°）")
    elif ang > HOOK_DIR_WARN_DEG:
        res.note(f"手の向きが掛けた形から {ang:.1f}° ずれる")


def check(p: dscr.ScriptParams, g: dcl.DoorGeom, pelvis_z: float, urdf: Optional[str] = None,
          poses: Optional[Sequence[Tuple[str, dfr.PlanarPose]]] = None) -> Tuple[bool, bool, List[PoseResult]]:
    """返り値: (通ったか, 注意があるか, 構えの位置ごとの結果)"""
    chain = fk.load_chain(urdf)
    ik = aik.ArmIK(chain, "left" if g.lever_left else "right", urdf)
    st = dscr.script_stance(p, g.width, g.backset, g.lever_height, g.lever_left)
    ps = list(poses or stance_poses(st))
    results = [run_pose(p, g, pose, pelvis_z, chain, ik, name) for name, pose in ps]
    if p.hook_deg > 0.0:  # 11-13 夜 手首を下げて掛ける形: 同じ構えの位置で 2d の終わりまで
        results += [run_pose(p, g, pose, pelvis_z, chain, ik, f"{name}・掛ける", hook=True) for name, pose in ps]
    if poses is None:  # 空振りの押し直し: 構えの真ん中で、ずらした先でも壁・板に当たらず腕が届くか
        results += [run_pose(p, g, st.target, pelvis_z, chain, ik, f"押し直し 横 {100 * v:+.0f} cm", shift=v) for v in p.miss_shifts]
    return all(r.ok for r in results), any(r.warn for r in results), results


def file_hash(*paths: Optional[str]) -> str:
    h = hashlib.sha256()
    for path in paths:
        h.update(b"|")
        if path:
            h.update(Path(path).read_bytes())
    return h.hexdigest()


def ok_path(params_path: str) -> Path:
    return Path(str(params_path) + ".ok")


def stamp(params_path: str, geom_path: Optional[str], door: str, pelvis_z: float, warn: bool) -> dict:
    return dict(sha256=file_hash(params_path, geom_path), door=door, pelvis_z=round(float(pelvis_z), 4),
                geom=Path(geom_path).name if geom_path else "", warn=bool(warn), when=time.strftime("%Y-%m-%d %H:%M:%S"))


def verify_ok(params_path: str, geom_path: Optional[str], door: str, pelvis_z: float) -> str:
    """<定数>.ok が今の定数・寸法・DOOR・pelvis の高さと合えば ""、合わなければ理由（走らせ役が d の前に呼ぶ・軽い）"""
    okp = ok_path(params_path)
    if not okp.exists():
        return f"門番を通っていない（{okp.name} が無い）"
    try:
        rec = json.loads(okp.read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        return f"{okp.name} を読めない（{type(e).__name__}）"
    if rec.get("sha256") != file_hash(params_path, geom_path):
        return f"定数か寸法のファイルが門番を通った後に変わった（{okp.name} と合わない）"
    if rec.get("door") != door or abs(float(rec.get("pelvis_z", -1)) - float(pelvis_z)) > 1e-4:
        return f"門番を通した DOOR・pelvis の高さ（{rec.get('door')}・{rec.get('pelvis_z')}）と今（{door}・{pelvis_z}）が違う"
    return ""


def report(results: Sequence[PoseResult]) -> str:
    out = []
    for r in results:
        gaps = "・".join(f"{o} {100 * d:+.1f}" for o, (d, _) in r.min_gap.items() if math.isfinite(d))
        head = "○" if r.ok and not r.warn else ("△" if r.ok else "×")
        out.append(f"[gate] {head} {r.name:10s} すき間 cm: {gaps}・端まで最小 {r.margin:.2f} rad・"
                   f"棒の上 すき間 {100 * r.bar[0]:+.1f} cm・付け根から {100 * r.bar[1]:.1f} cm")
        out.extend(f"[gate]     {line}" for line in r.lines)
    return "\n".join(out)


# ── 11-12 の各回の定数（会話と作業ログ 2 章から起こした）。門番の線の答え合わせ ─────────────
HISTORY = [
    # (回, 結果の要約, 門番が落とすべきか, 定数の差分)
    ("1 回目", "手の先が棒の下に掛かった（76 °C・23 cm 後ずさり）", True,
     dict(via=((-0.12, 0.0, -0.25), (-0.12, 0.0, 0.08)), above=(0.0, 0.0, 0.08), hand_dir_deg=-1.0)),
    ("2 回目", "手の先が板に届いた（8 cm 後ずさり）", True,
     dict(via=((-0.16, 0.18, -0.25), (-0.16, 0.18, 0.12), (-0.16, 0.0, 0.12)), above=(0.0, 0.0, 0.10), hand_dir_deg=-1.0)),
    ("4 回目", "手を 60° 振った・棒の脇（台本は空振りと判定）", True,
     dict(via=((-0.16, 0.18, -0.25), (-0.16, 0.18, 0.12), (-0.16, 0.06, 0.12)), above=(-0.03, 0.06, 0.10), hand_dir_deg=60.0,
          posture=(-0.8, 1.1, -0.25, 0.4, 1.4, -1.0, 0.55))),
    ("5 回目", "手を 80° 振った・肘と手首が可動域の端に付いて手が止まった", True,
     dict(via=((-0.16, 0.18, -0.25), (-0.16, 0.18, 0.12), (-0.16, 0.05, 0.12)), above=(-0.025, 0.05, 0.10), hand_dir_deg=80.0,
          posture=(-0.85, 1.1, -0.4, 0.4, 1.7, -0.7, 1.1))),
    ("7 回目", "2d まで通った（少しかすった・許容）", False,
     dict(via=((-0.16, 0.18, -0.25), (-0.16, 0.18, 0.12), (-0.16, 0.0, 0.12)), above=(-0.10, 0.0, 0.10), hand_dir_deg=0.0)),
    ("10 回目", "扉 10° まで通った", False,
     dict(via=((-0.16, 0.18, -0.25), (-0.16, 0.18, 0.12), (-0.16, 0.03, 0.12)), above=(-0.10, 0.03, 0.10), hand_dir_deg=0.0)),
]


def run_history(geom_path: Optional[str], door: str, pelvis_z: float) -> int:
    g = dcl.load_geom(geom_path, [float(v) for v in door.split(",")])
    base = dscr.ScriptParams()
    wrong = 0
    for name, what, should_fail, diff in HISTORY:
        p = replace(base, **diff)
        # 答え合わせは構えの真ん中だけで見る（その回に機体が立った所。隅は実機で通った回でも落ちうる）
        st = dscr.script_stance(p, g.width, g.backset, g.lever_height, g.lever_left)
        ok, warn, res = check(p, g, pelvis_z, poses=[("真ん中", st.target)])
        verdict = "落とす" if not ok else ("注意つきで通す" if warn else "通す")
        right = (not ok) == should_fail
        wrong += not right
        why = next((line for r in res for line in r.lines if line.startswith("×")), "")
        print(f"[gate] {'✓' if right else '✗'} {name}（{what}）→ 門番は{verdict}{'・' + why[2:] if why else ''}")
    print(f"[gate] 答え合わせ: {len(HISTORY) - wrong}/{len(HISTORY)} が実機の結果と合う")
    return 1 if wrong else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("params", nargs="?", help="台本の定数の JSON（script_params.json）")
    ap.add_argument("--door", default="0.94,2.05,0.99,0.064", help="幅,高さ,レバーの高さ,バックセット [m]（走らせ役の DOOR と同じ文字列）")
    ap.add_argument("--pelvis-z", type=float, default=0.73)
    ap.add_argument("--geom", default=None, help="扉の周りの寸法の JSON（door_geom.json。無ければ 11-11 の巻き尺の値）")
    ap.add_argument("--lever-right", action="store_true")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--check", action="store_true", help="走らせずに <定数>.ok が今と合うかだけ見る")
    ap.add_argument("--history", action="store_true", help="11-12 の各回の定数で門番の線を答え合わせする")
    a = ap.parse_args(argv)
    if a.history:
        return run_history(a.geom, a.door, a.pelvis_z)
    if not a.params:
        ap.error("定数の JSON を渡す（--history の時は要らない）")
    if a.check:
        why = verify_ok(a.params, a.geom, a.door, a.pelvis_z)
        print(f"[gate] {'合う' if not why else '合わない: ' + why}")
        return 1 if why else 0
    p = dscr.load_params(a.params)
    g = dcl.load_geom(a.geom, [float(v) for v in a.door.split(",")], lever_left=not a.lever_right)
    t0 = time.perf_counter()
    ok, warn, res = check(p, g, a.pelvis_z, a.urdf)
    print(report(res))
    okp = ok_path(a.params)
    if ok:
        okp.write_text(json.dumps(stamp(a.params, a.geom, a.door, a.pelvis_z, warn), ensure_ascii=False, indent=1), encoding="utf-8")
        print(f"[gate] 通った{'（注意あり）' if warn else ''}: {okp} を書いた（{time.perf_counter() - t0:.1f} 秒）")
        return 0
    if okp.exists():
        okp.unlink()  # 古い .ok を残さない（落ちた定数で走らせない）
    print(f"[gate] 落ちた: {okp.name} は書かない（{time.perf_counter() - t0:.1f} 秒）")
    return 1


if __name__ == "__main__":
    sys.exit(main())
