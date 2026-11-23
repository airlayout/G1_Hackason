"""机の上のお菓子を置いたシミュレーション。人の手の動き（前へ伸ばす・高さ・指の開き）から、reach_map の逆運動学で腕を動かし、
グリッパーの先端がお菓子の所へ届くか、肩が後ろ上へ振り上がらないかを、MuJoCo の公式 SO-101 モデルで確かめる。

  .venv/bin/python crane_sim.py                      # 手の動きは、人が「上げる→伸ばす→下ろす→つまむ→持ち上げる→戻す」を想定した台本（合成。録画ではない）
  .venv/bin/python crane_sim.py --log <名前>         # 録画（debug/follow-<名前>.jsonl）の手の入力をそのまま使う
  .venv/bin/python crane_sim.py --snack-x 0.25       # お菓子の位置を変える（台座から前へ m）
出力: debug/crane-<名前>.rrd（Rerun で開く）と、確認結果の文章。

【前提（未測定。推測を値に使っている）】お菓子の位置・大きさは仮置き。机の高さは、初期姿勢（グリッパーが机に接した姿勢）の先端の高さ。
つかめるか（指の力・滑り）は物理では確かめていない。確かめるのは「先端がお菓子の位置へ、道具を下に向けて届く」まで。
"""
import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import rerun as rr
import rerun.blueprint as rrb

import reach_map
import sim_replay as sr
from ik_follow import COMMAND_TIME_CONSTANT, INPUT_BETA, INPUT_MIN_CUTOFF     # 実機サーバーの IK 追従と同じ値
from smoothing import CriticallyDamped, OneEuro
from home import neutral_map, relative
from limits import speed_raw
from replay import SimArm

HERE = Path(__file__).resolve().parent
DT = 1 / 15
MAX_DT = 0.25                                # 録画のサンプルが途切れたときの、1 ステップで進める時間の上限（秒）
SIDE = 0.25                                  # 録画と新しい腕を並べるときの、左右へずらす距離（m）
SNACK_X, SNACK_Y = 0.30, 0.0                 # 仮置き（未測定）
SNACK_HALF = (0.015, 0.015, 0.025)           # 3 × 3 × 5 cm（仮置き。爪のすき間 3〜4.5 cm に入り、背が高く挟みやすい形）
GRASP_XY_TOL, GRASP_Z_TOL = 0.015, 0.03      # 閉じ始めの先端がこの範囲ならお菓子の位置に届いたとする
CLOSE_BELOW = 0.25                           # 指の開き（0〜1）がこれ未満＝閉じる指令
SHOULDER_BACK_LIMIT = reach_map.USABLE["shoulder_lift"][0]
DEADBAND_MIN = 0.3                           # これ未満の差は押し込まない（揺れ防止）
OPEN = 1.0                                  # 台本で指を開くときの開き（0〜1）。公開データ lerobot/svla_so101_pickplace は約 0.25（関節値 20〜27）で開くが、校正が違い、3 cm の箱では 0.5 以下だと挟み損ねた（物理の掃引 2026-11-22）。全開が最も広く成功
HAND_UP = 0.6                              # 持ち上げる・運ぶときの、手の高さ（基準から。-1〜1）
PHASES = ["静止", "手を上げる", "手を前へ伸ばす", "手を下ろす", "狙いを定める（静止）", "指を閉じる", "持ち上げる", "手を戻す（前）", "手を下ろす（戻り）"]


def input_for(value, low, high, base):
    """reach_map.neutral の逆: 先端を value にする手の入力（-1〜1）。"""
    base = min(high, max(low, base))
    return (value - base) / ((high - base) if value >= base else (base - low) or 1.0)


def script_for(snack, anchor):
    """お菓子の位置へ先端を持っていく、人の手の動きの台本。(秒数, 前へ伸ばす, 高さ, 指の開き) の区間の終わりの値。区間の間は直線で変える。
    基準（動き出す前）が 0。人が上手に狙った場合を想定した合成で、録画ではない。"""
    reach = input_for(snack[0], reach_map.X_MIN, reach_map.X_MAX, anchor[0])
    low = input_for(snack[2], reach_map.Z_MIN, reach_map.Z_MAX, anchor[1])      # 先端がお菓子の中心の高さになる手の高さ
    o = OPEN
    return [(1.0, 0, 0, o), (2.0, 0, HAND_UP, o), (3.0, reach, HAND_UP, o), (2.0, reach, low, o), (1.0, reach, low, o),
            (1.0, reach, low, 0), (2.0, reach, HAND_UP, 0), (2.0, 0, HAND_UP, 0), (1.0, 0, 0, 0)]


def scripted_inputs(script):
    """台本を DT ごとの (相, 伸ばす, 高さ, 開き) にする。"""
    last, out = (0, 0, OPEN), []
    for phase, (length, reach, height, opening) in enumerate(script):
        for i in range(round(length / DT)):
            f = (i + 1) / round(length / DT)
            out.append((phase, *(a + (b - a) * f for a, b in zip(last, (reach, height, opening)))))
        last = (reach, height, opening)
    return out


def recorded_inputs(name):
    """録画の手の入力を、動き出す前の手の構え（最初の約 0.3 秒の平均）を基準にして、台本と同じ形にする。
    (入力の列, 各サンプルの時刻 [s]（録画の最初の行から）, そのときの実機の実測の姿勢) を返す。"""
    rows = [json.loads(line) for line in (sr.DEBUG / f"follow-{name}.jsonl").read_text().splitlines()]
    t0 = rows[0]["ts"]
    samples = [r for r in rows if r.get("ev") == "sample" and r.get("inp") and r.get("cur")]
    base_reach = sum(r["inp"]["reach"] for r in samples[:5]) / 5
    base_height = sum(-r["inp"]["lift"] for r in samples[:5]) / 5
    inputs = [(0, relative(r["inp"]["reach"], base_reach), relative(-r["inp"]["lift"], base_height), r["inp"]["opening"])
              for r in samples]
    return inputs, [r["ts"] - t0 for r in samples], [r["cur"] for r in samples]


FREE_KEYS = {"shoulder_pan": "pan", "wrist_roll": "roll"}     # 録画の入力の項目名（実機サーバーの FREE_INPUTS と同じ）


def recorded_free(name, joints=("shoulder_pan", "wrist_roll")):
    """録画の台座（pan）・手首の回転（roll）の入力を、動き出す前の構えを 0 にした -1〜1 にして、recorded_inputs と同じ並びで返す。
    {関節: 入力の列}。注意：実機の録画では追従の対象ではなかった関節（例: roll）の入力も、ここでは追従したものとして動かす。"""
    rows = [json.loads(line) for line in (sr.DEBUG / f"follow-{name}.jsonl").read_text().splitlines()]
    samples = [r for r in rows if r.get("ev") == "sample" and r.get("inp") and r.get("cur")]
    out = {}
    for joint in joints:
        key = FREE_KEYS[joint]
        base = sum(r["inp"].get(key, 0.0) for r in samples[:5]) / 5
        out[joint] = [relative(r["inp"].get(key, 0.0), base) for r in samples]
    return out


def free_target(joint, rel, home_value, limits):
    """台座・手首の回転の入力（-1〜1）を、実機サーバーと同じ写し方（home.neutral_map）で、関節の目標（-100〜100）にする。"""
    lo, hi = (2 * limits[joint][k] - 100 for k in ("min", "max"))
    return neutral_map(rel, lo, hi, home_value)


def make_arm(home, limits):
    arm = SimArm(home)
    percent = lambda lo, hi: dict(min=(lo + 100) / 2, max=(hi + 100) / 2)
    arm.limits = {j: dict(limits[j], **(percent(*reach_map.USABLE[j]) if j in reach_map.USABLE else {})) for j in limits}
    return arm


def push_through_deadband(goal, arm, units):
    """サーボの不感帯（replay.DEAD）の中で目標の手前に止まらないよう、不感帯に入った関節だけ、不感帯の分だけ余分に指令する。
    補正なしだと先端が最大 2 cm ずれる（実機ログから当てはめた不感帯での計算）。実機で効くかは未確認の仮定。
    押し込んだ指令が包絡（高さ・向き・肩）の外になる関節は、押し込まない。"""
    pose, out = arm.read(), dict(goal)
    for joint in reach_map.USABLE:
        error = goal[joint] - pose[joint]
        dead = arm.dead[joint]
        dead = (dead[0] if error > 0 else dead[1]) if isinstance(dead, tuple) else dead
        if DEADBAND_MIN < abs(error) < dead:
            pushed = {**out, joint: goal[joint] + math.copysign(dead, error)}
            if reach_map.inside_envelope(pushed, units):
                out = pushed
    return out


def run(inputs, units, home, limits, snack, times=None, smooth=True, free=None):
    """入力を流して腕を動かす。毎ステップの (関節の姿勢, 先端, 道具の向き, 相, 開き, 時刻) を返す。
    times（各入力の時刻 [s]）があれば、その間隔で時間を進める（録画は約 0.09 秒間隔）。なければ DT ごと。
    free（{関節: 入力の列 -1〜1}。recorded_free）があれば、台座・手首の回転もそれで動かす（なければ基準のまま）。
    smooth=True（既定）: 手の入力（伸ばす・高さ）を One Euro で、関節の指令を臨界減衰で滑らかにする（ブルブルを抑える。2026-11-22）。"""
    arm = make_arm(home, limits)
    anchor = reach_map.tip_of(home, units)[0]
    goal = {j: float(home[j]) for j in home}
    safe = {j: float(home[j]) for j in reach_map.USABLE}
    in_filters = [OneEuro(INPUT_MIN_CUTOFF, INPUT_BETA) for _ in range(2 + len(free or {}))]
    cmd_filters = {j: CriticallyDamped(COMMAND_TIME_CONSTANT) for j in (*reach_map.USABLE, *(free or {}))}
    frames = []
    for i, (phase, reach, height, opening) in enumerate(inputs):
        dt = DT if times is None or i == 0 else min(MAX_DT, max(1e-3, times[i] - times[i - 1]))
        if smooth:
            reach, height = (f(v, dt if i else 0.0) for f, v in zip(in_filters, (reach, height)))
        (x, z), want = reach_map.target(reach, height, anchor, units, safe)
        for joint, value in want.items():
            step = speed_raw(joint, arm.limits) * dt
            goal[joint] += max(-step, min(step, value - goal[joint]))
        for k, (joint, series) in enumerate((free or {}).items()):
            rel = in_filters[2 + k](series[i], dt if i else 0.0) if smooth else series[i]
            value = free_target(joint, rel, float(home[joint]), limits)
            step = speed_raw(joint, arm.limits) * dt
            goal[joint] += max(-step, min(step, value - goal[joint]))
        goal["gripper"] = 100.0 * opening
        command = push_through_deadband(goal, arm, units)
        if smooth:
            command = {**command, **{j: cmd_filters[j](command[j], dt) for j in cmd_filters}}
        arm.write(command)
        arm.advance(dt)
        pose = arm.read()
        tip, pitch = reach_map.tip_of(pose, units)
        frames.append(dict(pose=dict(pose), tip=tip, pitch=pitch, phase=phase, opening=opening, want_tip=(x, z),
                           t=times[i] if times else i * DT))
    return frames


def judge(frames, snack):
    """確認結果。閉じ始め＝開きが CLOSE_BELOW を下回った最初のステップ。"""
    close = next((f for f in frames if f["opening"] < CLOSE_BELOW), None)
    shoulders = [f["pose"]["shoulder_lift"] for f in frames]
    sx, sz = snack[0], snack[2]
    nearest = min(frames, key=lambda f: math.hypot(f["tip"][0] - sx, f["tip"][1] - sz))
    out = dict(shoulder_min=min(shoulders), shoulder_max=max(shoulders),
               tip_x=(min(f["tip"][0] for f in frames), max(f["tip"][0] for f in frames)),
               tip_z=(min(f["tip"][1] for f in frames), max(f["tip"][1] for f in frames)),
               height_max=max(f["tip"][1] for f in frames) - reach_map.Z_DESK,
               pitch=(min(f["pitch"] for f in frames), max(f["pitch"] for f in frames)),
               nearest_dist=math.hypot(nearest["tip"][0] - sx, nearest["tip"][1] - sz))
    if close:
        height = close["tip"][1] - reach_map.Z_DESK        # 机からの先端の高さ
        out.update(close_dx=close["tip"][0] - sx, close_height=height, close_pitch=close["pitch"],
                   reached=abs(close["tip"][0] - sx) <= GRASP_XY_TOL and height <= GRASP_Z_TOL and close["pitch"] < -60)
    return out


def blueprint(with_camera, side_by_side):
    title = ("左: 録画の実機の動き（関節ごとに手へ追従）/ 右: 新しい動き（先端の位置を決めて逆運動学）" if side_by_side
             else "SO-101（公式モデル）＋机＋お菓子（仮置き）")
    view = rrb.Spatial3DView(origin="world", name=title)
    top = rrb.Horizontal(rrb.Spatial2DView(origin="camera", name="録画（カメラ）"), view) if with_camera else view
    return rrb.Blueprint(rrb.Vertical(top, rrb.TextLogView(origin="log", name="手の動き"), row_shares=[4, 1]),
                         collapse_panels=True)


def log_scene(snack, side_by_side):
    rr.log("world/desk", rr.Boxes3D(centers=[[0.28, 0, reach_map.Z_DESK - 0.01]], half_sizes=[[0.30, 0.45, 0.01]],
                                    colors=[[110, 110, 120, 120]]), static=True)
    if snack is not None:
        rr.log("world/snack", rr.Boxes3D(centers=[snack], half_sizes=[SNACK_HALF], colors=[[240, 150, 40]]), static=True)
    for y in ((-SIDE, SIDE) if side_by_side else (0.0,)):
        rr.log(f"world/axes_{y:+.2f}", rr.Arrows3D(origins=[[0, y, 0]] * 3, vectors=[[.08, 0, 0], [0, .08, 0], [0, 0, .08]],
                                                  colors=[[255, 60, 60], [60, 200, 60], [60, 60, 255]]), static=True)


def to_rrd(frames, snack, cal, out, name, recorded=None, images=None):
    """Rerun に書き出す。recorded（録画の実機の姿勢の列）があれば左に実機の動き、右に新しい動きを並べる。images はカメラ映像のフレームの
    フォルダ。戻り値は、MuJoCo の公式モデルで測った先端（IK が使う平面の運動学との差・机からの最大高さ）。"""
    model = mujoco.MjModel.from_xml_path(str(sr.MODEL_XML))
    data, rec_data = mujoco.MjData(model), mujoco.MjData(model)
    rr.init(f"crane-sim-{name}", recording_id=f"crane-sim-{name}")
    rr.save(str(out))
    rr.send_blueprint(blueprint(images is not None, recorded is not None))
    sr.log_meshes(model, "world/robot", None)
    side = recorded is not None
    if side:
        sr.log_meshes(model, "world/recorded", None)
        rr.log("world/recorded", rr.Transform3D(translation=[0, -SIDE, 0]), static=True)
        rr.log("world/robot", rr.Transform3D(translation=[0, SIDE, 0]), static=True)
        rr.log("world/label_recorded", rr.Points3D([[0.1, -SIDE, 0.32]], labels=["録画の実機（関節ごと）"], radii=0.004), static=True)
        rr.log("world/label_new", rr.Points3D([[0.1, SIDE, 0.32]], labels=["新（先端を決める）"], radii=0.004), static=True)
    log_scene(snack, side)
    trail, rec_trail, last_phase = [], [], None
    worst_gap, height_max = 0.0, -1.0
    for i, f in enumerate(frames):
        rr.set_time("sec", duration=f["t"])
        sr.apply_pose(model, data, cal, f["pose"])
        sr.log_transforms(model, data, "world/robot")
        tip = sr.tip_position(model, data)
        trail.append(tip + np.array([0, SIDE if side else 0, 0]))
        worst_gap = max(worst_gap, math.hypot(math.hypot(tip[0], tip[1]) - f["tip"][0], tip[2] - f["tip"][1]))
        height_max = max(height_max, tip[2] - reach_map.Z_DESK)
        if side:
            sr.apply_pose(model, rec_data, cal, recorded[i])
            sr.log_transforms(model, rec_data, "world/recorded")
            rec_trail.append(sr.tip_position(model, rec_data) + np.array([0, -SIDE, 0]))
        image = sr.frame_path(images, f["t"]) if images else None
        if image:
            rr.log("camera", rr.EncodedImage(path=str(image)))
        if f["phase"] != last_phase:
            rr.log("log", rr.TextLog(f"手: {PHASES[f['phase']]}"))
            last_phase = f["phase"]
    rr.log("world/tip_trail", rr.LineStrips3D([np.array(trail)], colors=[[40, 120, 220]], radii=0.0015), static=True)
    if rec_trail:
        rr.log("world/tip_trail_recorded", rr.LineStrips3D([np.array(rec_trail)], colors=[[255, 140, 30]], radii=0.0015), static=True)
    return dict(planar_vs_mujoco_gap=worst_gap, mujoco_height_max=height_max)


def report(result, with_snack=True):
    """with_snack=False は録画（手がお菓子を狙っていない）。お菓子の判定は出さない。"""
    print(f"肩の関節値 {result['shoulder_min']:+.0f}〜{result['shoulder_max']:+.0f}（使ってよい下限 {SHOULDER_BACK_LIMIT:+.0f}）"
          f" → {'後ろ上へ振り上がらない' if result['shoulder_min'] >= SHOULDER_BACK_LIMIT - 1 else '【異常】後ろ上へ振り上がった'}")
    print(f"先端の動き 前後 x={result['tip_x'][0]:.2f}〜{result['tip_x'][1]:.2f} m / 高さ z={result['tip_z'][0]:+.2f}〜{result['tip_z'][1]:+.2f} m")
    print(f"先端の机からの最大高さ {result['height_max'] * 100:.1f} cm（上限 {reach_map.HEIGHT_MAX * 100:.0f} cm）"
          f" / 道具の向き {result['pitch'][0]:+.0f}〜{result['pitch'][1]:+.0f}°（許す範囲 {reach_map.PITCH_RANGE[0]:+.0f}〜{reach_map.PITCH_RANGE[1]:+.0f}°）")
    if not with_snack:
        return
    print(f"先端がお菓子の中心に最も近づいた距離 {result['nearest_dist'] * 100:.1f} cm")
    if "reached" in result:
        print(f"閉じ始め: お菓子との前後差 {result['close_dx'] * 100:+.1f} cm / 机からの先端の高さ {result['close_height'] * 100:.1f} cm"
              f" / 道具の向き {result['close_pitch']:+.0f}° → {'届いた（位置・向き）' if result['reached'] else '届かなかった'}")
    else:
        print("指を閉じる動きは入力にありませんでした")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", help="録画の名前（例: 20261122-221606-624）。省略すると台本")
    ap.add_argument("--snack-x", type=float, default=SNACK_X)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--frames", type=Path, help="録画から切り出したフレーム（0001.jpg から連番、10fps）のフォルダ。--log と併用")
    args = ap.parse_args()
    cal = sr.load_calibration()
    units = reach_map.joint_units(cal)
    home = json.loads((HERE / "home.json").read_text())
    limits = json.loads((HERE / "limits.json").read_text())
    snack = (args.snack_x, SNACK_Y, reach_map.Z_DESK + SNACK_HALF[2])
    anchor = reach_map.tip_of(home, units)[0]
    recorded = times = None
    if args.log:
        inputs, times, recorded = recorded_inputs(args.log)
    else:
        inputs = scripted_inputs(script_for(snack, anchor))
    frames = run(inputs, units, {**home, "gripper": 100.0}, limits, snack, times)
    name = args.log or f"script-x{args.snack_x:.2f}"
    out = args.out or sr.DEBUG / f"crane-{name}.rrd"
    check = to_rrd(frames, None if args.log else snack, cal, out, name, recorded, args.frames)
    print(f"[crane_sim] {name}: 入力 {len(inputs)} ステップ（{'録画' if args.log else '台本（合成）'}）、お菓子 x={snack[0]:.2f} m")
    report(judge(frames, snack), with_snack=not args.log)
    print(f"MuJoCo 公式モデルとの照合: 先端の差 最大 {check['planar_vs_mujoco_gap'] * 1000:.1f} mm / 机からの最大高さ "
          f"{check['mujoco_height_max'] * 100:.1f} cm（上限 {reach_map.HEIGHT_MAX * 100:.0f} cm）")
    print(f"[crane_sim] 書き出しました: {out}")


if __name__ == "__main__":
    main()
