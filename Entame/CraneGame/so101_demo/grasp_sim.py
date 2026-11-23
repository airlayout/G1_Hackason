"""接触つきの物理シミュレーション（MuJoCo）で、閉じた指がお菓子を挟んで、持ち上げたとき付いてくるかを確かめる。

crane_sim.py は「先端がお菓子の位置へ届くか」までを、運動学（接触なし）で見る。ここでは同じ手の動きの台本（crane_sim.script_for）で動かした
腕の関節の姿勢を、公式 SO-101 モデルのサーボ（位置アクチュエータ・トルク上限つき）への指令にして、机とお菓子（自由に動く箱）の接触を解く。

  .venv/bin/python grasp_sim.py                 # お菓子 x=0.25 m
  .venv/bin/python grasp_sim.py --snack-x 0.20
出力: debug/grasp-x<位置>.rrd（Rerun。腕・机・お菓子の物理の動き）と、確認結果の文章。

【前提（未測定。推測を値に使っている）】お菓子の質量・摩擦、机の摩擦は仮置き。サーボの強さはモデル付属の値（lerobot の実機の設定とは 1 対 1 でない、とモデルのコメントにある）。
お菓子の形は箱（実物は袋・箱・缶などで違う）。ここで確かめられるのは「この仮置きの値なら挟んで持ち上がる」まで。
"""
import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import rerun as rr
import rerun.blueprint as rrb

import crane_sim
import reach_map
import sim_replay as sr
import smoothing

HERE = Path(__file__).resolve().parent
SNACK_MASS = 0.03                    # kg（仮置き）
SNACK_FRICTION = (1.0, 0.005, 0.0001)  # 滑り・ねじり・転がり（仮置き）
DESK_FRICTION = (0.8, 0.005, 0.0001)
TIP_Y = 0.007                        # 先端の y（MuJoCo の順運動学で測った値。平面の逆運動学は y を無視するので、お菓子をここへ置く）
AIM_DX = -0.025                     # 先端の狙いは、お菓子の中心より手前（台座側）へ 3 cm（固定爪が手前、可動爪が奥に来る。物理の掃引で決めた値）
AIM_DZ = -0.015                      # 先端の狙いは、お菓子の中心より 1.5 cm 低く（背の高い箱の下寄りを挟む）
IMPRATIO = 5.0                       # 滑り方向の摩擦をより硬く解く（把持が滑るのを抑える。調整値）
SETTLE = 0.5                       # 初めに、お菓子が机に落ち着くまで待つ秒数
LIFT_OK = 0.02                       # 持ち上げ後、お菓子の底が机からこれ以上浮いていれば「持ち上がった」（m）
CARRY_TIP_TOL = 0.04                 # 運んだ後、お菓子の中心が先端からこの距離以内なら「付いてきた」（m）
LIFT_PHASE, RETRACT_PHASE = 6, 7     # crane_sim.PHASES の「持ち上げる」「手を戻す（前）」


def build_model(snack):
    """公式モデルに、机（固定の箱）とお菓子（自由な箱）を足す。"""
    spec = mujoco.MjSpec.from_file(str(sr.MODEL_XML))
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC       # 摩擦円錐は楕円（既定の角錐は斜めの摩擦を大きく見積もる）
    spec.option.impratio = IMPRATIO
    world = spec.worldbody
    desk = world.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.30, 0.45, 0.01], pos=[0.28, 0, reach_map.Z_DESK - 0.01],
                          rgba=[0.43, 0.43, 0.48, 0.5], friction=list(DESK_FRICTION))
    desk.name = "desk"
    body = world.add_body(name="snack", pos=list(snack))
    body.add_freejoint()
    geom = body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=list(crane_sim.SNACK_HALF), mass=SNACK_MASS,
                         rgba=[0.94, 0.59, 0.16, 1.0], friction=list(SNACK_FRICTION), condim=4)
    geom.name = "snack_box"
    return spec.compile()


def controls(model, cal, pose):
    """関節の姿勢（-100〜100）を、アクチュエータの指令（rad）にする。"""
    out = {}
    for joint in sr.ARM_JOINTS:
        out[joint] = sr.joint_radians(cal, joint, pose[joint])
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "gripper")
    low, high = model.jnt_range[gid]
    out["gripper"] = low + pose.get("gripper", 0.0) / 100 * (high - low)
    return out


def simulate(frames, snack, cal):
    """frames（crane_sim.run の結果）の姿勢を指令に、物理を進める。毎フレームの (腕の姿勢, お菓子の位置・向き, 先端) を返す。"""
    model = build_model(snack)
    data = mujoco.MjData(model)
    act = {j: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, j) for j in (*sr.ARM_JOINTS, "gripper")}
    snack_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "snack")
    sr.apply_pose(model, data, cal, frames[0]["pose"])       # 最初の姿勢から始める（立ち上がりの飛びを避ける）
    for joint, value in controls(model, cal, frames[0]["pose"]).items():
        data.ctrl[act[joint]] = value
    mujoco.mj_forward(model, data)
    for _ in range(round(SETTLE / model.opt.timestep)):
        mujoco.mj_step(model, data)
    out, last_t = [], frames[0]["t"]
    previous = controls(model, cal, frames[0]["pose"])
    for f in frames:
        current = controls(model, cal, f["pose"])
        steps = max(1, round((f["t"] - last_t or crane_sim.DT) / model.opt.timestep))
        for k in range(1, steps + 1):                    # 指令はフレーム間を直線で補う（階段状だとサーボが毎フレーム叩かれて震える）
            for joint, value in smoothing.interpolate(previous, current, k / steps).items():
                data.ctrl[act[joint]] = value
            mujoco.mj_step(model, data)
        previous, last_t = current, f["t"]
        out.append(dict(t=f["t"], phase=f["phase"], opening=f.get("opening"), tip=sr.tip_position(model, data),
                        snack=data.xpos[snack_id].copy(), snack_mat=data.xmat[snack_id].reshape(3, 3).copy(),
                        geom_xpos=data.geom_xpos.copy(), geom_xmat=data.geom_xmat.copy()))
    return model, out


def judge(steps, snack):
    """持ち上げ後・運んだ後のお菓子の様子。机の面の高さは reach_map.Z_DESK。"""
    floor = reach_map.Z_DESK + crane_sim.SNACK_HALF[2]          # 机に置いたときのお菓子の中心の高さ
    lifted = [s for s in steps if s["phase"] == LIFT_PHASE]
    carried = [s for s in steps if s["phase"] == RETRACT_PHASE]
    out = dict(moved=float(np.linalg.norm(steps[-1]["snack"][:2] - np.array(snack[:2]))))
    if lifted:
        out["lift_height"] = max(s["snack"][2] for s in lifted) - floor
    if carried:
        end = carried[-1]
        out["carry_distance"] = float(np.linalg.norm(end["snack"] - end["tip"]))
        out["carry_height"] = end["snack"][2] - floor
    out["ok"] = out.get("lift_height", 0) >= LIFT_OK and out.get("carry_distance", 1) <= CARRY_TIP_TOL
    return out


def close_open_events(steps):
    """指の開き（0〜1）が CLOSE_BELOW を下回った時刻（閉じる）と、再び上回った時刻（離す）。(種類, ステップ番号) の列。"""
    events, closed = [], False
    for i, s in enumerate(steps):
        if s.get("opening") is None:
            continue
        now = s["opening"] < crane_sim.CLOSE_BELOW
        if now != closed:
            events.append(("閉じる" if now else "離す", i))
            closed = now
    return events


def to_rrd(model, steps, snack, out, name, images=None):
    """Rerun に書き出す。images は録画から切り出したカメラ映像のフォルダ（あれば左に映像、右に 3D）。"""
    rr.init(f"grasp-sim-{name}", recording_id=f"grasp-sim-{name}")
    rr.save(str(out))
    view = rrb.Spatial3DView(origin="world", name="物理（接触つき）: 公式 SO-101 ＋ 机 ＋ お菓子（仮置き）")
    top = rrb.Horizontal(rrb.Spatial2DView(origin="camera", name="録画（カメラ）"), view) if images else view
    rr.send_blueprint(rrb.Blueprint(rrb.Vertical(top, rrb.TextLogView(origin="log", name="手の動き"), row_shares=[4, 1]),
                                    collapse_panels=True))
    sr.log_meshes(model, "world/robot", None)
    rr.log("world/desk", rr.Boxes3D(centers=[[0.28, 0, reach_map.Z_DESK - 0.01]], half_sizes=[[0.30, 0.45, 0.01]],
                                    colors=[[110, 110, 120, 120]]), static=True)
    rr.log("world/snack", rr.Boxes3D(centers=[[0, 0, 0]], half_sizes=[crane_sim.SNACK_HALF], colors=[[240, 150, 40]]), static=True)
    events = dict((i, kind) for kind, i in close_open_events(steps))
    trail, last_phase = [], None
    for i, s in enumerate(steps):
        rr.set_time("sec", duration=s["t"])
        for gid in sr.mesh_geoms(model):
            rr.log(f"world/robot/g{gid}", rr.Transform3D(translation=s["geom_xpos"][gid], mat3x3=s["geom_xmat"][gid].reshape(3, 3)))
        rr.log("world/snack", rr.Transform3D(translation=s["snack"], mat3x3=s["snack_mat"]))
        trail.append(s["snack"])
        image = sr.frame_path(images, s["t"]) if images else None
        if image:
            rr.log("camera", rr.EncodedImage(path=str(image)))
        if s["phase"] != last_phase:
            if s["phase"] or last_phase is not None:
                rr.log("log", rr.TextLog(f"手: {crane_sim.PHASES[s['phase']]}"))
            last_phase = s["phase"]
        if i in events:
            rr.log("log", rr.TextLog(f"指を{events[i]}（開き {s['opening']:.2f}）"))
    rr.log("world/snack_trail", rr.LineStrips3D([np.array(trail)], colors=[[240, 150, 40]], radii=0.0015), static=True)


def pick_close(frames, model_free_tip):
    """録画の中で「お菓子を挟みに行った」閉じ始めを選ぶ。指の開きが CLOSE_BELOW を下がる瞬間のうち、先端が机から GRASP_Z_TOL 以内・
    前後 X_MIN〜X_MAX に入る最初のもの。なければ、最も低い閉じ始め（その旨を呼び出し側が表示する）。無ければ None。
    戻り値: (フレーム番号, 条件に合ったか)。"""
    # 先頭は録画の初期状態（指が閉じたまま始まる）なので除き、開いた状態から閉じた瞬間だけを数える
    cands = [i for i, f in enumerate(frames) if i > 0 and f["opening"] < crane_sim.CLOSE_BELOW <= frames[i - 1]["opening"]]
    if not cands:
        return None
    height = lambda i: model_free_tip(frames[i])[2] - reach_map.Z_DESK
    fits = [i for i in cands if height(i) <= crane_sim.GRASP_Z_TOL and reach_map.X_MIN <= model_free_tip(frames[i])[0] <= reach_map.X_MAX + 0.04]
    return (fits[0], True) if fits else (min(cands, key=height), False)


def run_recorded(name, cal=None):
    """録画の手の入力で腕を動かし、手が最初に低く閉じた場所へお菓子を置いて、物理を進める。
    録画の手はお菓子を狙っていないので、お菓子の位置は「閉じた先端の、AIM_DX だけ奥」に後から決める（実機で測った位置ではない）。"""
    cal = cal or sr.load_calibration()
    units = reach_map.joint_units(cal)
    home = json.loads((HERE / "home.json").read_text())
    limits = json.loads((HERE / "limits.json").read_text())
    inputs, times, _ = crane_sim.recorded_inputs(name)
    frames = crane_sim.run(inputs, units, {**home, "gripper": 100.0 * inputs[0][3]}, limits, None, times,
                           free=crane_sim.recorded_free(name))
    probe = mujoco.MjModel.from_xml_path(str(sr.MODEL_XML))
    probe_data = mujoco.MjData(probe)

    def tip_of_frame(f):
        sr.apply_pose(probe, probe_data, cal, f["pose"])
        return sr.tip_position(probe, probe_data)

    chosen = pick_close(frames, tip_of_frame)
    if chosen is None:
        return None
    index, fits = chosen
    tip = tip_of_frame(frames[index])
    snack = (tip[0] - AIM_DX, tip[1], reach_map.Z_DESK + crane_sim.SNACK_HALF[2] + 0.0005)
    model, steps = simulate(frames, snack, cal)
    return dict(model=model, steps=steps, snack=snack, close_index=index, close_fits=fits, close_tip=tip)


def judge_recorded(steps, snack, close_index):
    """録画駆動の結果。閉じ始めの前にお菓子が動いたか、閉じている間に持ち上がったか、先端に付いてきたか、離した後はどうか。"""
    floor = reach_map.Z_DESK + crane_sim.SNACK_HALF[2]
    events = close_open_events(steps)
    t_close = steps[close_index]["t"]
    opens = [i for kind, i in events if kind == "離す" and i > close_index]
    end = opens[0] if opens else len(steps) - 1
    held = steps[close_index:end + 1]
    out = dict(knocked=float(np.linalg.norm(steps[close_index]["snack"][:2] - np.array(snack[:2]))),
               hold_seconds=steps[end]["t"] - t_close, released=bool(opens),
               lift_height=max(s["snack"][2] for s in held) - floor,
               carry_distance=float(np.linalg.norm(steps[end]["snack"] - steps[end]["tip"])),
               moved=float(np.linalg.norm(steps[-1]["snack"][:2] - np.array(snack[:2]))))
    out["ok"] = out["lift_height"] >= LIFT_OK and out["carry_distance"] <= CARRY_TIP_TOL
    return out


def run_script(snack_x, cal=None):
    cal = cal or sr.load_calibration()
    units = reach_map.joint_units(cal)
    home = json.loads((HERE / "home.json").read_text())
    limits = json.loads((HERE / "limits.json").read_text())
    snack = (snack_x, TIP_Y, reach_map.Z_DESK + crane_sim.SNACK_HALF[2] + 0.0005)
    anchor = reach_map.tip_of(home, units)[0]
    aim = (snack[0] + AIM_DX, snack[1], snack[2] + AIM_DZ)
    frames = crane_sim.run(crane_sim.scripted_inputs(crane_sim.script_for(aim, anchor)), units,
                           {**home, "gripper": 100.0 * crane_sim.OPEN}, limits, aim)
    model, steps = simulate(frames, snack, cal)
    return model, steps, snack


def report_script(result, snack_x):
    print(f"[grasp_sim] お菓子 x={snack_x:.2f} m（質量 {SNACK_MASS * 1000:.0f} g・摩擦 {SNACK_FRICTION[0]}、仮置き）")
    print(f"持ち上げ中の、お菓子の机からの浮き {result.get('lift_height', float('nan')) * 100:+.1f} cm（{LIFT_OK * 100:.0f} cm 以上で持ち上がった）")
    print(f"手を戻した後の、お菓子と先端の距離 {result.get('carry_distance', float('nan')) * 100:.1f} cm / 机からの浮き {result.get('carry_height', float('nan')) * 100:+.1f} cm"
          f"（先端から {CARRY_TIP_TOL * 100:.0f} cm 以内なら付いてきた）")
    print(f"お菓子が机の上で水平に動いた距離（最終）{result['moved'] * 100:.1f} cm")
    print("→", "挟んで持ち上げ、運べた" if result["ok"] else "挟んで運ぶことはできなかった")


def report_recorded(name, run, result):
    snack = run["snack"]
    print(f"[grasp_sim] 録画 {name}: お菓子を閉じた先端の奥 x={snack[0]:.2f} y={snack[1]:+.3f} に置いた（仮置き・質量 {SNACK_MASS * 1000:.0f} g・摩擦 {SNACK_FRICTION[0]}）")
    print(f"選んだ閉じ始め: t={run['steps'][run['close_index']]['t']:.1f}s / 先端 x={run['close_tip'][0]:.2f} 机から {(run['close_tip'][2] - reach_map.Z_DESK) * 100:.1f} cm"
          f" → {'条件（低く・届く範囲）に合う' if run['close_fits'] else '【条件に合うものが無く、最も低い閉じ始めを使った】'}")
    print(f"閉じ始めの前に、お菓子が水平に動いた距離 {result['knocked'] * 100:.1f} cm（腕が当たった目安）")
    print(f"閉じていた時間 {result['hold_seconds']:.1f} s（{'その後、指を開いた' if result['released'] else '録画の最後まで閉じたまま'}）")
    print(f"閉じている間の、お菓子の机からの浮き 最大 {result['lift_height'] * 100:+.1f} cm（{LIFT_OK * 100:.0f} cm 以上で持ち上がった）")
    print(f"開く直前（または最後）の、お菓子と先端の距離 {result['carry_distance'] * 100:.1f} cm（{CARRY_TIP_TOL * 100:.0f} cm 以内なら付いてきた）")
    print("→", "挟んで持ち上げた" if result["ok"] else "持ち上げられなかった（この録画の手の動きでは）")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snack-x", type=float, default=0.25)
    ap.add_argument("--log", help="録画の名前（例: 20261122-221606-624）。省略すると台本")
    ap.add_argument("--frames", type=Path, help="録画から切り出したフレーム（0001.jpg から連番、10fps）のフォルダ。--log と併用")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    if args.log:
        run = run_recorded(args.log)
        if run is None:
            print(f"[grasp_sim] 録画 {args.log}: 指を閉じる動きが無いので、物理を動かしません")
            return
        result = judge_recorded(run["steps"], run["snack"], run["close_index"])
        out = args.out or sr.DEBUG / f"grasp-{args.log}.rrd"
        to_rrd(run["model"], run["steps"], run["snack"], out, args.log.replace("-", "_"), args.frames)
        report_recorded(args.log, run, result)
    else:
        model, steps, snack = run_script(args.snack_x)
        result = judge(steps, snack)
        out = args.out or sr.DEBUG / f"grasp-x{args.snack_x:.2f}.rrd"
        to_rrd(model, steps, snack, out, f"x{args.snack_x:.2f}".replace(".", "p"))
        report_script(result, args.snack_x)
    print(f"[grasp_sim] 書き出しました: {out}")


if __name__ == "__main__":
    main()
