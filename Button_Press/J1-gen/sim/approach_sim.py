"""立つ位置まで寄る（common/approach.py）を、Yada の評価環境（MuJoCo）で通す。寄ったあと、その位置で押して点灯するかまで見る。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/approach_sim.py --trials 20                     # ラボの車いす用、乱しなし（basic）
    $P Button_Press/J1-gen/sim/approach_sim.py --trials 20 --set realistic     # 深度の雑音・カメラの取り付けのずれ・揺れなど
    $P Button_Press/J1-gen/sim/approach_sim.py --trials 5 --no-press           # 寄るところだけ（速い）

評価環境は腰（pelvis）を固定していて歩けない。そこで、歩いたぶん乗り場のほうを逆に動かす（評価セット realistic の
「立ち位置のずれ」stance_xy / stance_yaw の口を使い、1 回動くたびにモデルを作り直す）。歩く量は偽物の足（common/loco.py の
FakeLoco。進む向きが −25〜+5° ずれ、動く量は指令の 60〜90%、ばらつき 15%。試行ごとに乱数で決める）。頭カメラの画像・深度、
▲▼ の検出、柱の面、押せるかの計画（PlanFeasibility）は本物（評価環境の画像と、J1-gen の部品そのもの）。

置き方（ラボの車いす用、--scene lab）: 柱の面まで 0.7〜1.2 m、▲▼ の中点が体の左右 −25〜+15 cm、向き ±10°（乱数）。
押す組は wheelchair（車いす用の ▲▼。高さは実機と同じ pelvis から ▲ +0.21・▼ +0.14 m。sim/record_trial_video.py と同じ）。
押す: ready になったら、その位置のモデルで今までのエージェント（contest_agent。J1GEN_ARM に ready の腕）を評価環境の判定にかける。

保存: _local/button_press_yada/approach_sim/<時刻>/summary.json と、観測ごとの頭カメラの画像（trial<N>_step<K>.png）。
"""

from __future__ import annotations

import argparse
import copy
import dataclasses
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

FEATURE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = FEATURE_DIR.parents[1]
YADA_DIR = FEATURE_DIR.parent / "Yada"
OUT_DIR = REPO_ROOT / "_local" / "button_press_yada" / "approach_sim"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--seed0", type=int, default=0, help="最初の試行の乱数の種（試行 i は seed0 + i）")
    ap.add_argument("--set", default="basic", choices=["basic", "realistic"], help="評価セット（乱しの有無）")
    ap.add_argument("--scene", default="lab", choices=["lab"], help="置くボタン（今はラボの車いす用だけ）")
    ap.add_argument("--arms", nargs="+", choices=["right", "left"], default=None, help="計画を試す腕（既定は approach.yaml）")
    ap.add_argument("--no-press", action="store_true", help="寄るところだけ（押さない）")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    # Yada の評価環境は自分のフォルダを import の検索先に入れて使う。J1-gen の部品は j1gen_bridge で別名で読む
    sys.path.insert(0, str(YADA_DIR))
    sys.path.insert(0, str(FEATURE_DIR / "sim"))
    from common.config import load_config
    from common.j1gen_bridge import j1gen, j1gen_config
    from common.realism import sample_realism
    from contest.interface import load_agent
    from contest.robots.mujoco_robot import MujocoRobot
    from contest.runner import run_episode
    from contest.task import make_trial
    from record_trial_video import _place_lab_wheelchair

    approach = j1gen("approach")
    loco = j1gen("loco")
    column_face = j1gen("column_face")
    elevator_buttons = j1gen("elevator_buttons")
    robot_model = j1gen("robot_model")

    contest_cfg = load_config("contest.yaml")
    acfg = j1gen_config("approach.yaml")
    app = copy.deepcopy(acfg["approach"])
    if args.arms:
        app["arms"] = args.arms
    elevator = j1gen_config("elevator.yaml")
    elevator["geometry"]["button_set"] = "wheelchair"
    robot_cfg = j1gen_config("robot.yaml")
    os.environ["J1GEN_BUTTON_SET"] = "wheelchair"

    out = Path(args.out) if args.out else OUT_DIR / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    feas = approach.PlanFeasibility.from_config(robot_cfg, j1gen_config("press.yaml"), j1gen_config("arm.yaml"),
                                                elevator, list(app["arms"]), "up")
    face_finder = column_face.FaceFinder.from_config(robot_cfg, acfg["face"])
    button_finder = elevator_buttons.ButtonFinder.from_config(robot_cfg, elevator)

    rows = []
    for i in range(args.trials):
        seed = args.seed0 + i
        rng = np.random.default_rng(1000 + seed)
        trial = make_trial(seed, contest_cfg, eval_set=args.set)
        target = "up" if rng.random() < 0.5 else "down"
        trial = dataclasses.replace(trial, target=target, instruction=contest_cfg["instructions"][target][0])
        d0, y0, yaw0 = rng.uniform(0.7, 1.2), rng.uniform(-0.25, 0.15), np.radians(rng.uniform(-10, 10))
        scene = copy.deepcopy(trial.scene_cfg)
        _place_lab_wheelchair(scene, d0, y0)
        base_realism = trial.realism
        if base_realism is None:
            base_realism = quiet_realism(sample_realism(seed, contest_cfg["realism"]), MujocoRobot, scene, contest_cfg,
                                         robot_model)
        fake = loco.FakeLoco(rate_ratio=rng.uniform(0.6, 0.9), heading_error_deg=rng.uniform(-25, 5), noise=0.15,
                             seed=seed)
        fake.pose[2] = yaw0
        feas.target = target

        state: dict = {"pose": None, "robot": None, "t": 0.0, "step": 0, "saved": -1}

        def robot_at(pose: np.ndarray) -> MujocoRobot:
            r = dataclasses.replace(base_realism, **hall_offset(pose, base_realism))
            return MujocoRobot(scene, contest_cfg, realism=r)

        def get_frame():
            if state["pose"] is None or not np.allclose(state["pose"], fake.pose):
                if state["robot"] is not None:
                    state["robot"].close()
                state["pose"] = fake.pose.copy()
                state["robot"] = robot_at(fake.pose)
                state["robot"].advance(float(contest_cfg.get("settle_s", 0.0)))
                state["t"] = 0.0
                tr = true_rel(fake.pose, d0, y0)
                print(f"  （本当の値: 柱まで {tr['dist_m'] * 100:.1f} cm、ボタン {tr['side_m'] * 100:+.1f} cm、向き {tr['yaw_deg']:+.1f}°）")
            r = state["robot"]
            ob = r.observe(state["t"])
            r.advance(0.1)
            state["t"] += 0.1
            if state["saved"] != state["step"]:
                cv2.imwrite(str(out / f"trial{seed}_step{state['step']}.png"), ob.rgb[:, :, ::-1])
                state["saved"] = state["step"]
            return approach.Frame(ob.rgb, ob.depth, ob.K, ob.q, ob.imu_quat)

        perception = approach.HeadPerception(get_frame, button_finder, face_finder, elevator, acfg["face"], app)
        orig_reset = perception.reset

        def reset() -> None:
            state["step"] += 1
            orig_reset()

        perception.reset = reset
        face_finder.reset()
        mover = loco.LocoMover(fake, fake, acfg["loco"], go=True, sleep=fake.sleep, now=fake.now, log=lambda s: None)
        t0 = time.time()
        try:
            res = approach.Approacher(perception, feas, mover, app, log=lambda s: print(f"  {s}")).run()
        except Exception:  # 1 回の失敗で全体を止めない（記録して次の試行へ）
            import traceback

            tb = traceback.format_exc()
            print(tb)
            res = approach.ApproachResult("error", tb.strip().splitlines()[-1], None, [])
        row = {"seed": seed, "target": target, "start": {"dist_m": d0, "side_m": y0, "yaw_deg": float(np.degrees(yaw0))},
               "legs": {"rate_ratio": fake.rate_ratio, "heading_error_deg": float(np.degrees(fake.heading_error))},
               "outcome": res.outcome, "reason": res.reason, "arm": res.arm, "moves": len(fake.calls),
               "steps": res.steps, "approach_s": round(time.time() - t0, 1), "true_final": true_rel(fake.pose, d0, y0)}
        if res.outcome == "ready" and not args.no_press:
            state["robot"].close()
            os.environ["J1GEN_ARM"] = res.arm
            robot = robot_at(fake.pose)
            try:
                ep = run_episode(robot, load_agent(str(FEATURE_DIR / "contest_agent")), trial, contest_cfg)
            finally:
                robot.close()
            state["robot"] = None
            row["press"] = ep.outcome
            row["press_time_s"] = ep.time_s
            if ep.error:
                row["press_error"] = ep.error[-500:]
        elif state["robot"] is not None:
            state["robot"].close()
        rows.append(row)
        tf = row["true_final"]
        print(f"[approach_sim] 試行 {seed}（{target}）: 柱まで {d0 * 100:.0f} cm・左右 {y0 * 100:+.0f} cm・向き {np.degrees(yaw0):+.0f}° から"
              f" → {res.outcome}（{len(fake.calls)} 回動いた、着いた位置: 柱まで {tf['dist_m'] * 100:.1f} cm・ボタン {tf['side_m'] * 100:+.1f} cm）"
              + (f" → 押した: {row['press']}" if "press" in row else "") + f"。{res.reason[:100]}")

    summary = summarize(rows, app)
    (out / "summary.json").write_text(json.dumps({"args": vars(args), "summary": summary, "trials": rows}, indent=2,
                                                 ensure_ascii=False, default=float))
    print(f"[approach_sim] まとめ: {json.dumps(summary, ensure_ascii=False)}")
    print(f"[approach_sim] 保存: {out}")
    return 0


def hall_offset(pose: np.ndarray, base) -> dict:
    """ロボットが最初の位置から pose = (x, y, yaw) 歩いたときの、乗り場のずらし（stance_xy / stance_yaw）。

    乗り場の点 p（最初の pelvis の座標）は、歩いたあとの pelvis の座標で R(−yaw)(p − t)。乗り場の体は「pelvis + o を中心に
    yaw_h 回す」なので、yaw_h = −yaw、o = −R(−yaw) t。評価セットのずれ（base）があれば、その外側に重ねる。
    """
    x, y, yaw = (float(v) for v in pose)
    c, s = np.cos(yaw), np.sin(yaw)
    o = -np.array([c * x + s * y, -s * x + c * y])
    yh = -yaw
    ob = np.asarray(base.stance_xy, dtype=float)
    ch, sh = np.cos(yh), np.sin(yh)
    o = o + np.array([ch * ob[0] - sh * ob[1], sh * ob[0] + ch * ob[1]])
    return {"stance_xy": o, "stance_yaw": yh + float(base.stance_yaw)}


def true_rel(pose: np.ndarray, d0: float, y0: float) -> dict:
    """歩いたあとの、本当の柱までの距離・ボタンの左右（体の座標）・向き（乱しの立ち位置のずれは含めない）。"""
    x, y, yaw = (float(v) for v in pose)
    c, s = np.cos(yaw), np.sin(yaw)
    rel = np.array([d0 - x, y0 - y])
    return {"dist_m": float(d0 - x), "side_m": float(-s * rel[0] + c * rel[1]), "yaw_deg": float(np.degrees(-yaw))}


def quiet_realism(r, MujocoRobot, scene: dict, contest_cfg: dict, robot_model):
    """乱しをすべて 0 にした realism（評価セット basic で、立ち位置のずれの口だけを使うため）。

    armature・摩擦は、乱しなし（realism=None）で作ったモデルと同じ値にする（realism を渡すと上書きされるため）。
    """
    plain = MujocoRobot(scene, contest_cfg, realism=None)
    dofs = [plain.model.joint(n).dofadr[0] for n in robot_model.JOINT_NAMES]
    armature, friction = float(np.median(plain.model.dof_armature[dofs])), float(np.median(plain.model.dof_frictionloss[dofs]))
    plain.close()
    return dataclasses.replace(
        r, depth_sigma_at_1m=0.0, depth_hole_ratio=0.0, depth_edge_hole_prob=0.0, depth_edge_jump_m=0.0,
        color_noise_sigma=0.0, color_gain=1.0, camera_xyz_offset=np.zeros(3), camera_rpy_offset=np.zeros(3),
        image_latency_s=0.0, image_fps=30.0, command_latency_s=0.0, q_sigma=0.0, dq_sigma=0.0,
        kp_scale=np.ones(29), armature=armature, friction_nm=friction, sway_amp=np.zeros(6),
        stance_xy=np.zeros(2), stance_yaw=0.0)


def summarize(rows: list[dict], app: dict) -> dict:
    n = len(rows)
    ready = [r for r in rows if r["outcome"] == "ready"]
    pressed = [r for r in ready if r.get("press") == "success"]
    closest = min((min([s["distance_m"] for s in r["steps"]] + [r["true_final"]["dist_m"]]) for r in rows), default=None)
    out = {"trials": n, "ready": len(ready), "press_success": len(pressed),
           "press_tried": sum("press" in r for r in rows),
           "moves_median": float(np.median([r["moves"] for r in ready])) if ready else None,
           "closest_dist_m": closest,
           "aborted_reasons": [r["reason"][:80] for r in rows if r["outcome"] != "ready"]}
    if ready:
        e = np.array([[s["distance_m"] - r["true_final"]["dist_m"], (s["side_m"] or 0.0) - r["true_final"]["side_m"]]
                      for r in ready for s in r["steps"][-1:]])
        out["final_measure_error_cm"] = {"dist_mean": float(e[:, 0].mean() * 100), "dist_max": float(np.abs(e[:, 0]).max() * 100),
                                         "side_mean": float(e[:, 1].mean() * 100), "side_max": float(np.abs(e[:, 1]).max() * 100)}
    return out


if __name__ == "__main__":
    sys.exit(main())
