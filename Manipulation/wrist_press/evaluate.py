"""Full-cycle evaluation: keep running after the official button-light score."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time
import numpy as np

from .adapter import SampleBuilder
from .controller import PressController
from .runtime import enable_yada
from .settings import load_settings, resolve_path


def run_episode(robot, cfg, task, control_dt=.02, snapshots=None, recorder=None):
    cfg=dict(cfg)
    if cfg["gravity_mode"]=="torque" and not getattr(robot,"supports_torque_feedforward",False):
        cfg["gravity_mode"]="position"
    controller = PressController(cfg)
    kp, _ = robot.upper_gains()
    controller.reset(task.target, kp)
    builder = SampleBuilder(controller.robot, robot.name)
    first_light = None
    lit_names = set()
    max_force = 0.0
    bottle_contact = False
    max_bottle_force = 0.0
    command = None
    trace = []
    started = time.perf_counter()
    t = 0.0
    last_phase = None
    try:
        while t <= cfg["cycle_timeout_s"]+.2:
            obs = robot.observe(t)
            command = controller.act(builder.build(obs))
            if command.phase != last_phase:
                print(f"[cycle] {t:.2f}s {command.phase}",flush=True)
                if snapshots:
                    from PIL import Image
                    snapshots.mkdir(parents=True,exist_ok=True)
                    Image.fromarray(obs.rgb).save(snapshots/f"{command.phase}_head.png")
                    if getattr(obs,"wrist",None):
                        Image.fromarray(obs.wrist.rgb).save(snapshots/f"{command.phase}_wrist.png")
                last_phase = command.phase
            if recorder is not None:
                recorder.capture(obs, command)
            if command.done:
                break
            if command.tau_ff is not None:
                robot.command_with_feedforward(command.q[12:],np.zeros(3),command.tau_ff)
            else:
                robot.command(command.q[12:], np.zeros(3))
            info = robot.advance(control_dt)
            t += control_dt
            lit = {name for name, on in info.lit.items() if on}
            lit_names |= lit
            if lit and first_light is None:
                first_light = t
            if info.max_contact_force is not None:
                max_force = max(max_force, info.max_contact_force, getattr(robot,"unintended_force",0.0))
            bottle_contact |= bool(getattr(robot, "bottle_contact", False))
            max_bottle_force = max(max_bottle_force, getattr(robot, "bottle_force", 0.0))
            trace.append({"t": round(t,4), "phase": command.phase, "q": obs.q[22:].tolist()})
    finally:
        result = controller.result()
        controller.close()
    force_available=robot.name=="mujoco"
    result.update(time_to_light_s=first_light, full_cycle_s=round(t,4), wall_time_s=time.perf_counter()-started,
                  gravity_mode=cfg["gravity_mode"],
                  max_unintended_force_n=max_force if force_available else None,
                  bottle_contact=bottle_contact, max_bottle_force_n=max_bottle_force,
                  target=task.target, lit_buttons=sorted(lit_names), trace=trace)
    physical = lit_names == {task.target} if cfg["mode"] == "button" else bottle_contact if hasattr(robot,"bottle_contact") else None
    result["physical_press_success"] = physical
    result["success"] = bool(physical and result["returned"] and not result["error"] and force_available and max_force < 1.0)
    result["within_time_target"] = result["success"] and t <= cfg["cycle_target_s"]
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config")
    ap.add_argument("--mode", choices=["button","bottle"])
    ap.add_argument("--detector", choices=["yolo","classic"], help="classic is an explicit button-only diagnostic")
    ap.add_argument("--weights",help="explicit YOLO weights for the selected target mode")
    ap.add_argument("--sim", choices=["mujoco","isaac"], default="mujoco")
    ap.add_argument("--seeds", type=int, nargs="+", default=[0,1])
    ap.add_argument("--eval-set", choices=["basic","realistic"], default="basic")
    ap.add_argument("--head-only", action="store_true")
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--output", default="_local/wrist_press/results/full_cycle.json")
    ap.add_argument("--snapshots", help="optional directory for phase transition camera images")
    ap.add_argument("--video-dir", help="MuJoCo bottle WebM recordings: overview, head RGB and wrist RGB together")
    args = ap.parse_args()
    if args.video_dir and args.sim != "mujoco":
        ap.error("--video-dir currently requires --sim mujoco")
    cfg = load_settings(args.config, mode=args.mode, detector=args.detector)
    if args.video_dir and cfg["mode"] != "bottle":
        ap.error("--video-dir currently requires --mode bottle")
    if args.weights:
        cfg[f"{cfg['mode']}_weights"]=args.weights
    if args.head_only:
        cfg["use_wrist"] = False
    app=None
    if args.sim=="isaac":
        from isaaclab.app import AppLauncher
        app=AppLauncher({"headless":not args.view,"enable_cameras":True}).app
    enable_yada()
    from common.config import load_config
    from contest.task import make_trial
    from contest.interface import TaskInfo
    contest_cfg = load_config("contest.yaml")
    results = []
    for seed in args.seeds:
        trial = make_trial(seed, contest_cfg, eval_set=args.eval_set)
        if cfg["mode"] == "bottle":
            # Move the hall away from the fixture; bottle remains fixed to its base.
            trial.scene_cfg["wall"]["front_x"] = .65
        if args.sim == "mujoco":
            from .simulation import make_mujoco_robot
            robot = make_mujoco_robot(trial.scene_cfg, contest_cfg, cfg, args.view, trial.realism)
        else:
            from .isaac_backend import make_isaac_robot
            robot = make_isaac_robot(trial.scene_cfg,contest_cfg,cfg)
        kp, kd = robot.upper_gains()
        task = TaskInfo(trial.instruction, "bottle" if cfg["mode"] == "bottle" else trial.target,
                        args.sim, cfg["cycle_timeout_s"], .02, False, kp, kd)
        recorder = None
        try:
            if args.video_dir:
                from .recording import SynchronizedRecorder
                recorder = SynchronizedRecorder(robot, resolve_path(args.video_dir)/f"{cfg['mode']}_{seed}.webm")
            result = run_episode(robot, cfg, task,
                                 snapshots=resolve_path(args.snapshots)/str(seed) if args.snapshots else None,
                                 recorder=recorder)
            result.update(seed=seed, sim=args.sim, eval_set=args.eval_set)
            results.append(result)
            print(json.dumps({k:v for k,v in result.items() if k not in ("trace","perception_ms","planning_ms")}, ensure_ascii=False))
        finally:
            if recorder is not None:
                recorder.close()
            robot.close()
    out = resolve_path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    times = [r["full_cycle_s"] for r in results if r["success"]]
    payload = {"config": cfg, "summary": {"trials": len(results), "successes": sum(r["success"] for r in results),
               "median_full_cycle_s": float(np.median(times)) if times else None,
               "p95_full_cycle_s": float(np.quantile(times,.95)) if times else None}, "results": results}
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=2)+"\n")
    if app:
        app.close()
    return 0 if all(r["success"] for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
