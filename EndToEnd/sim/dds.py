"""Unitree HG LowStateを配信し、LowCmdのhybrid PDでG1を動かす。"""

import argparse
from dataclasses import replace
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time


def main():
    from isaaclab.app import AppLauncher
    from g1_e2e.dds.codec import LowLevelCodec
    from g1_e2e.robot import RobotSpec

    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=42)
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--mode-machine", type=int, default=5)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--steps", type=int, default=5000)
    parser.add_argument("--watchdog", type=float, default=0.1, help="wall-clock秒")
    parser.add_argument("--wait-seconds", type=float, default=30)
    parser.add_argument("--fixed-base", action="store_true")
    parser.add_argument("--output", type=Path)
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(headless=True, device="cpu")
    args = parser.parse_args()
    if args.steps < 1 or not 0 < args.dt <= 0.02 or args.wait_seconds <= 0:
        parser.error("steps/dt/wait-secondsの範囲外です")
    codec = LowLevelCodec(args.mode_machine)
    output = args.output or root / "runs" / datetime.now(timezone.utc).strftime("dds-%Y%m%dT%H%M%S.%fZ")
    output.mkdir(parents=True, exist_ok=False)
    from sim.runtime import configure_app

    gpu = configure_app(args, root)
    app = AppLauncher(args, active_gpu=gpu, multi_gpu=False).app
    env = bridge = None
    try:
        import numpy as np
        import omni.timeline
        from g1_e2e.data.config import SensorConfig
        from g1_e2e.dds.bridge import DdsBridge
        from sim.envs.config import FlatConfig
        from sim.envs.flat import G1FlatEnv
        from sim.sensors.imu import UrdfImuRig

        spec = RobotSpec.load(root / "configs/robot/g1_29dof.yaml")
        # PhysXにPDを二重適用しない。LowCmdのゲインから各物理stepで合力を計算する。
        effort_spec = replace(spec, stiffness=np.zeros(29, np.float32), damping=np.zeros(29, np.float32))
        config = FlatConfig(physics_dt=args.dt, control_dt=args.dt, episode_seconds=args.steps * args.dt + 1)
        env = G1FlatEnv(root, effort_spec, config, device=args.device, fixed_base=args.fixed_base)
        sensors = SensorConfig.load(root / "configs/sensors/g1_rgb_lidar.yaml", robot_urdf=root / spec.urdf)
        imus = UrdfImuRig(env, sensors.body_imus)
        observation = env.reset()
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_auto_update(False)
        timeline.commit()
        bridge = DdsBridge(codec, domain=args.domain, interface=args.interface, timeout_s=args.watchdog)

        def publish(current, previous=None):
            frames = imus.capture(current.episode_id, current.simulation_time_s)
            # モデル入力のIMU選択から、公式SDKの配信先の対応を独立させる。
            lowstate = codec.encode_state(current, imu=frames["pelvis"], previous=previous, dt=args.dt)
            bridge.publish(lowstate, secondary_imu=codec.encode_imu(frames["torso"]))

        print(
            f"[DDS READY] domain={args.domain} interface={args.interface} dt={args.dt}; waiting for LowCmd",
            flush=True,
        )
        deadline = time.monotonic() + args.wait_seconds
        while app.is_running() and bridge.mailbox.current() is None:
            publish(observation)
            if time.monotonic() > deadline:
                raise TimeoutError("LowCmdを受信しませんでした")
            app.update()
            time.sleep(0.01)
        rows = {name: [] for name in ["time_s", "q", "target_q", "effort_nm", "command_active", "base_xyz"]}
        started = time.monotonic()
        watchdog_steps = 0
        fell = False
        for step in range(args.steps):
            if not app.is_running():
                break
            targets = bridge.mailbox.current()
            active = targets is not None
            watchdog_steps += int(not active)
            if targets is None:
                targets = bridge.mailbox.damping()
            command = targets.effort_command(observation, spec.effort_limit)
            result = env.step(command, render=step % max(1, round(0.02 / args.dt)) == 0)
            previous, observation = observation, result.observation
            publish(observation, previous)
            for name, value in {
                "time_s": observation.simulation_time_s,
                "q": observation.joint_position_rad,
                "target_q": targets.q,
                "effort_nm": result.applied_command.feedforward_torque_nm,
                "command_active": active,
                "base_xyz": observation.privileged["base_position_world_m"],
            }.items():
                rows[name].append(value)
            if result.terminated or result.truncated:
                fell = result.terminated
                break
            if (step + 1) % 500 == 0:
                print(f"[DDS] {step + 1}/{args.steps}; {observation.simulation_time_s:.2f} sim s", flush=True)
            time.sleep(max(0, started + (step + 1) * args.dt - time.monotonic()))
        elapsed = time.monotonic() - started
        np.savez_compressed(output / "trajectory.npz", **{k: np.asarray(v) for k, v in rows.items()})
        summary = {
            "steps": len(rows["time_s"]),
            "requested_steps": args.steps,
            "fell": fell,
            "simulated_seconds": observation.simulation_time_s,
            "wall_seconds": elapsed,
            "wall_steps_per_second": len(rows["time_s"]) / elapsed,
            "physics_dt_s": args.dt,
            "domain": args.domain,
            "interface": args.interface,
            "mode_pr": 0,
            "mode_machine": args.mode_machine,
            "sdk_commit": codec.sdk.commit,
            "isaac_sim_version": (Path(os.environ["ISAAC_SIM_PATH"]) / "VERSION").read_text().strip(),
            "watchdog_seconds": args.watchdog,
            "watchdog_steps": watchdog_steps,
            "published_states": bridge.published,
            "published_secondary_imu": bridge.secondary_published,
            "imu": {**imus.metadata, "lowstate": "pelvis", "secondary_imu": "torso"},
            **bridge.mailbox.stats(),
            "joint_names_sdk_order": spec.joint_names,
            "fixed_base": args.fixed_base,
            "unmodeled_fields": ["temperature", "vol", "sensor", "motorstate", "wireless_remote", "reserve"],
            "motor_mode": "simulated body slots report 1; unused 29..34 report 0",
        }
        (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(f"[SAVED] {json.dumps(summary)}", flush=True)
        if len(rows["time_s"]) != args.steps:
            raise RuntimeError("指定step数より前に終了しました")
    finally:
        if bridge is not None:
            bridge.close()
        if env is not None:
            env.close()
        app.close()


if __name__ == "__main__":
    main()
