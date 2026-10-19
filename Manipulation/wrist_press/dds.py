"""Run the same controller on Yada DDS or G1; dry-run is the default."""
from __future__ import annotations

import argparse
import json
from types import SimpleNamespace
import time
import numpy as np

from .adapter import SampleBuilder
from .controller import PressController
from .runtime import enable_yada, j1_module
from .settings import load_settings, resolve_path
from .types import CameraFrame


def capture_time(epoch_stamp, now_epoch, now_control):
    age = now_epoch-float(epoch_stamp)
    if not np.isfinite(age) or age < -.02 or age > .5:
        raise ValueError("camera clock skew/stale exposure; synchronize camera and controller clocks")
    return now_control-age


class TorqueCommandFactory:
    """Add feed-forward to the pinned backend's existing JointCommand factory."""
    def __init__(self,factory):
        self.factory=factory
        self.tau=np.zeros(29)

    def __call__(self):
        command=self.factory()
        command.tau[22:]=self.tau[22:]
        return command


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config")
    ap.add_argument("--mode", choices=["button","bottle"])
    ap.add_argument("--target", choices=["up","down"], default="up")
    ap.add_argument("--network-interface", default="lo")
    ap.add_argument("--camera-host", default="127.0.0.1")
    ap.add_argument("--rgbd-port", type=int, default=5556)
    ap.add_argument("--wrist-host")
    ap.add_argument("--wrist-port", type=int, default=5557)
    ap.add_argument("--wrist-rgb-only", action="store_true")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--output", default="_local/wrist_press/results/dds.json")
    args = ap.parse_args()
    cfg = load_settings(args.config, mode=args.mode)
    if args.execute and args.network_interface != "lo" and not cfg.get("real_calibrated", False):
        ap.error("real execution requires real_calibrated: true in a measured camera/tip/obstacle configuration")
    cfg["async_perception"] = True
    enable_yada()
    from common.config import load_config
    from contest.robots.real_g1 import RealG1Robot
    controller = PressController(cfg)
    robot = wrist = torque_factory = None
    dry_plan_ready=False
    try:
        robot = RealG1Robot(load_config("contest.yaml"), args.network_interface, args.camera_host,
                            args.rgbd_port, dry_run=not args.execute)
        torque_factory=TorqueCommandFactory(robot._JointCommand)
        robot._JointCommand=torque_factory
        if args.execute and robot._loco is not None:
            robot._loco.StopMove()
        kp, _ = robot.upper_gains()
        controller.reset(args.target, kp)
        builder = SampleBuilder(controller.robot, robot.name)
        if args.wrist_host:
            wrist = j1_module("camera_rgbd").RgbdZmqSource(args.wrist_host,args.wrist_port,timeout_ms=1)
            wrist.open()
        started = time.monotonic()
        last_wrist = None
        initial_imu=None
        while time.monotonic()-started <= cfg["cycle_timeout_s"]:
            tick = time.monotonic()
            t = tick-started
            state = robot._fresh_state()
            if np.max(np.abs(state.dq[:15]))>.15:
                raise RuntimeError("lower body is moving; pressing requires a stopped robot")
            imu=np.asarray(state.imu_quat,float)
            if initial_imu is None:
                initial_imu=imu/np.linalg.norm(imu)
            if not np.isfinite(imu).all() or np.linalg.norm(imu)<.5:
                raise ValueError("invalid pelvis IMU quaternion")
            angle=2*np.arccos(np.clip(abs((imu/np.linalg.norm(imu))@initial_imu),0,1))
            if angle>np.radians(2):
                raise RuntimeError("pelvis attitude changed during the stationary press")
            with robot._frame_lock:
                frame = robot._frame
            if frame is None:
                raise RuntimeError("head RGB-D not received")
            stamp = capture_time(frame.timestamp,time.time(),t)
            ins = frame.intrinsics
            K = np.array([[ins.fx,0,ins.cx],[0,ins.fy,ins.cy],[0,0,1.]])
            obs = SimpleNamespace(q=state.q.copy(),dq=state.dq.copy(),t=t,image_t=stamp,
                                  rgb=frame.color_bgr[...,::-1].copy(),depth=frame.depth_m(),K=K)
            sample = builder.build(obs)
            if wrist:
                wf = wrist.read_rgbd()
                if wf is not None:
                    wt = capture_time(wf.timestamp,time.time(),t)
                    q_at = builder.at(wt)
                    if q_at is not None:
                        wi = wf.intrinsics
                        last_wrist = CameraFrame(wf.color_bgr[...,::-1].copy(),
                            None if args.wrist_rgb_only else wf.depth_m(),
                            np.array([[wi.fx,0,wi.cx],[0,wi.fy,wi.cy],[0,0,1.]]),wt,q_at,
                            controller.robot.wrist_transform(q_at),"wrist",wf.frame_id)
            sample.wrist = last_wrist
            command = controller.act(sample)
            if not args.execute and command.phase=="transit":
                dry_plan_ready=True
                break
            if command.tau_ff is not None:
                torque_factory.tau=command.tau_ff.copy()
            if command.done:
                break
            robot.command(command.q[12:],np.zeros(3))
            time.sleep(max(0,.02-(time.monotonic()-tick)))
    except Exception as exc:
        controller.report.error=str(exc)
        raise
    finally:
        if robot:
            # Release gravity support without retaining a pressing-force term.
            state=robot.backend.read_state()
            torque_factory.tau=(controller.robot.gravity_torque(state.q)
                                if state is not None and time.monotonic()-state.stamp<.1 else np.zeros(29))
            robot.close()
        if wrist:
            wrist.close()
        controller.close()
        out = resolve_path(args.output)
        out.parent.mkdir(parents=True,exist_ok=True)
        result = controller.result()
        result.update(dry_run=not args.execute,interface=args.network_interface,planned=dry_plan_ready)
        if dry_plan_ready:
            result["nominal_trajectory"]=controller.trajectory.path.tolist()
        out.write_text(json.dumps(result,indent=2)+"\n")
        print(json.dumps(result,ensure_ascii=False))
    return 0 if dry_plan_ready or result["returned"] and not result["error"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
