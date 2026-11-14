#!/usr/bin/env python3
"""One bounded right-shoulder test using the official rt/arm_sdk route."""
import argparse
import json
import math
import signal
import threading
import time


ARM = tuple(range(15, 29))
HZ = 50
DELTA = math.radians(12)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC

    ChannelFactoryInitialize(0, "eth0")
    lock = threading.Lock()
    latest = [None, 0.0]

    def receive(msg):
        with lock:
            latest[:] = [msg, time.monotonic()]

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(receive, 1)
    baseline_rpy = None

    def state(check_motion=True):
        with lock:
            msg, at = latest
        if msg is None or time.monotonic() - at > 0.25:
            raise RuntimeError("Fresh LowState unavailable")
        q = [float(msg.motor_state[i].q) for i in ARM]
        rpy = [float(x) for x in msg.imu_state.rpy]
        gyro = [float(x) for x in msg.imu_state.gyroscope]
        leg_speed = max(abs(float(msg.motor_state[i].dq)) for i in range(12))
        if msg.mode_machine != 5 or not all(math.isfinite(x) for x in q + rpy + gyro + [leg_speed]):
            raise RuntimeError("Unexpected robot model or invalid telemetry")
        if check_motion:
            if abs(rpy[0]) > 0.15 or abs(rpy[1]) > 0.15 or max(abs(x) for x in gyro) > 0.5 or leg_speed > 0.5:
                raise RuntimeError("Robot is not stationary and upright")
            if baseline_rpy and max(abs(rpy[i] - baseline_rpy[i]) for i in (0, 1)) > 0.08:
                raise RuntimeError("Body attitude changed; aborting arm test")
        return q, rpy

    deadline = time.monotonic() + 3
    while latest[0] is None and time.monotonic() < deadline:
        time.sleep(0.02)
    loco = LocoClient()
    loco.SetTimeout(2)
    loco.Init()
    code, fsm = loco.GetFsmId()
    if code != 0 or fsm != 501:
        raise RuntimeError("Expected stable standing FSM 501, got %r" % ((code, fsm),))
    start, baseline_rpy = state()
    for _ in range(10):
        q, _ = state()
        if max(abs(a - b) for a, b in zip(q, start)) > 0.03:
            raise RuntimeError("Arms are already moving")
        time.sleep(0.02)
    goal = start.copy()
    goal[7] -= DELTA  # Joint 22: right shoulder pitch. Negative delta raises forward.
    if not -3.0692 < goal[7] < 2.6504:
        raise RuntimeError("Shoulder target outside joint limits")
    print(json.dumps({"plan": "right shoulder up and return", "fsm": fsm,
                      "initial_shoulder_rad": start[7], "target_shoulder_rad": goal[7],
                      "amplitude_deg": 12, "raise_seconds": 2, "return_seconds": 2}), flush=True)
    if not args.execute:
        sub.Close()
        print("PREFLIGHT_OK_NO_COMMANDS_SENT", flush=True)
        return

    pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()
    cmd = unitree_hg_msg_dds__LowCmd_()
    crc = CRC()
    current_target = start.copy()
    weight = 0.0
    peak_raise = 0.0
    max_other_deviation = 0.0

    def interrupted(signum, frame):
        raise KeyboardInterrupt("Interrupt received")

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)

    def publish(target, new_weight, monitor=True):
        nonlocal current_target, weight, peak_raise, max_other_deviation
        if monitor:
            observed, _ = state()
            if max(abs(a - b) for a, b in zip(observed, current_target)) > 0.25:
                raise RuntimeError("Arm tracking error exceeds 0.25 rad")
            peak_raise = max(peak_raise, start[7] - observed[7])
            other = max(abs(observed[i] - start[i]) for i in range(14) if i != 7)
            max_other_deviation = max(max_other_deviation, other)
        for joint, q in zip(ARM, target):
            motor = cmd.motor_cmd[joint]
            motor.q, motor.dq, motor.tau = q, 0.0, 0.0
            motor.kp = 15.0 if joint in (19, 20, 21, 26, 27, 28) else 40.0
            motor.kd = 1.5
        cmd.motor_cmd[29].q = new_weight
        cmd.crc = crc.Crc(cmd)
        current_target, weight = target.copy(), new_weight
        if not pub.Write(cmd):
            raise RuntimeError("Arm command transport failed")

    def phase(name, duration, first, last, w0, w1, monitor=True):
        print("PHASE " + name, flush=True)
        begin = time.monotonic()
        steps = round(duration * HZ)
        for i in range(steps + 1):
            delay = begin + i / HZ - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            alpha = i / steps
            blend = alpha * alpha * (3 - 2 * alpha)
            target = [a + blend * (b - a) for a, b in zip(first, last)]
            publish(target, w0 + blend * (w1 - w0), monitor)

    returned = False
    peak_observed = None
    return_error = None
    try:
        phase("engage_at_current_pose", 2, start, start, 0, 1)
        phase("raise_right_arm", 2, start, goal, 1, 1)
        phase("hold", 0.6, goal, goal, 1, 1)
        peak_observed, _ = state()
        phase("return_to_initial_pose", 2, goal, start, 1, 1)
        phase("settle", 0.5, start, start, 1, 1)
        final_q, _ = state()
        return_error = max(abs(a - b) for a, b in zip(final_q, start))
        returned = True
    finally:
        # Only arm blending is released; no Damp, mode switch, leg or waist command.
        if weight > 0:
            try:
                phase("release_arm_blending", 2, current_target, current_target, weight, 0, False)
            finally:
                pub.Close()
                sub.Close()
    result = {"completed_return": returned,
              "observed_peak_raise_deg": math.degrees(peak_raise),
              "observed_shoulder_at_hold_rad": peak_observed[7] if peak_observed else None,
              "max_other_arm_deviation_deg": math.degrees(max_other_deviation),
              "return_max_error_deg": math.degrees(return_error) if return_error is not None else None,
              "arm_control_weight": weight}
    print("ARM_TEST_RESULT " + json.dumps(result), flush=True)
    if peak_raise < math.radians(4) or return_error is None or return_error > math.radians(5):
        raise RuntimeError("Movement or return not confirmed; no automatic retry")
    print("SMALL_RIGHT_ARM_TEST_OK", flush=True)


if __name__ == "__main__":
    main()
