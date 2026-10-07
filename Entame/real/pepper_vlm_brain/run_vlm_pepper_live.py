"""Latest-frame-wins Pepper camera -> Qwen3-VL -> throttled LOOK/SPEAK loop."""

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import subprocess
import time

from PIL import Image

from adapters.pepper.motion import Ros2HeadTransport
from brain.decision import Brain
from brain.vlm import QwenVLM
from config import Config


YAW_DELTA = {"LEFT": 0.15, "CENTER": 0.00, "RIGHT": -0.15}
GENERIC_SPEECH = {"こんにちは", "おはようございます", "こんばんは", "何かお手伝いしましょうか"}


@dataclass
class ReactionState:
    last_target_present: bool = False
    last_direction: str | None = None
    last_salient_fact: str | None = None
    last_speech: str | None = None
    last_reaction_time: float = 0.0
    last_head_yaw: float | None = None
    last_look_time: float = 0.0


def capture_latest_frame(path: Path, worker: subprocess.Popen, baseline_mtime: float,
                         timeout_s: float = 8.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if (path.exists() and path.stat().st_size > 0
                and path.stat().st_mtime > baseline_mtime):
            return
        if worker.poll() is not None:
            raise RuntimeError("camera capture worker exited")
        time.sleep(0.1)
    raise TimeoutError("no Pepper camera frame received")


def run_vlm_on_frame(brain: Brain, path: Path):
    with Image.open(path) as source:
        image = source.copy()
    return brain.decide(image)


def read_head_pose(path: Path) -> tuple[float, float]:
    payload = json.loads(path.read_text())
    return float(payload["yaw"]), float(payload["pitch"])


def map_direction_to_head_pose(direction: str, current_yaw: float,
                               current_pitch: float) -> tuple[float, float, float]:
    delta = YAW_DELTA[direction]
    target = max(-0.80, min(0.80, current_yaw + delta))
    return target, current_pitch, delta


def _normalize_speech(text: str) -> str:
    return text.strip().rstrip("。！!？? ")


def grounded_speech(decision) -> tuple[str, bool, bool]:
    """Return text, grounded flag, and whether generic model output was rejected."""
    speech = (decision.speech or "").strip()
    fact = (decision.salient_fact or "").strip()
    fact_ja = (decision.salient_fact_ja or "").strip()
    has_fact = bool(fact) and fact.casefold() != "none"
    generic = _normalize_speech(speech) in GENERIC_SPEECH
    if not has_fact:
        return speech, not generic, False
    if not generic:
        return speech, True, False
    if not fact_ja or fact_ja.casefold() == "none":
        return "", False, True
    base = fact_ja.rstrip("。！!？? ")
    if base.endswith("ている"):
        rewritten = base[:-3] + "ていますね。"
    elif base.endswith("いる"):
        rewritten = base[:-2] + "いますね。"
    elif base.endswith("ある"):
        rewritten = base[:-2] + "ありますね。"
    else:
        rewritten = base + "ですね。"
    return rewritten, True, True


def should_react(result, state: ReactionState, now: float, cooldown: float,
                 look_threshold: float, current_yaw: float,
                 minimum_look_interval: float) -> tuple[bool, bool, list[str], str, bool]:
    decision = result.decision
    present = decision.target.value == "PERSON" and decision.direction.value in YAW_DELTA
    if not present:
        state.last_target_present = False
        return False, False, ["no_person"], "", False
    direction = decision.direction.value
    target_yaw, _, _ = map_direction_to_head_pose(direction, current_yaw, 0.15)
    new_person = not state.last_target_present
    direction_changed = state.last_direction is not None and direction != state.last_direction
    yaw_changed = abs(target_yaw - current_yaw) >= look_threshold
    look_ready = now - state.last_look_time >= minimum_look_interval
    look = direction != "CENTER" and yaw_changed and look_ready
    fact = (decision.salient_fact or "").strip().casefold()
    speech, _, generic_rejected = grounded_speech(decision)
    fact_changed = bool(fact) and fact != (state.last_salient_fact or "").casefold()
    speech_changed = bool(speech) and speech != (state.last_speech or "")
    cooled_down = now - state.last_reaction_time >= cooldown
    # A new person or a concrete fact change is an event and bypasses cooldown.
    # Cooldown only permits a changed utterance about an otherwise unchanged fact;
    # exact repeats (including generic greetings) remain suppressed.
    speak = bool(speech) and (new_person or fact_changed or (cooled_down and speech_changed))
    reasons = [name for name, enabled in (("new_person", new_person),
               ("direction_changed", direction_changed), ("yaw_threshold", yaw_changed),
               ("fact_changed", fact_changed), ("speech_changed", speech_changed),
               ("cooldown_ready", cooled_down)) if enabled]
    return look, speak, reasons or ["duplicate"], speech, generic_rejected


def perform_reaction(transport: Ros2HeadTransport, target_yaw: float,
                     target_pitch: float, speech: str, look: bool, speak: bool,
                     decision_time: float) -> None:
    if look:
        transport.look_at_joints(target_yaw, target_pitch)
        print(f"[REACTION] look_latency_s={time.monotonic() - decision_time:.3f}")
    if speak:
        transport.speak(speech)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--speak-cooldown", type=float, default=4.0)
    parser.add_argument("--look-threshold", type=float, default=0.08)
    parser.add_argument("--minimum-look-interval", type=float, default=1.0)
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--send", action="store_true", help="enable real LOOK/SPEAK")
    parser.add_argument("--frame", type=Path, default=Path("reports/pepper/live_loop_latest.png"))
    parser.add_argument("--joint-state", type=Path,
                        default=Path("reports/pepper/live_loop_joints.json"))
    args = parser.parse_args()
    if args.interval < 2.0 or args.duration_sec <= 0 or args.speak_cooldown < 0:
        parser.error("interval must be >=2.0; duration positive; cooldown nonnegative")

    capture_command = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /tmp/pepper_naoqi_ws.BwBAe7/install2/setup.bash && "
        f"/usr/bin/python3 capture_pepper_ros_image.py --continuous --output {args.frame} "
        f"--joint-state-output {args.joint_state}"
    )
    baseline_mtime = args.frame.stat().st_mtime if args.frame.exists() else 0.0
    worker = subprocess.Popen(["/bin/bash", "-lc", capture_command],
                              stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    joint_baseline = args.joint_state.stat().st_mtime if args.joint_state.exists() else 0.0
    transport = Ros2HeadTransport(dry_run=not args.send)
    brain = Brain(QwenVLM(Config(max_new_tokens=192)))
    state = ReactionState()
    started = time.monotonic()
    previous_cycle_start = None
    effective_cycles: list[float] = []
    inference_count = look_count = speech_count = suppressed_speech_count = 0
    try:
        capture_latest_frame(args.frame, worker, baseline_mtime)
        capture_latest_frame(args.joint_state, worker, joint_baseline)
        while time.monotonic() - started < args.duration_sec:
            cycle_started = time.monotonic()
            if previous_cycle_start is not None:
                effective_cycles.append(cycle_started - previous_cycle_start)
            previous_cycle_start = cycle_started
            frame_timestamp = args.frame.stat().st_mtime
            result = run_vlm_on_frame(brain, args.frame)
            inference_count += 1
            print(f"[FRAME] timestamp={frame_timestamp:.6f}")
            if result.fallback:
                print(f"[VLM] fallback=yes error={result.error}")
            else:
                decision = result.decision
                print(f"[VLM] direction={decision.direction.value} target={decision.target.value} "
                      f"salient_fact={decision.salient_fact} "
                      f"salient_fact_ja={decision.salient_fact_ja} speech={decision.speech}")
                now = time.monotonic()
                current_yaw, current_pitch = read_head_pose(args.joint_state)
                look, speak, reasons, speech, generic_rejected = should_react(
                    result, state, now, args.speak_cooldown, args.look_threshold,
                    current_yaw, args.minimum_look_interval)
                print(f"[DECISION] look={'yes' if look else 'no'} speak={'yes' if speak else 'no'} "
                      f"reason={','.join(reasons)}")
                if decision.target.value == "PERSON" and decision.direction.value in YAW_DELTA:
                    yaw, pitch, delta = map_direction_to_head_pose(
                        decision.direction.value, current_yaw, current_pitch)
                    if decision.direction.value == "CENTER":
                        print(f"[TRACK] current_yaw={current_yaw:.3f} direction=CENTER action=HOLD")
                    else:
                        print(f"[TRACK] current_yaw={current_yaw:.3f} "
                              f"direction={decision.direction.value} delta={delta:+.2f} "
                              f"target_yaw={yaw:.3f}")
                    print(f"[SPEECH] grounded={'true' if speech else 'false'} "
                          f"generic_rejected={'true' if generic_rejected else 'false'} text={speech}")
                    if speak:
                        print(f"[SPEECH] text={speech}")
                    decision_time = time.monotonic()
                    perform_reaction(transport, yaw, pitch, speech, look, speak, decision_time)
                    look_count += int(look)
                    speech_count += int(speak)
                    suppressed_speech_count += int(bool(decision.speech) and not speak)
                    state.last_target_present = True
                    state.last_direction = decision.direction.value
                    state.last_salient_fact = decision.salient_fact
                    state.last_speech = speech
                    if look:
                        state.last_head_yaw = yaw
                        state.last_look_time = now
                    if speak:
                        state.last_reaction_time = now
            remaining = args.interval - (time.monotonic() - cycle_started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("[LOOP] Ctrl+C received")
    finally:
        worker.terminate()
        try:
            worker.wait(timeout=3)
        except subprocess.TimeoutExpired:
            worker.kill()
            worker.wait()
        print("[LOOP] stopped; no shutdown motion sent")
        average_cycle = (sum(effective_cycles) / len(effective_cycles)
                         if effective_cycles else 0.0)
        print(f"[SUMMARY] inferences={inference_count} effective_cycle_s={average_cycle:.3f} "
              f"looks={look_count} speeches={speech_count} "
              f"speech_suppressed={suppressed_speech_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
