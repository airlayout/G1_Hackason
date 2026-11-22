"""Read-only Pepper camera probe and Phase 3.6 YOLO/hybrid dry-run."""

import argparse
import json
from pathlib import Path
from statistics import mean
from time import perf_counter, sleep


def percentile(values, q):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * q
    low = int(index)
    high = min(low + 1, len(ordered) - 1)
    fraction = index - low
    return ordered[low] * (1 - fraction) + ordered[high] * fraction


def summary(values):
    return {
        "count": len(values),
        "mean_s": mean(values) if values else None,
        "p95_s": percentile(values, .95),
        "max_s": max(values) if values else None,
    }


def snapshot(ip, path, fps):
    import cv2
    from adapters.pepper.camera import PepperCameraSource
    path.parent.mkdir(parents=True, exist_ok=True)
    with PepperCameraSource(ip, fps=fps) as camera:
        frame = camera.read()
    if not cv2.imwrite(str(path), frame.bgr):
        raise RuntimeError(f"OpenCV could not save {path}")
    return frame


def detect_snapshot(frame, model_path, device, confidence):
    from ultralytics import YOLO
    from hybrid.workers import person_detection
    model = YOLO(str(model_path))
    started = perf_counter()
    prediction = model.predict(
        frame.bgr, device=device, classes=[0], imgsz=640, conf=confidence,
        iou=.7, verbose=False, save=False, half=False, augment=False, max_det=100,
    )[0]
    finished = perf_counter()
    result = person_detection(prediction.boxes.data.cpu().tolist(), frame.width, confidence)
    result.update(
        yolo_latency_s=finished - started,
        capture_to_geometry_s=finished - frame.host_receive_monotonic,
        ultralytics_speed_ms=prediction.speed,
    )
    return result


def run_live(ip, seconds, model_path, device, confidence, fps, hybrid):
    from brain.split_schemas import PlannerDecision
    from hybrid.core import (EventLatch, GeometryStore, LatestFrameSlot, RecordingRobot,
                             bind_latest, immediate_tracking)
    from hybrid.pepper import PepperProducer
    from hybrid.process import WorkerProcess

    slot = LatestFrameSlot()
    producer = PepperProducer(ip, slot, fps=fps)
    fast = WorkerProcess("yolo", model_path, device, confidence=confidence)
    slow = WorkerProcess("qwen") if hybrid else None
    report = {
        "duration_s": seconds,
        "fast_worker": fast.ready(),
        "slow_worker": slow.ready() if slow else None,
        "fast_results": [], "commands": [], "events": [], "slow_results": [],
    }
    store = GeometryStore()
    latch = EventLatch()
    robot = RecordingRobot()
    last_fast = -1
    producer.start()
    if not producer.ready.wait(10) or producer.error:
        raise RuntimeError(producer.error or "Pepper producer initialization timeout")
    started = perf_counter()
    try:
        while perf_counter() - started < seconds or fast.busy or (slow and slow.busy):
            now = perf_counter()
            result = fast.poll()
            if result is not None:
                last_fast = max(last_fast, result["generation"])
                accepted, reason = store.accept_fast(result, perf_counter())
                result.update(accepted=accepted, rejection_reason=reason,
                              state_after=store.state.snapshot.model_dump(mode="json"),
                              state_update_timestamp=perf_counter())
                report["fast_results"].append(result)
                if accepted:
                    transition = store.state.snapshot.transition.value
                    if hybrid and transition in {"UNKNOWN", "APPEARED", "DISAPPEARED"}:
                        event = "INITIAL" if transition == "UNKNOWN" else transition
                        if latch.offer(event, store.combined(now), store.state.snapshot,
                                       store.result, store.episode, now):
                            report["events"].append({**latch.pending, "type": "planner_trigger"})
                    decision = immediate_tracking(store, now)
                    if decision is not None:
                        robot.execute(decision)
                        ready = perf_counter()
                        report["commands"].append({
                            "channel": "immediate_geometry",
                            "generation": result["generation"],
                            "capture_timestamp": result["capture_timestamp"],
                            "ready_timestamp": ready,
                            "event_to_ready_age_s": ready - result["capture_timestamp"],
                            "decision": decision.model_dump(mode="json", exclude_none=True),
                            "mock_robot_output": robot.last_output,
                        })
            if slow:
                slow_result = slow.poll()
                if slow_result is not None:
                    request = slow.request
                    slow_result["request_context"] = {k: v for k, v in request.items() if k != "bgr"}
                    report["slow_results"].append(slow_result)
                    if not slow_result.get("cancelled"):
                        original = PlannerDecision.model_validate(slow_result["decision"])
                        valid = request.get("episode") == store.episode and now - request["capture_timestamp"] <= 4
                        resolved = bind_latest(original, store, now) if valid else None
                        if resolved is not None:
                            robot.execute(resolved)
                            ready = perf_counter()
                            report["commands"].append({
                                "channel": "vlm_semantic", "trigger": request.get("event_kind"),
                                "capture_timestamp": request["capture_timestamp"],
                                "ready_timestamp": ready,
                                "decision": resolved.model_dump(mode="json", exclude_none=True),
                                "raw_decision": slow_result["decision"],
                                "mock_robot_output": robot.last_output,
                            })
            active = perf_counter() - started < seconds
            frame = slot.latest(last_fast)
            if active and not fast.busy and frame is not None:
                fast.submit({
                    "kind": "fast", "generation": frame.generation,
                    "source_timestamp": frame.source_timestamp,
                    "capture_timestamp": frame.capture_timestamp, "bgr": frame.bgr,
                })
                last_fast = frame.generation
            if hybrid and slow and not slow.busy:
                event = latch.take(now)
                if event:
                    slow.submit({**event, "kind": "planner", "event_kind": event["kind"]})
                    report["events"].append({"type": "slow_dispatch", "kind": "planner",
                                             "generation": event["generation"], "timestamp": now})
            sleep(.001)
    finally:
        producer.stop.set()
        producer.join(timeout=5)
        fast.close()
        if slow:
            slow.close()
    report.update(
        producer_error=producer.error,
        producer_frames=producer.frames,
        camera_read_failures=producer.read_failures,
        latest_slot_overwrites=slot.overwritten,
        latest_frame_slot_capacity=1,
        camera_latency=summary([row["camera_latency_s"] for row in producer.metadata]),
        acquisition_interval=summary([
            b["host_receive_monotonic"] - a["host_receive_monotonic"]
            for a, b in zip(producer.metadata, producer.metadata[1:])
        ]),
        yolo_latency=summary([row["inference_latency_s"] for row in report["fast_results"]]),
        frame_age=summary([row["result_age_s"] for row in report["fast_results"]]),
        update_interval=summary([
            b["processing_finish"] - a["processing_finish"]
            for a, b in zip(report["fast_results"], report["fast_results"][1:])
        ]),
        look_ready=summary([
            row["event_to_ready_age_s"] for row in report["commands"]
            if row["channel"] == "immediate_geometry"
        ]),
        effective_hz=(len(report["fast_results"]) / seconds if seconds else None),
        final_state=store.state.snapshot.model_dump(mode="json"),
    )
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", required=True)
    parser.add_argument("--snapshot", action="store_true")
    parser.add_argument("--yolo", action="store_true")
    parser.add_argument("--live-seconds", type=float, default=0)
    parser.add_argument("--hybrid-dry-run", action="store_true")
    parser.add_argument("--fps", type=int, default=5)
    parser.add_argument("--confidence", type=float, default=.1)
    parser.add_argument("--device", default="0")
    parser.add_argument("--model", type=Path, default=Path(".cache/phase36/yolo11n.pt"))
    parser.add_argument("--image", type=Path, default=Path("reports/pepper/latest.png"))
    parser.add_argument("--report", type=Path, default=Path("reports/pepper/phase4a.json"))
    args = parser.parse_args()
    if not (args.snapshot or args.yolo or args.live_seconds or args.hybrid_dry_run):
        parser.error("select --snapshot, --yolo, --live-seconds, or --hybrid-dry-run")
    if args.live_seconds < 0 or args.live_seconds > 30:
        parser.error("live-seconds must be in 0..30")
    if args.confidence != .1:
        parser.error("Phase 3.6 confidence is fixed at 0.10")
    report = {"ip": args.ip, "confidence": args.confidence, "motion_sent": False}
    try:
        frame = snapshot(args.ip, args.image, args.fps)
        report["snapshot"] = {
            "status": "pass", "width": frame.width, "height": frame.height,
            "layers": frame.layers, "colorspace": frame.colorspace,
            "pepper_timestamp_s": frame.pepper_timestamp_s,
            "host_receive_timestamp": frame.host_receive_timestamp,
            "payload_bytes": frame.payload_bytes,
            "camera_latency_s": frame.acquisition_latency_s,
            "saved_path": str(args.image.resolve()),
        }
        if args.yolo or args.live_seconds or args.hybrid_dry_run:
            report["yolo_snapshot"] = detect_snapshot(
                frame, args.model, args.device, args.confidence
            )
        duration = args.live_seconds or (15 if args.hybrid_dry_run else 0)
        if duration:
            report["live"] = run_live(
                args.ip, duration, args.model, args.device, args.confidence,
                args.fps, args.hybrid_dry_run,
            )
        report["status"] = "pass"
    except Exception as exc:
        report.update(status="fail", error=f"{type(exc).__name__}: {exc}")
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False, default=str) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=False, default=str))
    return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
