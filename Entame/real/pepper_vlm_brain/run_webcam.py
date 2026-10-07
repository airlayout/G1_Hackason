"""Responsive preview, a single inference worker and no queued old frames."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from time import monotonic
import cv2
from adapters.camera_webcam import WebcamCamera
from adapters.robot_mock import MockRobot
from brain.decision import Brain, DecisionResult
from config import Config, MODEL_2B


def overlay(frame, result, busy, age):
    decision = result.decision
    latency = result.metrics.get("inference_latency_s")
    lines = [f"Action: {decision.action}", f"Direction: {decision.direction}",
             f"Target: {decision.target}", f"Distance: {decision.distance}",
             f"Confidence: {decision.confidence:.2f}",
             f"Inference latency: {latency:.2f}s" if latency is not None else "Inference latency: --",
             f"Snapshot age: {age:.1f}s" if age is not None else "Snapshot age: --",
             "INFERENCING" if busy else "READY", "q / ESC: exit"]
    if result.fallback:
        lines.append("FALLBACK: WAIT (see console error)")
    panel_height = min(frame.shape[0], 26 * len(lines) + 10)
    cv2.rectangle(frame, (0, 0), (min(430, frame.shape[1]), panel_height), (0, 0, 0), -1)
    for i, line in enumerate(lines):
        cv2.putText(frame, line, (10, 24 + i * 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return frame


def main():
    parser = argparse.ArgumentParser(description="Webcam -> periodic high-level VLM decisions")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--backend", choices=["auto", "dshow", "msmf"], default="auto")
    parser.add_argument("--interval", type=float, default=2.0, help="Minimum interval between inference starts")
    parser.add_argument("--probe", action="store_true", help="Check one camera frame without loading VLM")
    parser.add_argument("--headless", action="store_true", help="No preview; use Ctrl+C or --max-decisions")
    parser.add_argument("--max-decisions", type=int, default=0, help="Exit after N decisions; 0=unlimited")
    parser.add_argument("--model", default=MODEL_2B)
    parser.add_argument("--revision", default=None)
    parser.add_argument("--4bit", dest="quantize_4bit", action="store_true")
    args = parser.parse_args()
    if args.interval < 0.1 or args.max_decisions < 0:
        parser.error("interval must be >=0.1, max-decisions must be >=0")
    robot = MockRobot()
    camera = WebcamCamera(args.camera, args.backend)
    pool = None
    future = None
    result = DecisionResult()
    count = 0
    next_start = 0.0
    decision_snapshot_at = None
    pending_snapshot_at = None
    try:
        camera.open()
        frame = camera.read()
        if args.probe:
            print(json.dumps({"camera": args.camera, "backend": camera.capture.getBackendName(),
                              "frame_read": True, "frame_shape": list(frame.shape)}))
            return 0
        from brain.vlm import QwenVLM
        brain = Brain(QwenVLM(Config(model_name=args.model, revision=args.revision,
                                    quantize_4bit=args.quantize_4bit)))
        pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vlm")
        while True:
            now = monotonic()
            if future is not None and future.done():
                result = future.result()
                future = None
                decision_snapshot_at = pending_snapshot_at
                print(json.dumps(result.to_dict(), ensure_ascii=True))
                robot.execute(result.decision)
                count += 1
                if args.max_decisions and count >= args.max_decisions:
                    break
            # One running request only. Snapshot the latest unmodified frame.
            # Preview never waits for CUDA inference or model loading.
            if future is None and now >= next_start:
                image = camera.to_image(frame)
                pending_snapshot_at = now
                future = pool.submit(brain.decide, image)
                next_start = now + args.interval
            if not args.headless:
                age = now - decision_snapshot_at if decision_snapshot_at is not None else None
                cv2.imshow("Robot VLM Brain v0.1", overlay(frame.copy(), result, future is not None, age))
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
                if cv2.getWindowProperty("Robot VLM Brain v0.1", cv2.WND_PROP_VISIBLE) < 1:
                    break
            frame = camera.read()
    except KeyboardInterrupt:
        print("Stopping webcam.")
    except Exception as exc:
        result = DecisionResult(fallback=True, error=f"Webcam failed: {type(exc).__name__}: {exc}")
        print(json.dumps(result.to_dict(), ensure_ascii=True))
        robot.execute(result.decision)
        return 1
    finally:
        camera.close()
        cv2.destroyAllWindows()
        if pool is not None:
            # Preview/camera are already closed; an active GPU call finishes before exit.
            pool.shutdown(wait=True, cancel_futures=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
