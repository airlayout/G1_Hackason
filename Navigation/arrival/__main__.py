"""python -m Navigation.arrival --help"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .session import initialize, mark, read_json, read_lines, write_json


def annotate(args):
    import numpy as np

    root = Path(args.session)
    frames = read_lines(root/"streams/frames.jsonl")
    frame = next((f for f in frames if f["id"] == args.frame_id), None)
    if frame is None:
        raise ValueError("unknown frame ID; see streams/frames.jsonl")
    if args.panel and args.button:
        polygon, button = json.loads(args.panel), args.button
    elif args.panel or args.button:
        raise ValueError("provide both --panel and --button")
    else:
        import cv2

        with np.load(root/frame["path"], allow_pickle=False) as data:
            image = data["rgb"][..., ::-1].copy()
        points = []

        def click(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN and len(points) < 5:
                points.append([x, y])

        title = "Panel: 4 corners, then button center. Enter=save, R=reset, Esc=cancel"
        cv2.namedWindow(title)
        cv2.setMouseCallback(title, click)
        try:
            while True:
                canvas = image.copy()
                for i, point in enumerate(points):
                    cv2.circle(canvas, tuple(point), 3, (0, 0, 255) if i == 4 else (0, 255, 0), -1)
                if len(points) >= 4:
                    cv2.polylines(canvas, [np.array(points[:4])], True, (0, 255, 0), 1)
                cv2.imshow(title, canvas)
                key = cv2.waitKey(50) & 255
                if key == 27:
                    return
                if key in (ord("r"), ord("R")):
                    points.clear()
                if key in (10, 13) and len(points) == 5:
                    break
        finally:
            cv2.destroyAllWindows()
        polygon, button = points[:4], points[4]
    from .geometry import polygon_mask
    with np.load(root/frame["path"], allow_pickle=False) as data:
        polygon_mask(data["rgb"].shape[:2], polygon)
    path = root/"annotations.json"
    existing = read_json(path)
    existing["frames"] = [a for a in existing["frames"] if a["frame_id"] != frame["id"]]
    existing["frames"].append({"frame_id": frame["id"], "panel_polygon": polygon, "button_uv": button,
                               "method": "manual", "button_exclusion_px": args.exclude_radius})
    write_json(path, existing)


def positive(value):
    import math
    x = float(value)
    if not math.isfinite(x) or x <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return x


def main():
    parser = argparse.ArgumentParser(description="Arrival measurement only; never sends robot motion commands.")
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("init", "mark", "annotate", "analyze", "synthetic", "record-ros", "record-body", "record-camera"):
        command = commands.add_parser(name)
        command.add_argument("--session", required=True, help="measurement session directory")
        if name == "mark":
            command.add_argument("event", choices=["start", "end", "manual_intervention", "bind_goal"])
            command.add_argument("--trial", required=True)
            command.add_argument("--goal-id")
            command.add_argument("--reason", default="")
        if name == "annotate":
            command.add_argument("--frame-id", required=True)
            command.add_argument("--panel", help="JSON polygon, e.g. '[[10,10],[60,10],[60,50],[10,50]]'")
            command.add_argument("--button", nargs=2, type=float)
            command.add_argument("--exclude-radius", type=positive, default=4.)
        if name == "analyze":
            command.add_argument("--output")
        if name.startswith("record-"):
            command.add_argument("--duration", type=positive, default=300.)
        if name == "record-body":
            command.add_argument("--interface", required=True)
            command.add_argument("--domain", type=int, default=0)
            command.add_argument("--hz", type=positive, default=50.)
            command.add_argument("--time-uncertainty", type=positive,
                                 help="measured bound on body receive-time error [s]; omitted = unmeasured")
        if name == "record-camera":
            command.add_argument("--host", required=True)
            command.add_argument("--port", type=int, default=5556)
            command.add_argument("--fps", type=positive, default=5.)
            command.add_argument("--optical-frame", required=True)
            command.add_argument("--exposure-verified", action="store_true", help="source timestamp is verified as exposure time")
        if name == "record-ros":
            command.add_argument("--base-frame", default="base_link")
            command.add_argument("--odom-topic", default="/odom", help="measured SLAM odometry, never integrated SDK commands")
    args = parser.parse_args()
    try:
        if args.command == "init":
            initialize(args.session)
        elif args.command == "mark":
            mark(args.session, args.trial, args.event, args.goal_id, args.reason)
        elif args.command == "annotate":
            annotate(args)
        elif args.command == "analyze":
            from .analysis import analyze
            report = analyze(args.session, args.output)
            print(json.dumps({k: report[k] for k in ("data_kind", "expected_attempts", "record_counts", "automatic_settled_measurements")}, ensure_ascii=False))
        elif args.command == "synthetic":
            from .synthetic import generate
            generate(args.session)
        else:
            from . import collect
            getattr(collect, args.command.replace("-", "_"))(args)
    except KeyboardInterrupt:
        return 130
    except (ValueError, OSError, KeyError) as exc:
        parser.exit(2, f"arrival: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
