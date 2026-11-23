"""録った流れのこまの並びをまとめる（段 0 の「こま落ちと解像度の動き」の記録）。

    python stream_report.py rec/cam1_1117_1200.mkv [--gap-ms 100] [--json out.json]

ffprobe でこまごとの時刻と大きさを読み、次を出す:
- 長さ・こまの数・平均の fps
- 間が空いた所（既定 100 ms 超）の数と、いちばん長い空き
- 解像度が変わった時刻と、それぞれの解像度で過ごした長さ
"""

from __future__ import annotations

import argparse
import json
import pathlib
import shutil
import subprocess
import sys
from collections import Counter


def probe_frames(path: pathlib.Path) -> list[tuple[float, int, int]]:
    """こまごとの (時刻, 幅, 高さ)。ffprobe の csv は列の順が求めた順にならないので json で読む"""
    ffprobe = shutil.which("ffprobe") or str(pathlib.Path(sys.executable).parent / "ffprobe")
    out = subprocess.run(
        [
            ffprobe, "-v", "error", "-select_streams", "v:0",
            "-show_entries", "frame=best_effort_timestamp_time,width,height",
            "-of", "json", str(path),
        ],
        check=True, capture_output=True, text=True,
    ).stdout
    frames = []
    for f in json.loads(out).get("frames", []):
        t = f.get("best_effort_timestamp_time")
        if t in (None, "N/A") or "width" not in f:
            continue
        frames.append((float(t), int(f["width"]), int(f["height"])))
    frames.sort()
    return frames


def summarize(frames: list[tuple[float, int, int]], gap_ms: float) -> dict:
    if len(frames) < 2:
        return {"frames": len(frames), "error": "こまが 2 つ未満"}
    ts = [f[0] for f in frames]
    dur = ts[-1] - ts[0]
    dts = [(b - a) * 1000 for a, b in zip(ts, ts[1:])]
    gaps = [(ts[i], d) for i, d in enumerate(dts) if d > gap_ms]
    changes = []
    time_at = Counter()
    for i, (t, w, h) in enumerate(frames):
        if i == 0 or (w, h) != frames[i - 1][1:]:
            changes.append({"t": round(t - ts[0], 2), "size": f"{w}x{h}"})
        if i + 1 < len(frames):
            time_at[f"{w}x{h}"] += frames[i + 1][0] - t
    return {
        "frames": len(frames),
        "duration_s": round(dur, 2),
        "mean_fps": round((len(frames) - 1) / dur, 2) if dur > 0 else 0.0,
        "gap_threshold_ms": gap_ms,
        "gaps": len(gaps),
        "max_gap_ms": round(max(dts), 1),
        "worst_gaps": [{"t": round(t - ts[0], 2), "ms": round(d, 1)} for t, d in sorted(gaps, key=lambda x: -x[1])[:5]],
        "size_changes": changes,
        "time_per_size_s": {k: round(v, 1) for k, v in time_at.most_common()},
    }


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("path")
    p.add_argument("--gap-ms", type=float, default=100.0)
    p.add_argument("--json", default="")
    args = p.parse_args()
    rep = summarize(probe_frames(pathlib.Path(args.path)), args.gap_ms)
    text = json.dumps(rep, ensure_ascii=False, indent=2)
    print(text)
    out = pathlib.Path(args.json) if args.json else pathlib.Path(args.path).with_suffix(".report.json")
    out.write_text(text + "\n")
    print(f"[report] {out}")


if __name__ == "__main__":
    main()
