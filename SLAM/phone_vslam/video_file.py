"""録画（mp4・mkv）を ffmpeg で解き、決まった大きさのこまを順に渡す読み口。

iPhone の Safari の送りは始めの約 2 秒が 360x640、その後 720x1280 になり、MediaMTX の録画にもそのまま入る。
OpenCV の読み口（cv2.VideoCapture）は途中で大きさが変わると `_step >= minstep` で落ちる（11-17 の録画で確認）。
生の映像の読み口（rtsp_grabber）と同じく ffmpeg に解かせ、最初の大きさの縦横比のまま長い辺 long_side に揃える。

こまの時刻は ffprobe で読んだ包み（packet）の時刻を順に当てる（WebRTC の映像は B こまが無く、包みとこまが 1 対 1）。
"""

from __future__ import annotations

import json
import subprocess

import numpy as np

from rtsp_grabber import LIVE_LONG_SIDE, _env_tool, live_out_size

DEFAULT_FPS = 30.0


def probe_video(path: str) -> tuple[int, int, list[float]]:
    """(最初の幅, 最初の高さ, 包みの時刻の並び)"""
    r = subprocess.run(
        [_env_tool("ffprobe"), "-v", "error", "-select_streams", "v:0",
         "-show_entries", "stream=width,height:packet=pts_time", "-of", "json", path],
        capture_output=True, text=True, timeout=120,
    )
    if r.returncode != 0:
        raise FileNotFoundError(f"録画を読めない: {path}: {r.stderr.strip()[-300:]}")
    d = json.loads(r.stdout or "{}")
    streams = d.get("streams") or []
    if not streams:
        raise ValueError(f"録画に映像が無い: {path}")
    w, h = int(streams[0]["width"]), int(streams[0]["height"])
    pts = sorted(float(p["pts_time"]) for p in d.get("packets", []) if p.get("pts_time") not in (None, "N/A"))
    return w, h, pts


class FfmpegVideoReader:
    """録画のこまを頭から順に読む。read() は (録画の中の秒, RGB の uint8) か、終わりなら None"""

    def __init__(self, path: str, long_side: int = LIVE_LONG_SIDE) -> None:
        self.path = path
        w, h, self.pts = probe_video(path)
        self.src_wh = (w, h)
        self.out_wh = live_out_size(w, h, long_side=long_side)
        ow, oh = self.out_wh
        vf = f"scale={ow}:{oh}:force_original_aspect_ratio=decrease,pad={ow}:{oh}:(ow-iw)/2:(oh-ih)/2"
        # 記録の段は fatal: こまをそのまま出す（passthrough）と同じ時刻のこまで「dts が増えない」の行が
        # こまごとに出る（害は無い）。読めなかったこまは read() の数で分かる
        cmd = [_env_tool("ffmpeg"), "-nostdin", "-hide_banner", "-loglevel", "fatal", "-i", path,
               "-an", "-vf", vf, "-fps_mode", "passthrough", "-pix_fmt", "rgb24", "-f", "rawvideo", "pipe:1"]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stdin=subprocess.DEVNULL)
        self.n = 0
        self.t0 = self.pts[0] if self.pts else 0.0
        self.last_t = 0.0

    def read(self) -> tuple[float, np.ndarray] | None:
        ow, oh = self.out_wh
        size = ow * oh * 3
        buf = self.proc.stdout.read(size)
        if len(buf) < size:
            return None
        t = self.pts[self.n] if self.n < len(self.pts) else self.t0 + self.n / DEFAULT_FPS
        self.n += 1
        self.last_t = t - self.t0
        return self.last_t, np.frombuffer(buf, np.uint8).reshape(oh, ow, 3)

    def close(self) -> None:
        if self.proc.poll() is None:
            self.proc.kill()
        self.proc.wait(timeout=10)
