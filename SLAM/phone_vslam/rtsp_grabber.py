"""スマホの映像（RTSP）を ffmpeg の子プロセスで解き続ける読み口（MASt3R にも DROID にも依らない）。

iPhone の Safari は始めの約 2 秒を 360x640、その後 720x1280 で送る。OpenCV の RTSP の読み口はこの途中の
変化を追えず、決まった大きさのまま壊れたこまを渡すか `_step >= minstep` で落ちる（11-17）。
ffmpeg は入力の大きさが変わると絵の変換を作り直し、決まった大きさに揃えて渡せる。
"""

from __future__ import annotations

import collections
import dataclasses
import pathlib
import shutil
import subprocess
import sys
import threading
import time

import numpy as np

RECONNECT_WAIT_S = 1.0
OPEN_FAIL_LOG_EVERY = 10  # 開けない知らせはこの回数に 1 度だけ出す
# つないだ直後のこまを捨てる秒。途中から読むと次の鍵のこままで壊れたこまが出る
# （11-17 の録画で "reference picture missing"）。WebRTC は始めの 2 秒ほど解像度も低い
STARTUP_SKIP_S = 0.5
LIVE_LONG_SIDE = 640  # 生の流れを解いて揃える長い辺（MASt3R は 512 に縮めるので、それより大きく）


@dataclasses.dataclass
class SourceStats:
    """取り込み口の数（表示用）"""

    received: int = 0  # 受けたこま
    used: int = 0  # SLAM に渡したこま
    reconnects: int = 0
    last_shape: tuple[int, int] = (0, 0)  # 受けた生のこまの (h, w)
    receive_fps: float = 0.0


def _env_tool(name: str) -> str:
    """conda の環境の ffmpeg・ffprobe（無ければ PATH の物）"""
    local = pathlib.Path(sys.executable).parent / name
    return str(local) if local.exists() else (shutil.which(name) or name)


def live_out_size(w: int, h: int, long_side: int = LIVE_LONG_SIDE) -> tuple[int, int]:
    """入力の縦横比のまま、長い辺を long_side にした (w, h)。偶数にそろえる"""
    if w >= h:
        return long_side, max(2, int(round(long_side * h / w / 2)) * 2)
    return max(2, int(round(long_side * w / h / 2)) * 2), long_side


class LatestFrameGrabber:
    """RTSP を ffmpeg で解き続け、いちばん新しいこまを持つ（別の糸）。

    ステレオ用に、届いた時刻つきの直近のこまも history_len 枚まで持つ（frame_near で時刻の近い物を引く）。
    out_wh を渡すと、流れの縦横比に関わらずその大きさに揃える（2 台を同じ大きさで扱うため）。
    """

    def __init__(
        self,
        url: str,
        long_side: int = LIVE_LONG_SIDE,
        out_wh: tuple[int, int] | None = None,
        history_len: int = 0,
    ) -> None:
        self.url = url
        self.long_side = long_side
        self.history: collections.deque[tuple[float, int, np.ndarray]] = collections.deque(maxlen=history_len or 1)
        self.stats = SourceStats()
        self._lock = threading.Condition()
        self._frame: np.ndarray | None = None
        self._seq = 0
        self._stop = False
        self._fails = 0
        self._proc: subprocess.Popen | None = None
        self._out_wh: tuple[int, int] | None = out_wh  # 最初に決めた大きさを、つなぎ直しても保つ
        self._fps_t0 = time.time()
        self._fps_n = 0
        self._thread = threading.Thread(target=self._run, name="rtsp-grabber", daemon=True)
        self._thread.start()

    def _probe(self) -> tuple[int, int] | None:
        """流れの (w, h)。まだ送られていなければ None"""
        r = subprocess.run(
            # 大きさだけ分かればよいので 0.5 秒で切り上げる（既定は 5 秒まで読み、送信開始から画が出るまでが
            # 5〜7 秒かかって、待ちきれずに止められた）
            [_env_tool("ffprobe"), "-v", "error", "-rtsp_transport", "tcp", "-select_streams", "v:0",
             "-analyzeduration", "500000", "-probesize", "200000",
             "-show_entries", "stream=width,height", "-of", "csv=p=0", self.url],
            capture_output=True, text=True, timeout=15,
        )
        try:
            w, h = (int(x) for x in r.stdout.strip().split(",")[:2])
        except ValueError:
            return None
        return (w, h) if w > 0 and h > 0 else None

    def _run(self) -> None:
        while not self._stop:
            try:
                wh = self._probe()
            except subprocess.TimeoutExpired:
                wh = None
            if wh is None:
                if self._fails % OPEN_FAIL_LOG_EVERY == 0:
                    print(f"[取り込み] {self.url} を開けない（まだ送られていない）。{RECONNECT_WAIT_S} 秒おきにやり直す", flush=True)
                self._fails += 1
                time.sleep(RECONNECT_WAIT_S)
                continue
            self._fails = 0
            if self._out_wh is None:
                self._out_wh = live_out_size(*wh, long_side=self.long_side)
            self._read_stream(wh)
            time.sleep(RECONNECT_WAIT_S)

    def _read_stream(self, src_wh: tuple[int, int]) -> None:
        ow, oh = self._out_wh
        vf = (f"scale={ow}:{oh}:force_original_aspect_ratio=decrease,"
              f"pad={ow}:{oh}:(ow-iw)/2:(oh-ih)/2")
        cmd = [_env_tool("ffmpeg"), "-nostdin", "-hide_banner", "-loglevel", "error",
               "-rtsp_transport", "tcp", "-fflags", "nobuffer", "-flags", "low_delay",
               "-i", self.url, "-an", "-vf", vf,
               # 届いたこまをそのまま出す（既定の一定の速さに合わせる形だと、RTSP の時刻の目盛りで
               # 同じこまを毎秒 7,000 枚も複製した）
               "-fps_mode", "passthrough",
               "-pix_fmt", "bgr24", "-f", "rawvideo", "pipe:1"]
        self._proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stdin=subprocess.DEVNULL)
        print(f"[取り込み] {self.url} を開いた（流れ {src_wh[0]}x{src_wh[1]} → {ow}x{oh} に揃える）", flush=True)
        size = ow * oh * 3
        opened_at = time.time()
        try:
            while not self._stop:
                buf = self._proc.stdout.read(size)
                if len(buf) < size:
                    print("[取り込み] 流れが切れた。つなぎ直す", flush=True)
                    self.stats.reconnects += 1
                    break
                if time.time() - opened_at < STARTUP_SKIP_S:
                    continue
                self._push(np.frombuffer(buf, np.uint8).reshape(oh, ow, 3))
        finally:
            self._kill()

    def _kill(self) -> None:
        proc, self._proc = self._proc, None
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()

    def _push(self, bgr: np.ndarray) -> None:
        now = time.time()
        self._fps_n += 1
        if now - self._fps_t0 >= 1.0:
            self.stats.receive_fps = self._fps_n / (now - self._fps_t0)
            self._fps_t0, self._fps_n = now, 0
        with self._lock:
            self._frame = bgr
            self._seq += 1
            self.history.append((now, self._seq, bgr))
            self.stats.received += 1
            self.stats.last_shape = tuple(bgr.shape[:2])
            self._lock.notify_all()

    def frame_near(self, t: float) -> tuple[float, int, np.ndarray] | None:
        """届いた時刻が t にいちばん近いこま（history から）。(届いた時刻, 番号, こま)"""
        with self._lock:
            if not self.history:
                return None
            return min(self.history, key=lambda item: abs(item[0] - t))

    def latest(self) -> tuple[float, int, np.ndarray] | None:
        with self._lock:
            return self.history[-1] if self.history else None

    def wait_newer(self, last_seq: int, timeout: float) -> tuple[int, np.ndarray]:
        """last_seq より新しいこまを待って返す"""
        with self._lock:
            ok = self._lock.wait_for(lambda: self._seq > last_seq, timeout=timeout)
            if not ok or self._frame is None:
                raise TimeoutError(f"{timeout:.0f} 秒こまが来ない（{self.url}）")
            return self._seq, self._frame

    def close(self) -> None:
        self._stop = True
        self._kill()
