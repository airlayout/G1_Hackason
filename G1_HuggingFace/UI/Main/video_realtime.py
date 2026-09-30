"""動画ファイルを**実時間で**流す。

⚠️ **素の `VideoFileSource` は「次の1フレーム」を返すだけで、時間の概念が無い。**
24fps の動画を 10fps で読むと 20 秒の映像が 48 秒かかる（**2.4 倍遅い**）。
UI の見た目が間延びするだけでなく、**検知ログの継続時間が同じ倍率で水増しされる**
ので、警備の記録としては誤りになる（2026-09-20 に実測して判明）。

ここでは壁時計に合わせて**余ったフレームを捨てる**。YOLO に渡る枚数は
`target_fps` のままで、捨てる分は復号するだけなので負荷はほとんど増えない
（720p の復号は推論より桁違いに軽い）。

⚠️ **実機の ZMQ には要らない。** `ZmqFrameSource` は `CONFLATE=True` で常に最新の
1枚しか持たないため、何 Hz で読んでも実時間の標本になる。この包みは
**動画ファイル専用**。
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2
import numpy as np

_PERCEPTION = Path(__file__).resolve().parents[3] / "Perception"
if str(_PERCEPTION) not in sys.path:
    sys.path.insert(0, str(_PERCEPTION))

from common.camera.base import FrameSource          # noqa: E402
from common.camera.video_file import VideoFileSource  # noqa: E402

# 一度に捨てる上限。これを超えるほど遅れたら追いかけるのを諦めて基準を引き直す
# （推論が詰まったときに「遅れを取り戻そうとしてさらに詰まる」のを防ぐ）
_MAX_SKIP = 120


class RealtimeVideoSource(FrameSource):
    """動画ファイルを実時間の速さで流す `FrameSource`。"""

    def __init__(self, path: str, loop: bool = True, fps: float | None = None) -> None:
        self._path = path
        self._inner = VideoFileSource(path, loop=loop)
        self._fps = float(fps) if fps else self._probe_fps(path)
        self._t0: float | None = None
        self._index = 0          # これまでに消費したフレーム数
        self._dropped = 0        # 捨てた枚数（動いているかの確認用）

    @staticmethod
    def _probe_fps(path: str) -> float:
        cap = cv2.VideoCapture(path)
        fps = cap.get(cv2.CAP_PROP_FPS)
        cap.release()
        if not fps or fps <= 0 or fps > 1000:
            # 読めない動画がある。その場合は捨てずにそのまま流す（従来どおり）
            print(f"[映像] fps を読めなかったので実時間再生を諦める: {path}")
            return 0.0
        return float(fps)

    def open(self) -> None:
        self._inner.open()
        if self._fps > 0:
            print(f"[映像] 実時間で再生する（{self._fps:.1f} fps ぶんを消費し、"
                  f"余りは捨てる）")

    def read(self) -> np.ndarray | None:
        if self._fps <= 0:
            return self._inner.read()

        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now

        want = int((now - self._t0) * self._fps)
        skip = want - self._index
        if skip > _MAX_SKIP:
            # 追いつけないほど遅れた。基準を引き直して無限に追いかけない
            self._t0 = now - self._index / self._fps
            skip = _MAX_SKIP
        for _ in range(max(0, skip)):
            if self._inner.read() is None:
                return None
            self._index += 1
            self._dropped += 1

        frame = self._inner.read()
        if frame is None:
            return None
        self._index += 1
        return frame

    def close(self) -> None:
        self._inner.close()

    @property
    def dropped(self) -> int:
        """捨てた枚数。実時間再生が効いているかの確認に使う。"""
        return self._dropped
