"""カメラ映像を取り込み、YOLO で人を検出し、枠を焼き込んだ JPEG を持ち回る。

⚠️ **ここは ROS と無関係。** 映像は G1 本体の ZMQ から直接来る（ROS トピックではない）。
UI のこの部分は航法側が落ちていても動く。

検出と配信を別スレッドにしてあるのは、**ブラウザの本数や描画の遅さが推論の周期に
影響しないようにする**ため。ブラウザは「いまある最新の1枚」を取るだけ。

枠は**サーバ側で焼き込む**。座標だけ送って JavaScript 側で重ねる手もあるが、
フレームと枠がずれて見えるので、まずは焼き込みで作る。

⚠️ **枠あり・枠なしの2系統を持つ。** UI で表示を切り替えられるようにするため。
焼き込み方式では切り替えがサーバまで届く必要があるが、1つの絵を共有すると
**誰かが切り替えると全員の画面が変わる**ので、両方を用意して見る側に選ばせる。
検出が無いフレームでは2枚が同じ絵になるため、**二度エンコードしない**
（サンプル動画では 46% のフレームが該当し、その分の無駄が消える）。
"""
from __future__ import annotations

import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import cv2
import numpy as np

# Perception トラックの実装をそのまま使う（UI 側で作り直さない）
_PERCEPTION = Path(__file__).resolve().parents[3] / "Perception"
if str(_PERCEPTION) not in sys.path:
    sys.path.insert(0, str(_PERCEPTION))

from common.camera.base import FrameSource          # noqa: E402
from common.detector.yolo_detector import Detection, YoloDetector  # noqa: E402

# 枠と文字の色（BGR）。暗い映像の上でも見えるように彩度の高い色にする
_BOX_COLOR = (64, 220, 64)
_TEXT_COLOR = (16, 32, 16)


@dataclass
class DetectionEvent:
    """検知の始まりと終わり。警備ログに残す単位。"""

    kind: str           # "検知" | "消失"
    at: float           # epoch 秒
    count: int          # そのときの人数
    confidence: float   # 最も高い信頼度
    duration_s: float = 0.0   # kind="消失" のときだけ意味を持つ

    def as_dict(self) -> dict[str, object]:
        return {
            "kind": self.kind,
            "at": self.at,
            "count": self.count,
            "confidence": round(self.confidence, 2),
            "duration_s": round(self.duration_s, 1),
        }


class VisionWorker:
    """映像取得 -> YOLO -> JPEG 化 を裏で回し続ける。"""

    def __init__(
        self,
        source: FrameSource,
        detector: YoloDetector,
        target_fps: float = 10.0,
        jpeg_quality: int = 80,
        present_frames: int = 2,
        absent_grace_s: float = 1.5,
        on_event: Callable[[DetectionEvent], None] | None = None,
    ) -> None:
        self._source = source
        self._detector = detector
        self._interval = 1.0 / max(1e-3, target_fps)
        self._jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)]
        # 検知の不感帯。実測(2026-09-20)で、素の判定だと 0.1 秒の「消失」が
        # 21 件中 10 件を占めてログが読めなくなった
        self._present_frames = max(1, int(present_frames))
        self._absent_grace_s = float(absent_grace_s)
        self._on_event = on_event

        self._lock = threading.Lock()
        self._frame_ready = threading.Condition(self._lock)
        self._jpeg_plain: bytes | None = None    # 枠なし
        self._jpeg_boxed: bytes | None = None     # 枠あり
        self._seq = 0
        self._detections: list[Detection] = []
        self._fps_window: deque[float] = deque(maxlen=30)
        self._last_error = ""
        self._alive = False
        self._present_since: float | None = None   # 人が映り始めた時刻（確定後）
        self._last_seen: float = 0.0               # 最後に人が映っていた時刻
        self._streak: int = 0                      # 連続して映っているフレーム数
        # ⚠️ 猶予(absent_grace_s)の間は検出数が 0 になる。そのまま画面に出すと
        # 「人を検知 0 人」という矛盾した警報になるので、その回の値を保っておく
        self._episode_count: int = 0
        self._episode_conf: float = 0.0
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    # ---- 起動と停止 ---------------------------------------------------------

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="vision", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        try:
            self._source.close()
        except Exception as exc:                    # 後始末で落ちても本体は止めない
            print(f"[映像] 後始末で例外: {exc}")

    # ---- 本体 ---------------------------------------------------------------

    def _run(self) -> None:
        try:
            self._source.open()
            self._alive = True
            print(f"[映像] 取り込みを開始した（推論は {1 / self._interval:.0f} fps 上限、"
                  f"device={self._detector.device}）")
        except Exception as exc:
            self._last_error = f"映像ソースを開けません: {exc}"
            print(f"[映像] {self._last_error}")
            return

        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                frame = self._source.read()
            except Exception as exc:
                self._last_error = f"読み取りで例外: {exc}"
                frame = None

            if frame is None:
                # 動画の終端やストリーム断。loop=True の動画ならここには来ない
                self._alive = False
                self._last_error = self._last_error or "映像が途絶えました"
                time.sleep(0.5)
                continue

            self._alive = True
            self._last_error = ""
            detections = self._detector.detect(frame)
            self._note_events(detections)

            ok, buf = cv2.imencode(".jpg", frame, self._jpeg_params)
            if not ok:
                continue
            plain = buf.tobytes()
            if detections:
                ok2, buf2 = cv2.imencode(".jpg", self._annotate(frame, detections),
                                         self._jpeg_params)
                boxed = buf2.tobytes() if ok2 else plain
            else:
                boxed = plain      # 枠が無いので同じ絵。エンコードし直さない

            with self._frame_ready:
                self._jpeg_plain = plain
                self._jpeg_boxed = boxed
                self._detections = detections
                self._seq += 1
                self._fps_window.append(time.monotonic())
                self._frame_ready.notify_all()

            # 目標周期まで待つ（推論が速すぎてCPU/GPUを無駄に使わないように）
            rest = self._interval - (time.monotonic() - t0)
            if rest > 0:
                self._stop.wait(rest)

    def _note_events(self, detections: list[Detection]) -> None:
        """人がいる/いないの切り替わりを検知イベントにする。

        ⚠️ **フレームごとに出さない。** 毎フレーム記録するとログが埋まって読めなくなる。
        「映り始め」と「消えた」の2つだけを残す。

        ⚠️ **不感帯を入れてある。** 素の判定（1フレームでも外れたら消失）にすると、
        検出が1フレームだけ落ちるたびに「消失 0.1秒」→「検知」が並んでログが使い物に
        ならない（2026-09-20 の実測で「消失」21件中10件が1秒未満だった）。
          - 検知の確定: `present_frames` フレーム連続で映っていること
          - 消失の確定: `absent_grace_s` 秒のあいだ一度も映らないこと
        消失の継続時間は**猶予時間を含めない**（最後に映っていた時刻までを数える）。
        """
        now = time.time()
        if detections:
            self._last_seen = now
            self._streak += 1
            self._episode_count = len(detections)
            self._episode_conf = max(d.confidence for d in detections)
            if self._present_since is None and self._streak >= self._present_frames:
                self._present_since = now
                best = max(d.confidence for d in detections)
                self._emit(DetectionEvent("検知", now, len(detections), best))
            return

        self._streak = 0
        if self._present_since is not None and (now - self._last_seen) >= self._absent_grace_s:
            dur = max(0.0, self._last_seen - self._present_since)
            self._present_since = None
            self._emit(DetectionEvent("消失", now, 0, 0.0, duration_s=dur))

    def _emit(self, event: DetectionEvent) -> None:
        if self._on_event is not None:
            self._on_event(event)

    def _annotate(self, frame: np.ndarray, detections: list[Detection]) -> np.ndarray:
        out = frame.copy()
        for det in detections:
            x1, y1, x2, y2 = (int(v) for v in det.bbox)
            cv2.rectangle(out, (x1, y1), (x2, y2), _BOX_COLOR, 2)
            label = f"{det.class_name} {det.confidence:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(out, (x1, max(0, y1 - th - 6)), (x1 + tw + 6, y1), _BOX_COLOR, -1)
            cv2.putText(out, label, (x1 + 3, max(th, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, _TEXT_COLOR, 1, cv2.LINE_AA)
        return out

    # ---- 取り出し -----------------------------------------------------------

    def wait_for_frame(self, last_seq: int, boxes: bool = True,
                       timeout: float = 2.0) -> tuple[bytes | None, int]:
        """`last_seq` より新しいフレームが来るまで待って返す。

        MJPEG 配信で使う。同じ絵を送り直さないための番号付き。
        `boxes=False` なら検出枠を焼いていない方を返す（見る側ごとに選べる）。
        """
        with self._frame_ready:
            if self._seq == last_seq:
                self._frame_ready.wait(timeout)
            return (self._jpeg_boxed if boxes else self._jpeg_plain), self._seq

    @property
    def status(self) -> dict[str, object]:
        with self._lock:
            fps = 0.0
            if len(self._fps_window) >= 2:
                span = self._fps_window[-1] - self._fps_window[0]
                if span > 0:
                    fps = (len(self._fps_window) - 1) / span
            return {
                "alive": self._alive,
                # ⚠️ count は生の検出数（ちらつく）。present は不感帯を通した判定
                "present": self._present_since is not None,
                "fps": round(fps, 1),
                "count": (len(self._detections) or
                          (self._episode_count if self._present_since is not None else 0)),
                "max_confidence": round(
                    max((d.confidence for d in self._detections),
                        default=(self._episode_conf if self._present_since is not None else 0.0)), 2),
                "device": self._detector.device,
                "error": self._last_error,
            }
