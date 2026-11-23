"""スマホの映像（RTSP）と録画を、MASt3R-SLAM の dataset の形で渡す。

上流の Webcam は cv2 の受け皿を順に読むので、処理が遅いと古いこまが溜まって遅れが伸びる。
ここでは別の糸で読み続けて「いちばん新しい 1 こま」だけを残す（処理が追いつかない分は捨てる）。

スマホの向きが変わると縦横が入れ替わる・WebRTC が帯に合わせて解像度を変える。
SLAM は最初のこまの大きさで地図の箱を作るので、2 こま目以降は最初の向き・縦横比・大きさに揃える。

生の流れは OpenCV ではなく ffmpeg の子プロセスで解く。iPhone の Safari は始めの約 2 秒を 360x640、
その後 720x1280 で送る。OpenCV の RTSP の読み口はこの途中の変化を追えず、決まった大きさのまま
壊れたこまを渡すか、`_step >= minstep` で落ちる（11-17 に生の SLAM が 2 こま目から見失い続けた）。
ffmpeg は入力の大きさが変わると絵の変換を作り直し、決まった大きさに揃えて渡せる。
"""

from __future__ import annotations

import time

import cv2
import numpy as np
from mast3r_slam.dataloader import MonocularDataset

from rtsp_grabber import (  # noqa: F401  （phone_source から使っていた名前を残す）
    LIVE_LONG_SIDE,
    LatestFrameGrabber,
    SourceStats,
    live_out_size,
)

LIVE_LEN = 10**9  # 生の流れは終わりが無い
MAX_PLAUSIBLE_FPS = 240.0  # これを超える fps の欄は時間の目盛り（mkv の 1000 など）とみなす
FIRST_FRAME_TIMEOUT_S = 60.0


class FrameConformer:
    """2 こま目以降を、最初のこまの向き・縦横比・大きさに揃える"""

    def __init__(self) -> None:
        self.ref_hw: tuple[int, int] | None = None

    def __call__(self, img: np.ndarray) -> np.ndarray:
        h, w = img.shape[:2]
        if self.ref_hw is None:
            self.ref_hw = (h, w)
            return img
        rh, rw = self.ref_hw
        if (h > w) != (rh > rw):
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
            h, w = img.shape[:2]
        if (h, w) == (rh, rw):
            return img
        # 縦横比を合わせて中央を切り、最初の大きさに伸ばす
        ref_ratio, ratio = rw / rh, w / h
        if ratio > ref_ratio:
            cw = int(round(h * ref_ratio))
            x0 = (w - cw) // 2
            img = img[:, x0 : x0 + cw]
        elif ratio < ref_ratio:
            ch = int(round(w / ref_ratio))
            y0 = (h - ch) // 2
            img = img[y0 : y0 + ch]
        return cv2.resize(img, (rw, rh), interpolation=cv2.INTER_AREA)


class LiveRtspDataset(MonocularDataset):
    """スマホの生の流れ。i 番目を求められたら、その時点でいちばん新しいこまを返す"""

    def __init__(self, url: str, frame_timeout_s: float = FIRST_FRAME_TIMEOUT_S) -> None:
        super().__init__()
        self.use_calibration = False
        self.save_results = False
        self.dataset_path = None
        self.url = url
        self.frame_timeout_s = frame_timeout_s
        self.grabber = LatestFrameGrabber(url)
        self.conform = FrameConformer()
        self._seq = 0
        self._first: np.ndarray | None = None
        self.timestamps = []

    @property
    def stats(self) -> SourceStats:
        return self.grabber.stats

    def __len__(self) -> int:
        return LIVE_LEN

    def subsample(self, subsample: int) -> None:
        # 生の流れは「いちばん新しいこま」を取るので間引きは要らない
        del subsample

    def read_img(self, idx: int) -> np.ndarray:
        if idx == 0 and self._first is not None and not self.timestamps:
            # get_img_shape で読んだ最初のこまを、そのまま 0 番として使う
            img = self._first
        else:
            self._seq, bgr = self.grabber.wait_newer(self._seq, self.frame_timeout_s)
            img = self.conform(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
            if self._first is None:
                self._first = img
                return img  # get_img_shape から（時刻は積まない）
        self.timestamps.append(time.time())
        self.stats.used += 1
        return img

    def get_timestamp(self, idx: int) -> float:
        return self.timestamps[idx]


class VideoFileDataset(MonocularDataset):
    """録画のファイル（mp4・mkv・mov）。realtime なら生の流れと同じく、間に合わないこまを捨てる。

    こまの時刻は fps の欄ではなく、こまごとの時刻（CAP_PROP_POS_MSEC）で追う。record.sh が RTSP を
    そのまま写した mkv は時間の目盛りが 1/1000 秒で、fps の欄が 1000 になる（11-17 に realtime の
    流しが 2 こま目で「録画の終わり」になった）。
    """

    def __init__(self, path: str, realtime: bool = False, stride: int = 1) -> None:
        super().__init__()
        self.use_calibration = False
        self.save_results = True
        self.dataset_path = path
        self.realtime = realtime
        self.stride = max(1, stride)
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise FileNotFoundError(f"録画を開けない: {path}")
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.nominal_fps = fps if 0 < fps <= MAX_PLAUSIBLE_FPS else 0.0  # 表示用。時刻には使わない
        self.total = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.conform = FrameConformer()
        self.stats = SourceStats(receive_fps=self.nominal_fps)
        self._pos = 0  # 次に読むこまの番号
        self._t_first: float | None = None  # 最初のこまの録画の中の時刻
        self._t_last = 0.0  # いま読んだこまの、最初のこまからの秒
        self._t0: float | None = None  # 0 番を SLAM に渡した壁の時刻
        self._first: np.ndarray | None = None
        self._ended = False
        self.timestamps = []

    def __len__(self) -> int:
        if self.realtime:
            return LIVE_LEN if not self._ended else len(self.timestamps)
        return max(1, self.total // self.stride)

    def subsample(self, subsample: int) -> None:
        self.stride = max(1, self.stride * subsample)

    def _grab(self) -> bool:
        if not self.cap.grab():
            return False
        t = self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        if self._t_first is None:
            self._t_first = t
        self._t_last = t - self._t_first
        self._pos += 1
        self.stats.received = self._pos
        return True

    def _retrieve(self) -> np.ndarray | None:
        ok, bgr = self.cap.retrieve()
        if not ok:
            return None
        self.stats.last_shape = tuple(bgr.shape[:2])
        return self.conform(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))

    def _read_skip(self, n_skip: int) -> np.ndarray | None:
        """n_skip こまを解かずに飛ばし、その次のこまを返す"""
        for _ in range(n_skip + 1):
            if not self._grab():
                return None
        return self._retrieve()

    def _read_until(self, t_target: float) -> np.ndarray | None:
        """録画の中の時刻が t_target に届くまで解かずに飛ばし、届いたこまを返す"""
        while True:
            if not self._grab():
                return None
            if self._t_last >= t_target:
                return self._retrieve()

    def read_img(self, idx: int) -> np.ndarray:
        if self._first is None:
            self._first = self._read_skip(0)
            if self._first is None:
                raise ValueError(f"録画が空: {self.dataset_path}")
            return self._first  # get_img_shape から
        if idx == 0:
            img = self._first
            self._t0 = time.time()
        elif self.realtime:
            img = self._read_until(time.time() - self._t0)
        else:
            img = self._read_skip(self.stride - 1)
        if img is None:
            self._ended = True
            raise StopIteration
        self.timestamps.append(self._t_last)
        self.stats.used += 1
        return img

    def get_timestamp(self, idx: int) -> float:
        return self.timestamps[idx]
