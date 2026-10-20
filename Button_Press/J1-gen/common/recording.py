"""収録（RGB、深度、lowstate を時刻付きで保存）と再生。タスク5。

収録 1 回分のフォルダ（_local/button_press/recordings/<日時>_<ラベル>/）:

    meta.json        内部パラメータ、深度の単位、使った設定、git のコミット、コマンドの引数
    frames.jsonl     1 行 1 フレーム: 番号、受信した時刻、送信側の時刻、ファイル名、そのときの腰の角度
    frames/          000001_color.png（または .jpg）、000001_depth.png（16bit）
    lowstate.csv     時刻、mode_machine、関節角 q0..q28、関節速度 dq0..dq28、IMU の四元数（w, x, y, z）

- 時刻はすべてラボ PC の time.time()（PC2 の時計とずれていても、フレームと lowstate の対応が取れるように）
- 途中で落ちてもそれまでの分が残るよう、1 行ずつ追記して flush する
- 深度が使えないとき（RGB だけ）は、depth のファイルが無い行になる

実機で撮ったデータは取り直せない。収録が終わったら、必ずバックアップする（README 参照）。
"""

from __future__ import annotations

import csv
import json
import subprocess
import time
from bisect import bisect_left
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .arm.types import JointState
from .config import REPO_ROOT
from .rgbd_protocol import Intrinsics, RgbdFrame
from .robot_model import NUM_MOTORS, WAIST_IDX

LOWSTATE_HEADER = (["t", "mode_machine"] + [f"q{i}" for i in range(NUM_MOTORS)]
                   + [f"dq{i}" for i in range(NUM_MOTORS)] + ["qw", "qx", "qy", "qz"])


def git_commit() -> str:
    try:
        return subprocess.run(["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"], capture_output=True,
                              text=True, check=True).stdout.strip()
    except Exception:  # noqa: BLE001  git が無くても収録は続ける
        return "unknown"


class RecordingWriter:
    def __init__(self, root: Path, label: str, meta: dict[str, Any], color_format: str = "png",
                 jpeg_quality: int = 95) -> None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        name = f"{stamp}_{label}" if label else stamp
        self.dir = root / name
        (self.dir / "frames").mkdir(parents=True, exist_ok=False)
        if color_format not in ("png", "jpg"):
            raise ValueError(f"color_format は png か jpg: {color_format}")
        self.color_format = color_format
        self.jpeg_quality = jpeg_quality
        self.meta = dict(meta, created=stamp, git_commit=git_commit(), color_format=color_format)
        self._write_meta()
        self._frames = open(self.dir / "frames.jsonl", "a", encoding="utf-8")
        self._lowstate = open(self.dir / "lowstate.csv", "a", newline="", encoding="utf-8")
        self._csv = csv.writer(self._lowstate)
        self._csv.writerow(LOWSTATE_HEADER)
        self.n_frames = 0
        self.n_lowstate = 0

    def _write_meta(self) -> None:
        (self.dir / "meta.json").write_text(json.dumps(self.meta, indent=2, ensure_ascii=False))

    def add_frame(self, color_bgr: np.ndarray, depth: np.ndarray | None, t_recv: float,
                  frame: RgbdFrame | None, state: JointState | None) -> int:
        """1 フレームを保存する。深度が無い（RGB だけ）ときは depth=None、frame=None でよい。"""
        self.n_frames += 1
        i = self.n_frames
        color_name = f"frames/{i:06d}_color.{self.color_format}"
        params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality] if self.color_format == "jpg" else []
        if not cv2.imwrite(str(self.dir / color_name), color_bgr, params):
            raise RuntimeError(f"書き込めない: {color_name}")
        rec: dict[str, Any] = {"index": i, "t_recv": t_recv, "color": color_name}
        if depth is not None:
            depth_name = f"frames/{i:06d}_depth.png"
            if not cv2.imwrite(str(self.dir / depth_name), depth):
                raise RuntimeError(f"書き込めない: {depth_name}")
            rec["depth"] = depth_name
        if frame is not None:
            rec.update(t_sender=frame.timestamp, frame_id=frame.frame_id)
            if "intrinsics" not in self.meta:
                self.meta.update(intrinsics=frame.intrinsics.__dict__, depth_scale=frame.depth_scale)
                self._write_meta()
        if state is not None:
            rec["waist_q"] = [float(v) for v in state.q[list(WAIST_IDX)]]
            rec["lowstate_age_s"] = time.monotonic() - state.stamp
        self._frames.write(json.dumps(rec) + "\n")
        self._frames.flush()
        return i

    def add_lowstate(self, t: float, st: JointState) -> None:
        self._csv.writerow([f"{t:.6f}", st.mode_machine] + [f"{v:.6f}" for v in st.q]
                           + [f"{v:.6f}" for v in st.dq] + [f"{v:.6f}" for v in st.imu_quat])
        self.n_lowstate += 1
        if self.n_lowstate % 50 == 0:
            self._lowstate.flush()

    def close(self) -> None:
        self.meta.update(n_frames=self.n_frames, n_lowstate=self.n_lowstate, closed=time.strftime("%Y%m%d_%H%M%S"))
        self._write_meta()
        self._frames.close()
        self._lowstate.close()


@dataclass
class RecordedFrame:
    index: int
    t_recv: float
    color_bgr: np.ndarray
    rgbd: RgbdFrame | None  # 深度が無ければ None
    waist_q: np.ndarray | None  # 収録したときの腰の角度（無ければ lowstate から近い時刻の値）


class Recording:
    """収録 1 回分を読む。"""

    def __init__(self, path: Path) -> None:
        self.dir = Path(path)
        self.meta = json.loads((self.dir / "meta.json").read_text())
        with open(self.dir / "frames.jsonl", encoding="utf-8") as f:
            self.frames = [json.loads(line) for line in f if line.strip()]
        self._t: list[float] = []
        self._q: list[np.ndarray] = []
        ls = self.dir / "lowstate.csv"
        if ls.exists():
            with open(ls, encoding="utf-8") as f:
                r = csv.reader(f)
                next(r, None)
                for row in r:
                    if len(row) != len(LOWSTATE_HEADER):
                        continue  # 途中で落ちたときの書きかけの行
                    self._t.append(float(row[0]))
                    self._q.append(np.array([float(v) for v in row[2:2 + NUM_MOTORS]]))
        intr = self.meta.get("intrinsics")
        self.intrinsics = Intrinsics(**intr) if intr else None
        self.depth_scale = float(self.meta.get("depth_scale", 0.001))

    def __len__(self) -> int:
        return len(self.frames)

    @property
    def lowstate_count(self) -> int:
        return len(self._t)

    def q_at(self, t: float) -> np.ndarray | None:
        """時刻 t に最も近い lowstate の関節角（29）。lowstate が無ければ None。"""
        if not self._t:
            return None
        i = bisect_left(self._t, t)
        cands = [j for j in (i - 1, i) if 0 <= j < len(self._t)]
        j = min(cands, key=lambda k: abs(self._t[k] - t))
        return self._q[j]

    def load(self, k: int) -> RecordedFrame:
        """k 番目（0 から）のフレームを読む。"""
        rec = self.frames[k]
        color = cv2.imread(str(self.dir / rec["color"]), cv2.IMREAD_COLOR)
        if color is None:
            raise FileNotFoundError(self.dir / rec["color"])
        rgbd = None
        if "depth" in rec and self.intrinsics is not None:
            depth = cv2.imread(str(self.dir / rec["depth"]), cv2.IMREAD_UNCHANGED)
            rgbd = RgbdFrame(color_bgr=color, depth=depth, depth_scale=self.depth_scale, intrinsics=self.intrinsics,
                             timestamp=float(rec.get("t_sender", rec["t_recv"])),
                             frame_id=int(rec.get("frame_id", rec["index"])))
        if "waist_q" in rec:
            waist = np.asarray(rec["waist_q"], dtype=float)
        else:
            q = self.q_at(float(rec["t_recv"]))
            waist = None if q is None else q[list(WAIST_IDX)].copy()
        return RecordedFrame(index=int(rec["index"]), t_recv=float(rec["t_recv"]), color_bgr=color,
                             rgbd=rgbd, waist_q=waist)

    def __iter__(self) -> Iterator[RecordedFrame]:
        for k in range(len(self.frames)):
            yield self.load(k)
