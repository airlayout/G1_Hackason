"""全体をつなぐスクリプトの 1 回分の記録（dry-run のときも必ず残す）。タスク6。

保存先: _local/button_press/runs/<日時>_<ラベル>/

    meta.json       コマンドの引数、経路、dry-run か、使った設定（較正値を含む）、git のコミット、結果
    events.jsonl    1 行 1 出来事（段階の開始・終了、検出結果、IK の結果、確認の結果、エラーなど）
    frames/         段階ごとの RGB（PNG）と深度（16bit PNG）、検出結果を描いた画像
    plan_*.json     押し込みの計画（目標、手前・押し込み終わりの点、IK の結果の関節角、軌道の全点）
    commands.csv    送った（dry-run では送るはずだった）指令の全周期: 時刻、weight、q / kp / kd / tau（29 関節）
    lowstate.csv    制御の周期ごとの lowstate（時刻、関節角、関節速度、IMU）
"""

from __future__ import annotations

import csv
import json
import time
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .arm.backend import ArmBackend
from .arm.types import JointCommand, JointState
from .recording import LOWSTATE_HEADER, git_commit
from .rgbd_protocol import RgbdFrame
from .robot_model import NUM_MOTORS

COMMAND_HEADER = (["t", "weight"] + [f"q{i}" for i in range(NUM_MOTORS)] + [f"kp{i}" for i in range(NUM_MOTORS)]
                  + [f"kd{i}" for i in range(NUM_MOTORS)] + [f"tau{i}" for i in range(NUM_MOTORS)])


def to_jsonable(x: Any) -> Any:
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    if is_dataclass(x) and not isinstance(x, type):
        return {k: to_jsonable(v) for k, v in asdict(x).items()}
    if isinstance(x, dict):
        return {str(k): to_jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [to_jsonable(v) for v in x]
    if isinstance(x, Path):
        return str(x)
    return x


class RunLogger:
    def __init__(self, root: Path, label: str, meta: dict[str, Any]) -> None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.dir = Path(root) / (f"{stamp}_{label}" if label else stamp)
        (self.dir / "frames").mkdir(parents=True, exist_ok=False)
        self.meta = dict(meta, created=stamp, git_commit=git_commit())
        self._write_meta()
        self._events = open(self.dir / "events.jsonl", "a", encoding="utf-8")
        self._cmd_f = open(self.dir / "commands.csv", "a", newline="", encoding="utf-8")
        self._cmd = csv.writer(self._cmd_f)
        self._cmd.writerow(COMMAND_HEADER)
        self._ls_f = open(self.dir / "lowstate.csv", "a", newline="", encoding="utf-8")
        self._ls = csv.writer(self._ls_f)
        self._ls.writerow(LOWSTATE_HEADER)
        self._n_frames = 0
        self.n_commands = 0
        self._t0 = time.time()
        print(f"[run] 記録: {self.dir}")

    def _write_meta(self) -> None:
        (self.dir / "meta.json").write_text(json.dumps(to_jsonable(self.meta), indent=2, ensure_ascii=False))

    def update_meta(self, **kw: Any) -> None:
        self.meta.update(kw)
        self._write_meta()

    def event(self, name: str, **data: Any) -> None:
        rec = {"t": time.time(), "elapsed_s": round(time.time() - self._t0, 3), "event": name}
        rec.update(to_jsonable(data))
        self._events.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self._events.flush()

    def save_frame(self, tag: str, frame: RgbdFrame | None, **extra: Any) -> str | None:
        if frame is None:
            return None
        self._n_frames += 1
        stem = f"frames/{self._n_frames:04d}_{tag}"
        cv2.imwrite(str(self.dir / f"{stem}_color.png"), frame.color_bgr)
        cv2.imwrite(str(self.dir / f"{stem}_depth.png"), frame.depth)
        self.event("frame", tag=tag, file=stem, frame_id=frame.frame_id, timestamp=frame.timestamp,
                   intrinsics=frame.intrinsics, depth_scale=frame.depth_scale, **extra)
        return stem

    def save_image(self, tag: str, img: np.ndarray) -> str:
        self._n_frames += 1
        name = f"frames/{self._n_frames:04d}_{tag}.png"
        cv2.imwrite(str(self.dir / name), img)
        return name

    def save_json(self, name: str, obj: Any) -> None:
        (self.dir / name).write_text(json.dumps(to_jsonable(obj), indent=2, ensure_ascii=False))

    def log_command(self, cmd: JointCommand) -> None:
        self._cmd.writerow([f"{time.time():.6f}", f"{cmd.weight:.4f}"] + [f"{v:.6f}" for v in cmd.q]
                           + [f"{v:.3f}" for v in cmd.kp] + [f"{v:.3f}" for v in cmd.kd]
                           + [f"{v:.4f}" for v in cmd.tau])
        self.n_commands += 1
        if self.n_commands % 50 == 0:
            self._cmd_f.flush()

    def log_state(self, st: JointState) -> None:
        self._ls.writerow([f"{time.time():.6f}", st.mode_machine] + [f"{v:.6f}" for v in st.q]
                          + [f"{v:.6f}" for v in st.dq] + [f"{v:.6f}" for v in st.imu_quat])

    def close(self, **result: Any) -> None:
        self.update_meta(result=result, n_commands=self.n_commands, closed=time.strftime("%Y%m%d_%H%M%S"))
        for f in (self._events, self._cmd_f, self._ls_f):
            f.close()


class LoggingBackend(ArmBackend):
    """送信側（ArmBackend）に記録の層をかぶせる。送った（dry-run では送るはずだった）指令と、
    制御の周期ごとの lowstate を RunLogger に書く。"""

    def __init__(self, inner: ArmBackend, logger: RunLogger) -> None:
        self.inner = inner
        self.logger = logger
        self.name = inner.name
        self.uses_weight = inner.uses_weight
        self.dry_run = inner.dry_run
        self.needs_support_check = inner.needs_support_check

    def open(self) -> None:
        self.inner.open()

    def read_state(self) -> JointState | None:
        return self.inner.read_state()

    def send(self, cmd: JointCommand) -> None:
        self.logger.log_command(cmd)
        self.inner.send(cmd)

    def tick(self, dt: float) -> None:
        self.inner.tick(dt)
        st = self.inner.read_state()
        if st is not None:
            self.logger.log_state(st)

    def verify_peer(self, state: JointState) -> None:
        self.inner.verify_peer(state)

    def close(self) -> None:
        self.inner.close()
