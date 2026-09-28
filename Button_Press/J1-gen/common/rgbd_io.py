"""RgbdFrame のファイルへの保存と読み込み。

- カラー: PNG（BGR）
- 深度: 16bit PNG（値をそのまま保存できる。JPEG にはしない）
- 内部パラメータなど: JSON

probe_rgbd.py --save と、収録ツール（タスク5）で使う。
"""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np

from .rgbd_protocol import Intrinsics, RgbdFrame


def save_rgbd(frame: RgbdFrame, stem: Path, extra: dict | None = None) -> None:
    """<stem>_color.png、<stem>_depth.png、<stem>_meta.json に保存する。"""
    stem.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(f"{stem}_color.png", frame.color_bgr)
    cv2.imwrite(f"{stem}_depth.png", frame.depth)
    meta = {"intrinsics": asdict(frame.intrinsics), "depth_scale": frame.depth_scale,
            "timestamp": frame.timestamp, "frame_id": frame.frame_id, "camera": frame.camera}
    meta.update(extra or {})
    Path(f"{stem}_meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))


def load_rgbd(stem: Path) -> tuple[RgbdFrame, dict]:
    """save_rgbd で保存したものを読む。(RgbdFrame, メタデータ全体) を返す。

    probe_rgbd.py の古い保存形式（<stem>_intrinsics.json）も読める。
    """
    color = cv2.imread(f"{stem}_color.png", cv2.IMREAD_COLOR)
    depth = cv2.imread(f"{stem}_depth.png", cv2.IMREAD_UNCHANGED)
    if color is None or depth is None:
        raise FileNotFoundError(f"{stem}_color.png / {stem}_depth.png が読めない")
    if depth.dtype != np.uint16:
        raise ValueError(f"{stem}_depth.png が 16bit ではない（{depth.dtype}）")
    meta_path = Path(f"{stem}_meta.json")
    if not meta_path.exists():
        meta_path = Path(f"{stem}_intrinsics.json")
    meta = json.loads(meta_path.read_text())
    frame = RgbdFrame(
        color_bgr=color, depth=depth, depth_scale=float(meta["depth_scale"]),
        intrinsics=Intrinsics(**meta["intrinsics"]), timestamp=float(meta.get("timestamp", 0.0)),
        frame_id=int(meta.get("frame_id", 0)), camera=meta.get("camera", "head_camera"),
    )
    return frame, meta
