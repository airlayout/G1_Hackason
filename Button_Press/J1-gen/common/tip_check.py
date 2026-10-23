"""ボトルが倒れたかの自動判定（頭カメラの RGB + 深度）。ボトルを倒す動作（real/knock_bottle.py）で使う。

方法: **ボトルのいちばん高い点の高さ**（pelvis 座標の z）を、押す前と押したあとで比べる。
立っていれば約 20 cm の高さ、倒れて寝ていれば約 6.5 cm（ボトルの太さ）で、差は約 13 cm になる。

なぜ枠の縦横比ではないか: ボトルは机の奥へ倒れる（カメラから遠ざかる向き）ので、寝たボトルは画像の上では縦に長いまま見える
（MuJoCo の頭カメラの画像で確かめた）。奥行きの向きに寝るので、枠の形は立っているときとあまり変わらない。

いちばん高い点の求め方: 検出した枠の上端の帯（枠の高さの top_strip_fraction）の深度のうち、最も手前の画素（立っていればキャップ、
寝ていれば遠いほうの端）だけを使い、その pelvis 座標の z の中央値をとる。枠が少し大きくても、背景（机・壁。ボトルより奥）は
使わない。深度が取れなければ「判定できない」を返す（透明な PET は深度が取れないことがある）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .camera_geometry import HeadCameraTransform
from .localize import Detector
from .rgbd_protocol import RgbdFrame

# 状態
FALLEN = "fallen"      # 倒れた
STANDING = "standing"  # 立っている
LEANING = "leaning"    # 傾いている（倒れきっていない。机に寄りかかっている、など）
UNKNOWN = "unknown"    # 判定できない（見つからない、深度が取れない）

# 終了コード（common/pipeline.py の EXIT_* とは別。0 は倒れた）
EXIT_NOT_TIPPED = 20
EXIT_TIP_UNKNOWN = 21


@dataclass
class TipResult:
    state: str
    message: str
    top_before_m: float | None = None
    top_after_m: float | None = None
    data: dict[str, Any] = field(default_factory=dict)

    @property
    def drop_m(self) -> float | None:
        if self.top_before_m is None or self.top_after_m is None:
            return None
        return self.top_before_m - self.top_after_m

    @property
    def fallen(self) -> bool:
        return self.state == FALLEN

    @property
    def exit_code(self) -> int:
        return 0 if self.state == FALLEN else EXIT_TIP_UNKNOWN if self.state == UNKNOWN else EXIT_NOT_TIPPED


def top_height(frame: RgbdFrame, bbox: tuple[float, float, float, float], transform: HeadCameraTransform,
               q_waist: np.ndarray, cfg: dict[str, Any]) -> tuple[float | None, str]:
    """枠の上端の帯のいちばん手前の画素の高さ（pelvis の z [m]）。求まらなければ (None, 理由)。"""
    depth = frame.depth_m()
    h, w = depth.shape
    x1, y1, x2, y2 = bbox
    u0, u1 = int(max(0, np.floor(x1))), int(min(w, np.ceil(x2)))
    v0 = int(max(0, np.floor(y1)))
    v1 = int(min(h, max(v0 + 2, np.ceil(y1 + (y2 - y1) * float(cfg["top_strip_fraction"])))))
    strip = depth[v0:v1, u0:u1]
    vs, us = np.nonzero((strip > 0) & (strip >= float(cfg["min_m"])) & (strip <= float(cfg["max_m"])))
    if vs.size < int(cfg["min_valid_pixels"]):
        return None, f"上端の帯で有効な深度が {vs.size} 画素（必要 {cfg['min_valid_pixels']}）"
    z = strip[vs, us]
    # 最も手前（5 パーセンタイル。速い雑音を避ける）から near_band_m 以内の画素 = ボトル。それより奥は背景
    near = z <= np.percentile(z, 5) + float(cfg["near_band_m"])
    if int(near.sum()) < int(cfg["min_valid_pixels"]) // 2:
        return None, f"最も手前の画素が {int(near.sum())} 画素しかない"
    intr = frame.intrinsics
    zz = z[near].astype(float)
    pts = np.column_stack([(us[near] + u0 - intr.cx) * zz / intr.fx, (vs[near] + v0 - intr.cy) * zz / intr.fy, zz])
    r, p = transform.pelvis_from_optical(q_waist)
    pel = pts @ r.T + p + transform.offset
    return float(np.median(pel[:, 2])), ""


class TipOverCheck:
    """押す前のいちばん高い点の高さを覚え、押したあとの高さと比べて、倒れたかを判定する。

    使い方（common/pipeline.py が呼ぶ）: observe_before(frames) → 押す → evaluate(frames)。
    frames は (RgbdFrame, 腰の角度 q_waist) のリスト。複数枚の中央値をとる。
    """

    def __init__(self, robot_cfg: dict[str, Any], loc_cfg: dict[str, Any], detector: Detector,
                 cfg: dict[str, Any]) -> None:
        self.cfg = cfg
        self.detector = detector
        self.transform = HeadCameraTransform(robot_cfg, loc_cfg["calibration"]["offset_pelvis_m"])
        self.top_before: float | None = None

    def _measure(self, frames: list[tuple[RgbdFrame, np.ndarray]]) -> tuple[float | None, str]:
        tops: list[float] = []
        reasons: list[str] = []
        for f, qw in frames:
            dets = self.detector.detect(f.color_bgr)
            if not dets:
                reasons.append("ボトルが見つからない")
                continue
            best = max(dets, key=lambda d: d.confidence)
            top, why = top_height(f, tuple(best.bbox), self.transform, qw, self.cfg)
            if top is None:
                reasons.append(why)
            else:
                tops.append(top)
        if not tops:
            return None, " / ".join(sorted(set(reasons))) or "フレームが無い"
        return float(np.median(tops)), ""

    def observe_before(self, frames: list[tuple[RgbdFrame, np.ndarray]]) -> float | None:
        """押す前のいちばん高い点の高さを覚える。求まらなければ None（このあとの判定は「判定できない」）。"""
        self.top_before, why = self._measure(frames)
        if self.top_before is None:
            print(f"[tip] ⚠️ 押す前のボトルの高さが求まらない: {why}")
        else:
            print(f"[tip] 押す前のボトルのいちばん高い点: z = {self.top_before:+.3f} m（pelvis 座標）")
        return self.top_before

    def evaluate(self, frames: list[tuple[RgbdFrame, np.ndarray]]) -> TipResult:
        c = self.cfg
        after, why = self._measure(frames)
        if self.top_before is None:
            return TipResult(UNKNOWN, "押す前の高さが求まっていないので判定できない")
        if after is None:
            return TipResult(UNKNOWN, f"押したあとのボトルの高さが求まらない: {why}"
                             "（机から落ちた、視野の外、深度が取れない）", top_before_m=self.top_before)
        drop = self.top_before - after
        if drop >= float(c["fallen_drop_m"]):
            state, msg = FALLEN, "倒れた"
        elif drop <= float(c["standing_tolerance_m"]):
            state, msg = STANDING, "立っている（倒れなかった）"
        else:
            state, msg = LEANING, "傾いている（倒れきっていない）"
        return TipResult(state, f"{msg}（いちばん高い点が {drop * 100:.1f} cm 下がった。押す前 {self.top_before:+.3f} m → "
                         f"押したあと {after:+.3f} m）", self.top_before, after)
