"""シミュレーションの頭カメラの画像を、実機に近づける乱し（実機を触れない間に、壊れ方を先に見つけるため）。

MuJoCo の深度は欠けもノイズも無いが、実物のゴミ箱はステンレスの鏡面で、上面の深度が欠けやすい（矢田さんの指摘）。
ここでは次を入れる（どれも Realism の値が 0 なら入れない）:
- 上面（すり鉢とフラップ）の深度を、ひとかたまりずつ欠けさせる（鏡面で反射が返ってこない画素の代わり）
- 深度のノイズ（距離に比例。RealSense D435 は 1 m で 1 cm 前後のばらつき）
- 頭カメラの向き・較正のずれは、シミュレーションのカメラ（本当の向き）と、計算で使う向きを別にして入れる
  （sim/approach_sim.py の --cam-pitch-true / --calib-error-m）
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from typing import Any

import numpy as np

MIRROR_GEOMS = ("bin_funnel", "bin_flap")  # 鏡面として深度を欠けさせる部品（名前の頭）


@dataclass
class Realism:
    depth_dropout: float = 0.0   # 上面の画素のうち、深度を欠けさせる割合（0〜1）
    blob_px: int = 24            # 欠ける塊の大きさ [画素]
    depth_noise: float = 0.0     # 深度のノイズの標準偏差（1 m あたり [m]。距離の 2 乗に比例させる）

    def active(self) -> bool:
        return self.depth_dropout > 0 or self.depth_noise > 0


def mirror_mask(model: Any, data: Any, renderer: Any, camera_name: str) -> np.ndarray:
    """頭カメラの画像で、鏡面として扱う部品（すり鉢とフラップ）が写っている画素（True / False）。"""
    import mujoco

    # 「NO」のステッカー（bin_flap_sticker）はつや消しなので鏡面にしない
    ids = [g for g in range(model.ngeom) if (model.geom(g).name or "").startswith(MIRROR_GEOMS)
           and not (model.geom(g).name or "").endswith("_sticker")]
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera=camera_name)
    seg = renderer.render().copy()
    renderer.disable_segmentation_rendering()
    return np.isin(seg[:, :, 0], ids) & (seg[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))


def corrupt(frame: Any, mask: np.ndarray, real: Realism, rng: np.random.Generator) -> Any:
    """RgbdFrame の深度に、欠けとノイズを入れたものを返す（元の frame は変えない）。"""
    import cv2

    depth = frame.depth.astype(np.float32) * frame.depth_scale
    h, w = depth.shape
    if real.depth_dropout > 0 and mask.any():
        # 粗い格子に乱数を置いて拡大し、ひとかたまりずつ欠けさせる。割合が depth_dropout になるように、しきい値を決める
        gh, gw = max(1, h // real.blob_px), max(1, w // real.blob_px)
        noise = cv2.resize(rng.random((gh, gw)).astype(np.float32), (w, h), interpolation=cv2.INTER_LINEAR)
        vals = noise[mask]
        thr = np.quantile(vals, real.depth_dropout) if len(vals) else 0.0
        depth[mask & (noise < thr)] = 0.0
    if real.depth_noise > 0:
        ok = depth > 0
        depth[ok] += rng.normal(0.0, 1.0, size=int(ok.sum())).astype(np.float32) * real.depth_noise * depth[ok] ** 2
    out = np.where(depth > 0, np.round(depth / frame.depth_scale), 0).astype(np.uint16)
    return dataclasses.replace(frame, depth=out)
