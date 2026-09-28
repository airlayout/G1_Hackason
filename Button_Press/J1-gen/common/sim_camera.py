"""MuJoCo の頭カメラで、RealSense と同じ形の RgbdFrame（カラー + 位置合わせ済みの深度 + 内部パラメータ）を作る。

タスク4（座標変換の確認）と タスク6（全体をつなぐスクリプトのシミュレーション）で使う。
深度は 1 mm 単位の uint16（RealSense の既定の depth_scale = 0.001 と同じ）にする。

MuJoCo のカメラは画素が正方形で、画角 fovy は縦方向。内部パラメータは
fy = fx = (高さ / 2) / tan(fovy / 2)、画像の中心 cx = (幅 − 1) / 2、cy = (高さ − 1) / 2（画素の中心が整数の座標）。
"""

from __future__ import annotations

import time
from typing import Any

import numpy as np

from .rgbd_protocol import Intrinsics, RgbdFrame


def mujoco_intrinsics(width: int, height: int, fovy_deg: float) -> Intrinsics:
    f = (height / 2) / np.tan(np.radians(fovy_deg) / 2)
    return Intrinsics(width=width, height=height, fx=float(f), fy=float(f),
                      cx=(width - 1) / 2, cy=(height - 1) / 2)


class SimHeadCamera:
    def __init__(self, model: Any, data: Any, robot_cfg: dict[str, Any]) -> None:
        import mujoco

        self._mj = mujoco
        cam = robot_cfg["head_camera"]
        self.name = cam["name"]
        self.model, self.data = model, data
        self.width, self.height = int(cam["width"]), int(cam["height"])
        self.intrinsics = mujoco_intrinsics(self.width, self.height, float(cam["fovy_deg"]))
        self._renderer = mujoco.Renderer(model, self.height, self.width)
        self._frame_id = 0

    def render(self) -> RgbdFrame:
        r = self._renderer
        r.disable_depth_rendering()
        r.disable_segmentation_rendering()
        r.update_scene(self.data, camera=self.name)
        rgb = r.render().copy()
        r.enable_depth_rendering()
        r.update_scene(self.data, camera=self.name)
        depth_m = r.render().copy()
        r.disable_depth_rendering()
        # 遠すぎる（何も無い）ところと、RealSense が測れない近さは 0 にする
        depth = np.where((depth_m > 0.1) & (depth_m < 10.0), np.round(depth_m * 1000.0), 0).astype(np.uint16)
        self._frame_id += 1
        return RgbdFrame(color_bgr=rgb[:, :, ::-1].copy(), depth=depth, depth_scale=0.001,
                         intrinsics=self.intrinsics, timestamp=time.time(), frame_id=self._frame_id,
                         camera=self.name)

    def segmentation_bbox(self, geom_prefix: str) -> tuple[float, float, float, float] | None:
        """名前が geom_prefix で始まる geom が写っている範囲の枠 (x1, y1, x2, y2)。写っていなければ None。"""
        mj, m = self._mj, self.model
        ids = {i for i in range(m.ngeom) if (mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, i) or "").startswith(geom_prefix)}
        r = self._renderer
        r.enable_segmentation_rendering()
        r.update_scene(self.data, camera=self.name)
        seg = r.render().copy()
        r.disable_segmentation_rendering()
        mask = np.isin(seg[:, :, 0], list(ids)) & (seg[:, :, 1] == int(mj.mjtObj.mjOBJ_GEOM))
        if not mask.any():
            return None
        vs, us = np.nonzero(mask)
        return float(us.min()), float(vs.min()), float(us.max() + 1), float(vs.max() + 1)

    def ray_hit_pelvis(self, u: float, v: float) -> np.ndarray | None:
        """画素 (u, v) の方向へカメラから光線を飛ばし、最初に当たる点を pelvis 座標で返す（正解の値）。

        深度画像を使わずに求めるので、深度 → 逆投影 → pelvis 座標 の変換を確かめるのに使う。
        """
        mj, m, d = self._mj, self.model, self.data
        c = m.camera(self.name).id
        intr = self.intrinsics
        x, y = (u - intr.cx) / intr.fx, (v - intr.cy) / intr.fy
        # 光学座標（x 右、y 下、z 前）→ MuJoCo のカメラ座標（x 右、y 上、−z 前）
        vec = d.cam_xmat[c].reshape(3, 3) @ np.array([x, -y, -1.0])
        geomid = np.zeros(1, dtype=np.int32)
        dist = mj.mj_ray(m, d, d.cam_xpos[c], vec, None, 1, -1, geomid)
        if dist < 0:
            return None
        hit = d.cam_xpos[c] + dist * vec
        pel = m.body("pelvis").id
        return d.xmat[pel].reshape(3, 3).T @ (hit - d.xpos[pel])

    def close(self) -> None:
        self._renderer.close()


class SegmentationDetector:
    """シミュレーション用の検出器（YOLO の代わり）。MuJoCo のセグメンテーション（画素ごとに写っている物の番号）
    から、名前が geom_prefix で始まる物の枠を返す。

    YOLO（事前学習済み）は、MuJoCo の円柱を組み合わせた作り物のボトルを bottle と検出しなかった（2026-09-28）。
    YOLO 自体は実機の画像で確かめる。
    """

    def __init__(self, camera: SimHeadCamera, geom_prefix: str, class_name: str = "bottle") -> None:
        self.camera = camera
        self.prefix = geom_prefix
        self.class_name = class_name

    def detect(self, frame: np.ndarray) -> list:  # noqa: ARG002（画像ではなく、今のシミュレーションの状態を使う）
        bb = self.camera.segmentation_bbox(self.prefix)
        from .localize import Detection

        return [] if bb is None else [Detection(class_name=self.class_name, confidence=1.0, bbox=bb)]
