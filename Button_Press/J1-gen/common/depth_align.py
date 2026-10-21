"""深度画像を、別のカメラ（カラー）の画素に並べ直す（位置合わせ、align）。

pyrealsense2 の rs.align は、カラーと深度を同じ pipeline で開いたときにしか使えない。
PC2 ではカラー（/dev/video4）を Unitree の videohub が開いているので、カラーは videohub から、深度は
RealSense から別々に受け取り、位置合わせはここで自分で計算する（2026-09-29 に決めた）。

やり方は librealsense の align と同じ:
1. 深度の各画素を、深度カメラの内部パラメータで 3 次元の点にする
2. 外部パラメータ（深度 → カラーの回転と平行移動）でカラーカメラの座標に移す
3. カラーの内部パラメータで画素に投影する。深度の 1 画素はカラーでは 1 画素より大きく写ることがあるので
   （深度 fx ≈ 385、カラー 640x360 の fx ≈ 460）、深度の画素の左上と右下の角を投影し、その間に中心が入る
   カラーの画素をすべて埋める（埋めないと、網目のように穴が空く）。librealsense は角を四捨五入して両端を含めるので
   1 画素はみ出すが、ここでは中心で判定してはみ出さないようにした（同じカメラどうしなら入力と完全に一致する）
4. 同じカラーの画素に複数の深度が来たら、近いほうを残す（手前の物が奥の物を隠す）

書き込む値は、カラーカメラから見た距離（z）。受け取る側は、カラーの内部パラメータと合わせて
x = (u − cx) z / fx のように使える。

レンズのゆがみは無視する（D435 の深度はゆがみの係数が 0、カラーもほぼ 0）。
numpy だけを使う（PC2 のシステムの Python 3.8 と、古い numpy でも動くように ufunc.at は使わない）。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from common.rgbd_protocol import Intrinsics


@dataclass
class Extrinsics:
    """深度カメラの座標 → カラーカメラの座標: p_color = rotation @ p_depth + translation [m]。"""

    rotation: np.ndarray  # (3, 3)
    translation: np.ndarray  # (3,)

    @staticmethod
    def from_realsense(rotation: list[float], translation: list[float]) -> Extrinsics:
        """pyrealsense2 の rs.extrinsics の値から作る。rotation は 9 個を列の順（column-major）に並べたもの。"""
        return Extrinsics(np.asarray(rotation, np.float64).reshape(3, 3).T.copy(),
                          np.asarray(translation, np.float64).reshape(3))

    @staticmethod
    def identity() -> Extrinsics:
        return Extrinsics(np.eye(3), np.zeros(3))


def scale_intrinsics(intr: Intrinsics, width: int, height: int) -> Intrinsics:
    """画像を width x height に拡大・縮小したときの内部パラメータ（画角はそのまま）。

    画素の中心を基準にする（画素 0 の中心は 0.0、画像の左端は −0.5）ので、cx は (cx + 0.5) × 倍率 − 0.5。
    """
    sx = width / float(intr.width)
    sy = height / float(intr.height)
    return Intrinsics(
        width=int(width), height=int(height), fx=intr.fx * sx, fy=intr.fy * sy,
        cx=(intr.cx + 0.5) * sx - 0.5, cy=(intr.cy + 0.5) * sy - 0.5,
        model=intr.model, coeffs=list(intr.coeffs),
    )


def align_depth_to_color(
    depth: np.ndarray,
    depth_scale: float,
    depth_intr: Intrinsics,
    color_intr: Intrinsics,
    extr: Extrinsics,
    max_span: int = 4,
) -> np.ndarray:
    """深度（depth_intr の大きさの uint16）を、カラー（color_intr の大きさ）の画素に並べ直した uint16 を返す。

    値の単位は入力と同じ（距離 [m] = 値 × depth_scale）。0 は「測れなかった」（カラーの画素に深度が来なかった）。
    max_span: 深度の 1 画素が埋めるカラーの画素の、縦・横それぞれの上限（面に対して斜めから見たとき、
    角どうしが大きく離れて広い範囲を埋めてしまうのを防ぐ）。
    """
    if depth.dtype != np.uint16 or depth.ndim != 2:
        raise ValueError(f"深度は (H, W) の uint16: {depth.dtype} {depth.shape}")
    if depth.shape != (depth_intr.height, depth_intr.width):
        raise ValueError(f"深度の大きさ {depth.shape} が内部パラメータ {depth_intr.width}x{depth_intr.height} と違う")
    cw, ch = int(color_intr.width), int(color_intr.height)
    out = np.zeros((ch, cw), np.uint16)
    v, u = np.nonzero(depth)
    if u.size == 0:
        return out
    z = depth[v, u].astype(np.float64) * depth_scale
    uf = u.astype(np.float64)
    vf = v.astype(np.float64)
    r, t = extr.rotation, extr.translation

    def to_color(du: float, dv: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """深度の画素 (u + du, v + dv) をカラーの画素に投影する。(カラーの u, v, カラーから見た z) を返す。"""
        x = (uf + du - depth_intr.cx) / depth_intr.fx * z
        y = (vf + dv - depth_intr.cy) / depth_intr.fy * z
        xc = r[0, 0] * x + r[0, 1] * y + r[0, 2] * z + t[0]
        yc = r[1, 0] * x + r[1, 1] * y + r[1, 2] * z + t[1]
        zc = r[2, 0] * x + r[2, 1] * y + r[2, 2] * z + t[2]
        with np.errstate(divide="ignore", invalid="ignore"):
            return xc / zc * color_intr.fx + color_intr.cx, yc / zc * color_intr.fy + color_intr.cy, zc

    u_lo, v_lo, _ = to_color(-0.5, -0.5)
    u_hi, v_hi, _ = to_color(0.5, 0.5)
    _, _, zc = to_color(0.0, 0.0)
    # 埋める画素: 中心（整数の位置）が [左上の角, 右下の角) に入るもの。1 つも入らないほど小さく写ったときは、
    # 右隣の 1 画素を埋める（穴を空けないため）
    u0 = np.ceil(np.minimum(u_lo, u_hi))
    u1 = np.maximum(np.ceil(np.maximum(u_lo, u_hi)) - 1, u0)
    v0 = np.ceil(np.minimum(v_lo, v_hi))
    v1 = np.maximum(np.ceil(np.maximum(v_lo, v_hi)) - 1, v0)
    ok = (zc > 0) & np.isfinite(u0) & np.isfinite(u1) & np.isfinite(v0) & np.isfinite(v1)
    ok &= (u1 >= 0) & (v1 >= 0) & (u0 <= cw - 1) & (v0 <= ch - 1)
    if not ok.any():
        return out
    u0 = np.clip(u0[ok], 0, cw - 1).astype(np.int64)
    u1 = np.clip(u1[ok], 0, cw - 1).astype(np.int64)
    v0 = np.clip(v0[ok], 0, ch - 1).astype(np.int64)
    v1 = np.clip(v1[ok], 0, ch - 1).astype(np.int64)
    u1 = np.minimum(u1, u0 + max_span - 1)
    v1 = np.minimum(v1, v0 + max_span - 1)
    val = np.clip(np.round(zc[ok] / depth_scale), 1, 65535).astype(np.int64)

    # 埋める (画素の番号, 値) を集める
    idx_parts: list[np.ndarray] = []
    val_parts: list[np.ndarray] = []
    for dy in range(max_span):
        for dx in range(max_span):
            m = (u0 + dx <= u1) & (v0 + dy <= v1)
            if not m.any():
                continue
            idx_parts.append((v0[m] + dy) * cw + (u0[m] + dx))
            val_parts.append(val[m])
    idx = np.concatenate(idx_parts)
    vals = np.concatenate(val_parts)
    # 同じ画素には近いほう（小さい値）を残す: (画素の番号, 値) の順に並べて、画素ごとの先頭を取る
    key = np.sort(idx * 65536 + vals)
    pix = key >> 16
    first = np.empty(pix.size, bool)
    first[0] = True
    np.not_equal(pix[1:], pix[:-1], out=first[1:])
    out.reshape(-1)[pix[first]] = (key[first] & 0xFFFF).astype(np.uint16)
    return out
