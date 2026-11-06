"""11-05: 画像だけの生徒の画像の網（ResNet18 → 線形 128 → LayerNorm、VIRAL6_vis の door_vision_feat.DoorVisionEncoder）を
実機で回す。PC2 に何が入っているか分からないので 2 通り:
  ort    onnxruntime（export_vision_student.py が書いた .onnx）
  numpy  numpy だけ（BatchNorm を畳み込みに畳んだ重みの npz）。PC2 の Python 3.8 の numpy でも動く
どちらも入力は「正規化した RGB [3, 108, 192]」。カメラの画像からそこまでは preprocess_bgr（cv2 が要る）。

シミュレータのカメラ（11-05 に直した）: 焦点 1.88・開口 2.6035×1.4621＝D435 のカラーの 69°×42°、108×192 で描く。
実機は純正の映像サービスの 1920×1080 JPEG（同じセンサーの全画角）を 1/4 で復号して 192×108 へ縮める。
"""
from __future__ import annotations

import json
import time
from typing import List, Optional, Tuple

import numpy as np

IMAGENET_MEAN = np.array([0.485, 0.456, 0.406])
IMAGENET_STD = np.array([0.229, 0.224, 0.225])
IMG_H, IMG_W = 108, 192


def preprocess_rgb(rgb_u8: np.ndarray) -> np.ndarray:
    """uint8 の RGB [H, W, 3] → 正規化した [3, H, W] float32（door_vision_feat.DoorVisionEncoder.preprocess と同じ）"""
    x = rgb_u8[..., :3].astype(np.float32) / 255.0
    x = (x - IMAGENET_MEAN.astype(np.float32)) / IMAGENET_STD.astype(np.float32)
    return np.ascontiguousarray(x.transpose(2, 0, 1))


def decode_small_bgr(jpeg: bytes) -> np.ndarray:
    """JPEG → 192×108 の BGR。1/4 で復号（1920×1080 → 480×270）してから面積平均で縮める"""
    import cv2

    buf = np.frombuffer(jpeg, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_REDUCED_COLOR_4)
    if img is None:
        raise ValueError("JPEG を復号できない")
    if img.shape[1] * IMG_H != img.shape[0] * IMG_W:
        raise ValueError(f"画像の縦横比が 16:9 でない: {img.shape[1]}×{img.shape[0]}（画角が学習と合わない）")
    return cv2.resize(img, (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)


def bgr_to_rgb(bgr: np.ndarray) -> np.ndarray:
    return np.ascontiguousarray(bgr[..., ::-1])


# --- 11-06: カラー＋深度（VIRAL6_vis の door_vision_feat.DoorRGBDEncoder・パッチ 0094） ---
# シミュレータは深度をカラーと同じカメラ・同じ画角（焦点 1.88・開口 2.6035×1.4621）で描き、縦横 1/2 の 54×96 で使う。
# 実機の D435i の深度（左の赤外線の位置・87°×58°）からは、その画角の中央を最近傍で取り出して同じ形にする（素子の位置の差 約 1.5 cm は無視）
DEPTH_MAX = 3.0
DEPTH_H, DEPTH_W = IMG_H // 2, IMG_W // 2
SIM_TAN_HALF = (2.6035 / 2 / 1.88, 1.4621 / 2 / 1.88)  # シミュレータのカメラの半画角の tan（横・縦）


def depth_from_intrinsics(depth_m: np.ndarray, fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    """実機の深度 [H, W]（m・0 は測れない）→ シミュレータと同じ画角・形の深度 [54, 96]（float32・0 は測れない・3 m で詰める）。
    出力の各画素の中心の向きを深度のカメラの内部の値（fx・fy・cx・cy、画素）で写し、最近傍で取る。画面の外は 0"""
    tx = (np.arange(DEPTH_W) + 0.5) / DEPTH_W * 2.0 - 1.0
    ty = (np.arange(DEPTH_H) + 0.5) / DEPTH_H * 2.0 - 1.0
    u = np.rint(cx + fx * tx * SIM_TAN_HALF[0] - 0.5).astype(int)
    v = np.rint(cy + fy * ty * SIM_TAN_HALF[1] - 0.5).astype(int)
    h, w = depth_m.shape
    inside = (u >= 0) & (u < w)
    inside_v = (v >= 0) & (v < h)
    out = np.zeros((DEPTH_H, DEPTH_W), dtype=np.float32)
    sub = depth_m[np.clip(v, 0, h - 1)][:, np.clip(u, 0, w - 1)].astype(np.float32)
    out[:] = np.where(inside_v[:, None] & inside[None, :], sub, 0.0)
    out[~np.isfinite(out)] = 0.0
    return np.clip(out, 0.0, DEPTH_MAX)


def depth_input(d: np.ndarray) -> np.ndarray:
    """深度 [54, 96]（m）→ 深度の枝の入力 [2, 54, 96]（距離 / DEPTH_MAX と測れた所の 1。door_vision_feat.depth_input と同じ）"""
    return np.ascontiguousarray(np.stack([d / DEPTH_MAX, (d > 0).astype(np.float32)]).astype(np.float32))


class OrtRGBDEncoder:
    """onnxruntime で回すカラー＋深度の網（入力 rgb [1, 3, 108, 192]・depth [1, 2, 54, 96]・color_off [1] → [1, feat_dim]）"""

    uses_depth = True

    def __init__(self, path: str, threads: int = 4) -> None:
        import onnxruntime as ort

        so = ort.SessionOptions()
        so.intra_op_num_threads = int(threads)
        self.sess = ort.InferenceSession(path, sess_options=so, providers=["CPUExecutionProvider"])
        self.names = [i.name for i in self.sess.get_inputs()]

    def __call__(self, x_rgb: np.ndarray, x_depth: np.ndarray, color_off: bool = False) -> np.ndarray:
        feed = {"rgb": x_rgb[None].astype(np.float32), "depth": x_depth[None].astype(np.float32),
                "color_off": np.array([1.0 if color_off else 0.0], dtype=np.float32)}
        y = self.sess.run(None, {k: feed[k] for k in self.names})[0]
        return y[0].astype(np.float64)


# --- numpy の ResNet18 ---


def _pad(x: np.ndarray, p: int, value: float = 0.0) -> np.ndarray:
    return x if p == 0 else np.pad(x, ((0, 0), (p, p), (p, p)), constant_values=value)


def conv2d(x: np.ndarray, w: np.ndarray, b: np.ndarray, stride: int, pad: int) -> np.ndarray:
    """x [C, H, W]・w [O, C, kh, kw] → [O, Ho, Wo]（im2col と行列積）"""
    c, _, _ = x.shape
    o, _, kh, kw = w.shape
    xp = _pad(x, pad)
    ho = (xp.shape[1] - kh) // stride + 1
    wo = (xp.shape[2] - kw) // stride + 1
    cols = np.empty((c, kh, kw, ho, wo), dtype=x.dtype)
    for i in range(kh):
        for j in range(kw):
            cols[:, i, j] = xp[:, i:i + stride * ho:stride, j:j + stride * wo:stride]
    y = w.reshape(o, -1) @ cols.reshape(c * kh * kw, ho * wo)
    return (y + b[:, None]).reshape(o, ho, wo)


def maxpool3x3s2(x: np.ndarray) -> np.ndarray:
    xp = _pad(x, 1, -np.inf)
    ho = (xp.shape[1] - 3) // 2 + 1
    wo = (xp.shape[2] - 3) // 2 + 1
    out = np.full((x.shape[0], ho, wo), -np.inf, dtype=x.dtype)
    for i in range(3):
        for j in range(3):
            np.maximum(out, xp[:, i:i + 2 * ho:2, j:j + 2 * wo:2], out=out)
    return out


class NumpyResNetEncoder:
    """export_vision_student.py の encoder npz（BN を畳んだ畳み込み・線形・LayerNorm）を numpy で回す"""

    def __init__(self, path: str) -> None:
        z = np.load(path, allow_pickle=False)
        self.meta = json.loads(str(z["meta_json"]))
        f32 = lambda k: z[k].astype(np.float32)  # noqa: E731
        self.conv1 = (f32("conv1_w"), f32("conv1_b"))
        self.blocks: List[Tuple] = []
        for name in self.meta["blocks"]:
            down = (f32(f"{name}_down_w"), f32(f"{name}_down_b")) if f"{name}_down_w" in z.files else None
            stride = int(self.meta["strides"][name])
            self.blocks.append((stride, (f32(f"{name}_c1_w"), f32(f"{name}_c1_b")), (f32(f"{name}_c2_w"), f32(f"{name}_c2_b")), down))
        self.proj = (f32("proj_w"), f32("proj_b"))
        self.ln = (f32("ln_w"), f32("ln_b"), float(self.meta["ln_eps"]))
        self.feat_dim = int(self.proj[0].shape[0])

    def __call__(self, x: np.ndarray) -> np.ndarray:
        """正規化した [3, H, W] → 特徴 [feat_dim]（float64）"""
        y = np.maximum(conv2d(x.astype(np.float32), *self.conv1, 2, 3), 0.0)
        y = maxpool3x3s2(y)
        for stride, c1, c2, down in self.blocks:
            r = y if down is None else conv2d(y, *down, stride, 0)
            t = np.maximum(conv2d(y, *c1, stride, 1), 0.0)
            y = np.maximum(conv2d(t, *c2, 1, 1) + r, 0.0)
        v = y.mean(axis=(1, 2))
        v = self.proj[0] @ v + self.proj[1]
        g, b, eps = self.ln
        v = (v - v.mean()) / np.sqrt(v.var() + eps)
        return (v * g + b).astype(np.float64)


class OrtEncoder:
    """onnxruntime で回す（入力 [1, 3, H, W] float32 → [1, feat_dim]）"""

    def __init__(self, path: str, threads: int = 4) -> None:
        import onnxruntime as ort

        so = ort.SessionOptions()
        so.intra_op_num_threads = int(threads)
        self.sess = ort.InferenceSession(path, sess_options=so, providers=["CPUExecutionProvider"])
        self.name = self.sess.get_inputs()[0].name

    def __call__(self, x: np.ndarray) -> np.ndarray:
        y = self.sess.run(None, {self.name: x[None].astype(np.float32)})[0]
        return y[0].astype(np.float64)


def make_encoder(npz_path: str, onnx_path: Optional[str] = None, backend: str = "auto", threads: int = 4):
    """(網, 使った方の名前)。auto は onnxruntime があればそれ、無ければ numpy"""
    if backend in ("auto", "ort") and onnx_path:
        try:
            return OrtEncoder(onnx_path, threads), "ort"
        except ImportError:
            if backend == "ort":
                raise
    if backend == "ort":
        raise ValueError("--backend ort には .onnx が要る")
    return NumpyResNetEncoder(npz_path), "numpy"


def bench(enc, n: int = 20) -> float:
    """1 枚あたりの時間 [ms]（最初の 3 回は捨てる）"""
    x = preprocess_rgb(np.random.default_rng(0).integers(0, 256, (IMG_H, IMG_W, 3), dtype=np.uint8))
    for _ in range(3):
        enc(x)
    t0 = time.monotonic()
    for _ in range(n):
        enc(x)
    return (time.monotonic() - t0) / n * 1000.0
