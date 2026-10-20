"""深度付きストリーム（RGBD）の送受信の形式。サーバ（PC2）とクライアント（ラボ PC）で共有する。タスク3。

このファイルは numpy / cv2 / 標準ライブラリだけを使う（PC2 に余計な依存を入れないため）。

## RGBD の形式（既定ポート 5556）

1 フレーム = ZMQ の 1 メッセージ（1 パート）。受信側で CONFLATE（常に最新の 1 件だけを保持）を使えるように、
複数パートには分けない:

    MAGIC（8 バイト "G1RGBD1\\0"）| ヘッダの長さ（uint32、リトルエンディアン）| ヘッダ（JSON, UTF-8）
    | カラー画像（JPEG）| 深度（uint16 の生データ、または zlib で圧縮したもの）

- カラー: JPEG。**デコードすると BGR**（OpenCV の普通の順番。既存の RGB 互換ストリームとは違う）
- 深度: カラー画像に位置合わせ済み（aligned_to = "color"）で、画素ごとの距離 = 値 × depth.scale_m [m]。
  0 は「測れなかった」。16bit のまま送る（JPEG にはしない）
- 内部パラメータ（intrinsics）: カラー画像のもの。深度も同じ画素の並びなので、そのまま使える

## RGB 互換の形式（既定ポート 5555）

lerobot の ImageServer（run_g1_server.py --camera）と同じ JSON 文字列:
    {"timestamps": {"head_camera": 時刻}, "images": {"head_camera": "<base64 の JPEG>"}}
JPEG は **RGB の並びの配列をそのまま cv2.imencode したもの**（lerobot と同じ）。既存の ZmqFrameSource は
これをデコードしてから RGB→BGR に並べ替えるので、そのまま動く。
"""

from __future__ import annotations

import base64
import json
import struct
import zlib
from dataclasses import asdict, dataclass, field

import cv2
import numpy as np

MAGIC = b"G1RGBD1\x00"
VERSION = 1


@dataclass
class Intrinsics:
    """カメラの内部パラメータ（ピンホールモデル）。画素 (u, v) と深度 z から、カメラ座標
    x = (u − cx) z / fx、y = (v − cy) z / fy を求めるのに使う。"""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    # レンズのゆがみのモデルと係数（RealSense の値をそのまま入れる。D435 のカラーは歪みがほぼ 0）
    model: str = "none"
    coeffs: list[float] = field(default_factory=lambda: [0.0] * 5)


@dataclass
class RgbdFrame:
    color_bgr: np.ndarray  # (H, W, 3) uint8、BGR
    depth: np.ndarray  # (H, W) uint16。距離 [m] = depth × depth_scale
    depth_scale: float  # [m / 1]
    intrinsics: Intrinsics
    timestamp: float  # 送信側で撮った時刻（time.time()）
    frame_id: int
    camera: str = "head_camera"

    def depth_m(self) -> np.ndarray:
        """深度を m 単位の float32 にしたもの（0 は測れなかった画素）。"""
        return self.depth.astype(np.float32) * np.float32(self.depth_scale)


def encode_rgbd(frame: RgbdFrame, jpeg_quality: int = 90, depth_compression: str = "zlib") -> bytes:
    """RgbdFrame を 1 メッセージのバイト列にする。"""
    if frame.depth.dtype != np.uint16 or frame.depth.ndim != 2:
        raise ValueError(f"深度は (H, W) の uint16: {frame.depth.dtype} {frame.depth.shape}")
    h, w = frame.depth.shape
    if frame.color_bgr.shape[:2] != (h, w):
        raise ValueError(f"カラーと深度の大きさが違う（位置合わせされていない）: {frame.color_bgr.shape} {frame.depth.shape}")
    ok, jpg = cv2.imencode(".jpg", np.ascontiguousarray(frame.color_bgr), [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)])
    if not ok:
        raise RuntimeError("JPEG へのエンコードに失敗した")
    color = jpg.tobytes()
    raw = np.ascontiguousarray(frame.depth, dtype="<u2").tobytes()
    if depth_compression == "zlib":
        depth = zlib.compress(raw, 1)  # 速さ優先（レベル 1）
    elif depth_compression == "none":
        depth = raw
    else:
        raise ValueError(f"depth_compression は none か zlib: {depth_compression}")
    header = {
        "version": VERSION,
        "camera": frame.camera,
        "frame_id": int(frame.frame_id),
        "timestamp": float(frame.timestamp),
        "color": {"encoding": "jpeg", "order": "bgr", "width": w, "height": h, "nbytes": len(color)},
        "depth": {"encoding": depth_compression, "dtype": "uint16le", "width": w, "height": h,
                  "nbytes": len(depth), "scale_m": float(frame.depth_scale), "aligned_to": "color"},
        "intrinsics": asdict(frame.intrinsics),
    }
    hb = json.dumps(header).encode("utf-8")
    return MAGIC + struct.pack("<I", len(hb)) + hb + color + depth


def decode_rgbd(msg: bytes) -> RgbdFrame:
    """encode_rgbd のバイト列を RgbdFrame に戻す。形式が違えば ValueError。"""
    if len(msg) < 12 or msg[:8] != MAGIC:
        raise ValueError("RGBD のメッセージではない（先頭の MAGIC が違う）")
    (hlen,) = struct.unpack("<I", msg[8:12])
    header = json.loads(msg[12:12 + hlen].decode("utf-8"))
    if header.get("version") != VERSION:
        raise ValueError(f"RGBD の形式の版が違う: {header.get('version')}（このコードは {VERSION}）")
    c, d = header["color"], header["depth"]
    off = 12 + hlen
    color_bytes = msg[off:off + c["nbytes"]]
    off += c["nbytes"]
    depth_bytes = msg[off:off + d["nbytes"]]
    if len(depth_bytes) != d["nbytes"]:
        raise ValueError("RGBD のメッセージが途中で切れている")
    color = cv2.imdecode(np.frombuffer(color_bytes, np.uint8), cv2.IMREAD_COLOR)
    if color is None:
        raise ValueError("カラー画像（JPEG）をデコードできない")
    raw = zlib.decompress(depth_bytes) if d["encoding"] == "zlib" else depth_bytes
    depth = np.frombuffer(raw, dtype="<u2").reshape(d["height"], d["width"]).astype(np.uint16)
    return RgbdFrame(
        color_bgr=color,
        depth=depth,
        depth_scale=float(d["scale_m"]),
        intrinsics=Intrinsics(**header["intrinsics"]),
        timestamp=float(header["timestamp"]),
        frame_id=int(header["frame_id"]),
        camera=header.get("camera", "head_camera"),
    )


def encode_legacy_rgb(color_rgb: np.ndarray, camera: str, timestamp: float, jpeg_quality: int = 80) -> str:
    """lerobot の ImageServer と同じ JSON 文字列を作る（RGB の配列をそのまま imencode する）。"""
    ok, jpg = cv2.imencode(".jpg", color_rgb, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)])
    if not ok:
        raise RuntimeError("JPEG へのエンコードに失敗した")
    b64 = base64.b64encode(jpg.tobytes()).decode("utf-8")
    return json.dumps({"timestamps": {camera: timestamp}, "images": {camera: b64}})
