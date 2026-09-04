#!/usr/bin/env python3
"""G1 PC2のカメラノードを調べ、カラーストリームと内部パラメータを特定する。

`camera_only_server.py --device N` に渡すNを決めるために使う。RealSenseは
depth / IR / color を別々の /dev/videoN へ出すため、OpenCVで開けるノードが
必ずカラーとは限らない（実際に2026-09-04の計測ではIRを記録してしまった）。

標準ライブラリだけでフォーマット一覧を出し、cv2があれば実際に1枚取って
彩度で色の有無を判定する。pyrealsense2があればcolor streamの内部パラメータも出す。
"""

from __future__ import annotations

import argparse
import fcntl
import glob
import struct

# v4l2 の VIDIOC_ENUM_FMT: _IOWR('V', 2, struct v4l2_fmtdesc)
# struct v4l2_fmtdesc は 64 bytes（index, type, flags, description[32],
# pixelformat, mbus_code, reserved[3]）。
_FMTDESC_SIZE = 64
_VIDIOC_ENUM_FMT = (3 << 30) | (_FMTDESC_SIZE << 16) | (ord("V") << 8) | 2
_BUF_TYPE_VIDEO_CAPTURE = 1


def _fourcc_to_text(value: int) -> str:
    """pixelformatの32bit値を4文字のfourccへ戻す。"""

    return "".join(chr((value >> shift) & 0xFF) for shift in (0, 8, 16, 24)).strip()


def list_formats(path: str) -> list[tuple[str, str]]:
    """1つのvideoノードが公開するpixel formatを列挙する。"""

    formats: list[tuple[str, str]] = []
    try:
        file_descriptor = open(path, "rb", buffering=0)
    except OSError as error:
        print(f"[WARN] {path}: open失敗 ({error})")
        return formats

    with file_descriptor:
        for index in range(32):
            payload = bytearray(_FMTDESC_SIZE)
            struct.pack_into("II", payload, 0, index, _BUF_TYPE_VIDEO_CAPTURE)
            try:
                fcntl.ioctl(file_descriptor, _VIDIOC_ENUM_FMT, payload, True)
            except OSError:
                break
            description = bytes(payload[12:44]).split(b"\x00", 1)[0].decode(
                "ascii", "replace"
            )
            pixelformat = struct.unpack_from("I", payload, 44)[0]
            formats.append((_fourcc_to_text(pixelformat), description))
    return formats


def check_color(device_index: int, width: int, height: int) -> str:
    """実際に1フレーム取得し、彩度から色の有無を判定する。"""

    try:
        import cv2
    except ImportError:
        return "cv2なし（色判定を省略）"

    capture = cv2.VideoCapture(device_index)
    if not capture.isOpened():
        capture.release()
        return "OpenCVで開けない"

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    # 露光が安定するまで数フレーム捨てる。
    frame = None
    for _ in range(5):
        success, candidate = capture.read()
        if success:
            frame = candidate
    capture.release()

    if frame is None:
        return "フレーム取得失敗"

    frame_height, frame_width = frame.shape[:2]
    if frame.ndim != 3 or frame.shape[2] != 3:
        return f"{frame_width}x{frame_height} 単チャンネル＝モノクロ"

    # BGRの画素ごとの最大差。0ならグレースケール。
    spread = frame.max(axis=2).astype(int) - frame.min(axis=2).astype(int)
    mean_spread = float(spread.mean())
    max_spread = int(spread.max())
    verdict = "カラー" if max_spread > 8 else "モノクロ（IRの可能性）"
    return (
        f"{frame_width}x{frame_height} 平均彩度{mean_spread:.2f} "
        f"最大彩度{max_spread} → {verdict}"
    )


def print_realsense_intrinsics() -> None:
    """pyrealsense2があればcolor streamの内部パラメータを出す。"""

    try:
        import pyrealsense2 as rs
    except ImportError:
        print("[INFO] pyrealsense2なし。内部パラメータは別途取得が必要")
        return

    try:
        pipeline = rs.pipeline()
        profile = pipeline.start()
    except Exception as error:  # noqa: BLE001 - 実機側の例外種別は環境依存
        print(f"[WARN] RealSense pipelineを開けません: {error}")
        return

    try:
        stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intrinsics = stream.get_intrinsics()
        print("[INTRINSICS] color stream")
        print(f"  CAMERA_WIDTH={intrinsics.width}")
        print(f"  CAMERA_HEIGHT={intrinsics.height}")
        print(f"  CAMERA_FX={intrinsics.fx}")
        print(f"  CAMERA_FY={intrinsics.fy}")
        print(f"  CAMERA_CX={intrinsics.ppx}")
        print(f"  CAMERA_CY={intrinsics.ppy}")
    finally:
        pipeline.stop()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument(
        "--skip-capture",
        action="store_true",
        help="フォーマット一覧だけ出し、実フレーム取得はしない",
    )
    arguments = parser.parse_args()

    paths = sorted(glob.glob("/dev/video*"))
    if not paths:
        print("[ERROR] /dev/video* が見つかりません")
        return

    for path in paths:
        index = int(path.replace("/dev/video", ""))
        print(f"=== {path}")
        formats = list_formats(path)
        if formats:
            for fourcc, description in formats:
                print(f"  format: {fourcc:<6} {description}")
        else:
            print("  format: 取得できず（capture非対応ノードの可能性）")
        if not arguments.skip_capture:
            print(f"  capture: {check_color(index, arguments.width, arguments.height)}")

    print()
    print_realsense_intrinsics()


if __name__ == "__main__":
    main()
