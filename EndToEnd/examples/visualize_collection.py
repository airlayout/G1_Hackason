"""同時記録した外部映像・RGB・LiDARだけから、同期した3画面動画を作る。"""

import argparse
import json
from pathlib import Path
import shutil
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

from g1_e2e.data.dataset import DatasetReader
from g1_e2e.data.observer import synchronized_frames
from g1_e2e.data.geometry import transform_points

BACKGROUND = (12, 18, 28)
TEXT = (222, 232, 243)


def font(size):
    path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default(size=size)


def point_cloud_panel(frame):
    image = Image.new("RGB", (512, 384), BACKGROUND)
    draw = ImageDraw.Draw(image)

    def project(points):
        points = np.asarray(points)
        x, y, z = points.T
        return np.column_stack([210 + 18 * (x - 0.7 * y), 302 - 8 * (x + 0.7 * y) - 45 * z])

    for x in range(-2, 14, 2):
        a, b = project([(x, -6, 0), (x, 6, 0)])
        draw.line([tuple(a), tuple(b)], fill=(32, 45, 60))
    for y in range(-6, 7, 2):
        a, b = project([(-2, y, 0), (12, y, 0)])
        draw.line([tuple(a), tuple(b)], fill=(32, 45, 60))
    lidar = frame.lidar
    if lidar is None:
        draw.text((20, 24), "—", font=font(16), fill=TEXT)
        return image
    points = (
        lidar["points_world_m"][lidar["valid"]].copy()
        if "points_world_m" in lidar
        else transform_points(lidar["points_sensor_m"][lidar["valid"]], lidar["world_from_sensor"])
    )
    base = frame.base_pose_world[:3, 3]
    points[:, :2] -= base[:2]
    visible = (np.abs(points[:, 1]) < 7) & (points[:, 0] > -3) & (points[:, 0] < 14)
    points = points[visible & (points[:, 2] > -0.2) & (points[:, 2] < 4)]
    pixels = np.rint(project(points)).astype(int)
    rgb = np.asarray(image).copy()
    height = np.clip(points[:, 2] / 2, 0, 1)
    colors = np.column_stack([70 + 180 * height, 185 - 30 * height, 240 - 150 * height]).astype("u1")
    for dx, dy in [(0, 0), (1, 0), (0, 1)]:
        u, v = pixels[:, 0] + dx, pixels[:, 1] + dy
        valid = (u >= 0) & (u < 512) & (v >= 0) & (v < 384)
        rgb[v[valid], u[valid]] = colors[valid]
    image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(image)
    center = project([(0, 0, 0)])[0]
    for endpoint, color, label in [
        ((2, 0, 0), (240, 120, 100), "X"),
        ((0, 2, 0), (100, 220, 150), "Y"),
        ((0, 0, 2), (140, 165, 255), "Z"),
    ]:
        end = project([endpoint])[0]
        draw.line([tuple(center), tuple(end)], fill=color, width=2)
        draw.text(tuple(end + [3, -15]), label, font=font(14), fill=color)
    draw.ellipse((center[0] - 4, center[1] - 4, center[0] + 4, center[1] + 4), fill=(245, 245, 245))
    draw.text((20, 24), f"{int(lidar['valid'].sum()):,} points", font=font(16), fill=TEXT)
    return image


def compose(frame, *, camera_hz, lidar_hz):
    image = Image.new("RGB", (1536, 464), BACKGROUND)
    draw = ImageDraw.Draw(image)
    for x, title in [(20, "G1"), (532, "RGB camera"), (1044, "LiDAR")]:
        draw.text((x, 13), title, fill=TEXT, font=font(21))
    image.paste(Image.fromarray(frame.observer_rgb), (0, 48))
    camera = ImageOps.contain(Image.fromarray(frame.camera["rgb"]), (512, 384), Image.Resampling.LANCZOS)
    image.paste(camera, (512 + (512 - camera.width) // 2, 48 + (384 - camera.height) // 2))
    image.paste(point_cloud_panel(frame), (1024, 48))
    draw = ImageDraw.Draw(image)
    time_s = frame.time_s
    for x, label in [
        (20, f"t = {time_s:05.2f} s"),
        (532, f"{camera_hz:g} Hz   |   t = {float(frame.camera['time_s']):05.2f} s"),
        (
            1044,
            f"{lidar_hz:g} Hz   |   scan —"
            if frame.lidar is None
            else f"{lidar_hz:g} Hz   |   scan t = {float(frame.lidar['time_s']):05.2f} s",
        ),
    ]:
        draw.text((x, 440), label, fill=(145, 165, 187), font=font(14))
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("collection", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--ffmpeg", default=shutil.which("ffmpeg"))
    args = parser.parse_args()
    if not args.ffmpeg:
        try:
            import imageio_ffmpeg

            args.ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
        except ImportError:
            parser.error(
                "setup.sh --with-videoでffmpegを取得するか、--ffmpegに実行ファイルを指定してください"
            )
    output = args.output or args.collection / "synchronized.mp4"
    if output.exists():
        parser.error(f"出力先は使用済みです: {output}")
    with DatasetReader(args.collection / "dataset.h5") as reader:
        metadata = reader.metadata
    period = metadata["sensor_config"]["camera"]["period_s"]
    process = None
    frames = 0
    try:
        process = subprocess.Popen(
            [
                args.ffmpeg,
                "-n",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                "1536x464",
                "-r",
                str(1 / period),
                "-i",
                "pipe:0",
                "-an",
                "-c:v",
                "libx264",
                "-preset",
                "fast",
                "-crf",
                "19",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(output),
            ],
            stdin=subprocess.PIPE,
        )
        for capture in synchronized_frames(args.collection):
            frame = compose(
                capture, camera_hz=1 / period, lidar_hz=1 / metadata["sensor_config"]["lidar"]["period_s"]
            )
            if frames == 0:
                frame.save(output.with_name(f"{output.stem}-0000.png"))
            process.stdin.write(frame.tobytes())
            frames += 1
        process.stdin.close()
        if process.wait() != 0:
            raise RuntimeError("動画のエンコードに失敗しました")
        print(json.dumps({"video": str(output.resolve()), "frames": frames, "fps": 1 / period}))
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait()


if __name__ == "__main__":
    main()
