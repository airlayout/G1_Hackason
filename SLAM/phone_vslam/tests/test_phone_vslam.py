"""phone_vslam の部品の試し（OMEN の mast3r 環境で回す）

    cd ~/work_inoue/phone_slam/phone_vslam
    ~/miniforge3/envs/mast3r/bin/python -m pytest -q tests

GPU も MediaMTX も要らない部品だけを試す。通しは feed_test.sh と start_slam.sh で見る。
"""

from __future__ import annotations

import pathlib
import subprocess
import sys
import time

import cv2
import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parents[1]
REPO = HERE.parent / "MASt3R-SLAM"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO))

from mast3r_slam.config import load_config  # noqa: E402

load_config(str(REPO / "config/base.yaml"))

from phone_source import FrameConformer, VideoFileDataset, live_out_size  # noqa: E402
from rerun_viz import VizOptions, sim3_parts, viewer_urls  # noqa: E402
from stream_report import summarize  # noqa: E402


def _img(h: int, w: int, value: int = 0) -> np.ndarray:
    img = np.full((h, w, 3), value, np.uint8)
    img[0, 0] = (255, 0, 0)  # 向きが分かる印（左上）
    return img


class TestFrameConformer:
    def test_first_frame_passes_unchanged(self) -> None:
        conform = FrameConformer()
        img = _img(720, 1280)
        assert conform(img) is img

    def test_same_size_passes_unchanged(self) -> None:
        conform = FrameConformer()
        conform(_img(720, 1280))
        img = _img(720, 1280, 7)
        assert conform(img) is img

    def test_lower_resolution_is_scaled_up_to_first_size(self) -> None:
        conform = FrameConformer()
        conform(_img(720, 1280))
        out = conform(_img(360, 640))
        assert out.shape == (720, 1280, 3)

    def test_portrait_frame_is_rotated_back_to_landscape(self) -> None:
        conform = FrameConformer()
        conform(_img(720, 1280))
        out = conform(_img(1280, 720))
        assert out.shape == (720, 1280, 3)

    def test_other_aspect_is_center_cropped_not_squashed(self) -> None:
        conform = FrameConformer()
        conform(_img(720, 1280))  # 16:9
        img = np.zeros((480, 640, 3), np.uint8)  # 4:3。真ん中の帯だけ白
        img[60:420, :] = 255
        out = conform(img)
        assert out.shape == (720, 1280, 3)
        assert out.mean() > 250  # 上下の黒い帯は切り落とされている


class TestLiveOutSize:
    def test_portrait_keeps_aspect_with_long_side_640(self) -> None:
        assert live_out_size(720, 1280) == (360, 640)

    def test_landscape_keeps_aspect_with_long_side_640(self) -> None:
        assert live_out_size(1280, 720) == (640, 360)

    def test_odd_sizes_are_rounded_to_even(self) -> None:
        w, h = live_out_size(1000, 333)
        assert (w, h) == (640, 214) and h % 2 == 0


class TestVideoFileDataset:
    @pytest.fixture()
    def video(self, tmp_path: pathlib.Path) -> pathlib.Path:
        path = tmp_path / "v.mp4"
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (64, 48))
        for k in range(20):
            writer.write(np.full((48, 64, 3), k * 10, np.uint8))
        writer.release()
        return path

    def test_reads_every_frame_without_realtime(self, video: pathlib.Path) -> None:
        ds = VideoFileDataset(str(video))
        (h, w), raw = ds.get_img_shape()
        assert raw == (48, 64)
        n = len(ds)
        assert n == 20
        values = [ds[i][1].mean() for i in range(n)]
        assert values == sorted(values)  # 順に読めている
        assert ds.timestamps[1] == pytest.approx(1 / 30)

    def test_stride_skips_frames(self, video: pathlib.Path) -> None:
        ds = VideoFileDataset(str(video), stride=5)
        ds.get_img_shape()
        assert len(ds) == 4
        _ = ds[0]
        _ = ds[1]
        assert ds.timestamps[1] == pytest.approx(5 / 30)

    def test_realtime_follows_frame_times_not_fps_field(self, tmp_path: pathlib.Path) -> None:
        # record.sh の mkv は fps の欄が 1000 になる。時刻はこまごとの時刻で追う
        src = tmp_path / "v.mp4"
        writer = cv2.VideoWriter(str(src), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (64, 48))
        for k in range(60):
            writer.write(np.full((48, 64, 3), k * 4, np.uint8))
        writer.release()
        mkv = tmp_path / "v.mkv"
        ffmpeg = pathlib.Path(sys.executable).parent / "ffmpeg"
        subprocess.run(
            [str(ffmpeg), "-v", "error", "-y", "-i", str(src), "-c", "copy",
             "-video_track_timescale", "1000", str(mkv)],
            check=True,
        )
        ds = VideoFileDataset(str(mkv), realtime=True)
        ds.get_img_shape()
        _ = ds[0]
        time.sleep(0.5)
        _ = ds[1]  # 0.5 秒たった＝録画の 0.5 秒あたりのこま（fps 1000 と読むと終わりを超える）
        assert 0.4 <= ds.timestamps[1] <= 0.7

    def test_end_of_file_raises_stop_iteration(self, video: pathlib.Path) -> None:
        ds = VideoFileDataset(str(video), realtime=True)
        ds.get_img_shape()
        with pytest.raises(StopIteration):
            for i in range(10_000):
                ds[i]


class TestRerunViz:
    def test_sim3_parts_splits_lietorch_layout(self) -> None:
        data = np.array([1, 2, 3, 0, 0, 0, 1, 2.5], np.float32)
        t, q, s = sim3_parts(data)
        assert t.tolist() == [1, 2, 3]
        assert q.tolist() == [0, 0, 0, 1]
        assert s == 2.5

    def test_viewer_urls_point_browser_at_websocket(self) -> None:
        urls = viewer_urls(VizOptions(hosts=("10.0.0.1",)))
        assert urls == ["http://10.0.0.1:9090/?url=ws://10.0.0.1:9877"]


class TestStreamReport:
    def test_counts_gaps_and_size_changes(self) -> None:
        frames = [(k / 30, 1280, 720) for k in range(30)]
        frames += [(1.5 + k / 30, 640, 360) for k in range(30)]  # 0.53 秒の空きの後に解像度が落ちた
        rep = summarize(frames, gap_ms=100)
        assert rep["frames"] == 60
        assert rep["gaps"] == 1
        assert rep["max_gap_ms"] == pytest.approx(533.3, abs=0.5)
        assert [c["size"] for c in rep["size_changes"]] == ["1280x720", "640x360"]

    def test_too_few_frames_is_reported_not_crashed(self) -> None:
        assert "error" in summarize([(0.0, 10, 10)], gap_ms=100)
