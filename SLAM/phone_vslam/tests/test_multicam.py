"""2 台で 1 つの地図を作る部品の試し（GPU・スマホ・MediaMTX は要らない）

    cd ~/work_inoue/phone_slam/phone_vslam
    ~/miniforge3/envs/mast3r/bin/python -m pytest -q tests/test_multicam.py

CamKeyframes は派生版（Multicam-MASt3R-SLAM）の mast3r_slam が要る。上流の mast3r_slam と同じ名前なので、
同じ pytest の中で混ざらないよう別のプロセスで試す。
"""

from __future__ import annotations

import os
import pathlib
import subprocess
import sys
import textwrap
import types

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

HERE = pathlib.Path(__file__).resolve().parents[1]
UPSTREAM = HERE.parent / "MASt3R-SLAM"
FORK = HERE.parent / "Multicam-MASt3R-SLAM"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "tests"))
sys.path.insert(0, str(UPSTREAM))

from eval_multicam import evaluate  # noqa: E402
from multicam_viz import MultiCamMapLogger, classify_edges  # noqa: E402
from run_multicam_slam import conform_to, source_name, view_problem  # noqa: E402


class TestClassifyEdges:
    def test_links_between_phones_and_loops_within_one(self) -> None:
        cam_ids = np.array([0, 0, 1, 0, 1, 1])
        ii = [0, 1, 0, 2, 3, 2, 9]
        jj = [1, 2, 3, 4, 5, 5, 0]
        links, loops = classify_edges(ii, jj, cam_ids)
        assert links == [(1, 2), (3, 5)]  # 別のカメラ同士＝合流
        assert loops == [(0, 3), (2, 5)]  # 同じカメラで 2 枚以上離れた
        # (0,1)・(2,4) は同じカメラの直前とのつなぎ（ふつうの前進）。9 は範囲の外

    def test_same_camera_neighbours_are_not_loops_even_if_indices_are_far(self) -> None:
        # cam0 のキーフレーム 0 と 5 の間に cam1 が 4 枚入っても、cam0 の中では隣
        cam_ids = np.array([0, 1, 1, 1, 1, 0])
        links, loops = classify_edges([0], [5], cam_ids)
        assert links == [] and loops == []


def test_growing_keyframes_are_the_latest_of_each_camera() -> None:
    # 追跡のたびに育つのはカメラごとの最新の 1 枚。これを間隔を空けずに送り直すと表示が溢れる
    logger = MultiCamMapLogger.__new__(MultiCamMapLogger)
    logger.keyframes = types.SimpleNamespace(cam_ids=lambda: np.array([0, 1, 0, 0, 1, 1]))
    assert logger.growing_keyframes(6) == {3, 5}
    assert logger.growing_keyframes(0) == set()


class TestEvalMulticam:
    def _traj(self, seed: int, n: int = 60) -> np.ndarray:
        rng = np.random.default_rng(seed)
        return np.cumsum(rng.normal(0, 0.05, (n, 3)), axis=0)

    def _to_map(self, g: np.ndarray) -> np.ndarray:
        """正解の座標から SLAM の地図の座標へ（縮尺も向きも違う）"""
        R = Rotation.from_euler("xyz", [20, -35, 70], degrees=True).as_matrix()
        return 0.37 * g @ R.T + np.array([1.0, -2.0, 0.5])

    def test_both_phones_in_one_map_give_small_joint_error(self) -> None:
        ga, gb = self._traj(1), self._traj(2) + np.array([0.5, 0, 0])
        res = evaluate([(self._to_map(ga), ga), (self._to_map(gb), gb)])
        assert res["scale_joint"] == pytest.approx(1 / 0.37, rel=1e-3)
        for cam in res["cams"]:
            assert cam["ate_joint_m"] < 1e-6 and cam["ate_own_m"] < 1e-6

    def test_misplaced_second_phone_shows_only_in_joint_error(self) -> None:
        ga, gb = self._traj(1), self._traj(2)
        eb = self._to_map(gb) + np.array([0.3, 0.0, 0.0])  # 2 台目が地図の中で 0.3 ずれて載った
        res = evaluate([(self._to_map(ga), ga), (eb, gb)])
        assert res["cams"][1]["ate_own_m"] < 1e-6  # 2 台目の軌跡の形は正しい
        assert res["cams"][1]["ate_joint_m"] > 0.1  # 重ね方がずれていると joint にだけ出る


class TestSourceName:
    @pytest.mark.parametrize(
        ("src", "name"),
        [
            ("rtsp://127.0.0.1:8554/cam1", "cam1"),
            ("rtsp://127.0.0.1:8554/cam2/", "cam2"),
            ("/data/tum/rgbd_dataset_freiburg1_desk", "rgbd_dataset_freiburg1_desk"),
            ("rec/cam1_20261117_170000_000000.mp4", "cam1_20261117_170000_000000"),
        ],
    )
    def test_names(self, src: str, name: str) -> None:
        assert source_name(src) == name


class TestConformTo:
    def test_second_phone_in_other_orientation_is_rotated_to_the_map_shape(self) -> None:
        portrait = np.zeros((640, 360, 3), np.uint8)
        landscape = np.zeros((360, 640, 3), np.uint8)
        assert conform_to(landscape, (640, 360)).shape == (640, 360, 3)
        assert conform_to(portrait, (640, 360)).shape == (640, 360, 3)

    def test_other_aspect_is_cropped_and_resized(self) -> None:
        img = np.zeros((480, 640, 3), np.uint8)  # 4:3 を 16:9 の箱へ
        assert conform_to(img, (360, 640)).shape == (360, 640, 3)


@pytest.mark.skipif(not (FORK / "mast3r_slam" / "frame.py").exists(), reason="派生版が無い")
def test_cam_keyframes_last_index_by_camera() -> None:
    script = textwrap.dedent(
        """
        import torch, torch.multiprocessing as mp
        from multicam_core import CamKeyframes
        kf = CamKeyframes(mp.Manager(), 32, 32, buffer=8, device="cpu")
        assert kf.last_index(0) is None
        kf.cam_id[:5] = torch.tensor([0, 1, 0, 1, 1], dtype=torch.int)
        kf.n_size.value = 5
        assert kf.last_index(0) == 2
        assert kf.last_index(1) == 4
        assert kf.last_index(1, before=4) == 3
        assert kf.last_index(0, before=2) == 0
        assert kf.last_index(2) is None
        assert kf.cam_ids().tolist() == [0, 1, 0, 1, 1]
        print("ok")
        """
    )
    env = dict(os.environ, PYTHONPATH=f"{FORK}:{HERE}")
    r = subprocess.run([sys.executable, "-c", script], env=env, capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr[-2000:]
    assert r.stdout.strip().endswith("ok")


def test_take_dirty_reads_a_copy_and_clears_only_what_it_saw() -> None:
    import torch

    from rerun_viz import take_dirty

    kf = types.SimpleNamespace(is_dirty=torch.tensor([[False], [True], [False], [True], [True]]))
    assert take_dirty(kf, 4) == [1, 3]
    assert kf.is_dirty.reshape(-1).tolist() == [False, False, False, False, True]  # n より後ろの印は残す
    assert take_dirty(kf, 0) == []


class TestViewProblem:
    def test_dark_frame_from_a_phone_lying_face_down(self) -> None:
        img = np.full((640, 360, 3), 6, np.uint8)
        img[200:260, 250:320] = 120  # 11-17 の伏せた Pixel: 真っ暗に小さな明かり
        assert view_problem(img) == "too dark"

    def test_flat_wall_has_no_texture(self) -> None:
        assert view_problem(np.full((640, 360, 3), 180, np.uint8)) == "no texture"

    def test_textured_scene_is_fine(self) -> None:
        import cv2

        rng = np.random.default_rng(0)
        img = (rng.random((160, 90)) * 255).astype(np.uint8)
        img = cv2.cvtColor(cv2.resize(img, (360, 640), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2RGB)
        assert view_problem(img) == ""


def test_recent_same_camera_keyframes_are_the_last_n_of_that_camera() -> None:
    script = textwrap.dedent(
        """
        import numpy as np
        from multicam_core import recent_same_camera
        ids = np.array([0, 1, 0, 0, 1, 0, 1, 0])
        assert recent_same_camera(ids, 0, 7, 2) == {3, 5}
        assert recent_same_camera(ids, 1, 7, 4) == {1, 4, 6}
        assert recent_same_camera(ids, 0, 7, 0) == set()
        print("ok")
        """
    )
    env = dict(os.environ, PYTHONPATH=f"{FORK}:{HERE}")
    r = subprocess.run([sys.executable, "-c", script], env=env, capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr[-2000:]


class TestRealtimeVideoFeed:
    """録画を生と同じ速さで渡す口は、何度呼ばれても壁の時計より先のこまを渡さない"""

    def _feed(self, monkeypatch, times):
        import run_multicam_slam as rms

        frames = [(t, np.zeros((4, 4, 3), np.uint8)) for t in times]

        class FakeReader:
            path, n, last_t = "fake.mp4", 0, 0.0
            proc = types.SimpleNamespace(poll=lambda: 0)

            def read(self):
                if self.n >= len(frames):
                    return None
                self.n += 1
                self.last_t = frames[self.n - 1][0]
                return frames[self.n - 1]

        feed = rms.VideoFeed.__new__(rms.VideoFeed)
        feed.reader, feed.realtime, feed.stride, feed.limit = FakeReader(), True, 1, None
        feed.used, feed.wall0, feed.ahead = 0, None, None
        feed.stats = types.SimpleNamespace(received=0)
        clock = {"t": 1000.0}
        monkeypatch.setattr(rms.time, "time", lambda: clock["t"])
        return feed, clock

    def test_fast_polling_does_not_fast_forward(self, monkeypatch) -> None:
        feed, clock = self._feed(monkeypatch, [i / 30 for i in range(300)])  # 10 秒・30 fps
        assert feed.poll()[0] == 0.0
        for _ in range(100):  # 時計が進まないうちに 100 回呼ぶ（主ループの空回り）
            assert feed.poll() is None
        clock["t"] += 1.0
        assert feed.poll()[0] == pytest.approx(1.0)  # 1 秒後に来ている中で最新のこま

    def test_last_frame_then_end(self, monkeypatch) -> None:
        feed, clock = self._feed(monkeypatch, [0.0, 0.5])
        assert feed.poll()[0] == 0.0
        clock["t"] += 5.0
        assert feed.poll()[0] == 0.5
        with pytest.raises(StopIteration):
            feed.poll()


def test_any_image_size_crops_to_multiples_of_16_and_keeps_512_as_upstream() -> None:
    script = textwrap.dedent(
        """
        import numpy as np
        import fast_mast3r
        import mast3r_slam.mast3r_utils as mu
        img = np.random.default_rng(0).random((640, 360, 3)).astype(np.float32)  # 縦長のスマホの画
        before = mu.resize_img(img, 512)["img"].shape
        fast_mast3r.allow_any_image_size()
        assert mu.resize_img(img, 512)["img"].shape == before == (1, 3, 512, 288)
        out = mu.resize_img(img, 384)
        assert out["img"].shape == (1, 3, 384, 208), out["img"].shape
        assert out["true_shape"].tolist() == [[384, 208]]
        import mast3r_slam.frame as fr
        assert fr.resize_img is mu.resize_img
        print("ok")
        """
    )
    env = dict(os.environ, PYTHONPATH=f"{FORK}:{HERE}:{UPSTREAM / 'thirdparty' / 'mast3r'}")
    r = subprocess.run([sys.executable, "-c", script], env=env, capture_output=True, text=True, timeout=180)
    assert r.returncode == 0, r.stderr[-2000:]


def test_reloc_needs_two_matches_except_while_the_map_is_tiny() -> None:
    script = textwrap.dedent(
        """
        from multicam_core import reloc_required
        assert reloc_required(2, 50) == 2   # ふつうは 2 枚以上（1 枚だけだと床の模様で別の所に重なった）
        assert reloc_required(2, 1) == 1    # 地図が 1 枚の間は 1 枚でよい（2 台目の合流が止まらない）
        assert reloc_required(1, 50) == 1
        assert reloc_required(3, 2) == 2
        print("ok")
        """
    )
    env = dict(os.environ, PYTHONPATH=f"{FORK}:{HERE}")
    r = subprocess.run([sys.executable, "-c", script], env=env, capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr[-2000:]


def test_buddy_keyframes_are_the_latest_of_the_other_cameras() -> None:
    script = textwrap.dedent(
        """
        import numpy as np
        from multicam_core import recent_other_cameras
        ids = np.array([0, 1, 0, 1, 1, 0, 1])
        assert recent_other_cameras(ids, 0, 2) == [6, 4]   # cam0 が見失ったら cam1 の最新 2 枚（新しい順）
        assert recent_other_cameras(ids, 1, 2) == [5, 2]
        assert recent_other_cameras(ids, 0, 0) == []
        assert recent_other_cameras(np.array([0, 0]), 0, 2) == []  # 相方のキーフレームがまだ無い
        print("ok")
        """
    )
    env = dict(os.environ, PYTHONPATH=f"{FORK}:{HERE}")
    r = subprocess.run([sys.executable, "-c", script], env=env, capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr[-2000:]
