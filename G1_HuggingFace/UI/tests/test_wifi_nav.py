"""無線（PC2 のアダプタに見るだけで繋ぐ）まわりのテスト。

守りたいのは3つ。
1. PC2 側（OccupancyGrid から）と UI 側（地図ファイルから）で、**同じ地図なら同じ指紋**
   になること。ずれると「同じ地図なのに違う」と警告し続け、警告が無視されるようになる
2. 見るだけのとき、**操作がアダプタへ届かない**こと
3. 表示中の地図が Nav2 の地図と違うとき、**黙って描かない**こと

⚠️ ROS も YOLO も使わない。アダプタは標準ライブラリの HTTP サーバで真似る。
"""
from __future__ import annotations

import json
import sys
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import yaml

MAIN = Path(__file__).resolve().parents[1] / "Main"
sys.path.insert(0, str(MAIN))

from mapimage import load_map                                         # noqa: E402
from nav.base import NavState                                         # noqa: E402
from nav.grid import (FREE, OCCUPIED, UNKNOWN, cells_digest,          # noqa: E402
                      cells_from_occupancy, png_from_cells)
from nav.http_source import HttpNavSource, describe_mismatch          # noqa: E402

GRIDS = (MAIN / "../../../Navigation/nav2_stable/maps/grids").resolve()
MAP_0911 = GRIDS / "room_a_map_20260911_edited.yaml"
MAP_1021 = GRIDS / "room_a_map_20261021.yaml"


def occupancy_like_map_server(yaml_path: Path) -> tuple[list[int], int, int]:
    """map_server（trinary）が地図ファイルから作る OccupancyGrid.data を真似る。"""
    meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    img = cv2.imread(str(yaml_path.parent / meta["image"]), cv2.IMREAD_UNCHANGED)
    p = (255 - img.astype(np.float64)) / 255.0
    data = np.full(img.shape, -1, dtype=np.int16)
    data[p > float(meta["occupied_thresh"])] = 100
    data[p < float(meta["free_thresh"])] = 0
    # OccupancyGrid は 0 行目が下端
    return np.flipud(data).ravel().tolist(), img.shape[1], img.shape[0]


class GridTest(unittest.TestCase):
    """PNG の向きと色、指紋の一致。"""

    def test_png_orientation_and_colors(self) -> None:
        # 2x2: 下の行(0行目) = 占有, 自由 / 上の行 = 未知, 未知
        cells = bytes([OCCUPIED, FREE, UNKNOWN, UNKNOWN])
        png = png_from_cells(cells, 2, 2)
        img = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(img.shape, (2, 2, 3))
        # PNG は上が 0 行目なので、占有（黒）は**下の行**の左に出るはず
        self.assertLess(int(img[1, 0, 0]), 0x30, "占有が下の行に出ていない（上下が逆）")
        self.assertGreater(int(img[1, 1, 0]), 0xD0, "自由が明るくない")
        self.assertEqual(int(img[0, 0, 0]), 0x44, "未知の色が違う")

    def test_cells_from_occupancy_thresholds(self) -> None:
        self.assertEqual(cells_from_occupancy([-1, 0, 19, 20, 65, 66, 100]),
                         bytes([UNKNOWN, FREE, FREE, UNKNOWN, UNKNOWN, OCCUPIED, OCCUPIED]))

    def test_same_map_same_digest(self) -> None:
        """地図ファイルから出した指紋と、map_server 経由の OccupancyGrid の指紋が一致する。"""
        for yaml_path in (MAP_0911, MAP_1021):
            with self.subTest(map=yaml_path.name):
                shown = load_map(yaml_path)
                data, w, h = occupancy_like_map_server(yaml_path)
                self.assertEqual(shown.digest, cells_digest(cells_from_occupancy(data), w, h))

    def test_different_maps_differ(self) -> None:
        self.assertNotEqual(load_map(MAP_0911).digest, load_map(MAP_1021).digest)


class _FakeAdapter(BaseHTTPRequestHandler):
    """アダプタの口だけ真似る。`posted` に操作が届いたかを残す。"""

    state: dict = {}
    map_info: dict | None = None
    posted: list = []

    def log_message(self, *args) -> None:
        pass

    def _json(self, obj: object, status: int = 200) -> None:
        body = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if self.path == "/api/state":
            self._json(self.state)
        elif self.path == "/api/map.json" and self.map_info:
            self._json(self.map_info)
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self) -> None:
        type(self).posted.append(self.path)
        self._json({"ok": True, "message": "受理"})


class HttpSourceTest(unittest.TestCase):
    """見るだけと、地図の照合。"""

    def setUp(self) -> None:
        _FakeAdapter.posted = []
        _FakeAdapter.state = {"connected": True, "bridge_state": "NAVIGATING",
                              "pose": {"x": 1.0, "y": 2.0, "yaw_deg": 90.0},
                              "patrol": {"state": "RUNNING", "index": 0, "total": 4},
                              "read_only": True}
        _FakeAdapter.map_info = None
        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), _FakeAdapter)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.url = f"http://127.0.0.1:{self.httpd.server_address[1]}"

    def tearDown(self) -> None:
        self.httpd.shutdown()
        self.httpd.server_close()

    def test_read_only_never_posts(self) -> None:
        src = HttpNavSource(self.url, read_only=True)
        r = src.send_command("stop")
        self.assertFalse(r.ok)
        self.assertEqual(_FakeAdapter.posted, [], "見るだけなのに操作がアダプタへ届いた")

    def test_read_only_from_adapter_is_shown(self) -> None:
        """UI 側の設定を忘れても、アダプタが見るだけなら画面も見るだけにする。"""
        s = HttpNavSource(self.url, read_only=False).get_state()
        self.assertTrue(s.read_only)
        self.assertEqual((s.pose.x, s.pose.y), (1.0, 2.0))
        self.assertIn("read_only", NavState().as_dict())

    def test_map_warning(self) -> None:
        shown = load_map(MAP_1021)
        src = HttpNavSource(self.url, map_yaml=MAP_1021)
        # Nav2 の地図がまだ無い
        self.assertIn("まだ届いていない", src.map_warning())
        # 同じ地図
        _FakeAdapter.map_info = shown.as_dict()
        src._nav2_map = None
        self.assertEqual(src.map_warning(), "")
        # 中身だけ違う
        _FakeAdapter.map_info = dict(shown.as_dict(), digest="0" * 16)
        src._nav2_map = None
        self.assertIn("中身が違います", src.map_warning())

    def test_describe_mismatch_size(self) -> None:
        a, b = load_map(MAP_1021).as_dict(), load_map(MAP_0911).as_dict()
        self.assertIn("大きさ・原点が違います", describe_mismatch(a, b, "x.yaml"))


if __name__ == "__main__":
    unittest.main()
