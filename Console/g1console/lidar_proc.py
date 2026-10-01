"""lidar 配信プロセス（Jetson の lidar_stream.py）の確認・起動・停止。

Jetson の ~/g1_console_lidar/lidar_ctl.sh を ssh で 1 回ずつ叩く（camera_proc と同じ作り。DDS・モーターには触れない）。
"""
import math
import struct
import time
from urllib.parse import parse_qs, urlencode, urlsplit

from g1console.camera_proc import SEP, SshCameraCtl, parse_status

REMOTE_DIR = "~/g1_console_lidar"
ACTIONS = ("status", "start", "stop")
MOCK_POINTS = 1500
MOCK_PERIOD_S = 20.0  # 模擬点群が 1 周する時間


VOXEL_MAX_M = 1.0
POINTS_MIN, POINTS_MAX = 100, 30000  # jetson/lidar_stream.py の範囲と揃える


def forward_query(path: str) -> str:
    """画面からの `/lidar?voxel=..&max=..` のうち、数値として妥当なものだけを上流へ渡す文字列にする（先頭 `?` 付き、無ければ空）。
    未知のキー・不正値・範囲外は捨てる（上流の URL に任意の文字列を入れさせない）。"""
    qs = parse_qs(urlsplit(path).query)
    out = {}
    try:
        voxel = float(qs["voxel"][0])
        if 0.0 <= voxel <= VOXEL_MAX_M:
            out["voxel"] = "%g" % voxel
    except (KeyError, ValueError):
        pass
    try:
        points = int(qs["max"][0])
        if POINTS_MIN <= points <= POINTS_MAX:
            out["max"] = str(points)
    except (KeyError, ValueError):
        pass
    return "?" + urlencode(out) if out else ""


class SshLidarCtl(SshCameraCtl):
    def __init__(self, target_fn, port_fn):
        super().__init__(target_fn)
        self._port_fn = port_fn  # () -> 配信ポート（検証済みの整数）

    def call(self, action: str) -> dict:
        if action not in ACTIONS:
            raise ValueError("unknown action")  # script に入るのは許可リストの固定語と整数だけ
        ctl = "LIDAR_PORT=%d %s/lidar_ctl.sh" % (int(self._port_fn()), REMOTE_DIR)
        steps = ([ctl + " " + action] if action != "status" else ["true"]) + [ctl + " status"]
        out = self._run(("; echo %s; " % SEP).join(steps))
        message, status = (out.split(SEP + "\n") + [""])[:2]
        return {"ok": True, **parse_status(status), "message": message.strip()}


class MockLidarCtl:
    """実機なしの模擬。起動中だけ、床の輪と壁のある部屋の点群を返す（frame()）。"""

    backend = "mock"

    def __init__(self):
        self.running = False

    def call(self, action: str) -> dict:
        if action not in ACTIONS:
            raise ValueError("unknown action")
        if action != "status":
            self.running = action == "start"
        return {"ok": True, "running": self.running, "pid": 4343 if self.running else None, "message": ""}

    def frame(self):
        """(バイナリ, 点数)。起動していなければ None。"""
        if not self.running:
            return None
        spin = (time.time() % MOCK_PERIOD_S) / MOCK_PERIOD_S * 2 * math.pi
        out = bytearray()
        for i in range(MOCK_POINTS):
            a = i / MOCK_POINTS * 2 * math.pi * 6 + spin
            ring = 1.0 + (i % 5) * 0.8
            if i % 3:  # 床の輪
                x, y, z = ring * math.cos(a), ring * math.sin(a), -0.6
            else:  # 4 m 四方の壁
                x, y, z = 4 * max(-1.0, min(1.0, 1.6 * math.cos(a))), 4 * max(-1.0, min(1.0, 1.6 * math.sin(a))), -0.6 + (i % 7) * 0.4
            out += struct.pack("<ffff", x, y, z, float(i % 100))
        return bytes(out), MOCK_POINTS
