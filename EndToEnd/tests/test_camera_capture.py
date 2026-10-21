"""複数のRGBが同じ描画更新から取得され、未準備のbufferを採用しないことを検証する。"""

import sys
from types import SimpleNamespace

import numpy as np
import pytest

from sim.sensors.camera import PinholeRenderer


@pytest.mark.parametrize("second_ready_after", [3, 4])
def test_rgb_cameras_share_render_updates_and_wait_for_all(monkeypatch, second_ready_after):
    clock = [0]

    def render():
        clock[0] += 1

    monkeypatch.setitem(
        sys.modules,
        "isaacsim.core.rendering_manager",
        SimpleNamespace(RenderingManager=SimpleNamespace(render=render)),
    )
    buffers = [np.zeros((2, 3, 1)), np.zeros((2, 3, 1))]

    def camera(index, annotator, ready_after):
        def get_data(name):
            assert name == annotator
            if clock[0] < ready_after:
                return [None]
            buffers[index][:] = clock[0]
            return [SimpleNamespace(numpy=lambda: buffers[index])]

        return SimpleNamespace(
            config=SimpleNamespace(render_updates=3),
            annotators=[annotator],
            sensor=SimpleNamespace(get_data=get_data),
        )

    frames = PinholeRenderer.capture_batch([camera(0, "rgb", 3), camera(1, "rgb", second_ready_after)])
    assert clock[0] == second_ready_after
    for buffer in buffers:
        buffer[:] = -1
    np.testing.assert_array_equal(frames[0]["rgb"], second_ready_after)
    np.testing.assert_array_equal(frames[1]["rgb"], second_ready_after)
