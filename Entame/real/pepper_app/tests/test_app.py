"""The Streamlit page through streamlit.testing (no browser)."""
import time
from pathlib import Path

import pytest
from streamlit.testing.v1 import AppTest

from pepperapp.weights import MODEL_DIR, WEIGHTS

TIMEOUT = 60
APP = str(Path(__file__).resolve().parents[1] / "app.py")


@pytest.fixture
def app():
    at = AppTest.from_file(APP, default_timeout=TIMEOUT).run()
    yield at
    for button in at.sidebar.button:
        if button.label == "停止" and not button.disabled:
            button.click().run()


def test_page_renders_while_stopped(app):
    assert not app.exception
    assert app.sidebar.radio(key="source_kind").value == "pepper"
    assert app.sidebar.text_input(key="pepper_ip").value == "192.168.0.85"
    assert app.title[0].value == "Pepper 認識アプリ"
    assert [tab.label for tab in app.tabs] == ["ライブ", "割り当て", "ログ", "接続確認"]
    assert any("開始" in info.value for info in app.info)


def test_pepper_mode_needs_ip_and_safety_check(app):
    assert app.sidebar.text_input(key="pepper_ip").value == "192.168.0.85"
    app.sidebar.text_input(key="pepper_ip").set_value("").run()
    app.sidebar.radio(key="robot_mode").set_value("pepper").run()
    app.sidebar.button[0].click().run()
    assert any("IP" in e.value for e in app.error)
    app.sidebar.text_input(key="pepper_ip").set_value("10.0.0.9").run()
    app.sidebar.button[0].click().run()
    assert any("腕の届く範囲" in e.value for e in app.error)


@pytest.mark.skipif(not all((MODEL_DIR / n).is_file() for n in WEIGHTS), reason="weights missing")
def test_start_with_sample_image_and_stop(app, sample_image):
    app.sidebar.radio(key="source_kind").set_value("file").run()
    app.sidebar.text_input(key="file_path").set_value(str(sample_image)).run()
    app.sidebar.button[0].click().run()
    assert not app.error, [e.value for e in app.error]
    time.sleep(2.0)
    app.run()
    assert any("認識中" in m.value for m in app.markdown)
    assert [m.label for m in app.metric][:3] == ["fps", "推論 ms", "人数"]
    stop = next(b for b in app.sidebar.button if b.label == "停止")
    stop.click().run()
    assert any("停止中" in m.value for m in app.markdown)
