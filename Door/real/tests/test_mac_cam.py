"""連係カメラの録り役（phone_cam/mac_cam.py）の試験。カメラの代わりに ffmpeg の作り物の入力（lavfi）で録る"""
from __future__ import annotations

import contextlib
import io
import json
import shutil
import sys
import time
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
CAM = HERE / "phone_cam"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(CAM))
import mac_cam as mc  # noqa: E402
import phone_sync as psy  # noqa: E402

needs_ffmpeg = pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="ffmpeg が無い")
LAVFI = ["-f", "lavfi", "-i", "testsrc=size=320x240:rate=30"]

# 11-13 の Mac の ffmpeg -f avfoundation -list_devices true の出力（名前の頭に U+200E が付いている）
DEVICES_1113 = """[AVFoundation indev @ 0x7f900c140] AVFoundation video devices:
[AVFoundation indev @ 0x7f900c140] [0] MacBook Airのカメラ
[AVFoundation indev @ 0x7f900c140] [1] ‎InoueのiPhoneのカメラ
[AVFoundation indev @ 0x7f900c140] [2] ‎InoueのiPhoneのデスクビューカメラ
[AVFoundation indev @ 0x7f900c140] [3] MacBook Airのデスクビューカメラ
[AVFoundation indev @ 0x7f900c140] [4] Capture screen 0
[AVFoundation indev @ 0x7f900c140] AVFoundation audio devices:
[AVFoundation indev @ 0x7f900c140] [0] MacBook Airのマイク
[AVFoundation indev @ 0x7f900c140] [1] ‎InoueのiPhoneのマイク
"""


class FakeFeed:
    def __init__(self) -> None:
        self.st = {"connected": True, "phase": "WAIT", "arms": 0, "age_s": 0.2}

    def snapshot(self) -> dict:
        return dict(self.st)

    def stop(self) -> None:
        pass


def _cam(tmp_path, feed=None, **kw):
    lines = []
    cam = mc.MacCam(tmp_path / "data", feed, vcodec="libx264", input_args=LAVFI, out=lines.append, **kw)
    return cam, lines


def _wait(cond, seconds=10.0):
    t_end = time.time() + seconds
    while time.time() < t_end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def test_parse_devices_picks_iphone_camera_not_desk_view():
    vids, auds = mc.parse_devices(DEVICES_1113)
    assert mc.pick(vids, "iPhone") == (1, "InoueのiPhoneのカメラ")
    assert mc.pick(auds, "iPhone") == (1, "InoueのiPhoneのマイク")
    assert mc.pick(vids, "Android") is None


def test_ffmpeg_cmd_takes_wallclock_start_and_fragments():
    cmd = mc.ffmpeg_cmd("ffmpeg", Path("/x/cam.mp4"), video=1, audio=1)
    i = cmd.index("-i")
    assert cmd[i + 1] == "1:1" and cmd.index("-use_wallclock_as_timestamps") < i
    assert "+frag_keyframe+empty_moov+default_base_moof" in cmd and cmd[-1] == "/x/cam.mp4"
    assert mc.ffmpeg_cmd("ffmpeg", Path("a.mp4"), video=1, audio=None)[i + 1] == "1:none"


def test_auto_policy_one_recording_per_a_and_ignores_stale_backlog():
    p = mc.AutoPolicy()
    st = {"connected": True, "phase": "DOOR", "arms": 3, "age_s": 300.0}   # つなぎ直した時の前の日の行
    assert p.tick(st, 0.0, None) is None                                      # 古い状態では録り始めない
    st.update(phase="WAIT", age_s=0.1)
    assert p.tick(st, 1.0, None) is None
    st.update(phase="ARMS_IN", arms=4)
    assert p.tick(st, 2.0, None) == "start"
    rec = mc.RecInfo(True, 4, 2.0)
    st.update(phase="DOOR")
    assert p.tick(st, 3.0, rec) is None
    st.update(phase="POST")
    assert p.tick(st, 4.0, rec) is None
    assert p.tick(st, 10.1, rec) == "stop"                                    # 止めて 6 秒
    assert p.tick(st, 10.2, None) is None                                     # 同じ a ではもう録らない
    st.update(phase="READY", arms=5)
    assert p.tick(st, 11.0, None) == "start"                                  # 次の a
    assert p.tick(dict(st, arms=6), 12.0, mc.RecInfo(True, 5, 11.0)) == "stop"  # 録っている間に次の a
    assert p.tick(dict(st, phase="WAIT"), 13.0, mc.RecInfo(False, 5, 11.0)) is None  # 手で録った物は止めない


@needs_ffmpeg
def test_manual_recording_writes_video_wallclock_start_and_marks(tmp_path):
    cam, lines = _cam(tmp_path)
    t0 = time.time()
    cam.cmds.put(("toggle", "", "", t0))
    cam.step()
    assert cam.rec is not None and _wait(lambda: cam.rec.start_mac is not None, 5)
    time.sleep(1.5)
    t_mark = time.time()
    cam.cmds.put(("mark", "当たった", "", t_mark))
    cam.step()
    cam.cmds.put(("toggle", "", "", time.time()))
    cam.step()
    assert cam.rec is None and any("録画を閉じた" in s for s in lines), lines
    rec_json = next((tmp_path / "data").glob("cam_*.json"))
    meta = json.loads(rec_json.read_text(encoding="utf-8"))
    assert meta["status"] == "済み" and meta["source"] == "continuity" and meta["duration"] == pytest.approx(1.6, abs=0.6)
    assert t0 <= meta["start_mac"] <= t0 + 2.0 and not meta.get("start_mac_estimated")
    _, video, clock, calib = psy.load_rec(rec_json, None)                     # 合わせの道具がそのまま読める
    assert video.exists() and calib == 0.0 and clock.start_mac == meta["start_mac"]
    ev = psy.mark_events(rec_json.parent / "marks.jsonl", meta["rec"], clock)
    assert sorted(e.video_t for e in ev)[-1] == pytest.approx(t_mark - meta["start_mac"], abs=1e-3)


@needs_ffmpeg
def test_auto_records_from_feed_and_closes_after_post(tmp_path, monkeypatch):
    monkeypatch.setattr(mc, "AUTO_STOP_AFTER_POST_S", 0.5)
    feed = FakeFeed()
    cam, lines = _cam(tmp_path, feed)
    cam.step()
    assert cam.rec is None
    feed.st.update(phase="ARMS_IN", arms=1)
    cam.step()
    assert cam.rec is not None and cam.rec.meta["auto"] is True and cam.rec.meta["cycle"] == 1
    time.sleep(1.0)
    feed.st.update(phase="POST")
    cam.step()
    time.sleep(0.6)
    cam.step()
    assert cam.rec is None
    metas = [json.loads(p.read_text(encoding="utf-8")) for p in (tmp_path / "data").glob("cam_*.json")]
    assert len(metas) == 1 and metas[0]["status"] == "済み" and metas[0]["stop_why"] == "自動"
    assert metas[0]["state_start"]["phase"] == "ARMS_IN" and metas[0]["state_stop"]["phase"] == "POST"


def test_devices_are_looked_up_by_name_at_each_start(tmp_path, monkeypatch):
    """11-13: Jabra をつなぐと iPhone のマイクの番号が 1 → 0 に変わった。録り始めるたびに名前で探し直す"""
    monkeypatch.setattr(mc.Recorder, "start", lambda self: None)            # ffmpeg は起こさない
    listings = [mc.parse_devices(DEVICES_1113),
                ([(0, "MacBook Airのカメラ"), (1, "InoueのiPhoneのカメラ")], [(0, "InoueのiPhoneのマイク"), (1, "Jabra Talk 45")]),
                ([(0, "MacBook Airのカメラ")], [(0, "MacBook Airのマイク")])]  # iPhone が離れた
    lines = []
    cam = mc.MacCam(tmp_path / "data", out=lines.append, want_video="iPhone", want_audio="iPhone",
                    lister=lambda ffmpeg: listings.pop(0))
    assert cam.start_rec(False, None) and "1:1" in cam.rec.cmd
    cam.rec = None
    assert cam.start_rec(False, None) and "1:0" in cam.rec.cmd
    cam.rec = None
    assert not cam.start_rec(False, None) and "見えない" in lines[-1]


def test_dead_ffmpeg_is_reported_with_hint(tmp_path):
    cam, lines = _cam(tmp_path)
    cam.input_args = ["-f", "lavfi", "-i", "no_such_filter_xyz"]               # すぐ落ちる入力（カメラを取られた代わり）
    cam.cmds.put(("toggle", "", "", time.time()))
    cam.step()
    assert _wait(lambda: not cam.rec.alive(), 5)
    cam.step()
    assert cam.rec is None and any("録れなかった" in s for s in lines)
    meta = json.loads(next((tmp_path / "data").glob("cam_*.json")).read_text(encoding="utf-8"))
    assert meta["status"] == "録れない" and meta["ffmpeg_tail"]
    assert "許可" in mc.failure_hint(["[avfoundation] Failed to create AV capture input device: not authorized"])


@needs_ffmpeg
def test_continuity_calib_is_kept_apart_from_phone(tmp_path):
    cam, _ = _cam(tmp_path)
    cam.start_rec(False, None)
    time.sleep(1.0)
    meta = cam.stop_rec("試し")
    rec_json = tmp_path / "data" / f"{meta['rec']}.json"
    calib = psy.calib_file(rec_json.parent)
    calib.write_text(json.dumps({"calib_ms": -300.0}), encoding="utf-8")      # 電話の Safari の遅れ
    with contextlib.redirect_stdout(io.StringIO()):
        assert psy.main([str(rec_json), "--set-calib", "-90"]) == 0
    d = json.loads(calib.read_text(encoding="utf-8"))
    assert d["calib_ms"] == -300.0 and d["calib_ms_continuity"] == -90.0
    _, _, clock, c = psy.load_rec(rec_json, None)
    assert c == -90.0 and clock.start_mac == pytest.approx(meta["start_mac"] - 0.09)


@needs_ffmpeg
def test_main_test_record_runs_without_feed(tmp_path):
    with contextlib.redirect_stdout(io.StringIO()) as out:
        rc = mc.main(["--data", str(tmp_path / "d"), "--test-record", "1.2", "--vcodec", "libx264",
                      "--input-args", " ".join(LAVFI)])
    assert rc == 0, out.getvalue()
    assert "--calib-frames" in out.getvalue()
    meta = json.loads(next((tmp_path / "d").glob("cam_*.json")).read_text(encoding="utf-8"))
    assert meta["status"] == "済み" and meta["stop_why"] == "試し"
