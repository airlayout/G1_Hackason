"""11-13: iPhone で撮りながら調べる道具（Door/real/phone_cam/）。機体も iPhone も要らない（ffmpeg・openssl・node は Mac のもの）。

  G1_Hackason/Navigation/.venv/bin/python -m pytest G1_Hackason/Door/real/tests/test_phone_cam.py -q

確かめる物: 時計の合わせ（電話↔Mac・PC2 の monotonic↔Mac・PC2 の起こし直し）・PC2 の出力の読み（11-12 の実際の行）・
見張り役（新しい行・起こし直しでファイルが替わる）・受け口（鍵・塊の順番・送り直し・mp4 に直す・印・https）・
合わせの道具（動画の秒に出来事と印が来る・記録の行で引き直す）・撮影のページの JS の構文。
"""
from __future__ import annotations

import contextlib
import io
import json
import math
import os
import shutil
import socket
import ssl
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
CAM = HERE / "phone_cam"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(CAM))
import access_log as al  # noqa: E402
import feed_parse as fp  # noqa: E402
import phone_server as ps  # noqa: E402
import phone_sync as psy  # noqa: E402
import robot_feed as rf  # noqa: E402
import timebase as tb  # noqa: E402

needs_ffmpeg = pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="ffmpeg が無い")
TOKEN = "test-token"

# 11-12 の console_20261112_080709.log から（10 回目・扉 10°）
LINES_1112 = [
    "[door_stock] WAIT → ARMS_IN（腕を借りる）",
    "[door_stock] ARMS_IN → READY（腕の差 0.01 rad（以後は届いた角度で保つ）・odom の基準を取った）",
    "[door_stock] READY    傾き  2.2° |dq|  0.0 足 L 196 N・R 193 N（体重の 119%） 腕の重み 1.00 歩き 立つ [0.0, 0.0, 0.0] "
    "RPC 0 ms code None 姿勢 (-1.00,+0.00,-0°) 扉 nan° 周期 20.4 ms（計算 6.3 ms） 腕 最高 55 °C（関節 15）",
    "[door_stock] READY → DOOR（180.0 秒・段に分けて（段 0 から））",
    "[script] 2a → 2b（手のひら−当てる点 前 -28.4 横 +0.9 上 -39.2 cm・扉 0.0°）",
    "[script] 2b → 2c（手のひら−当てる点 前 -9.0 横 +2.9 上 +8.3 cm・扉 0.0°）",
    "[script] 触れた（高さで）: 高さ 1.020 m（当てる点 1.020）・遅れ -0.3 cm・力の変化 2.4 N·m",
    "[door_stock] DOOR     傾き  3.1° |dq|  0.4 足 L 150 N・R 201 N（体重の 108%） 腕の重み 1.00 歩き 立つ [0.0, 0.0, 0.0] "
    "RPC 0 ms code None 姿勢 (-0.40,+0.09,-6°) 扉 3.0° 周期 21.0 ms（計算 9.1 ms） 腕 最高 83 °C（関節 15）",
    "[door_stock] DOOR → POST（扉 10°（--stop-door-deg 10））",
    "[ana] この回",
    "[ana] 止まった理由: 扉 10°",
]


# ── 時計 ──────────────────────────────────────────────
def test_best_ping_uses_shortest_round_trip():
    pings = [tb.Ping(10.0, 110.30, 10.40), tb.Ping(11.0, 111.02, 11.04), tb.Ping(12.0, 112.5, 12.9)]
    p = tb.best_ping(pings)
    assert p.rtt == pytest.approx(0.04) and p.offset == pytest.approx(100.0)
    vc = tb.video_clock(20.0, pings, calib_s=-0.1, duration=5.0)
    assert vc.to_mac(1.0) == pytest.approx(120.9)
    assert vc.contains(121.0) and not vc.contains(126.0)


def test_mono_clock_lower_envelope_and_reboot():
    rng = np.random.default_rng(0)
    d1, d2 = 1.0e6, 1.0e6 + 5000.0   # 起こし直すと差が大きく変わる
    s = [(m - d1, m + 0.003 + 0.02 * rng.random()) for m in np.arange(1000.0, 1300.0, 0.5)]
    s += [(m - d2, m + 0.003 + 0.02 * rng.random()) for m in np.arange(2000.0, 2300.0, 0.5)]
    mc = tb.MonoClock(s)
    assert mc.mono_to_mac(1100.0 - d1) == pytest.approx(1100.0, abs=0.01)
    assert mc.mono_to_mac(2100.0 - d2) == pytest.approx(2100.0, abs=0.01)
    assert mc.offset_at(1150.0) == pytest.approx(d1, abs=0.01)
    with pytest.raises(ValueError):
        mc.mono_to_mac(-5.0e6)


def test_mono_clock_hint_picks_boot_near_video():
    """同じ日に PC2 を起こし直すと、同じ monotonic が 2 つの起動のどちらにも合う → 動画の時刻に近い方"""
    a = [(m - 0.0, m + 0.003) for m in np.arange(1000.0, 1400.0, 0.5)]           # 起動 A: mono = Mac − 0
    b = [(m - 900.0, m + 0.003) for m in np.arange(1900.0, 2300.0, 0.5)]         # 起動 B: mono = Mac − 900
    mc = tb.MonoClock(a + b)
    assert mc.mono_to_mac(1200.0, hint_mac=1250.0) == pytest.approx(1200.0, abs=0.01)   # A の 1200
    assert mc.mono_to_mac(1200.0, hint_mac=2050.0) == pytest.approx(2100.0, abs=0.01)   # B の 2100


def test_best_ping_near_prefers_pings_around_start():
    far_short = tb.Ping(0.0, 100.0, 0.004)                   # 10 分前の短い往復（時計の歩みの差が入る）
    near = tb.Ping(600.0, 700.05, 600.02)
    assert tb.best_ping_near([far_short, near], 600.0) is near
    assert tb.best_ping_near([far_short], 600.0) is far_short   # 近くに無ければ全部から


# ── 出力の読み ─────────────────────────────────────────
def test_feed_parse_reads_1112_lines():
    st = fp.RobotState()
    for k, line in enumerate(LINES_1112):
        st = fp.apply_line(st, line, 100.0 + 0.1 * k)
    assert st.phase == "POST" and st.tries == 1 and st.arms == 1
    assert st.sub == "2c" and fp.sub_label(st.sub) == "2c 下ろして触れる"
    assert st.door_deg == pytest.approx(3.0) and st.temp_c == 83 and st.period_ms == pytest.approx(21.0)
    assert st.feet_n == (150.0, 201.0)
    assert st.ana == ("[ana] この回", "[ana] 止まった理由: 扉 10°")
    texts = [t for _, t in st.events]
    assert any("触れた" in t for t in texts) and texts[-1].startswith("DOOR → POST")
    j = fp.to_json(st, now=101.5)
    assert j["active"] is True and j["sub_label"] == "2c 下ろして触れる" and json.dumps(j, ensure_ascii=False)
    st2 = fp.apply_line(st, "[ana] 次の回", 100.0 + 0.1 * len(LINES_1112) + 5.0)  # 3 秒より空いたら新しい要約
    assert st2.ana == ("[ana] 次の回",)


# ── 見張り役 ───────────────────────────────────────────
def _agent(log: Path, seconds: float) -> subprocess.Popen:
    return subprocess.Popen([sys.executable, "-u", str(CAM / "pc2_feed_agent.py"), "--log", str(log), "--backlog", "2",
                             "--max-seconds", str(seconds)], stdout=subprocess.PIPE, text=True)


def test_agent_follows_new_lines_and_runner_restart(tmp_path):
    a, b, link = tmp_path / "console_a.log", tmp_path / "console_b.log", tmp_path / "console_latest.log"
    a.write_text("古い 1\n古い 2\n古い 3\n", encoding="utf-8")
    link.symlink_to(a.name)
    p = _agent(link, 2.0)
    time.sleep(0.4)
    with open(a, "a", encoding="utf-8") as f:
        f.write("新しい 1\n途中")
    time.sleep(0.3)
    with open(a, "a", encoding="utf-8") as f:
        f.write("まで\n")
    time.sleep(0.3)
    b.write_text("起こし直し 1\n", encoding="utf-8")
    link.unlink()
    link.symlink_to(b.name)                   # 走らせ役を起こし直した
    out, _ = p.communicate(timeout=10)
    msgs = [json.loads(s) for s in out.splitlines()]
    lines = [ln for m in msgs for ln in m["lines"]]
    assert lines == ["古い 2", "古い 3", "新しい 1", "途中まで", "起こし直し 1"]
    assert all("mono" in m and "wall" in m for m in msgs)
    # 行が無くても 0.5 秒あけずに出す（時計の見本・つながっている印）
    assert any(not m["lines"] for m in msgs)
    assert max(b["mono"] - a["mono"] for a, b in zip(msgs[:-1], msgs[1:])) < 0.65


def test_robot_feed_local_agent_updates_state_and_writes_feed(tmp_path):
    log = tmp_path / "console.log"
    log.write_text("", encoding="utf-8")
    feed = rf.RobotFeed(tmp_path, cmd=[sys.executable, "-u", str(CAM / "pc2_feed_agent.py"), "--log", str(log),
                                       "--max-seconds", "3"], stdin_script=False)
    feed.start()
    time.sleep(0.5)
    with open(log, "a", encoding="utf-8") as f:
        f.write("\n".join(LINES_1112[:5]) + "\n")
    time.sleep(0.8)
    snap = feed.snapshot()
    feed.stop()
    assert snap["connected"] and snap["phase"] == "DOOR" and snap["sub"] == "2b" and snap["arms"] == 1
    assert snap["mono_offset"] == pytest.approx(time.time() - time.monotonic(), abs=0.2)
    samples, lines = rf.load_feed([feed.feed_path])
    assert len(samples) >= 2 and [s for _, s in lines] == LINES_1112[:5]
    assert all(abs(t - time.time()) < 5 for t, _ in lines)


def test_robot_feed_pipes_agent_through_stdin_like_ssh(tmp_path):
    """ssh の時と同じく、見張り役の中身を標準入力で流し込んで起こす（python3 -u - --log …）"""
    log = tmp_path / "console.log"
    log.write_text("\n".join(LINES_1112[:2]) + "\n", encoding="utf-8")
    feed = rf.RobotFeed(tmp_path, cmd=[sys.executable, "-u", "-", "--log", str(log), "--max-seconds", "1.5"],
                        stdin_script=True)
    feed.start()
    assert _wait(lambda: feed.snapshot()["phase"] == "READY", 5.0)   # 末尾の行（backlog）で今の段が分かる
    feed.stop()


def test_reconnect_backlog_is_not_counted_twice(tmp_path):
    """つなぎ直すと見張り役は末尾の行をもう一度送る。数え直すと ARMS_IN が二重になり、自動の録画が勝手に切り替わる"""
    feed = rf.RobotFeed(tmp_path, cmd=["true"], stdin_script=False)
    msg = lambda lines, back, mono: json.dumps({"mono": mono, "lines": lines, **({"backlog": True} if back else {})})  # noqa: E731
    feed.handle(msg([LINES_1112[0]], True, 10.0))      # 初めてつないだ: 末尾の行で今の状態を作る
    assert feed.snapshot()["arms"] == 1
    feed.handle(msg([LINES_1112[0]], True, 20.0))      # つなぎ直した: 同じ行が来る
    assert feed.snapshot()["arms"] == 1
    feed.handle(msg(["[door_stock] POST → ARMS_IN（構え直す）"], False, 21.0))
    assert feed.snapshot()["arms"] == 2
    _, lines = rf.load_feed([feed.feed_path])
    assert [s for _, s in lines] == ["[door_stock] POST → ARMS_IN（構え直す）"]  # 時刻の分からない末尾の行は合わせに使わない


def test_agent_is_python38_grammar():
    import ast
    ast.parse((CAM / "pc2_feed_agent.py").read_text(encoding="utf-8"), feature_version=(3, 8))  # PC2 は 3.8


def test_ssh_command_uses_jump_and_key():
    cmd = rf.ssh_command({"G1_PC2_HOST": "192.168.0.82", "G1_JUMP": "ubuntu@100.99.102.70", "G1_KEY": "/k"})
    assert cmd[:3] == ["ssh", "-i", "/k"] and cmd[cmd.index("-J") + 1] == "ubuntu@100.99.102.70"
    assert cmd[-6:] == ["unitree@192.168.0.82", "python3", "-u", "-", "--log", rf.REMOTE_LOG]
    assert "-J" not in rf.ssh_command({"G1_KEY": "/k"})


# ── 受け口 ────────────────────────────────────────────
@contextlib.contextmanager
def _server(tmp_path, ctx=None, handler=ps.Handler, access=None):
    app = ps.App(tmp_path / "data", TOKEN, feed=None)
    srv = ps.TLSServer(("127.0.0.1", 0), handler, app, ctx, access)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    try:
        yield app, f"{'https' if ctx else 'http'}://127.0.0.1:{srv.server_address[1]}"
    finally:
        srv.shutdown()
        srv.server_close()


def _req(url, data=None, key=TOKEN, ctype=None, ctx=None):
    headers = {"X-Key": key} if key else {}
    if isinstance(data, dict):
        data, ctype = json.dumps(data).encode(), "application/json"
    if ctype:
        headers["Content-Type"] = ctype
    r = urllib.request.Request(url, data=data, headers=headers, method="POST" if data is not None else "GET")
    try:
        with urllib.request.urlopen(r, timeout=10, context=ctx) as resp:
            return resp.status, resp.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


def _video(path: Path, seconds: float = 2.0) -> Path:
    subprocess.run(["ffmpeg", "-y", "-v", "error", "-f", "lavfi", "-i", f"testsrc=size=320x240:rate=30:duration={seconds}",
                    "-f", "lavfi", "-i", f"sine=frequency=440:duration={seconds}", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                    "-c:a", "aac", "-shortest", str(path)], check=True)
    return path


def _wait(cond, seconds=15.0):
    t_end = time.time() + seconds
    while time.time() < t_end:
        if cond():
            return True
        time.sleep(0.1)
    return False


def test_server_requires_key_and_serves_page_without_it(tmp_path):
    with _server(tmp_path) as (_, base):
        assert _req(base + "/api/time", key="")[0] == 403
        assert _req(base + "/api/time", key="wrong-token")[0] == 403
        code, body = _req(base + "/api/time")
        assert code == 200 and abs(json.loads(body)["t"] / 1000 - time.time()) < 2
        code, body = _req(base + "/", key="")
        assert code == 200 and "G1 撮影".encode() in body
        assert _req(base + "/../phone_server.py", key="")[0] in (403, 404)
        assert json.loads(_req(base + "/api/state")[1])["connected"] is False


@needs_ffmpeg
def test_server_receives_chunks_in_order_and_makes_mp4(tmp_path):
    src = _video(tmp_path / "src.mp4")
    blob = src.read_bytes()
    parts = [blob[: len(blob) // 3], blob[len(blob) // 3: 2 * len(blob) // 3], blob[2 * len(blob) // 3:]]
    with _server(tmp_path) as (app, base):
        code, body = _req(base + "/api/rec/start", {"ua": "試験", "mime": "video/mp4", "width": 320, "height": 240})
        rec = json.loads(body)["rec"]
        assert code == 200 and ps.REC_ID_RE.match(rec)
        url = f"{base}/api/rec/chunk?rec={rec}&seq="
        assert _req(url + "0", parts[0], ctype="application/octet-stream")[0] == 200
        assert json.loads(_req(url + "0", parts[0], ctype="application/octet-stream")[1])["dup"] is True  # 送り直し
        code, body = _req(url + "2", parts[2], ctype="application/octet-stream")
        assert code == 409 and json.loads(body)["next"] == 1                                             # 飛んだ
        assert _req(url + "1", parts[1], ctype="application/octet-stream")[0] == 200
        # 電話は 3 つ送ったつもり・受け口は 2 つ（3 つ目が途中）→ 閉じない
        assert _req(base + "/api/rec/stop", {"rec": rec, "chunks": 3, "start_phone_ms": 1.0, "pings": []})[0] == 409
        assert _req(url + "2", parts[2], ctype="application/octet-stream")[0] == 200
        now = 1000 * time.time()
        pings = [[now - 30, now + 5000, now + 10], [now - 50, now + 5000, now + 50]]
        code, body = _req(base + "/api/rec/stop", {"rec": rec, "chunks": 3, "start_phone_ms": now, "pings": pings})
        assert code == 200 and json.loads(body)["rtt_ms"] == pytest.approx(40.0, abs=0.01)  # 今の時刻 1.8e12 ms の丸めで 2e-4 揺れる
        meta_path = app.data / f"{rec}.json"
        assert _wait(lambda: json.loads(meta_path.read_text(encoding="utf-8")).get("duration"))
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        assert meta["video"] == f"{rec}.mp4" and meta["duration"] == pytest.approx(2.0, abs=0.1)
        assert meta["start_mac"] == pytest.approx(now / 1000 + 5.01, abs=0.02)  # Mac − 電話 = 5000 − (−10) ms
        assert not (app.data / f"{rec}.raw").exists()
        assert _req(f"{base}/api/rec/chunk?rec=../../x&seq=0", b"x", ctype="application/octet-stream")[0] == 400


def _upload(base, blob, n=2):
    code, body = _req(base + "/api/rec/start", {"ua": "試験", "mime": "video/mp4", "pings": [], "call_phone_ms": 1.0})
    rec = json.loads(body)["rec"]
    step = len(blob) // n + 1
    for k in range(n):
        assert _req(f"{base}/api/rec/chunk?rec={rec}&seq={k}", blob[k * step:(k + 1) * step], ctype="application/octet-stream")[0] == 200
    return rec


@needs_ffmpeg
def test_stop_retry_is_idempotent_and_missing_duration_still_writes_meta(tmp_path, monkeypatch):
    blob = _video(tmp_path / "src.mp4").read_bytes()
    monkeypatch.setattr(ps, "probe_duration", lambda *a, **k: None)  # ffprobe が長さを返さない時
    with _server(tmp_path) as (app, base):
        rec = _upload(base, blob)
        now = 1000 * time.time()
        stop = {"rec": rec, "chunks": 2, "start_phone_ms": now, "pings": [[now - 10, now, now + 10]]}
        assert _req(base + "/api/rec/stop", stop)[0] == 200
        code, body = _req(base + "/api/rec/stop", stop)                 # 答えが失われて送り直した
        assert code == 200 and json.loads(body)["again"] is True
        meta_path = app.data / f"{rec}.json"
        assert _wait(lambda: json.loads(meta_path.read_text(encoding="utf-8")).get("status") == "済み")
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        assert meta["video"] == f"{rec}.mp4" and meta["duration"] is None and (app.data / meta["video"]).exists()


@needs_ffmpeg
def test_salvage_recovers_recording_without_stop(tmp_path):
    """電話が落ちて stop が来なかった録画は、受け口を起こし直した時に mp4 に直す"""
    blob = _video(tmp_path / "src.mp4").read_bytes()
    with _server(tmp_path) as (app, base):
        rec = _upload(base, blob)
    app2 = ps.App(tmp_path / "data", TOKEN, feed=None)
    with contextlib.redirect_stdout(io.StringIO()):
        assert app2.salvage() == 1
    meta = json.loads((app2.data / f"{rec}.json").read_text(encoding="utf-8"))
    assert meta["salvaged"] is True and meta["video"] == f"{rec}.mp4" and meta["duration"] == pytest.approx(2.0, abs=0.1)
    assert not (app2.data / f"{rec}.raw").exists()


def test_cut_off_chunk_is_not_written_and_resend_fills_it(tmp_path):
    """11-13 の実機: 遅い Wi-Fi で電話が塊を送る途中で打ち切り（16 KB の切れ目で 352 KB）、受け口がその欠けた本文を
    書いて「受けた」にしたので、送り直しは「もう受けた」で捨てられ、mp4 の断片が欠けた。切れた本文は書かない"""
    lines = []
    with _server(tmp_path, access=al.AccessLog(out=lines.append)) as (app, base):
        code, body = _req(base + "/api/rec/start", {"ua": "試験", "mime": "video/mp4", "pings": [], "call_phone_ms": 1.0})
        rec = json.loads(body)["rec"]
        blob = bytes(range(256)) * 64                                     # 16 KB
        host, port = base.split("//")[1].split(":")
        with socket.create_connection((host, int(port)), timeout=5) as s:  # 送る途中で切る（電話の時間切れの真似）
            s.sendall((f"POST /api/rec/chunk?rec={rec}&seq=0 HTTP/1.1\r\nHost: x\r\nX-Key: {TOKEN}\r\n"
                       f"Content-Type: application/octet-stream\r\nContent-Length: {len(blob)}\r\n\r\n").encode() + blob[:5000])
            s.shutdown(socket.SHUT_WR)
            s.settimeout(5)
            assert s.recv(100) == b""                                     # 答えずに閉じる
        assert _wait(lambda: any("途中で切れた" in s for s in lines), 5)
        assert app.recs[rec].next_seq == 0 and (app.data / f"{rec}.raw").read_bytes() == b""
        code, body = _req(f"{base}/api/rec/chunk?rec={rec}&seq=0", blob, ctype="application/octet-stream")
        assert code == 200 and json.loads(body) == {"next": 1, "bytes": len(blob)}  # 送り直しは「もう受けた」でなく書かれる
        assert (app.data / f"{rec}.raw").read_bytes() == blob


@needs_ffmpeg
def test_remux_keeps_raw_when_middle_is_broken(tmp_path):
    """11-13: 頭の断片が欠けた .raw を mp4 に入れ直すと、ffmpeg は壊れた所で読むのをやめて 1 割の大きさで「成功」し、
    .raw が消えた。大きく縮んだら直せない扱いにして .raw を残す"""
    frag = tmp_path / "frag.mp4"                                       # iPhone の MediaRecorder と同じ断片に分かれた mp4
    subprocess.run(["ffmpeg", "-y", "-v", "error", "-i", str(_video(tmp_path / "src.mp4", seconds=6.0)), "-c:v", "libx264",
                    "-g", "30", "-c:a", "aac", "-movflags", "frag_keyframe+empty_moov+default_base_moof", str(frag)], check=True)
    b = frag.read_bytes()
    m0 = b.find(b"moof") - 4
    d0 = b.find(b"mdat", m0) - 4
    m1 = b.find(b"moof", d0) - 4
    assert 0 < m0 < d0 < m1
    raw = tmp_path / "phone_20261113_074003.raw"
    raw.write_bytes(b[: d0 + (m1 - d0) // 4] + b[m1:])                 # 最初の断片の mdat を 1/4 で切る（11-13 と同じ形）
    (tmp_path / "phone_20261113_074003.json").write_text(json.dumps({"rec": "phone_20261113_074003"}), encoding="utf-8")
    with contextlib.redirect_stdout(io.StringIO()) as out:
        st = ps.remux_and_note(tmp_path, "phone_20261113_074003", raw, "video/mp4")
    assert st == "直せない" and raw.exists() and "途中が壊れている" in out.getvalue()


def test_chunk_write_failure_returns_507_and_resend_is_clean(tmp_path):
    app = ps.App(tmp_path / "data", TOKEN, feed=None)
    with contextlib.redirect_stdout(io.StringIO()):
        rec = app.start_rec({"mime": "video/mp4"})["rec"]
    raw = app.data / f"{rec}.raw"
    assert app.chunk(rec, 0, b"aaaa")[0] == 200
    raw.unlink()
    raw.mkdir()                                                          # 書けない（ディスクが一杯の代わり）
    assert app.chunk(rec, 1, b"bbbb")[0] == 507
    raw.rmdir()
    raw.write_bytes(b"aaaaXX")                                            # 書きかけが残った
    assert app.chunk(rec, 1, b"bbbb")[0] == 200                           # 送り直し: 書きかけを切り捨てて続ける
    assert raw.read_bytes() == b"aaaabbbb"


def test_server_mark_writes_mac_time(tmp_path):
    with _server(tmp_path) as (app, base):
        code, _ = _req(base + "/api/mark", {"rec": None, "phone_ms": 1000.0, "label": "当たった", "memo": "棒の右",
                                             "offset_ms": 500.0, "rtt_ms": 12})
        assert code == 200
        row = json.loads((app.data / "marks.jsonl").read_text(encoding="utf-8").splitlines()[0])
        assert row["label"] == "当たった" and row["memo"] == "棒の右" and row["mac"] == pytest.approx(1.5)
        assert _req(base + "/api/mark", {"label": "x"})[0] == 400  # 時刻が無い


def test_plain_port_serves_only_ca_and_clock(tmp_path):
    with _server(tmp_path, handler=ps.PlainHandler) as (_, base):
        assert _req(base + "/clock", key="")[0] == 200
        assert _req(base + "/api/time")[0] == 404
        assert _req(base + "/api/mark", {"phone_ms": 1})[0] == 403
        assert "証明書".encode() in _req(base + "/", key="")[1]


def test_make_cert_and_tls_handshake(tmp_path, monkeypatch):
    if shutil.which("openssl") is None:
        pytest.skip("openssl が無い")
    env = dict(os.environ, HOME=str(tmp_path))
    r = subprocess.run(["bash", str(CAM / "make_cert.sh")], env=env, capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stderr
    conf = tmp_path / ".config" / "g1_phone_cam"
    san = (conf / "server.san").read_text(encoding="utf-8")
    assert "IP:127.0.0.1" in san and "DNS:localhost" in san
    r2 = subprocess.run(["bash", str(CAM / "make_cert.sh")], env=env, capture_output=True, text=True, timeout=60)
    assert "そのまま使える" in r2.stdout                       # 2 回目は作り直さない
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(conf / "server.crt", conf / "server.key")
    client = ssl.create_default_context(cafile=str(conf / "ca.crt"))
    monkeypatch.setattr(ps, "CONF", conf)  # http の口が配る /ca.crt もこの試験の認証局にする
    lines = []
    access = al.AccessLog(plain_url="http://127.0.0.1:8080/", out=lines.append)
    with _server(tmp_path, ctx=ctx, access=access) as (_, base):
        code, body = _req(base.replace("127.0.0.1", "localhost") + "/api/time", ctx=client)
        assert code == 200
        with pytest.raises((urllib.error.URLError, ssl.SSLError)):  # 信頼していない相手（iPhone に入れる前）は握手で落ちる
            _req(base + "/api/time", ctx=ssl.create_default_context())
        # 断った理由と直し方が Mac の画面に出る（電話から「繋がらない」時に、来ているか・どこで断ったかが分かる）
        assert _wait(lambda: any("握手が通らない" in s for s in lines), 5)
        bad = next(s for s in lines if "握手が通らない" in s)
        assert "証明書を信頼していない" in bad and "http://127.0.0.1:8080/" in bad
        assert _req(base + "/api/time", key="old-key", ctx=client)[0] == 403
        assert any("鍵が合わない" in s for s in lines)
        with _server(tmp_path, handler=ps.PlainHandler) as (_, plain):
            port, http_port = int(base.rsplit(":", 1)[1]), int(plain.rsplit(":", 1)[1])
            ok, msg = al.self_check("127.0.0.1", port, http_port, TOKEN, conf / "ca.crt")
            assert ok, msg
            ok, msg = al.self_check("127.0.0.1", port, http_port, TOKEN, None)  # 手元の認証局を使わなければ落ちる
            assert not ok and "届かない" in msg


def test_access_log_maps_tls_errors_and_rate_limits():
    plain = "http://192.168.0.88:8080/"
    assert "信頼していない" in al.tls_hint(ssl.SSLError(1, "[SSL: TLSV1_ALERT_UNKNOWN_CA] tlsv1 alert unknown ca"), plain)
    assert "信頼していない" in al.tls_hint(ssl.SSLError(1, "[SSL: SSLV3_ALERT_CERTIFICATE_UNKNOWN] x"), plain)
    assert "http://" in al.tls_hint(ssl.SSLError(1, "[SSL: HTTP_REQUEST] http request"), plain)
    assert plain in al.tls_hint(ssl.SSLEOFError(8, "EOF occurred in violation of protocol"), plain)
    assert plain in al.tls_hint(ConnectionResetError(54, "Connection reset by peer"), plain)
    t = [0.0]
    lines = []
    log = al.AccessLog(plain_url=plain, out=lines.append, clock=lambda: t[0], quiet_ips=("192.168.0.88",))
    assert log.note("192.168.0.11", "page", "開いた")
    assert not log.note("192.168.0.11", "page", "開いた")       # 同じ相手・同じ種類は 30 秒に 1 回
    assert log.note("192.168.0.11", "key", "鍵")                 # 種類が違えば出す
    assert log.note("192.168.0.171", "page", "開いた")           # 相手が違えば出す
    assert not log.note("192.168.0.88", "page", "Mac 自身")      # 自己点検は出さない
    t[0] = al.REPEAT_S + 1
    assert log.note("192.168.0.11", "page", "開いた")
    assert "2 回目" in lines[-1]
    ua = ("Mozilla/5.0 (iPhone; CPU iPhone OS 18_7 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) "
          "Version/26.6.3 Mobile/15E148 Safari/604.1")                  # 11-13 の iPhone（iOS 26 は UA の OS を 18 で止めた）
    assert ps.device_name({"User-Agent": ua}) == "iPhone Safari 26.6.3（UA の OS 18.7）"


def test_empty_recording_is_marked_empty_and_logs_phone_diag(tmp_path):
    """11-13: 2 本目から電話の録画が塊を 1 つも出さず、0 バイトの .raw に「mp4 に直せない」が並んだ。
    空は空と書き、電話が送った中の様子（塊の数・カメラのトラック）を Mac の画面に出す"""
    with _server(tmp_path) as (app, base), contextlib.redirect_stdout(io.StringIO()) as out:
        rec = json.loads(_req(base + "/api/rec/start", {"mime": "video/mp4", "pings": [], "call_phone_ms": 1.0})[1])["rec"]
        now = 1000 * time.time()
        diag = {"events": 0, "empty": 0, "bytes": 0, "firstDataMs": None, "errors": [], "hidden": 0, "retried": False,
                "trackStart": "audio:live video:live", "trackStop": "audio:live video:live・muted", "recorderMime": "video/mp4"}
        code, _ = _req(base + "/api/rec/stop", {"rec": rec, "chunks": 0, "start_phone_ms": now, "pings": [[now - 5, now, now + 5]],
                                                "diag": diag})
        assert code == 200
    meta = json.loads((app.data / f"{rec}.json").read_text(encoding="utf-8"))
    assert meta["status"] == "空" and meta["diag"]["trackStop"].endswith("muted") and not (app.data / f"{rec}.raw").exists()
    assert "塊を 1 つも出さなかった" in out.getvalue() and "最初の塊 無し" in out.getvalue() and "muted" in out.getvalue()
    assert "古いページ" in ps.diag_text(None) and ps.diag_text({"events": "x"}).startswith("出た塊 ?")
    # 前に起こした時の 0 バイトの .raw は、起こし直した時に「空」で片付ける
    (app.data / "phone_20261113_081020.raw").write_bytes(b"")
    (app.data / "phone_20261113_081020.json").write_text(json.dumps({"rec": "phone_20261113_081020"}), encoding="utf-8")
    app2 = ps.App(app.data, TOKEN, feed=None)
    with contextlib.redirect_stdout(io.StringIO()):
        app2.salvage()
    assert json.loads((app.data / "phone_20261113_081020.json").read_text(encoding="utf-8"))["status"] == "空"
    assert not (app.data / "phone_20261113_081020.raw").exists()


# ── 合わせの道具 ───────────────────────────────────────
def _rec_dir(tmp_path, start_mac: float, seconds: float = 5.0):
    d = tmp_path / "phone" / "20261113"
    d.mkdir(parents=True)
    rec = "phone_20261113_101500"
    _video(d / f"{rec}.mp4", seconds)
    phone_ms = 1000 * (start_mac - 7.0)   # 電話の時計は Mac より 7 秒遅い
    pings = [[phone_ms - 20, 1000 * start_mac + 10, phone_ms + 20]]  # Mac − 電話 = 7.01 秒
    meta = {"rec": rec, "video": f"{rec}.mp4", "duration": seconds, "start_phone_ms": phone_ms, "pings": pings}
    (d / f"{rec}.json").write_text(json.dumps(meta), encoding="utf-8")
    return d, d / f"{rec}.json", rec


def _feed(d: Path, d_mono: float, rows):
    """rows: (Mac の時刻, 行の一覧)。見張り役は Mac から 3 ms 遅れて届く形"""
    with open(d / "feed_20261113_101000.jsonl", "w", encoding="utf-8") as f:
        for mac, lines in rows:
            f.write(json.dumps({"mac": mac + 0.003, "mono": mac - d_mono, "wall": mac - 3600, "file": "c.log",
                                "lines": lines}, ensure_ascii=False) + "\n")


@needs_ffmpeg
def test_sync_places_feed_events_and_marks_on_video_time(tmp_path, capsys):
    start = 1.8e9
    d, rec_json, rec = _rec_dir(tmp_path, start)
    v0 = start + 0.01                                          # 動画の 0 秒の Mac の時刻
    rows = [(v0 + t, []) for t in np.arange(-3.0, 6.0, 0.5)]
    rows += [(v0 + 1.0, [LINES_1112[3]]), (v0 + 2.0, [LINES_1112[4]]), (v0 + 3.0, [LINES_1112[6], LINES_1112[7]])]
    _feed(d, 5.0e5, sorted(rows, key=lambda r: r[0]))
    (d / "marks.jsonl").write_text(json.dumps({"rec": rec, "label": "当たった", "memo": "", "mac": v0 + 3.5}) + "\n",
                                   encoding="utf-8")
    assert psy.main([str(rec_json), "--at", "4.2"]) == 0
    ev = json.loads((d / f"{rec}_sync" / "events.json").read_text(encoding="utf-8"))
    t_of = {e["kind"]: e["video_t"] for e in ev if e["kind"] in ("段", "小段", "指定")}
    assert t_of["段"] == pytest.approx(1.0 - psy.FEED_LINE_LAG_S, abs=0.02)
    assert t_of["小段"] == pytest.approx(2.0 - psy.FEED_LINE_LAG_S, abs=0.02)
    assert t_of["指定"] == pytest.approx(4.2)
    marks = [e["video_t"] for e in ev if e["kind"] == "印"]
    assert sorted(marks) == pytest.approx([2.5, 3.0, 3.5], abs=0.02)  # 押した時と、その 0.5・1.0 秒前
    assert any("触れた" in e["text"] for e in ev)
    assert (d / f"{rec}_sync" / "sheet_1.jpg").exists() and (d / f"{rec}_sync" / "subs.srt").exists()
    assert "2b 構える" in (d / f"{rec}_sync" / "subs.srt").read_text(encoding="utf-8")


@needs_ffmpeg
def test_sync_with_npz_rows_uses_monotonic(tmp_path):
    start = 1.8e9
    d, rec_json, rec = _rec_dir(tmp_path, start)
    v0, d_mono = start + 0.01, 5.0e5
    _feed(d, d_mono, [(v0 + t, []) for t in np.arange(-30.0, 30.0, 0.5)])
    t = np.arange(v0 - 1.0, v0 + 4.0, 0.02) - d_mono           # 記録の t（PC2 の monotonic）
    n = len(t)
    phase = np.where(t + d_mono < v0 + 0.5, 2, 3)              # 0.5 秒で READY → DOOR
    script = np.full((n, 12), np.nan)
    script[t + d_mono >= v0 + 0.5, 0] = 0
    script[t + d_mono >= v0 + 1.7, 0] = 1                       # 1.7 秒で 2a → 2b
    script[:, 4:7] = [-0.3, 0.3, 0.6]
    script[:, 7] = 0.0
    meta = dict(args=dict(door="0.94,2.05,0.99,0.064"), phases=("WAIT", "ARMS_IN", "READY", "DOOR", "POST"))
    npz = d / "door_stock_x.npz"
    np.savez(npz, meta=np.array(repr(meta)), t=t, phase=phase, script=script, tau=np.zeros((n, 29)),
             arm_temp=np.full((n, 14), 60.0))
    assert psy.main([str(rec_json), "--npz", str(npz)]) == 0
    ev = json.loads((d / f"{rec}_sync" / "events.json").read_text(encoding="utf-8"))
    t_of = {e["kind"] + e["text"][:2]: e["video_t"] for e in ev}
    assert t_of["段RE"] == pytest.approx(0.5, abs=0.03)          # READY → DOOR
    assert t_of["小段2b"] == pytest.approx(1.7, abs=0.03)
    assert "手−当てる点" in next(e["detail"] for e in ev if e["text"].startswith("2b"))


@needs_ffmpeg
def test_set_calib_shifts_later_syncs(tmp_path):
    d, rec_json, rec = _rec_dir(tmp_path, 1.8e9)
    with contextlib.redirect_stdout(io.StringIO()):
        assert psy.main([str(rec_json), "--set-calib", "-120"]) == 0
    assert psy.read_calib(d) == -120.0
    _, _, clock, calib = psy.load_rec(rec_json, None)
    assert calib == -120.0 and clock.start_mac == pytest.approx(1.8e9 + 0.01 - 0.12, abs=1e-3)


# ── 撮影のページ ───────────────────────────────────────
def test_page_js_syntax_and_ids_exist():
    node = shutil.which("node")
    if node is None:
        pytest.skip("node が無い")
    r = subprocess.run([node, "--check", str(CAM / "web" / "app.js")], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    import re
    js = (CAM / "web" / "app.js").read_text(encoding="utf-8")
    html = (CAM / "web" / "index.html").read_text(encoding="utf-8")
    ids = set(re.findall(r"\$\('([A-Za-z]+)'\)", js))
    missing = [i for i in ids if f'id="{i}"' not in html]
    assert not missing, missing
