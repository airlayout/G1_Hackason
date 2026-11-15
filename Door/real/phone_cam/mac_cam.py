"""**Mac の上で**、連係カメラ（iPhone を Mac のカメラにする macOS の機能）で機体を録る。11-13 から撮影の本筋。

  bash Door/real/phone_cam/start_mac_cam.sh                    # ふだんはこれ（PC2 の走らせ役の a で自動で録る）
  bash Door/real/phone_cam/start_mac_cam.sh --test-record 10   # 10 秒だけ録って終わる（初めの試し・時計の合わせ）
  python mac_cam.py --list                                     # Mac から見えるカメラとマイク

iPhone の Safari で録って Mac へ送る形（phone_server.py・予備）は、11-13 に次でつまずいた: 証明書と https・Safari の録画が
2 本目から空・ルーターを通した送りの遅れと欠け・電話と Mac の時計の合わせ。連係カメラは Mac が直に録るので、どれも無い。

- iPhone は Mac の近く（Bluetooth が届く所）に三脚か挟み具で留める。Mac と同じ Apple ID・Wi-Fi と Bluetooth をオン・
  iPhone の「設定 → 一般 → AirPlay と連係 → 連係カメラ」をオン。録っている間、iPhone の画面は使えない
- 自動: 走らせ役が腕を借りたら（ARMS_IN・READY・DOOR）録り始め、止めて（POST）6 秒たつか次の a で 1 本にする（1 回の a に 1 本）
- 動画の 0 秒の Mac の時刻は ffmpeg の -use_wallclock_as_timestamps で取る（最初の画が Mac に届いた時）。
  写ってから届くまでの遅れは、1 日 1 回 Mac の画面の時計（open Door/real/phone_cam/web/clock.html）を写して
  phone_sync.py <cam_….json> --calib-frames → --set-calib で覚える（電話の録り方とは別に覚える）
- 印はこの端末のキー: 1 当たった・2 滑った・3 後ずさり・4 良い・m メモ・スペース 手で録る／止める・q 終わる

置き場は phone_server.py と同じ（Door/real/logs_real/phone/<日付>/）: cam_<日時>.mp4 と .json・marks.jsonl・feed_<日時>.jsonl。
回の後は phone_sync.py <cam_….json> [--npz …] で、段の移りと印の画を並べる（電話の時と同じ）。
"""
from __future__ import annotations

import argparse
import json
import queue
import re
import select
import shlex
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Deque, List, Optional, Sequence, Tuple

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import phone_server as ps  # noqa: E402  付帯情報の書き方と長さの測り方を同じにする

ARMED = ("ARMS_IN", "READY", "DOOR")
RECORDING_PHASES = ARMED + ("POST", "ARMS_OUT")
AUTO_STOP_AFTER_POST_S = 6.0
AUTO_STOP_STALE_S = 60.0     # 走らせ役の出力がこれだけ止まったら（落ちた）自動の録画を閉じる
START_FRESH_S = 10.0         # 状態がこれより古ければ録り始めない（つなぎ直した時の前の日の行で録り始めない）
MAX_AUTO_S = 600.0
TICK_S = 0.25
STOP_WAIT_S = 15.0
FIRST_FRAME_WAIT_S = 15.0    # 連係カメラは ffmpeg を起こしてから最初の画まで 4 秒かかった（11-13）
MARK_KEYS = {"1": "当たった", "2": "滑った", "3": "後ずさり", "4": "良い"}
AVOID = ("デスクビュー", "Desk View")
RE_DEV = re.compile(r"\[(\d+)\] (.+)$")
RE_START = re.compile(r"start: (\d+(?:\.\d+)?)")
PERMISSION_HINT = ("カメラの許可が無いかもしれない →「システム設定 → プライバシーとセキュリティ → カメラ」で、"
                   "この端末（ターミナル・VS Code）をオン")
NOT_FOUND_HINT = ("iPhone のカメラが見えない → iPhone を Mac の近くに置く・同じ Apple ID・Wi-Fi と Bluetooth をオン・"
                  "iPhone の「設定 → 一般 → AirPlay と連係 → 連係カメラ」をオン・Safari の撮影のページを閉じる")


# ── カメラを探す ──
def parse_devices(text: str) -> Tuple[List[Tuple[int, str]], List[Tuple[int, str]]]:
    """ffmpeg -f avfoundation -list_devices true の出力 → (映像, 音)"""
    video: List[Tuple[int, str]] = []
    audio: List[Tuple[int, str]] = []
    cur: Optional[List[Tuple[int, str]]] = None
    for line in text.splitlines():
        if "video devices" in line:
            cur = video
            continue
        if "audio devices" in line:
            cur = audio
            continue
        m = RE_DEV.search(line)
        if m and cur is not None:
            cur.append((int(m.group(1)), m.group(2).strip().strip("‎")))  # 名前の頭に左から右の印が付いている
    return video, audio


def list_devices(ffmpeg: str = "ffmpeg") -> Tuple[List[Tuple[int, str]], List[Tuple[int, str]]]:
    r = subprocess.run([ffmpeg, "-hide_banner", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
                       capture_output=True, text=True, timeout=30)
    return parse_devices(r.stderr)


def pick(devs: Sequence[Tuple[int, str]], want: str) -> Optional[Tuple[int, str]]:
    """名前に want を含む最初の物（デスクビューは除く）"""
    for i, name in devs:
        if want.lower() in name.lower() and not any(a in name for a in AVOID):
            return i, name
    return None


def ffmpeg_cmd(ffmpeg: str, out: Path, video: Optional[int] = None, audio: Optional[int] = None, fps: float = 30.0,
               size: str = "1920x1080", bitrate: str = "6M", vcodec: str = "h264_videotoolbox",
               input_args: Optional[Sequence[str]] = None) -> List[str]:
    """録る ffmpeg。動画の 0 秒の Mac の時刻を「start:」に出させる（-use_wallclock_as_timestamps）。
    出す mp4 は断片に分ける（途中で落ちても、それまでの分は見られる）"""
    src = list(input_args) if input_args else (
        ["-f", "avfoundation", "-framerate", f"{fps:g}"] + (["-video_size", size] if size else [])
        + ["-i", f"{video}:{'none' if audio is None else audio}"])
    return ([ffmpeg, "-hide_banner", "-y", "-thread_queue_size", "1024", "-use_wallclock_as_timestamps", "1"] + src
            + ["-c:v", vcodec, "-b:v", bitrate, "-g", f"{int(2 * fps)}", "-c:a", "aac", "-b:a", "128k",
               "-movflags", "+frag_keyframe+empty_moov+default_base_moof", str(out)])


# ── 自動で録る・止める ──
@dataclass
class RecInfo:
    auto: bool
    cycle: Optional[int]
    started: float


@dataclass
class AutoPolicy:
    """撮影のページ（app.js の autoTick）と同じ決め方。1 回の a（arms の数）に 1 本"""
    recorded_cycle: Optional[int] = None
    post_since: Optional[float] = None

    def tick(self, st: dict, now: float, rec: Optional[RecInfo]) -> Optional[str]:
        phase = st.get("phase") or ""
        if phase == "POST":
            if self.post_since is None:
                self.post_since = now
        else:
            self.post_since = None
        age = st.get("age_s")
        if rec is not None:
            if not rec.auto:
                return None
            after_post = self.post_since is not None and now - self.post_since > AUTO_STOP_AFTER_POST_S
            stale = age is not None and age > AUTO_STOP_STALE_S
            if after_post or phase not in RECORDING_PHASES or stale or st.get("arms") != rec.cycle \
                    or now - rec.started > MAX_AUTO_S:
                return "stop"
            return None
        if not st.get("connected"):
            return None
        if self.recorded_cycle is None:  # 起こした時に腕を借りていれば、その回から録る（-1 はどの回とも違う）
            self.recorded_cycle = -1 if phase in ARMED else st.get("arms")
        fresh = age is not None and age <= START_FRESH_S
        if phase in ARMED and fresh and st.get("arms") != self.recorded_cycle:
            self.recorded_cycle = st.get("arms")
            return "start"
        return None


# ── 1 本の録画 ──
class Recorder:
    def __init__(self, data: Path, rec_id: str, cmd: List[str], meta: dict, ffmpeg: str = "ffmpeg",
                 clock: Callable[[], float] = time.time) -> None:
        self.data, self.rec_id, self.cmd, self.meta, self.ffmpeg, self.clock = data, rec_id, cmd, meta, ffmpeg, clock
        self.video = data / f"{rec_id}.mp4"
        self.start_mac: Optional[float] = None
        self.launched: Optional[float] = None
        self.tail: Deque[str] = deque(maxlen=40)
        self.proc: Optional[subprocess.Popen] = None
        self.reader: Optional[threading.Thread] = None

    def start(self) -> None:
        self.launched = self.clock()
        ps.write_meta(self.data, self.rec_id, dict(self.meta, status="録画中", launched_mac=self.launched))
        self.proc = subprocess.Popen(self.cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                     text=True, encoding="utf-8", errors="replace", bufsize=1)
        self.reader = threading.Thread(target=self._read, daemon=True, name=f"ffmpeg_{self.rec_id}")
        self.reader.start()

    def _read(self) -> None:
        assert self.proc is not None and self.proc.stderr is not None
        for line in self.proc.stderr:  # 進み具合の行は \r で区切られるが、文字の読み方で行に分かれる
            line = line.strip()
            if self.start_mac is None:
                m = RE_START.search(line)
                if m:
                    self.start_mac = float(m.group(1))
            if line and not line.startswith(("frame=", "size=")):
                self.tail.append(line)

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def stop(self) -> None:
        """q で閉じさせる（mp4 の終わりを書かせる）。返らなければ SIGINT・最後は kill"""
        p = self.proc
        if p is None or p.poll() is not None:
            return
        try:
            assert p.stdin is not None
            p.stdin.write("q")
            p.stdin.flush()
        except (BrokenPipeError, OSError, ValueError):
            pass
        try:
            p.wait(STOP_WAIT_S)
        except subprocess.TimeoutExpired:
            p.send_signal(signal.SIGINT)
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()

    def finish(self, state_stop: dict, why: str) -> dict:
        self.stop()
        if self.reader is not None:
            self.reader.join(3)
        size = self.video.stat().st_size if self.video.exists() else 0
        dur = ps.probe_duration(self.video, self.ffmpeg) if size > 0 else None
        ok = dur is not None and dur > 0
        tail = list(self.tail)
        meta = dict(self.meta, status="済み" if ok else "録れない", video=self.video.name if ok else None, duration=dur,
                    bytes=size, start_mac=self.start_mac, launched_mac=self.launched, stopped_mac=self.clock(),
                    stop_why=why, state_stop=state_stop, ffmpeg_rc=None if self.proc is None else self.proc.returncode,
                    ffmpeg_tail=None if ok else tail[-10:])
        if ok and self.start_mac is None:  # 「start:」が読めなかった: 起こした時刻で仮に置く（数百 ms 早い）
            meta.update(start_mac=self.launched, start_mac_estimated=True)
        ps.write_meta(self.data, self.rec_id, meta)
        return meta


def failure_hint(tail: Sequence[str]) -> str:
    text = "\n".join(tail)
    if re.search(r"not authorized|permission|Failed to create AV capture input device|Unable to open", text, re.I):
        return PERMISSION_HINT
    if re.search(r"not supported by the device|Selected (framerate|video size)", text, re.I):
        return "その解像度か fps に iPhone が応えない → --size と --fps を替える（--list で名前だけは見える）"
    if re.search(r"Input/output error|No such|Invalid device", text, re.I):
        return NOT_FOUND_HINT
    return "（下の ffmpeg の行を Claude に見せる）"


# ── 全体 ──
class MacCam:
    def __init__(self, data: Path, feed=None, video: Optional[Tuple[int, str]] = None,
                 audio: Optional[Tuple[int, str]] = None, fps: float = 30.0, size: str = "1920x1080",
                 bitrate: str = "6M", vcodec: str = "h264_videotoolbox", ffmpeg: str = "ffmpeg", auto: bool = True,
                 input_args: Optional[Sequence[str]] = None, clock: Callable[[], float] = time.time,
                 out: Callable[[str], None] = lambda s: print(s, flush=True), want_video: str = "",
                 want_audio: str = "", lister: Callable[[str], Tuple[list, list]] = list_devices) -> None:
        self.data, self.feed, self.video_dev, self.audio_dev = data, feed, video, audio
        self.want_video, self.want_audio, self.lister = want_video, want_audio, lister  # 録るたびに名前で探し直す
        self.fps, self.size, self.bitrate, self.vcodec, self.ffmpeg = fps, size, bitrate, vcodec, ffmpeg
        self.auto, self.input_args, self.clock, self.out = auto, input_args, clock, out
        self.policy = AutoPolicy()
        self.rec: Optional[Recorder] = None
        self.info: Optional[RecInfo] = None
        self.cmds: "queue.Queue[Tuple[str, str, str, float]]" = queue.Queue()
        self.last_shown: Tuple = ()
        data.mkdir(parents=True, exist_ok=True)

    def state(self) -> dict:
        return self.feed.snapshot() if self.feed is not None else {}

    def _new_id(self) -> str:
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(self.clock()))
        rec_id, n = f"cam_{stamp}", 2
        while (self.data / f"{rec_id}.json").exists():
            rec_id, n = f"cam_{stamp}_{n}", n + 1
        return rec_id

    def _find_devices(self) -> bool:
        """番号は機器の抜き差しで変わる（11-13: Jabra をつなぐと iPhone のマイクが 1 → 0）。録り始めるたびに名前で探す"""
        if self.input_args or not self.want_video:
            return True
        try:
            vids, auds = self.lister(self.ffmpeg)
        except (OSError, subprocess.SubprocessError) as e:
            self.out(f"[cam] ⚠️ カメラの一覧を取れない（{e}）。前の番号で録る")
            return self.video_dev is not None
        video = pick(vids, self.want_video)
        if video is None:
            self.out(f"[cam] ⚠️ 録り始められない: {NOT_FOUND_HINT}（直したらスペースで録る）")
            return False
        self.video_dev = video
        self.audio_dev = pick(auds, self.want_audio) if self.want_audio else None
        return True

    def start_rec(self, auto: bool, cycle: Optional[int]) -> bool:
        if not self._find_devices():
            return False
        rec_id = self._new_id()
        out = self.data / f"{rec_id}.mp4"
        cmd = ffmpeg_cmd(self.ffmpeg, out, None if self.video_dev is None else self.video_dev[0],
                         None if self.audio_dev is None else self.audio_dev[0], self.fps, self.size, self.bitrate,
                         self.vcodec, self.input_args)
        meta = dict(rec=rec_id, source="continuity", device=None if self.video_dev is None else self.video_dev[1],
                    audio_device=None if self.audio_dev is None else self.audio_dev[1], auto=auto, cycle=cycle,
                    fps=self.fps, size=self.size, bitrate=self.bitrate, cmd=shlex.join(cmd), state_start=self.state())
        rec = Recorder(self.data, rec_id, cmd, meta, self.ffmpeg, self.clock)
        rec.start()
        self.rec, self.info = rec, RecInfo(auto, cycle, self.clock())
        self.out(f"[cam] 録り始めた: {rec_id}（{'自動' if auto else '手で'}・{meta['device'] or '試しの入力'}"
                 f"・{meta['audio_device'] or '音なし'}）")
        return True

    def stop_rec(self, why: str) -> Optional[dict]:
        rec, self.rec, self.info = self.rec, None, None
        if rec is None:
            return None
        meta = rec.finish(self.state(), why)
        if meta["status"] == "済み":
            at = "（0 秒の時刻は仮）" if meta.get("start_mac_estimated") else ""
            self.out(f"[cam] 録画を閉じた: {rec.video.name}（{meta['duration']:.1f} 秒・{meta['bytes'] / 1e6:.1f} MB・{why}）{at}")
        else:
            tail = meta.get("ffmpeg_tail") or []
            self.out(f"[cam] ⚠️ {rec.rec_id} を録れなかった（{why}）: {failure_hint(tail)}")
            for line in tail[-4:]:
                self.out(f"[cam]   ffmpeg: {line}")
        return meta

    def mark(self, label: str, memo: str, t: float) -> dict:
        st = self.state()
        row = dict(rec=None if self.rec is None else self.rec.rec_id, label=label, memo=memo[:2000], mac=t,
                   received=self.clock(), source="mac_key", phase=st.get("phase"), sub=st.get("sub"),
                   door_deg=st.get("door_deg"), tries=st.get("tries"))
        with open(self.data / "marks.jsonl", "a", encoding="utf-8") as f:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")
        self.out(f"[cam] 印: {label}" + (f"「{memo}」" if memo else "") + f"（{row['phase'] or '—'} {row['sub'] or ''}・"
                 f"{row['rec'] or '録画の外'}）")
        return row

    def step(self) -> bool:
        """1 周（0.25 秒おき）。q で False"""
        now = self.clock()
        while True:
            try:
                cmd, label, memo, t = self.cmds.get_nowait()
            except queue.Empty:
                break
            if cmd == "quit":
                return False
            if cmd == "mark":
                self.mark(label, memo, t)
            elif cmd == "toggle":
                if self.rec is None:
                    self.start_rec(False, self.state().get("arms"))
                else:
                    self.stop_rec("手で止めた")
        if self.rec is not None and not self.rec.alive():
            self.stop_rec("ffmpeg が自分で終わった（iPhone が離れた・カメラを取られた・許可が無い？）")
        st = self.state()
        if self.auto:
            act = self.policy.tick(st, now, self.info if self.rec is not None else None)
            if act == "start":
                self.start_rec(True, st.get("arms"))
            elif act == "stop":
                self.stop_rec("自動")
        self._show(st)
        return True

    def _show(self, st: dict) -> None:
        """段・小段・PC2 とのつながりが変わった時だけ 1 行出す"""
        key = (st.get("connected"), st.get("phase"), st.get("sub"))
        if not st or key == self.last_shown:
            return
        self.last_shown = key
        if not st.get("connected"):
            self.out(f"[cam] PC2 ✕ {st.get('error') or ''}")
            return
        door = st.get("door_deg")
        temp = st.get("temp_c")
        self.out(f"[cam] 機体: {st.get('phase') or '—'} {st.get('sub_label') or ''}"
                 + ("" if door is None else f"・扉 {door:.1f}°") + ("" if temp is None else f"・腕 {temp:.0f} °C")
                 + (f"・録画中 {self.rec.rec_id}" if self.rec is not None else ""))

    def close(self) -> None:
        if self.rec is not None:
            self.stop_rec("終わる")
        if self.feed is not None:
            self.feed.stop()


def read_keys(cmds: "queue.Queue", stop: threading.Event) -> None:
    """この端末のキーを 1 文字ずつ読む（Enter 無し）。端末でなければ何もしない"""
    if not sys.stdin.isatty():
        return
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not stop.is_set():
            r, _, _ = select.select([sys.stdin], [], [], 0.2)
            if not r:
                continue
            ch, t = sys.stdin.read(1), time.time()
            if ch in MARK_KEYS:
                cmds.put(("mark", MARK_KEYS[ch], "", t))
            elif ch == "m":
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                try:
                    memo = input("[cam] メモ（Enter で残す・空で取り消し）: ").strip()
                finally:
                    tty.setcbreak(fd)
                if memo:
                    cmds.put(("mark", "メモ", memo, t))
            elif ch == " ":
                cmds.put(("toggle", "", "", t))
            elif ch in ("q", "Q"):
                cmds.put(("quit", "", "", t))
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(argv=None) -> int:  # noqa: C901
    ap = argparse.ArgumentParser(description="連係カメラで機体を録る（Mac）")
    ap.add_argument("--data", default=str(HERE.parent / "logs_real" / "phone" / time.strftime("%Y%m%d")))
    ap.add_argument("--video", default="iPhone", help="カメラの名前に含む文字（既定 iPhone・デスクビューは除く）")
    ap.add_argument("--audio", default="iPhone", help="マイクの名前に含む文字（none で音なし）")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--size", default="1920x1080", help="空にすると iPhone の既定")
    ap.add_argument("--bitrate", default="6M")
    ap.add_argument("--vcodec", default="h264_videotoolbox")
    ap.add_argument("--no-feed", action="store_true", help="PC2 を見ない（自動で録らない。スペースで録る）")
    ap.add_argument("--manual", action="store_true", help="自動で録らない")
    ap.add_argument("--test-record", type=float, default=0.0, help="この秒数だけ手で録って終わる")
    ap.add_argument("--list", action="store_true", help="見えるカメラとマイクを出して終わる")
    ap.add_argument("--input-args", default="", help="試験用（avfoundation の代わりの ffmpeg の入力）")
    a = ap.parse_args(argv)
    say = lambda s: print(s, flush=True)  # noqa: E731
    vids, auds = ([], []) if a.input_args else list_devices()
    if a.list:
        say("[cam] カメラ: " + "・".join(f"[{i}] {n}" for i, n in vids))
        say("[cam] マイク: " + "・".join(f"[{i}] {n}" for i, n in auds))
        return 0
    video = audio = None
    if not a.input_args:
        video = pick(vids, a.video)
        if video is None:
            say(f"[cam] ⚠️ {NOT_FOUND_HINT}")
            say("[cam] 今見えるカメラ: " + ("・".join(n for _, n in vids) or "無し"))
            return 2
        audio = None if a.audio.lower() == "none" else pick(auds, a.audio)
    data = Path(a.data)
    feed = None
    if not (a.no_feed or a.test_record):
        import robot_feed as rf
        feed = rf.RobotFeed(data)
        feed.start()
    cam = MacCam(data, feed, video, audio, a.fps, a.size, a.bitrate, a.vcodec, auto=not (a.manual or a.no_feed),
                 input_args=shlex.split(a.input_args) if a.input_args else None, want_video=a.video,
                 want_audio="" if a.audio.lower() == "none" else a.audio)
    say(f"[cam] カメラ: {video[1] if video else a.input_args}・マイク: {audio[1] if audio else 'なし'}・置き場: {data}")
    stop = threading.Event()
    keys = threading.Thread(target=read_keys, args=(cam.cmds, stop), daemon=True)
    keys.start()
    if a.test_record:
        say(f"[cam] 試しに {a.test_record:g} 秒録る（時計の合わせなら、先に open {HERE / 'web' / 'clock.html'} で"
            " Mac の画面に時計を出しておき、「最初の画が届いた」と出てから 3 秒以上それを写す。"
            "連係カメラはつながるまで 4 秒ほど写らない）")
    elif cam.auto:
        say("[cam] 自動で待つ（走らせ役の a で録り始める）。キー: 1 当たった・2 滑った・3 後ずさり・4 良い・m メモ・"
            "スペース 手で録る／止める・q 終わる")
    else:
        say("[cam] 手で録る。キー: スペース 録る／止める・1〜4 印・m メモ・q 終わる")
    try:
        if a.test_record:
            if not cam.start_rec(False, None):
                return 2
            # 秒数は最初の画が届いてから数える（11-13: 連係カメラはつながるまで 4 秒かかり、10 秒の試しが 6.6 秒になった）
            t_first = time.time() + FIRST_FRAME_WAIT_S
            while cam.rec is not None and cam.rec.alive() and cam.rec.start_mac is None and time.time() < t_first:
                time.sleep(0.05)
            if cam.rec is not None and cam.rec.start_mac is not None:
                say(f"[cam] 最初の画が届いた（起こしてから {cam.rec.start_mac - (cam.rec.launched or cam.rec.start_mac):.1f} 秒）"
                    f"。ここから {a.test_record:g} 秒録る ← 時計はここから写す")
            t_end = time.time() + a.test_record
            while time.time() < t_end and cam.rec is not None and cam.rec.alive():
                time.sleep(TICK_S)
            meta = cam.stop_rec("試し")
            if meta and meta["status"] == "済み":
                say(f"[cam] 次: python {HERE / 'phone_sync.py'} {data / (meta['rec'] + '.json')} --calib-frames")
            return 0 if meta and meta["status"] == "済み" else 1
        while cam.step():
            time.sleep(TICK_S)
    except KeyboardInterrupt:
        say("[cam] 止めた")
    finally:
        stop.set()
        keys.join(1)
        cam.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
