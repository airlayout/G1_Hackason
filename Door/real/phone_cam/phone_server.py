"""**Mac の上で**動かす。iPhone の Safari で開く撮影のページを配り、録った動画・印・メモを受け取り、PC2 の今の状態を返す。

  bash Door/real/phone_cam/start_phone_cam.sh       # ふだんはこれ（証明書を確かめ、URL の QR を Mac の画面に出して起こす）
  python phone_server.py [--data DIR] [--port 8443] [--http-port 8080] [--no-feed] [--no-tls] [--qr PNG]

https（既定 8443）: 撮影のページ（/?k=鍵）と API。iPhone の Safari はカメラを https でしか使わせない。
http（既定 8080）: 証明書の配布（/ca.crt）と Mac の画面の時計（/clock）だけ。API は無い。
API は鍵（URL の ?k=… を撮影のページが X-Key で送る）が合わなければ 403。鍵と証明書は ~/.config/g1_phone_cam/（git の外）。

置き場（--data。既定 Door/real/logs_real/phone/<日付>/）:
  phone_<日時>.mp4 ／ .json（電話の時計・往復・始めと終わりの機体の状態）／ .raw（受けた塊をつないだ物。mp4 に直せたら消す）
  marks.jsonl（印とメモ）・feed_<日時>.jsonl（PC2 の出力。robot_feed.py）
"""
from __future__ import annotations

import argparse
import json
import re
import secrets
import shutil
import socket
import ssl
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, Optional, Tuple
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import access_log as al  # noqa: E402
import timebase as tb  # noqa: E402

CONF = Path.home() / ".config" / "g1_phone_cam"
WEB = HERE / "web"
REC_ID_RE = re.compile(r"^phone_\d{8}_\d{6}(_\d{1,3})?$")
MAX_CHUNK = 32 << 20        # 1 秒の塊は 6 Mbps で 0.75 MB。これを超えるのは壊れた要求
MAX_JSON = 1 << 20
MAX_REC_BYTES = 4 << 30     # 1 本 4 GB（6 Mbps で 1.5 時間）で打ち切る
KEEP_RAW_BELOW = 0.8        # mp4 に入れ直した物が元のこの割合より小さければ、.raw を消さない
STATIC = {"/": "index.html", "/index.html": "index.html", "/app.js": "app.js", "/style.css": "style.css",
          "/clock": "clock.html", "/clock.html": "clock.html"}
HTTP_ONLY = {"/clock", "/clock.html", "/ca.crt", "/"}
TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8", ".css": "text/css; charset=utf-8"}


class ShortBody(ValueError):
    """Content-Length より短い本文（送る途中で相手が切った）"""


@dataclass
class Recording:
    """受けている途中の 1 本（受け口の中だけで書き替える）"""
    rec_id: str
    raw: Path
    started: float
    state_start: dict
    next_seq: int = 0
    size: int = 0
    status: str = "受信中"
    meta: dict = field(default_factory=dict)
    lock: threading.Lock = field(default_factory=threading.Lock)  # 塊を書く間だけ（受け口全体の鍵は持たない）
    stop_resp: Optional[dict] = None                              # 閉じた時の答え（送り直しの stop に同じ答えを返す）


class App:
    def __init__(self, data_dir: Path, token: str, feed=None, ffmpeg: str = "ffmpeg", clock=time.time) -> None:
        self.data = data_dir
        self.data.mkdir(parents=True, exist_ok=True)
        self.token, self.feed, self.ffmpeg, self.clock = token, feed, ffmpeg, clock
        self.recs: Dict[str, Recording] = {}
        self.lock = threading.Lock()

    # ── 状態 ──
    def state(self) -> dict:
        if self.feed is None:
            return {"connected": False, "error": "PC2 を見ていない（--no-feed）", "phase": ""}
        return self.feed.snapshot()

    def calib_ms(self) -> Optional[float]:
        f = self.data.parent / "calib.json"
        try:
            return float(json.loads(f.read_text(encoding="utf-8"))["calib_ms"])
        except (OSError, ValueError, KeyError):
            return None

    # ── 録画 ──
    def start_rec(self, body: dict) -> dict:
        with self.lock:
            stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(self.clock()))
            rec_id, n = f"phone_{stamp}", 2
            while rec_id in self.recs or (self.data / f"{rec_id}.json").exists():
                rec_id, n = f"phone_{stamp}_{n}", n + 1
            rec = Recording(rec_id, self.data / f"{rec_id}.raw", self.clock(), self.state())
            rec.meta = {k: body.get(k) for k in ("ua", "mime", "width", "height", "fps", "bitrate", "auto")}
            rec.raw.write_bytes(b"")
            self.recs[rec_id] = rec
        # 仮の付帯情報を先に書く（電話が落ちて stop が来なくても、起こし直した時に .raw を救える: salvage）
        self._write_meta(rec_id, dict(rec.meta, rec=rec_id, status="受信中", pings=body.get("pings") or [],
                                      call_phone_ms=body.get("call_phone_ms"), start_phone_ms=body.get("call_phone_ms"),
                                      state_start=rec.state_start, received_start=rec.started))
        print(f"[phone] 録画を始めた: {rec_id}（{rec.meta.get('mime')}・{rec.meta.get('width')}×{rec.meta.get('height')}"
              f"・{'自動' if rec.meta.get('auto') else '手で'}）", flush=True)
        return {"rec": rec_id}

    def chunk(self, rec_id: str, seq: int, data: bytes) -> Tuple[int, dict]:
        with self.lock:
            rec = self.recs.get(rec_id)
        if rec is None:
            return 404, {"error": f"受けていない録画: {rec_id}"}
        with rec.lock:
            if rec.status != "受信中":
                return 404, {"error": f"閉じた録画: {rec_id}"}
            if seq < rec.next_seq:
                return 200, {"dup": True, "next": rec.next_seq}  # 送り直し（もう受けた）
            if seq > rec.next_seq:
                return 409, {"error": "塊が飛んだ", "next": rec.next_seq}
            if rec.size + len(data) > MAX_REC_BYTES:
                return 413, {"error": "1 本の上限を超えた"}
            try:  # 前の書きかけ（ディスクが一杯で途中まで書けた等）は切り捨ててから書く
                with open(rec.raw, "r+b") as f:
                    f.seek(rec.size)
                    f.write(data)
                    f.truncate()
            except OSError as e:
                return 507, {"error": f"Mac に書けない: {e}"}
            rec.next_seq += 1
            rec.size += len(data)
            return 200, {"next": rec.next_seq, "bytes": rec.size}

    def stop_rec(self, body: dict) -> Tuple[int, dict]:
        rec_id = str(body.get("rec", ""))
        chunks = int(body.get("chunks", -1))
        with self.lock:
            rec = self.recs.get(rec_id)
        if rec is None:
            return 404, {"error": f"受けていない録画: {rec_id}"}
        with rec.lock:
            if rec.status != "受信中":  # 答えが途中で失われて送り直した stop: 同じ答え
                if rec.stop_resp is not None and chunks == rec.next_seq:
                    return 200, dict(rec.stop_resp, again=True)
                return 404, {"error": f"閉じた録画: {rec_id}"}
            if chunks != rec.next_seq:
                return 409, {"error": "まだ届いていない塊がある", "next": rec.next_seq}
            rec.status = "仕上げ中"
        try:
            pings = tb.pings_from_ms(body.get("pings") or [])
            best = tb.best_ping_near(pings, float(body["start_phone_ms"]) / 1000.0)
            start_mac = float(body["start_phone_ms"]) / 1000.0 + best.offset
        except (ValueError, KeyError, TypeError) as e:
            pings, best, start_mac = [], None, None
            print(f"[phone] ⚠️ {rec_id}: 時計を合わせられない（{e}）。動画は残す", flush=True)
        meta = dict(rec.meta, rec=rec_id, status="仕上げ中", chunks=rec.next_seq, bytes=rec.size,
                    start_phone_ms=body.get("start_phone_ms"), call_phone_ms=body.get("call_phone_ms"),
                    stop_phone_ms=body.get("stop_phone_ms"), pings=body.get("pings") or [], broken=body.get("broken"),
                    offset_ms=None if best is None else 1000.0 * best.offset, rtt_ms=None if best is None else 1000.0 * best.rtt,
                    start_mac=start_mac, calib_ms=self.calib_ms(), state_start=rec.state_start, state_stop=self.state(),
                    received_start=rec.started, received_stop=self.clock(), diag=body.get("diag"))
        rec.stop_resp = {"rec": rec_id, "bytes": rec.size, "rtt_ms": meta["rtt_ms"]}
        print(f"[phone] {rec_id} を閉じた: {diag_text(meta['diag'])}", flush=True)
        if rec.size == 0:  # 11-13: 電話の録画が塊を 1 つも出さなかった。mp4 に直す物が無い
            meta.update(status="空", video=None, duration=None)
            self._write_meta(rec_id, meta)
            rec.raw.unlink(missing_ok=True)
            rec.status = "空"
            print(f"[phone] ⚠️ {rec_id}: 電話の録画が塊を 1 つも出さなかった（空）。ページを開き直してカメラを開く", flush=True)
            return 200, rec.stop_resp
        self._write_meta(rec_id, meta)
        threading.Thread(target=self.finalize, args=(rec_id,), daemon=True).start()
        return 200, rec.stop_resp

    def _write_meta(self, rec_id: str, meta: dict) -> None:
        write_meta(self.data, rec_id, meta)

    def finalize(self, rec_id: str) -> None:
        rec = self.recs[rec_id]
        try:
            status = remux_and_note(self.data, rec_id, rec.raw, str(rec.meta.get("mime") or ""), self.ffmpeg)
        except Exception as e:  # noqa: BLE001  仕上げの糸で落ちても、元の .raw は残す
            status = "直せない"
            print(f"[phone] ⚠️ {rec_id}: 仕上げで誤り（元の {rec.raw.name} を残す）: {type(e).__name__}: {e}", flush=True)
        rec.status = status

    def salvage(self) -> int:
        """前に起こした時に閉じられなかった録画（.raw と仮の付帯情報）を mp4 に直す。起こした時に 1 度だけ"""
        n = 0
        for raw in sorted(self.data.glob("phone_*.raw")):
            rec_id = raw.stem
            if rec_id in self.recs or not REC_ID_RE.match(rec_id) or not (self.data / f"{rec_id}.json").exists():
                continue
            meta = json.loads((self.data / f"{rec_id}.json").read_text(encoding="utf-8"))
            meta.update(salvaged=True, bytes=raw.stat().st_size)
            if meta["bytes"] == 0:  # 塊が 1 つも届かなかった録画は、直す物が無い
                meta.update(status="空", video=None, duration=None)
                self._write_meta(rec_id, meta)
                raw.unlink(missing_ok=True)
                print(f"[phone] 閉じられなかった録画は空だった: {rec_id}", flush=True)
                n += 1
                continue
            if meta.get("offset_ms") is None and meta.get("pings") and meta.get("start_phone_ms") is not None:
                try:  # 始めの往復と、録り始めを頼んだ時刻で仮に合わせる（本当の録り始めより数百 ms 早い）
                    best = tb.best_ping(tb.pings_from_ms(meta["pings"]))
                    meta.update(offset_ms=1000.0 * best.offset, rtt_ms=1000.0 * best.rtt,
                                start_mac=float(meta["start_phone_ms"]) / 1000.0 + best.offset)
                except (ValueError, TypeError):
                    pass
            self._write_meta(rec_id, meta)
            st = remux_and_note(self.data, rec_id, raw, str(meta.get("mime") or ""), self.ffmpeg)
            print(f"[phone] 閉じられなかった録画を救った: {rec_id}（{st}）", flush=True)
            n += 1
        return n

    # ── 印・メモ ──
    def mark(self, body: dict) -> dict:
        st = self.state()
        phone_ms = float(body["phone_ms"])
        off = body.get("offset_ms")
        row = dict(rec=body.get("rec"), label=str(body.get("label", ""))[:40], memo=str(body.get("memo", ""))[:2000],
                   phone_ms=phone_ms, offset_ms=off, rtt_ms=body.get("rtt_ms"),
                   mac=None if off is None else (phone_ms + float(off)) / 1000.0, received=self.clock(),
                   phase=st.get("phase"), sub=st.get("sub"), door_deg=st.get("door_deg"), tries=st.get("tries"))
        with self.lock, open(self.data / "marks.jsonl", "a", encoding="utf-8") as f:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")
        print(f"[phone] 印: {row['label']}" + (f"「{row['memo']}」" if row["memo"] else "")
              + f"（{row['phase']} {row['sub'] or ''}・{row['rec'] or '録画の外'}）", flush=True)
        return {"ok": True, "mac": row["mac"]}

    def list_recs(self) -> list:
        with self.lock:
            recs = sorted(self.recs.values(), key=lambda r: r.started)[-8:]
            return [dict(rec=r.rec_id, status=r.status, bytes=r.size, chunks=r.next_seq) for r in recs]


def diag_text(diag) -> str:
    """電話が閉じる時に送った中の様子を 1 行に（電話から来た値なので、形が違っても落ちない）"""
    if not isinstance(diag, dict):
        return "電話の中の様子なし（古いページ。読み込み直す）"
    g = lambda k, d=None: diag.get(k, d)  # noqa: E731
    try:
        first = g("firstDataMs")
        s = (f"出た塊 {int(g('events', 0))}（空 {int(g('empty', 0))}）・{float(g('bytes', 0)) / 1e6:.1f} MB・"
             f"最初の塊 {'無し' if first is None else f'{float(first) / 1000:.1f} 秒'}")
    except (TypeError, ValueError):
        s = "出た塊 ?"
    if g("hidden"):
        s += f"・裏に回った {g('hidden')} 回"
    if g("errors"):
        s += f"・誤り {g('errors')}"
    if g("noDataStop"):
        s += "・塊が出ず見張りが止めた"
    if g("retried"):
        s += "・撮り直しの 1 本"
    s += f"・カメラ {g('trackStart', '?')} → {g('trackStop', '?')}・{g('recorderMime', '')}"
    return s[:400]


def write_meta(data: Path, rec_id: str, meta: dict) -> None:
    tmp = data / f"{rec_id}.json.tmp"
    tmp.write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8")
    tmp.replace(data / f"{rec_id}.json")


def remux_and_note(data: Path, rec_id: str, raw: Path, mime: str, ffmpeg: str = "ffmpeg") -> str:
    """受けた塊をつないだ物を、そのまま（画を作り直さずに）mp4 に入れ直し、付帯情報に動画の名前と長さを書く。
    付帯情報を書いてから .raw を消す（途中で落ちても動画を失わない）。返すのは状態（済み／直せない）"""
    out = data / f"{rec_id}{'.webm' if 'webm' in mime else '.mp4'}"
    cmd = [ffmpeg, "-y", "-v", "error", "-i", str(raw), "-c", "copy"]
    cmd += (["-movflags", "+faststart"] if out.suffix == ".mp4" else []) + [str(out)]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
        ok = r.returncode == 0 and out.exists() and out.stat().st_size > 0
        why = r.stderr.strip()[-300:]
        # 入れ直しは大きさをほぼ変えない。大きく縮んだら途中の壊れた所で読むのをやめた（11-13: 11.5 MB → 1.5 MB で .raw を消した）
        if ok and out.stat().st_size < KEEP_RAW_BELOW * raw.stat().st_size:
            ok = False
            why = (f"入れ直した物が元の {out.stat().st_size / raw.stat().st_size:.0%}（{out.stat().st_size / 1e6:.1f}"
                   f"/{raw.stat().st_size / 1e6:.1f} MB）しかない。途中が壊れている。{out.name} は頭だけ")
    except (OSError, subprocess.TimeoutExpired) as e:
        ok, why = False, str(e)
    meta = json.loads((data / f"{rec_id}.json").read_text(encoding="utf-8"))
    if not ok:
        meta.update(video=raw.name, duration=None, remux_error=why, status="直せない")
        write_meta(data, rec_id, meta)
        print(f"[phone] ⚠️ {rec_id}: mp4 に直せない（元の {raw.name} を残す）: {why}", flush=True)
        return "直せない"
    dur = probe_duration(out, ffmpeg)
    meta.update(video=out.name, duration=dur, status="済み")
    write_meta(data, rec_id, meta)
    raw.unlink(missing_ok=True)
    nan = float("nan")
    rtt = meta.get("rtt_ms")
    print(f"[phone] 録画を受け取った: {out.name}（{(meta.get('bytes') or 0) / 1e6:.1f} MB・{nan if dur is None else dur:.1f} 秒・"
          f"時計の往復 {nan if rtt is None else rtt:.0f} ms）", flush=True)
    return "済み"


def probe_duration(path: Path, ffmpeg: str = "ffmpeg") -> Optional[float]:
    probe = shutil.which("ffprobe") or str(Path(ffmpeg).with_name("ffprobe"))
    try:
        r = subprocess.run([probe, "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", str(path)],
                           capture_output=True, text=True, timeout=60)
        return float(r.stdout.strip())
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return None


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"   # つなぎっぱなしにする（時計の往復に TLS の握手を混ぜない）
    server_version = "g1_phone_cam/1"
    plain = False                   # http の口（証明書と時計だけ）
    wbufsize = 1 << 16              # 見出しと本文を 1 回で送る（2 回に分けると Nagle と遅延 ACK で帰りだけ遅れ、時計が片寄る）

    @property
    def app(self) -> App:
        return self.server.app  # type: ignore[attr-defined]

    @property
    def access(self) -> al.AccessLog:
        return self.server.access  # type: ignore[attr-defined]

    @property
    def ip(self) -> str:
        return str(self.client_address[0])

    def setup(self) -> None:
        try:
            self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        if isinstance(self.request, ssl.SSLSocket):
            try:
                self.request.do_handshake()
            except (ssl.SSLError, OSError) as e:
                if not isinstance(e, (socket.timeout, TimeoutError)):  # 先回りの接続が黙って切れるのは誤りでない
                    self.access.tls_failed(self.ip, e)
                self.request.close()
                raise
        scheme = ("http（証明書の配布の口）" if self.plain
                  else "https" if isinstance(self.request, ssl.SSLSocket) else "http（--no-tls の試し）")
        self.access.note(self.ip, f"conn-{scheme}", f"{self.ip} から {scheme} につながった")
        super().setup()

    def _key_refused(self) -> None:
        self.access.note(self.ip, "key", f"⚠️ {self.ip}: 鍵が合わない（古い QR か、URL を手で打った）→ Mac の画面の QR から開き直す")

    def log_message(self, fmt: str, *args) -> None:  # 毎秒の状態の問い合わせは出さない
        return

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code: int, obj) -> None:
        self._send(code, json.dumps(obj, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")

    def _authed(self, q: dict) -> bool:
        k = self.headers.get("X-Key") or (q.get("k") or [""])[0]
        try:
            return secrets.compare_digest(k.encode("utf-8"), self.app.token.encode("utf-8"))
        except (TypeError, UnicodeError):
            return False

    def _body(self, limit: int) -> bytes:
        n = int(self.headers.get("Content-Length") or 0)
        if n < 0 or n > limit:
            raise ValueError(f"大きすぎる（{n} バイト）")
        data = self.rfile.read(n)
        if len(data) != n:  # 電話が送る途中で打ち切った（時間切れ・Wi-Fi の切り替え）。書くと塊が欠けたまま「受けた」になる
            raise ShortBody(f"本文が途中で切れた（{len(data)}/{n} バイト）")
        return data

    def do_GET(self) -> None:  # noqa: N802
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if self.plain and u.path not in HTTP_ONLY:
            return self._json(404, {"error": "http の口は /ca.crt と /clock だけ"})
        if u.path == "/ca.crt":
            ca = CONF / "ca.crt"
            if not ca.exists():
                return self._json(404, {"error": "証明書がまだ無い（make_cert.sh）"})
            self.access.note(self.ip, "ca", f"{self.ip} に証明書を渡した → iPhone の「設定」でプロファイルをインストールし、"
                                            "「一般 → 情報 → 証明書信頼設定」で g1_phone_cam をオン")
            return self._send(200, ca.read_bytes(), "application/x-x509-ca-cert")
        if self.plain and u.path == "/":
            self.access.note(self.ip, "plain", f"{self.ip} に証明書の配布のページを渡した（{device_name(self.headers)}）")
            return self._send(200, PLAIN_PAGE.encode("utf-8"), TYPES[".html"])
        if u.path in STATIC:
            if STATIC[u.path] == "index.html":
                self.access.note(self.ip, "page", f"{self.ip} に撮影のページを渡した（{device_name(self.headers)}）")
            f = WEB / STATIC[u.path]
            return self._send(200, f.read_bytes(), TYPES[f.suffix])
        if not self._authed(q):
            self._key_refused()
            return self._json(403, {"error": "鍵が合わない（Mac の画面の QR から開き直す）"})
        if u.path == "/api/time":
            self.access.note(self.ip, "api", f"{self.ip} の撮影のページが鍵つきで問い合わせている（ページは動いている）")
            return self._json(200, {"t": 1000.0 * self.app.clock()})
        if u.path == "/api/state":
            return self._json(200, dict(self.app.state(), recs=self.app.list_recs(), calib_ms=self.app.calib_ms()))
        return self._json(404, {"error": "無い"})

    def do_POST(self) -> None:  # noqa: N802
        u = urlparse(self.path)
        q = parse_qs(u.query)
        try:  # 本文は先に読み切る（つなぎっぱなしの接続に読み残すと、次の要求が壊れる）
            raw = self._body(MAX_CHUNK if u.path == "/api/rec/chunk" else MAX_JSON)
        except ShortBody as e:  # 相手はもう居ない。答えずに閉じる（電話は同じ塊を送り直す）
            self.close_connection = True
            self.access.note(self.ip, "short", f"⚠️ {self.ip} の送りが途中で切れた（{e}）。捨てて送り直しを待つ"
                                               "（続くなら Wi-Fi が遅い → ページの設定で画質・速さを下げる）")
            return None
        except ValueError as e:
            self.close_connection = True
            return self._json(413, {"error": str(e)})
        if self.plain or not self._authed(q):
            if not self.plain:
                self._key_refused()
            return self._json(403, {"error": "鍵が合わない"})
        try:
            if u.path == "/api/rec/chunk":
                rec_id, seq = (q.get("rec") or [""])[0], int((q.get("seq") or ["-1"])[0])
                if not REC_ID_RE.match(rec_id) or seq < 0:
                    return self._json(400, {"error": "録画の名前か番号が違う"})
                code, out = self.app.chunk(rec_id, seq, raw)
                return self._json(code, out)
            body = json.loads(raw or b"{}")
            if not isinstance(body, dict):
                raise ValueError("JSON の形が違う")
            if u.path == "/api/rec/start":
                return self._json(200, self.app.start_rec(body))
            if u.path == "/api/rec/stop":
                if not REC_ID_RE.match(str(body.get("rec", ""))):
                    return self._json(400, {"error": "録画の名前が違う"})
                code, out = self.app.stop_rec(body)
                return self._json(code, out)
            if u.path == "/api/mark":
                return self._json(200, self.app.mark(body))
            return self._json(404, {"error": "無い"})
        except (ValueError, KeyError, TypeError) as e:
            return self._json(400, {"error": f"要求が読めない: {e}"})


class PlainHandler(Handler):
    plain = True


PLAIN_PAGE = """<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>G1 撮影の準備</title><style>body{font:16px/1.6 -apple-system,"Hiragino Sans",sans-serif;margin:24px;max-width:36em}
a{font-size:20px}</style><h1>G1 撮影の準備（初めの 1 回だけ）</h1><ol>
<li><a href="/ca.crt">証明書を入れる</a> → 「設定」→「プロファイルがダウンロードされました」→ インストール</li>
<li>「設定」→「一般」→「情報」→「証明書信頼設定」→ g1_phone_cam をオン</li>
<li>Mac の画面の QR を iPhone のカメラで読んで、撮影のページを開く</li></ol>
<p>時計の合わせ（1 日 1 回）: Mac のブラウザで <a href="/clock">/clock</a> を開き、録画の初めの 2 秒だけ写す。</p>"""


class TLSServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, addr, handler, app: App, ctx: Optional[ssl.SSLContext],
                 access: Optional[al.AccessLog] = None) -> None:
        super().__init__(addr, handler)
        self.app, self.ctx = app, ctx
        self.access = access or al.AccessLog()

    def get_request(self):
        sock, addr = super().get_request()
        sock.settimeout(60)
        if self.ctx is not None:  # 握手は受けた側の糸で（遅い相手が他の接続を止めない）
            sock = self.ctx.wrap_socket(sock, server_side=True, do_handshake_on_connect=False)
        return sock, addr

    def handle_error(self, request, client_address) -> None:
        etype = sys.exc_info()[0]
        if etype is not None and issubclass(etype, (ssl.SSLError, ConnectionError, TimeoutError, socket.timeout)):
            return  # 証明書を信頼していない Safari や、切れた接続は毎回出さない
        super().handle_error(request, client_address)


RE_IOS = re.compile(r"(iPhone|iPad).*? OS (\d+(?:_\d+)*)")
RE_SAFARI = re.compile(r"Version/(\d+(?:\.\d+)*)")


def device_name(headers) -> str:
    """User-Agent から「iPhone Safari 26.6.3」くらいに縮める（版で録画・画面の消灯止めの有無が変わる）。
    iOS 26 から User-Agent の OS の版は 18 系のまま止まっている（11-13 の iPhone は「OS 18_7」で Safari 26.6.3）ので、
    Safari の版を先に出す"""
    ua = str(headers.get("User-Agent") or "")
    m, v = RE_IOS.search(ua), RE_SAFARI.search(ua)
    if m:
        return f"{m.group(1)} Safari {v.group(1) if v else '?'}（UA の OS {m.group(2).replace('_', '.')}）"
    return ua[:60] or "User-Agent なし"


def lan_ip(toward: str = "192.168.0.82") -> str:
    """PC2 の網に向いている Mac の口の IP（送らずに道だけ引く）"""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect((toward, 9))
            return s.getsockname()[0]
        except OSError:
            return "127.0.0.1"


def load_token() -> str:
    f = CONF / "token"
    if f.exists():
        return f.read_text(encoding="utf-8").strip()
    CONF.mkdir(parents=True, exist_ok=True)
    tok = secrets.token_urlsafe(12)
    f.write_text(tok, encoding="utf-8")
    f.chmod(0o600)
    return tok


def write_qr(url: str, path: Path) -> bool:
    try:
        import cv2
        import numpy as np
    except ImportError:
        return False
    img = cv2.QRCodeEncoder.create().encode(url)
    img = cv2.resize(img, None, fx=12, fy=12, interpolation=cv2.INTER_NEAREST)
    img = np.pad(img, 48, constant_values=255)
    return bool(cv2.imwrite(str(path), img))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(HERE.parent / "logs_real" / "phone" / time.strftime("%Y%m%d")))
    ap.add_argument("--port", type=int, default=8443)
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--cert", default=str(CONF / "server.crt"))
    ap.add_argument("--key", default=str(CONF / "server.key"))
    ap.add_argument("--no-tls", action="store_true", help="試験用（iPhone の Safari はカメラを使わせない）")
    ap.add_argument("--no-feed", action="store_true", help="PC2 を見ない（機体の電源が切れている時・撮影だけ）")
    ap.add_argument("--qr", default="", help="撮影のページの URL の QR を書く PNG")
    ap.add_argument("--token", default="", help="試験用（既定は ~/.config/g1_phone_cam/token）")
    a = ap.parse_args(argv)
    data = Path(a.data)
    feed = None
    if not a.no_feed:
        import robot_feed as rf
        feed = rf.RobotFeed(data)
        feed.start()
    app = App(data, a.token or load_token(), feed)
    app.salvage()
    ctx = None
    if not a.no_tls:
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(a.cert, a.key)
    ip = lan_ip()
    access = al.AccessLog(plain_url=f"http://{ip}:{a.http_port}/", out=lambda s: print(s, flush=True), quiet_ips=(ip,))
    main_srv = TLSServer((a.bind, a.port), Handler, app, ctx, access)
    plain_srv = TLSServer((a.bind, a.http_port), PlainHandler, app, None, access)
    threading.Thread(target=plain_srv.serve_forever, daemon=True).start()
    url = f"{'http' if a.no_tls else 'https'}://{ip}:{a.port}/?k={app.token}"
    print(f"[phone] 撮影のページ: {url}", flush=True)
    print(f"[phone] 初めの 1 回だけ: iPhone で http://{ip}:{a.http_port}/ を開いて証明書を入れる。"
          f"時計の合わせは Mac で http://localhost:{a.http_port}/clock", flush=True)
    print(f"[phone] 置き場: {data}", flush=True)
    if not ip.startswith("192.168.0."):
        print(f"[phone] ⚠️ Mac の IP が {ip}。G1 の Wi-Fi（Physical_AI・192.168.0.x）に居ない。iPhone と同じ Wi-Fi に入れる", flush=True)
    if a.qr and write_qr(url, Path(a.qr)):
        print(f"[phone] QR: {a.qr}", flush=True)

    def check() -> None:
        host = ip if a.bind in ("", "0.0.0.0") else a.bind  # 待つ口を絞った時（試し）はその IP で
        ok, msg = al.self_check(host, a.port, a.http_port, app.token, None if a.no_tls else CONF / "ca.crt", tls=not a.no_tls)
        print(f"[phone] {'自己点検 OK' if ok else '⚠️ 自己点検で誤り'}: {msg}", flush=True)
        if ok:
            print("[phone] 電話から開いても下に何も出なければ、電話の要求が Mac に来ていない（Wi-Fi・IP を確かめる）", flush=True)

    threading.Thread(target=check, daemon=True).start()
    try:
        main_srv.serve_forever()
    except KeyboardInterrupt:
        print("[phone] 止めた", flush=True)
    finally:
        if feed is not None:
            feed.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
