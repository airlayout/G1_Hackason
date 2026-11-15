"""**Mac の上で**: iPhone の 1 本の動画を、機体の出力（feed_*.jsonl）・あれば記録（door_stock_*.npz）と時刻で合わせ、
段の移り・印の瞬間の画を並べた 1 枚の絵を作る（Claude が Read で見る）。字幕（.srt）と字幕を焼いた動画も作れる。

  python phone_sync.py <置き場>/phone_<日時>.json               # 段の移り・出来事・印の画（feed の行の時刻 ±0.05 秒）
  python phone_sync.py <rec.json> --npz <door_stock_….npz>       # 記録の行で段の移りを引き直し、値（扉・手・力・温度）を添える
  python phone_sync.py <rec.json> --at 12.5 14.0                 # 動画の秒を指して画を出す
  python phone_sync.py <rec.json> --burn                         # 字幕を焼いた動画（<rec>_sync/<rec>_overlay.mp4）も作る
  python phone_sync.py <rec.json> --calib-frames                 # 初めの 3 秒の画に見積もりの時刻を添える（Mac の時計を写した回）
  python phone_sync.py <rec.json> --set-calib 120                # 写った時計 − 見積もり（ms）を覚える（以後の全部の動画に効く）

出す物（<rec>_sync/）: events.txt（動画の秒・Mac の時刻・出来事）・sheet_1.jpg …（3 列・1 枚 12 こま）・subs.srt
印の画は押した時と、その 0.5・1.0 秒前（見てから押すまでの遅れ）を並べる。
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import feed_parse as fp  # noqa: E402
import robot_feed as rf  # noqa: E402
import timebase as tb  # noqa: E402

FONT = "/System/Library/Fonts/ヒラギノ角ゴシック W4.ttc"
TILE_W, COLS, PER_SHEET = 640, 3, 12
CAPTION_LINES, LINE_H = 4, 24
MARK_BEFORE_S = (1.0, 0.5, 0.0)
FEED_LINE_LAG_S = 0.025   # 見張り役は 0.05 秒ごとに読むので、行が書かれたのは出した時より平均 0.025 秒前
SUBS_HOLD_S = 2.0


@dataclass(frozen=True)
class Event:
    video_t: float
    kind: str      # 段・小段・出来事・印・指定・合わせ
    text: str
    detail: str = ""


def load_rec(path: Path, calib_ms: Optional[float]) -> Tuple[dict, Path, tb.VideoClock, float]:
    meta = json.loads(path.read_text(encoding="utf-8"))
    video = path.with_name(meta.get("video") or f"{meta['rec']}.mp4")
    if not video.exists():
        raise SystemExit(f"[sync] 動画が無い: {video}（受け口が仕上げ中か、mp4 に直せなかった: {meta.get('remux_error')}）")
    source = rec_source(meta)
    if calib_ms is None:
        calib_ms = read_calib(path.parent, source)
    dur = float(meta.get("duration") or math.nan)
    if source == "continuity":  # 連係カメラ: Mac が直に録った。0 秒の Mac の時刻は ffmpeg の start（最初の画が届いた時）
        clock = tb.VideoClock(float(meta["start_mac"]) + calib_ms / 1000.0, dur)
    else:
        pings = tb.pings_from_ms(meta.get("pings") or [])
        clock = tb.video_clock(float(meta["start_phone_ms"]) / 1000.0, pings, calib_ms / 1000.0, dur)
    return meta, video, clock, calib_ms


CALIB_KEYS = {"phone": "calib_ms", "continuity": "calib_ms_continuity"}  # 写ってから届くまでの遅れは録り方で違う


def rec_source(meta: dict) -> str:
    return "continuity" if meta.get("source") == "continuity" else "phone"


def calib_file(data_dir: Path) -> Path:
    return data_dir.parent / "calib.json"


def read_calib(data_dir: Path, source: str = "phone") -> float:
    try:
        return float(json.loads(calib_file(data_dir).read_text(encoding="utf-8"))[CALIB_KEYS[source]])
    except (OSError, ValueError, KeyError):
        return 0.0


def write_calib(data_dir: Path, source: str, ms: float, meta: dict) -> Path:
    """録り方ごとに覚える（もう一方の値は残す）"""
    f = calib_file(data_dir)
    try:
        d = json.loads(f.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        d = {}
    d.update({CALIB_KEYS[source]: ms, f"{CALIB_KEYS[source]}_from": meta.get("rec"),
              f"{CALIB_KEYS[source]}_set_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
              f"{CALIB_KEYS[source]}_device": meta.get("device") or meta.get("ua")})
    f.write_text(json.dumps(d, ensure_ascii=False, indent=1), encoding="utf-8")
    return f


# ── 出来事を集める ──
def feed_events(lines: Sequence[Tuple[float, str]], clock: tb.VideoClock, margin: float = 2.0) -> List[Event]:
    out = []
    for mac, line in lines:
        t = mac - FEED_LINE_LAG_S
        if not clock.contains(t, margin):
            continue
        m = fp.RE_TRANSITION.match(line)
        if m:
            out.append(Event(clock.to_video(t), "段", f"{m.group(1)} → {m.group(2)}", m.group(3) or ""))
            continue
        m = fp.RE_SUB.match(line)
        if m:
            out.append(Event(clock.to_video(t), "小段", fp.sub_label(m.group(2)), m.group(3)))
            continue
        if any(k in line for k in fp.EVENT_KEYS):
            out.append(Event(clock.to_video(t), "出来事", line.replace("[script] ", "").replace("[staged] ", "")[:90]))
    return out


def status_lines(lines: Sequence[Tuple[float, str]], clock: tb.VideoClock) -> List[Tuple[float, fp.RobotState]]:
    """動画の秒ごとの状態（0.5 秒ごとの状態の行と小段の移りで作り直す）"""
    st, out = fp.RobotState(), []
    for mac, line in lines:
        st = fp.apply_line(st, line, mac)
        if clock.contains(mac, 1.0) and (fp.RE_STATUS.match(line) or fp.RE_SUB.match(line) or fp.RE_TRANSITION.match(line)):
            out.append((clock.to_video(mac - FEED_LINE_LAG_S), st))
    return out


def state_caption(st: Optional[fp.RobotState]) -> str:
    if st is None:
        return "（機体の出力なし）"
    door = "—" if st.door_deg != st.door_deg else f"{st.door_deg:.1f}°"
    temp = "—" if st.temp_c != st.temp_c else f"{st.temp_c:.0f} °C"
    sub = f" {fp.sub_label(st.sub)}" if st.phase in ("DOOR", "POST") and st.sub else ""
    return f"{st.phase}{sub}・扉 {door}・腕 {temp}"


def state_at(states: Sequence[Tuple[float, fp.RobotState]], t: float) -> Optional[fp.RobotState]:
    prev = None
    for vt, st in states:
        if vt > t:
            break
        prev = st
    return prev


def mono_to_mac_many(mono: tb.MonoClock, t, hint_mac: float):
    """記録の全部の行の t を Mac の時刻に（30 秒おきの点で差を引き、間は直線で。行ごとに引くと 9 万行で遅い）。
    hint_mac は動画の 0 秒（PC2 を起こし直した日に、別の起動と取り違えないため）"""
    import numpy as np
    anchors = np.unique(np.concatenate([np.arange(t[0], t[-1], 30.0), [t[-1]]]))
    pts = []
    for v in anchors:
        try:
            pts.append((v, mono.mono_to_mac(float(v), hint_mac) - v))
        except ValueError:
            continue
    if not pts:
        raise SystemExit("[sync] 記録の時刻に合う機体の出力が無い（その記録の間、受け口が PC2 を見ていなかった）")
    xs, ds_ = (np.array(c) for c in zip(*pts))
    return t + np.interp(t, xs, ds_)


def npz_events(path: Path, mono: tb.MonoClock, clock: tb.VideoClock) -> Tuple[List[Event], dict]:
    """記録の行から段・小段の移りを引き、各行の値を返す（見出しに添える）"""
    import numpy as np
    import ana_script_run as asr
    import door_script as dscr
    import door_staged as ds
    z = np.load(path, allow_pickle=True)
    meta = asr.load_meta(z)
    args = meta.get("args", {}) or {}
    phases = list(meta.get("phases") or [])
    t = np.asarray(z["t"], dtype=float)
    vt = mono_to_mac_many(mono, t, clock.start_mac) - clock.start_mac  # 動画の秒
    out: List[Event] = []
    ph = np.asarray(z["phase"]).astype(int)
    for k in np.where(np.diff(ph) != 0)[0] + 1:
        out.append(Event(float(vt[k]), "段", f"{phases[ph[k - 1]]} → {phases[ph[k]]}" if phases else f"段 {ph[k]}"))
    rows = {"vt": vt}
    if "script" in z.files:
        sc = np.asarray(z["script"], dtype=float)
        w, _, lh, bs = (float(v) for v in str(args.get("door") or "0.94,2.05,0.99,0.064").split(","))
        ref = ds.lever_ref_point(w, bs, lh, not args.get("lever_right", False))
        sub = sc[:, 0]
        ok = np.isfinite(sub)
        for k in np.where(ok[1:] & (sub[1:] != sub[:-1]))[0] + 1:
            d = 100 * (sc[k, 4:7] - ref)
            out.append(Event(float(vt[k]), "小段", fp.sub_label(dscr.SUBS[int(sub[k])]),
                             f"手−当てる点 前 {d[0]:+.1f} 横 {d[1]:+.1f} 上 {d[2]:+.1f} cm"))
        rows.update(sub=sub, door=sc[:, 7], palm=100 * (sc[:, 4:7] - ref))
    rows.update(tau=np.asarray(z["tau"], dtype=float), temp=np.nanmax(np.asarray(z["arm_temp"], dtype=float), axis=1))
    return out, rows


def row_caption(rows: dict, t: float) -> str:
    import numpy as np
    k = int(np.argmin(np.abs(rows["vt"] - t)))
    if abs(rows["vt"][k] - t) > 0.1:
        return "（記録の外）"
    s = f"肩pitch {rows['tau'][k, 15]:+.1f}・肘 {rows['tau'][k, 18]:+.1f} N·m・腕 {rows['temp'][k]:.0f} °C"
    if "door" in rows and np.isfinite(rows["door"][k]):
        p = rows["palm"][k]
        s = f"扉 {rows['door'][k]:.1f}°・手−点 [{p[0]:+.1f},{p[1]:+.1f},{p[2]:+.1f}] cm・" + s
    return s


def mark_events(marks: Path, rec_id: str, clock: tb.VideoClock) -> List[Event]:
    out: List[Event] = []
    if not marks.exists():
        return out
    for raw in marks.read_text(encoding="utf-8").splitlines():
        try:
            m = json.loads(raw)
        except ValueError:
            continue
        if m.get("mac") is None or not (m.get("rec") == rec_id or clock.contains(float(m["mac"]))):
            continue
        t0 = clock.to_video(float(m["mac"]))
        label = m.get("label", "") + (f"「{m['memo']}」" if m.get("memo") else "")
        for back in MARK_BEFORE_S:
            out.append(Event(t0 - back, "印", f"{label}" + (f"（{back:.1f} 秒前）" if back else "（押した時）")))
    return out


# ── 画 ──
def extract_frame(video: Path, t: float, out: Path) -> bool:
    cmd = ["ffmpeg", "-y", "-v", "error", "-ss", f"{max(0.0, t):.3f}", "-i", str(video), "-frames:v", "1",
           "-vf", f"scale={TILE_W}:-2", "-q:v", "3", str(out)]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    return r.returncode == 0 and out.exists()


def wrap(text: str, font, width: float) -> List[str]:
    """こまの幅に収まるよう、文字ごとに折り返す（日本語は語の切れ目が無いので文字で）"""
    out, cur = [], ""
    for ch in text:
        if cur and font.getlength(cur + ch) > width:
            out.append(cur)
            cur = ch
        else:
            cur += ch
    return out + ([cur] if cur else [])


def make_sheets(tiles: Sequence[Tuple[Path, List[str]]], out_dir: Path) -> List[Path]:
    from PIL import Image, ImageDraw, ImageFont
    try:
        font = ImageFont.truetype(FONT, 19)
    except OSError:
        font = ImageFont.load_default()
    paths = []
    for n in range(0, len(tiles), PER_SHEET):
        part = tiles[n:n + PER_SHEET]
        imgs = [Image.open(p).convert("RGB") for p, _ in part]
        th = max(i.height for i in imgs)
        cell_h = th + CAPTION_LINES * LINE_H + 8
        rows = math.ceil(len(part) / COLS)
        sheet = Image.new("RGB", (COLS * TILE_W, rows * cell_h), (16, 16, 18))
        draw = ImageDraw.Draw(sheet)
        for k, (img, (_, cap)) in enumerate(zip(imgs, part)):
            x, y = (k % COLS) * TILE_W, (k // COLS) * cell_h
            sheet.paste(img, (x, y))
            # 出来事は折り返して残りの行まで、補足と状態は 1 行ずつ（はみ出す分は … で切る）
            budget = [max(1, CAPTION_LINES - (len(cap) - 1))] + [1] * (len(cap) - 1)
            lines = []
            for j, (c, b) in enumerate(zip(cap, budget)):
                w = wrap(c, font, TILE_W - 12)
                if len(w) > b:
                    w = w[:b]
                    w[-1] = w[-1][:-1] + "…"
                lines += [(j, s) for s in w]
            lines = lines[:CAPTION_LINES]
            for row, (j, line) in enumerate(lines):
                draw.text((x + 6, y + th + 4 + row * LINE_H), line, font=font, fill=(240, 236, 226) if j == 0 else (190, 190, 190))
        p = out_dir / f"sheet_{n // PER_SHEET + 1}.jpg"
        sheet.save(p, quality=88)
        paths.append(p)
    return paths


def write_srt(states: Sequence[Tuple[float, fp.RobotState]], events: Sequence[Event], path: Path, duration: float) -> None:
    """字幕: 状態（扉・小段・温度）を下に、印と出来事を 2 秒だけ上に"""
    def ts(s: float) -> str:
        s = max(0.0, s)
        return f"{int(s // 3600):02d}:{int(s % 3600 // 60):02d}:{int(s % 60):02d},{int(round(s % 1 * 1000)) % 1000:03d}"

    cues: List[Tuple[float, float, str]] = []
    for k, (t, st) in enumerate(states):
        end = states[k + 1][0] if k + 1 < len(states) else min(t + SUBS_HOLD_S, duration if duration == duration else t + SUBS_HOLD_S)
        if end > t:
            cues.append((t, end, state_caption(st)))
    for e in events:
        if e.kind in ("印", "小段", "段", "出来事") and not (e.kind == "印" and "秒前" in e.text):
            cues.append((e.video_t, e.video_t + SUBS_HOLD_S, f"{{\\an8}}{e.kind}: {e.text}"))
    cues.sort()
    with open(path, "w", encoding="utf-8") as f:
        for n, (a, b, text) in enumerate(cues, 1):
            f.write(f"{n}\n{ts(a)} --> {ts(b)}\n{text}\n\n")


def burn(video: Path, srt: Path, out: Path) -> bool:
    style = "FontName=Hiragino Sans,FontSize=16,Outline=2,Shadow=0,MarginV=24"
    cmd = ["ffmpeg", "-y", "-v", "error", "-i", str(video), "-vf", f"subtitles={srt.name}:force_style='{style}'",
           "-c:v", "libx264", "-preset", "veryfast", "-crf", "23", "-c:a", "copy", str(out)]
    r = subprocess.run(cmd, cwd=srt.parent, capture_output=True, text=True, timeout=3600)
    if r.returncode != 0:
        print(f"[sync] ⚠️ 字幕を焼けない: {r.stderr.strip()[-300:]}", flush=True)
    return r.returncode == 0


def hms(mac: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(mac)) + f".{int(mac % 1 * 1000):03d}"


def main(argv=None) -> int:  # noqa: C901
    ap = argparse.ArgumentParser()
    ap.add_argument("rec", help="phone_<日時>.json")
    ap.add_argument("--npz", default="", help="PC2 の記録（door_stock_*.npz を Mac へ持ってきた物）")
    ap.add_argument("--feed", nargs="*", default=None, help="feed_*.jsonl（既定は動画と同じ置き場の全部）")
    ap.add_argument("--at", nargs="*", type=float, default=[], help="画を出す動画の秒")
    ap.add_argument("--burn", action="store_true")
    ap.add_argument("--calib", type=float, default=None, help="カメラの遅れ ms（既定は calib.json）")
    ap.add_argument("--calib-frames", action="store_true")
    ap.add_argument("--set-calib", type=float, default=None)
    a = ap.parse_args(argv)
    rec_path = Path(a.rec)
    if a.set_calib is not None:
        meta = json.loads(rec_path.read_text(encoding="utf-8"))
        f = write_calib(rec_path.parent, rec_source(meta), a.set_calib, meta)
        name = "連係カメラ" if rec_source(meta) == "continuity" else "電話の Safari"
        print(f"[sync] {name}の遅れ {a.set_calib:+.0f} ms を {f} に書いた", flush=True)
        return 0
    meta, video, clock, calib_ms = load_rec(rec_path, 0.0 if a.calib_frames else a.calib)
    out_dir = rec_path.with_name(f"{meta['rec']}_sync")
    out_dir.mkdir(exist_ok=True)
    how = ("連係カメラ" + ("（0 秒の時刻は仮）" if meta.get("start_mac_estimated") else "")
           if rec_source(meta) == "continuity" else f"電話・時計の往復 {meta.get('rtt_ms') or math.nan:.0f} ms")
    print(f"[sync] {video.name}: 長さ {clock.duration:.1f} 秒・0 秒は {hms(clock.start_mac)}・{how}・"
          f"カメラの遅れ {calib_ms:+.0f} ms", flush=True)

    events: List[Event] = []
    if a.calib_frames:
        events = [Event(round(0.2 * k, 1), "合わせ", f"見積もり {hms(clock.to_mac(0.2 * k))}") for k in range(16)]
        states: List[Tuple[float, fp.RobotState]] = []
        rows = None
    else:
        feeds = [Path(p) for p in a.feed] if a.feed is not None else sorted(rec_path.parent.glob("feed_*.jsonl"))
        samples, lines = rf.load_feed(feeds) if feeds else ([], [])
        states = status_lines(lines, clock)
        events = feed_events(lines, clock)
        rows = None
        if a.npz:
            if not samples:
                raise SystemExit("[sync] 機体の出力（feed）が無いので、記録の時刻を Mac の時計に直せない")
            npz_ev, rows = npz_events(Path(a.npz), tb.MonoClock(samples), clock)
            events = [e for e in events if e.kind not in ("段", "小段")] + npz_ev  # 段の移りは記録の行の方が正確
        events += mark_events(rec_path.parent / "marks.jsonl", meta["rec"], clock)
        events += [Event(t, "指定", f"{t:.2f} 秒") for t in a.at]
        if not lines:
            print("[sync] ⚠️ 機体の出力が無い（受け口が PC2 を見ていなかった）。印と指定の画だけ出す", flush=True)
    dur = clock.duration if clock.duration == clock.duration else math.inf
    events = sorted((e for e in events if -0.5 <= e.video_t <= dur + 0.5), key=lambda e: e.video_t)

    (out_dir / "events.json").write_text(json.dumps(
        [dict(video_t=round(e.video_t, 3), mac=round(clock.to_mac(e.video_t), 3), kind=e.kind, text=e.text, detail=e.detail)
         for e in events], ensure_ascii=False, indent=1), encoding="utf-8")
    tiles = []
    with open(out_dir / "events.txt", "w", encoding="utf-8") as f:
        for n, e in enumerate(events):
            t = min(max(e.video_t, 0.0), max(0.0, dur - 0.05))
            line = f"{e.video_t:7.2f} 秒  {hms(clock.to_mac(e.video_t))}  {e.kind}  {e.text}" + (f"（{e.detail}）" if e.detail else "")
            f.write(line + "\n")
            print("[sync] " + line, flush=True)
            img = out_dir / f"f{n:03d}.jpg"
            if not extract_frame(video, t, img):
                continue
            cap = [f"{e.video_t:.2f} 秒 ｜ {e.kind}: {e.text}"]
            if e.detail:
                cap.append(e.detail[:60])
            cap.append(row_caption(rows, e.video_t) if rows is not None else state_caption(state_at(states, e.video_t)))
            tiles.append((img, cap))
    sheets = make_sheets(tiles, out_dir) if tiles else []
    for p in sheets:
        print(f"[sync] 絵: {p}", flush=True)
    if not a.calib_frames:
        srt = out_dir / "subs.srt"
        write_srt(states, events, srt, clock.duration)
        print(f"[sync] 字幕: {srt}", flush=True)
        if a.burn:
            out = out_dir / f"{meta['rec']}_overlay.mp4"
            if burn(video, srt, out):
                print(f"[sync] 字幕を焼いた動画: {out}", flush=True)
    else:
        print("[sync] 絵の時計の数字 − 見積もり（ms）を --set-calib で覚える（2〜3 こまの中央）", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
