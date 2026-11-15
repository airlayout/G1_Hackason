"""**PC2 の上で**動く見張り役。走らせ役の画面の出力（~/door_real/logs/console_latest.log）を追い、新しい行を 1 行の JSON で標準出力へ出す。

配らない。Mac の robot_feed.py が ssh で中身を流し込んで起こす（ssh ... python3 -u - --log ... < pc2_feed_agent.py）。
ssh が切れたら（標準出力に書けなくなったら）終わる。PC2 の Python 3.8・標準ライブラリだけ。CPU は 0.05 秒ごとにファイルを 1 回見るだけ。

出す形（1 行 1 つ）: {"mono": time.monotonic(), "wall": time.time(), "file": "console_….log", "lines": ["…", …]}
  mono は PC2 の記録（door_stock_*.npz の t）と同じ時計。Mac は受けた時刻との差で時計を合わせる（timebase.mono_offset）。
  行が無くても heartbeat 秒ごとに lines=[] を出す（時計を合わせる見本と、つながっている印）。
走らせ役を起こし直すと console_latest.log の行き先が変わるので、毎回たどり直す。新しいファイルは頭から送る。
"""
import argparse
import json
import os
import sys
import time

MAX_LINES_PER_MSG = 200
MAX_READ_BYTES = 1 << 20


def emit(lines, name, backlog=False):
    """backlog: つないだ時に送る、前から有った末尾の行（書かれた時刻は分からない。Mac は今の状態を作るのにだけ使う）"""
    msg = {"mono": time.monotonic(), "wall": time.time(), "file": name, "lines": lines}
    if backlog:
        msg["backlog"] = True
    sys.stdout.write(json.dumps(msg, ensure_ascii=False) + "\n")
    sys.stdout.flush()


def tail_lines(path, n):
    """ファイルの末尾 n 行と、読み終えた位置"""
    with open(path, "rb") as f:
        f.seek(0, os.SEEK_END)
        end = f.tell()
        f.seek(max(0, end - 64 * 1024))
        data = f.read()
    lines = data.decode("utf-8", "replace").splitlines()
    return lines[-n:] if n > 0 else [], end


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default=os.path.expanduser("~/door_real/logs/console_latest.log"))
    ap.add_argument("--tick", type=float, default=0.05)
    ap.add_argument("--heartbeat", type=float, default=0.5)
    ap.add_argument("--backlog", type=int, default=40, help="つないだ時に送る、今のファイルの末尾の行数")
    ap.add_argument("--max-seconds", type=float, default=0.0, help="試験用（0 は無限）")
    a = ap.parse_args(argv)
    t_end = time.monotonic() + a.max_seconds if a.max_seconds > 0 else None
    cur, pos, buf, last_emit, first = None, 0, b"", 0.0, True
    try:
        while t_end is None or time.monotonic() < t_end:
            real = os.path.realpath(a.log) if os.path.exists(a.log) else None
            lines = []
            if real != cur:
                cur, buf = real, b""
                if real is None:
                    pos = 0
                elif first:
                    back, pos = tail_lines(real, a.backlog)
                    if back:
                        emit(back, os.path.basename(real), backlog=True)
                else:
                    pos = 0  # 起こし直した: 新しいファイルを頭から
                first = False
            if cur is not None:
                try:
                    size = os.path.getsize(cur)
                    if size < pos:
                        pos, buf = 0, b""  # 切り詰められた
                    if size > pos:
                        with open(cur, "rb") as f:
                            f.seek(pos)
                            data = f.read(MAX_READ_BYTES)
                        pos += len(data)
                        buf += data
                        *done, buf = buf.split(b"\n")
                        lines += [d.decode("utf-8", "replace") for d in done]
                except OSError:
                    cur = None
            now = time.monotonic()
            name = os.path.basename(cur) if cur else ""
            for k in range(0, len(lines), MAX_LINES_PER_MSG):
                emit(lines[k:k + MAX_LINES_PER_MSG], name)
                last_emit = now
            if now - last_emit >= a.heartbeat:
                emit([], name)
                last_emit = now
            time.sleep(a.tick)
    except (BrokenPipeError, KeyboardInterrupt):
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
