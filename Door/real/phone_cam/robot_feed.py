"""**Mac の上で**、PC2 の見張り役（pc2_feed_agent.py）を ssh で起こし、走らせ役の出力を受けて「今の状態」を持つ。

  受けた物は全部 <記録の置き場>/feed_<日時>.jsonl に書く（1 行 = {"mac": 受けた時刻, "mono", "wall", "file", "lines"}）。
  phone_sync.py はこれで PC2 の monotonic（記録の t）を Mac の時計に直し、記録が無い時は行の時刻で段の移りを並べる。

接続先は deploy_to_pc2.sh と同じ環境変数（G1_PC2_HOST 既定 192.168.0.82・G1_PC2_USER unitree・G1_KEY ~/.ssh/id_ed25519_g1・
G1_JUMP があれば踏み台）。切れたら 3 秒おきに（最長 10 秒おきに）つなぎ直す。機体の電源が切れていても撮影のページは使える。
"""
from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Deque, List, Optional, Sequence, Tuple

import feed_parse as fp
import timebase as tb

HERE = Path(__file__).resolve().parent
AGENT = HERE / "pc2_feed_agent.py"
RETRY_S = (3.0, 5.0, 10.0)
SAMPLES_KEEP = 4000          # 時計の見本（0.5 秒に 1 つ以上・30 分ぶん強）
REMOTE_LOG = "~/door_real/logs/console_latest.log"


def ssh_command(env: Optional[dict] = None) -> List[str]:
    e = dict(os.environ if env is None else env)
    host = e.get("G1_PC2_HOST", "192.168.0.82")
    user = e.get("G1_PC2_USER", "unitree")
    key = os.path.expanduser(e.get("G1_KEY", "~/.ssh/id_ed25519_g1"))
    cmd = ["ssh", "-i", key, "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "-o", "IdentitiesOnly=yes",
           "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=3"]
    if e.get("G1_JUMP"):
        cmd += ["-J", e["G1_JUMP"]]
    return cmd + [f"{user}@{host}", "python3", "-u", "-", "--log", REMOTE_LOG]


class RobotFeed(threading.Thread):
    """cmd: 見張り役を起こすコマンド（既定は ssh で PC2。試験では手元の python で pc2_feed_agent.py を起こす）。
    stdin_script: True なら pc2_feed_agent.py の中身を標準入力へ流す（ssh の時）"""

    def __init__(self, data_dir: Path, cmd: Optional[Sequence[str]] = None, stdin_script: Optional[bool] = None,
                 clock=time.time) -> None:
        super().__init__(daemon=True, name="robot_feed")
        self.cmd = list(cmd) if cmd is not None else ssh_command()
        self.stdin_script = (cmd is None) if stdin_script is None else stdin_script
        self.clock = clock
        self.lock = threading.Lock()
        self.state = fp.RobotState()
        self.samples: Deque[Tuple[float, float]] = deque(maxlen=SAMPLES_KEEP)  # (PC2 の monotonic, Mac で受けた時刻)
        self.connected = False
        self.last_error = ""
        self.seen_lines = False
        self.stop_flag = threading.Event()
        self.proc: Optional[subprocess.Popen] = None
        data_dir.mkdir(parents=True, exist_ok=True)
        self.feed_path = data_dir / f"feed_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"

    # ── 外から読む ──
    def snapshot(self) -> dict:
        now = self.clock()
        with self.lock:
            st, conn, err = self.state, self.connected, self.last_error
            off = tb.mono_offset(list(self.samples)[-200:]) if self.samples else None
        d = fp.to_json(st, now)
        d.update(connected=conn, error=err, mono_offset=off)
        return d

    def stop(self) -> None:
        self.stop_flag.set()
        p = self.proc
        if p is not None and p.poll() is None:
            p.terminate()

    # ── 受ける ──
    def handle(self, raw: str) -> None:
        """見張り役の 1 行（JSON）"""
        mac = self.clock()
        msg = json.loads(raw)
        mono, lines = float(msg["mono"]), [str(s) for s in msg.get("lines", [])]
        if msg.get("backlog") and self.seen_lines:
            lines = []  # つなぎ直した時の末尾の行は前に読んだ物（読み直すと ARMS_IN を二重に数え、自動の録画が切り替わる）
        with open(self.feed_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(dict(mac=mac, **msg), ensure_ascii=False) + "\n")
        with self.lock:
            self.samples.append((mono, mac))
            off = tb.mono_offset(list(self.samples)[-200:])
            t_line = mono + off  # 見張り役が出した時の Mac の時刻（行が書かれたのはこれより最大 0.05 秒前）
            st = self.state
            for s in lines:
                st = fp.apply_line(st, s, t_line)
            self.state = st
            self.seen_lines = self.seen_lines or bool(lines)

    def _run_once(self) -> None:
        stdin = subprocess.PIPE if self.stdin_script else subprocess.DEVNULL
        self.proc = subprocess.Popen(self.cmd, stdin=stdin, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     text=True, encoding="utf-8", errors="replace", bufsize=1)
        if self.stdin_script:
            self.proc.stdin.write(AGENT.read_text(encoding="utf-8"))
            self.proc.stdin.close()
        for raw in self.proc.stdout:
            if self.stop_flag.is_set():
                break
            if not raw.strip():
                continue
            try:
                self.handle(raw)
            except (ValueError, KeyError, TypeError, AttributeError) as e:  # 1 行が変でも糸は止めない
                with self.lock:
                    self.last_error = f"見張り役の行を読めない: {e}"
                continue
            if not self.connected:
                with self.lock:
                    self.connected, self.last_error = True, ""
                print(f"[phone_feed] PC2 につながった（{self.feed_path.name} に書く）", flush=True)
        rc = self.proc.wait()
        err = (self.proc.stderr.read() or "").strip().splitlines()
        with self.lock:
            self.connected = False
            self.last_error = f"見張り役が終わった（{rc}）" + (f": {err[-1]}" if err else "")

    def run(self) -> None:
        n = 0
        while not self.stop_flag.is_set():
            try:
                self._run_once()
            except OSError as e:
                with self.lock:
                    self.connected, self.last_error = False, f"見張り役を起こせない: {e}"
            if self.stop_flag.is_set():
                break
            wait = RETRY_S[min(n, len(RETRY_S) - 1)]
            print(f"[phone_feed] PC2 から切れた（{self.last_error}）。{wait:.0f} 秒後につなぎ直す", flush=True)
            n += 1
            self.stop_flag.wait(wait)


def load_feed(paths: Sequence[Path]) -> Tuple[List[Tuple[float, float]], List[Tuple[float, str]]]:
    """feed_*.jsonl から、時計の見本 (monotonic, Mac) と行 (Mac の時刻, 行) を読む"""
    samples: List[Tuple[float, float]] = []
    msgs: List[Tuple[float, float, List[str]]] = []
    for p in paths:
        with open(p, encoding="utf-8") as f:
            for raw in f:
                try:
                    m = json.loads(raw)
                    samples.append((float(m["mono"]), float(m["mac"])))
                    ls = [] if m.get("backlog") else [str(s) for s in m.get("lines", [])]  # 末尾の行は書かれた時刻が分からない
                    if ls:
                        msgs.append((float(m["mono"]), float(m["mac"]), ls))
                except (ValueError, KeyError):
                    continue
    lines: List[Tuple[float, str]] = []
    if samples:
        clock = tb.MonoClock(samples)
        for mono, mac, ls in msgs:
            t = mono + clock.offset_at(mac)  # 見張り役が出した時の Mac の時刻
            lines += [(t, s) for s in ls]
    return samples, lines
