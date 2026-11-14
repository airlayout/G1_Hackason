"""押す流れの節目で、G1 に決まった言葉を喋らせる（configs/voice.yaml。contest_agent/agent.py が使う）。

    ann = Announcer.from_config(load_config("voice.yaml"), mode="local")   # PC2 の上で押す処理を動かすとき
    ann = Announcer.from_config(load_config("voice.yaml"), mode="ssh")     # 有線のノート PC から動かすとき
    ctl.on_event = ann.say      # ElevatorPressController が "press_start" / "lit" などを渡す
    ...
    ann.close(wait_s=8)         # 最後の言葉（点灯成功など）を言い終わるまで少し待つ

鳴らし方: 前もって PC2 の pc2.dir に置いた <名前>.wav を、PC2 の g1_speaker.py（PlayStream）で鳴らす。
- local: このプロセスと同じ PC2 の上で `python3 g1_speaker.py --wav ...` を別プロセスで動かす
- ssh:   ノート PC から `ssh PC2 'python3 g1_speaker.py --wav ...'`。PC から音声の DDS は叩かない
  （PC でカメラと音声の DDS を同時に使うと 3102/3104 になった記録がある。Voice/g1_speaker.py）

⚠️ 制御（50 Hz）を止めないことがいちばん大事。say() は言葉を待ち行列に入れるだけですぐ返す。鳴らすのは別スレッドで、
1 つずつ順に（重ねると同じ APP_NAME の PlayStream がぶつかる）。鳴らせなくても（PC2 に WAV が無い・ssh が切れた）
注意を 1 行出すだけで、押す流れは続ける。
"""

from __future__ import annotations

import os
import queue
import shlex
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

MODES = ("local", "ssh")


def speaker_command(mode: str, cfg: dict[str, Any], key: str) -> list[str]:
    """<名前>.wav を PC2 で鳴らすコマンド。"""
    pc2 = cfg["pc2"]
    d = str(pc2["dir"])
    remote = [str(pc2["python"]), f"{d}/g1_speaker.py", "--wav", f"{d}/{key}.wav", "--iface", str(pc2["iface"])]
    if mode == "local":
        return remote
    if mode == "ssh":
        host = os.environ.get("PC2") or str(pc2["host"])
        return ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", host, shlex.join(remote)]
    raise ValueError(f"喋らせ方は {' / '.join(MODES)}: {mode}")


class Announcer:
    def __init__(self, cfg: dict[str, Any], mode: str) -> None:
        if mode not in MODES:
            raise ValueError(f"喋らせ方は {' / '.join(MODES)}: {mode}")
        self.cfg = cfg
        self.mode = mode
        # 文が空・名前が無い出来事は喋らない
        self.phrases = {str(k): str(v) for k, v in (cfg.get("phrases") or {}).items() if v and str(v).strip()}
        self.timeout_s = float(cfg["pc2"].get("timeout_s", 20))
        self.said: list[dict[str, Any]] = []  # 鳴らした記録（名前・文・結果・かかった秒）。結果の JSON に入れる
        self._q: queue.Queue[str | None] = queue.Queue()
        self._worker = threading.Thread(target=self._run, name="announcer", daemon=True)
        self._worker.start()
        if mode == "local":
            missing = [k for k in self.phrases if not Path(f"{cfg['pc2']['dir']}/{k}.wav").exists()]
            if missing:
                print(f"[voice] ⚠️ PC2 に WAV が無い: {', '.join(missing)}（real/voice_phrases.py send を先に）")

    @classmethod
    def from_config(cls, cfg: dict[str, Any], mode: str) -> "Announcer":
        return cls(cfg, mode)

    def say(self, event: str) -> None:
        """出来事に結び付いた言葉を待ち行列に入れる（すぐ返る）。言葉が無い出来事は何もしない。"""
        if event in self.phrases:
            print(f"[voice] 「{self.phrases[event]}」（{event}）")
            self._q.put(event)

    def close(self, wait_s: float = 8.0) -> None:
        """待ち行列の言葉を、wait_s 秒まで待ってから終わる（鳴らしている途中のものは、それ以上は待たない）。"""
        self._q.put(None)
        self._worker.join(timeout=max(0.0, wait_s))
        if self._worker.is_alive():
            print(f"[voice] ⚠️ {wait_s:.0f} 秒待っても言い終わらなかった。待たずに終わる")

    def _run(self) -> None:
        while True:
            key = self._q.get()
            if key is None:
                return
            t0 = time.monotonic()
            try:
                r = subprocess.run(speaker_command(self.mode, self.cfg, key), capture_output=True, text=True,
                                   timeout=self.timeout_s)
                ok = r.returncode == 0
                why = "" if ok else (r.stdout.strip().splitlines() or r.stderr.strip().splitlines() or [f"終了コード {r.returncode}"])[-1]
            except (OSError, subprocess.TimeoutExpired) as e:  # 鳴らせなくても押す流れは止めない
                ok, why = False, str(e)
            dt = time.monotonic() - t0
            self.said.append({"event": key, "text": self.phrases[key], "ok": ok, "seconds": round(dt, 2),
                              **({"error": why} if not ok else {})})
            if not ok:
                print(f"[voice] ⚠️ 鳴らせなかった（{key}）: {why}")
