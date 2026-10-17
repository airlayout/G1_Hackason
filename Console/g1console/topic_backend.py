"""トピック通信のバックエンド（Jetson 上の DDS ヘルパーを子プロセスで持つ）。

初回の購読か送信のときに起動する（常駐させない）。直列で、応答が遅ければ捨てて次回起動し直す。
ヘルパー側にも送信許可リストと拒否リストを渡して二重に検査する。
"""
import json
import os
import queue
import subprocess
import sys
import threading
from pathlib import Path

from g1console.topics import DENY_PATTERNS, TopicError

HELPER_SRC = Path(__file__).resolve().parent.parent / "jetson" / "topic_helper.py"
HELPER_READY_S = 15.0
HELPER_TIMEOUT_S = 2.0


class HelperTopicBackend:
    name = "local"

    def __init__(self, allowlist, sdk_dir=None):
        self._allow = {t: r["max_bytes"] for t, r in allowlist.items()}
        self._sdk_dir = Path(sdk_dir or os.environ.get("UNITREE_SDK_DIR") or "~/unitree_sdk2_python").expanduser()
        self._lock = threading.Lock()
        self._proc = None
        self._lines = queue.Queue()
        self._subs = {}  # 再起動したときに購読し直すための控え

    def _command(self, src):
        return [sys.executable, "-u", "-c", src], str(self._sdk_dir)

    def _source(self):
        src = HELPER_SRC.read_text(encoding="utf-8")
        return src.replace("__PUBLISH__", repr(self._allow)).replace("__DENY__", repr(list(DENY_PATTERNS)))

    @staticmethod
    def _pump(proc, out):
        for line in proc.stdout:
            out.put(line)
        out.put(None)

    def _alive(self):
        return self._proc is not None and self._proc.poll() is None

    def _kill(self):
        if self._proc is not None:
            self._proc.kill()
            self._proc = None

    def _start(self):
        cmd, cwd = self._command(self._source())
        self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                      stderr=subprocess.DEVNULL, text=True, cwd=cwd)
        self._lines = queue.Queue()
        threading.Thread(target=self._pump, args=(self._proc, self._lines), daemon=True).start()
        try:
            ready = self._lines.get(timeout=HELPER_READY_S)
            ok = ready is not None and json.loads(ready).get("ready")
        except (queue.Empty, ValueError):
            ok = False
        if not ok:
            self._kill()
            raise TopicError(502, "トピックヘルパーが起動しません（SDK・DDS・eth0 を確認）")
        for topic, type_name in list(self._subs.items()):  # 再起動後は購読を復元する
            self._call_started({"op": "sub", "topic": topic, "type": type_name})

    def _call_started(self, req):
        try:
            self._proc.stdin.write(json.dumps(req) + "\n")
            self._proc.stdin.flush()
            line = self._lines.get(timeout=HELPER_TIMEOUT_S)
            if line is None:
                raise RuntimeError("helper exited")
            reply = json.loads(line)
        except (OSError, RuntimeError, queue.Empty, ValueError) as exc:
            self._kill()  # 遅れて届く古い応答と取り違えないよう捨てる
            raise TopicError(502, "トピックヘルパーが応答しません（%s）" % type(exc).__name__)
        if not reply.get("ok"):
            raise TopicError(502, "トピックヘルパーが拒否しました: %s" % reply.get("error", "不明"))
        return reply

    def _call(self, req):
        with self._lock:
            if not self._alive():
                self._start()
            return self._call_started(req)

    def subscribe(self, topic, type_name):
        self._call({"op": "sub", "topic": topic, "type": type_name})
        self._subs[topic] = type_name

    def unsubscribe(self, topic):
        self._subs.pop(topic, None)
        if self._alive():
            self._call({"op": "unsub", "topic": topic})

    def poll(self):
        return self._call({"op": "poll"}).get("items", [])

    def publish(self, topic, data):
        self._call({"op": "pub", "topic": topic, "data": data})

    def close(self):
        with self._lock:
            if self._proc is not None:
                try:
                    self._proc.stdin.close()  # EOF でヘルパーが終わる
                    self._proc.wait(timeout=2)
                except (OSError, subprocess.TimeoutExpired):
                    pass
                self._kill()
