"""カメラ配信プロセス（Jetson の camera_stream.py）の確認・起動・停止。

Jetson の ~/g1_console_camera/camera_ctl.sh を ssh で 1 回ずつ叩く（常駐ヘルパーとは別。DDS には触れない）。
holders は「映像デバイスを掴んでいるプロセス」。videohub_pc4 などが握っていると camera_stream は開けない。
"""
import subprocess

REMOTE_DIR = "~/g1_console_camera"
CTL_TIMEOUT_S = 30.0  # start は起動確認に約 2 秒待つ
ACTIONS = ("status", "start", "stop")
OWN_MARK = "camera_stream.py"


_RESOLVED = {}


def resolve_host(host: str) -> str:
    """ssh のエイリアス（~/.ssh/config の Host）を実ホスト名に直す。HTTP の宛先には別名が使えないため。
    解決できなければそのまま返す。結果は覚えておく（ssh を毎回叩かない）。"""
    if host not in _RESOLVED:
        try:
            done = subprocess.run(["ssh", "-G", "--", host], capture_output=True, text=True, timeout=5)
            names = [l.split(None, 1)[1] for l in done.stdout.splitlines() if l.startswith("hostname ")]
            _RESOLVED[host] = names[0].strip() if names else host
        except (OSError, subprocess.TimeoutExpired):
            return host
    return _RESOLVED[host]


def parse_holders(text: str) -> list:
    """camera_ctl.sh holders の出力（デバイス<TAB>PID<TAB>ユーザー<TAB>コマンド）を dict の列にする。"""
    out = []
    for line in text.splitlines():
        parts = line.split("\t", 3)
        if len(parts) == 4 and parts[1].isdigit():
            device, pid, user, cmd = parts
            out.append({"device": device, "pid": int(pid), "user": user, "cmd": cmd, "own": OWN_MARK in cmd})
    return out


def parse_status(text: str) -> dict:
    """camera_ctl.sh status の出力から起動中か・PID を取り出す。"""
    running = "起動中" in text
    pid = None
    if running and "pid=" in text:
        digits = text.split("pid=", 1)[1].rstrip(") \n")
        pid = int(digits) if digits.isdigit() else None
    return {"running": running, "pid": pid}


def summarize(status_text: str, holders_text: str) -> dict:
    holders = parse_holders(holders_text)
    return {"ok": True, **parse_status(status_text), "holders": holders,
            "foreign": [h for h in holders if not h["own"]]}  # 自前以外が掴んでいる＝競合の可能性


class SshCameraCtl:
    backend = "ssh"

    def __init__(self, target_fn):
        self._target_fn = target_fn  # () -> (user@host or None, key)

    def _run(self, *args: str) -> str:
        host, key = self._target_fn()
        if not host:
            raise RuntimeError("Jetson の接続先が未設定です（設定タブで入力）")
        key_opt = ["-i", key] if key else []
        cmd = ["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes", *key_opt, "--", host,
               "%s/camera_ctl.sh %s" % (REMOTE_DIR, " ".join(args))]
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=CTL_TIMEOUT_S)
        if done.returncode == 255:
            raise RuntimeError("Jetson に ssh できません: %s" % (done.stderr.strip().splitlines() or [""])[-1])
        return done.stdout

    def call(self, action: str) -> dict:
        if action not in ACTIONS:
            raise ValueError("unknown action")
        message = self._run(action) if action != "status" else ""
        return {**summarize(self._run("status"), self._run("holders")), "message": message.strip()}


class MockCameraCtl:
    """実機なしの模擬。videohub が video4 を掴み、camera_stream は起動／停止できる。"""

    backend = "mock"

    def __init__(self):
        self.running = False

    def call(self, action: str) -> dict:
        if action not in ACTIONS:
            raise ValueError("unknown action")
        if action != "status":
            self.running = action == "start"
        status = "[camera] 起動中 (pid=4242)" if self.running else "[camera] 停止中"
        holders = "/dev/video4\t9113\troot\t/unitree/module/video_hub_pc4/videohub_pc4 /dev/video4\n"
        if self.running:
            holders += "/dev/video0\t4242\tunitree\tpython -u camera_stream.py --camera std=/dev/video0\n"
        return {**summarize(status, holders), "message": ""}
