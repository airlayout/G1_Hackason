"""カメラ配信プロセス（Jetson の camera_stream.py）の確認・起動・停止。

Jetson の ~/g1_console_camera/camera_ctl.sh を ssh で 1 回ずつ叩く（常駐ヘルパーとは別。DDS には触れない）。
--local のときは ssh を使わず、このリポジトリの jetson/camera_ctl.sh を直接呼ぶ（配信は 127.0.0.1 だけで待ち受ける）。
ハブは起動しない限り配信を起動しない（ポートは画面で「起動」を押したときだけ使う）。
holders は「映像デバイスを掴んでいるプロセス」。videohub_pc4 などが握っていると camera_stream は開けない。
"""
import ipaddress
import re
import shlex
import subprocess
from pathlib import Path

REMOTE_DIR = "~/g1_console_camera"
LOCAL_CTL_DIR = Path(__file__).resolve().parent.parent / "jetson"  # --local のとき ctl スクリプトを呼ぶ場所
LOCAL_STATE_DIR = "~/.g1console/run"  # --local のときの pid・ログの置き場
LOOPBACK = "127.0.0.1"
CTL_TIMEOUT_S = 30.0  # start は起動確認に約 2 秒待つ
ACTIONS = ("status", "start", "stop")
OWN_MARK = "camera_stream.py"
SEP = "@@camera-ctl-sep@@"
OWN_RE = re.compile(r"(^|[\s/])python[0-9.]*\s+(-u\s+)?\S*camera_stream\.py(\s|$)")  # python … camera_stream.py だけを自前とみなす


_RESOLVED = {}


def resolve_host(host: str) -> str:
    """ssh のエイリアス（~/.ssh/config の Host）を実ホスト名に直す。HTTP の宛先には別名が使えないため。
    解決できなければそのまま返す。結果は覚えておく（ssh を毎回叩かない）。IP リテラルは ssh に聞かずそのまま返す。"""
    try:
        ipaddress.ip_address(host)
        return host
    except ValueError:
        pass
    if host not in _RESOLVED:
        try:
            done = subprocess.run(["ssh", "-G", "--", host], capture_output=True, text=True, timeout=5)
            names = [l.split(None, 1)[1] for l in done.stdout.splitlines() if l.startswith("hostname ")]
            _RESOLVED[host] = names[0].strip() if names else host
        except (OSError, subprocess.TimeoutExpired):
            _RESOLVED[host] = host  # 失敗も覚える（毎回 5 秒待たせない）。設定変更後は別の host 文字列になる
    return _RESOLVED[host]


def parse_holders(text: str) -> list:
    """camera_ctl.sh holders の出力（デバイス<TAB>PID<TAB>ユーザー<TAB>コマンド）を dict の列にする。"""
    out = []
    for line in text.splitlines():
        parts = line.split("\t", 3)
        if len(parts) == 4 and parts[1].isdigit():
            device, pid, user, cmd = parts
            out.append({"device": device, "pid": int(pid), "user": user, "cmd": cmd, "own": bool(OWN_RE.search(cmd))})
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
    ENV = "CAMERA"  # 配信ポートを渡す環境変数 CAMERA_PORT の接頭辞（lidar は LIDAR）

    def __init__(self, target_fn, port_fn=None):
        self._target_fn = target_fn  # () -> (user@host or None, key)
        self._port_fn = port_fn  # () -> 配信ポート（検証済みの整数）。None なら ctl スクリプトの既定

    def _dir(self) -> str:
        return REMOTE_DIR

    def _extra_env(self) -> list:
        return []

    def _ctl(self, script: str) -> str:
        """ctl スクリプトの呼び出し（環境変数つき）。入るのは固定の語・整数・このリポジトリ内のパスだけ。"""
        env = ["%s_PORT=%d" % (self.ENV, int(self._port_fn()))] if self._port_fn else []
        return " ".join(env + self._extra_env() + ["%s/%s" % (self._dir(), script)])

    def _run(self, script: str) -> str:
        host, key = self._target_fn()
        if not host:
            raise RuntimeError("Jetson の接続先が未設定です（設定タブで入力）")
        key_opt = ["-i", key] if key else []
        cmd = ["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes", *key_opt, "--", host, script]
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=CTL_TIMEOUT_S)
        if done.returncode == 255:
            raise RuntimeError("Jetson に ssh できません: %s" % (done.stderr.strip().splitlines() or [""])[-1])
        return done.stdout

    def call(self, action: str) -> dict:
        """操作（status 以外）・status・holders を 1 回の ssh で実行し、SEP 行で 3 つに分ける。"""
        if action not in ACTIONS:
            raise ValueError("unknown action")  # script に入るのは許可リストの固定語だけ
        ctl = self._ctl("camera_ctl.sh")
        steps = ([ctl + " " + action] if action != "status" else ["true"]) + [ctl + " status", ctl + " holders"]
        out = self._run(("; echo %s; " % SEP).join(steps))
        message, status, holders = (out.split(SEP + "\n") + ["", ""])[:3]
        return {**summarize(status, holders), "message": message.strip()}


class LocalRunMixin:
    """ssh を介さず、Jetson 上のこのプロセスから ctl スクリプトを直接実行する（コンソールを Jetson で動かす場合）。"""

    backend = "local"

    def _dir(self) -> str:
        return shlex.quote(str(LOCAL_CTL_DIR))  # ~/g1_console_* へのコピーは要らない。このリポジトリの jetson/ をそのまま使う

    def _extra_env(self) -> list:
        # 配信は自機（ハブ）からだけ読ませる（wlan0 へ認証なしで開かない）。pid・ログはリポジトリの外に置く
        return ["G1_CTL_STATE_DIR=%s" % shlex.quote(str(Path(LOCAL_STATE_DIR).expanduser())), "%s_BIND=%s" % (self.ENV, LOOPBACK)]

    def _run(self, script: str) -> str:
        done = subprocess.run(["bash", "-c", script], capture_output=True, text=True, timeout=CTL_TIMEOUT_S)
        return done.stdout


class LocalCameraCtl(LocalRunMixin, SshCameraCtl):
    def __init__(self, port_fn=None):
        super().__init__(lambda: (None, ""), port_fn)


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
