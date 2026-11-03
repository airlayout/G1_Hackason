"""人を検知したら G1 本体のスピーカーで声をかける（2026-11-03 追加）。

    検知イベント ──> VoiceAnnouncer ──ssh の標準入力に "play"──> PC2: Speaker/g1_speaker.py --serve
                                                                      └─ AudioClient.PlayStream

⚠️ **鳴らすのは PC2 の中。** UI サーバは ssh を1本張って「鳴らして」と1行送るだけで、
音声の DDS には触らない（理由は `Speaker/g1_speaker.py` の冒頭）。無線でも同じ経路で動く。

⚠️ **無線の「見るだけ」でも鳴らす**（2026-11-03 に決めた）。声は検知で自動的に出るもので、
画面からの操作ではない。機体は動かさない。

⚠️ **人が映っている間はずっと繰り返す**（2026-11-03 に決めた）。「検知」で鳴らし始め、
言い終えたら `repeat_gap_s` 空けてまた鳴らす。「消失」で止める（言いかけの台詞は最後まで言う）。
映っているかどうかは検知ログと同じ判定（`present_frames` / `absent_grace_s`）を使う。

⚠️⚠️ **カメラが途切れても止める。** 映像が途絶えると VisionWorker は「消失」を出さない
（検知したままになる）ので、イベントだけを頼りにすると**鳴りっぱなしになる**。
繰り返す前に毎回 `still_present()`（カメラが生きていて、まだ映っているか）を確かめる。

ssh の宛先が無いとき（モック・動画で画面を作るとき）は鳴らさず、ログに出すだけにする。
"""
from __future__ import annotations

import json
import subprocess
import threading
import time
from typing import TYPE_CHECKING, Callable, Sequence

if TYPE_CHECKING:   # ⚠️ 実行時に import しない。vision は ultralytics を引き込み、テストが重くなる
    from vision import DetectionEvent


class VoiceAnnouncer:
    """検知イベントを受けて、人が映っている間 PC2 の常駐プロセスに再生を頼み続ける。"""

    def __init__(self, command: Sequence[str] | None, phrase: str,
                 repeat_gap_s: float = 1.0,
                 still_present: Callable[[], bool] | None = None) -> None:
        """`command` は PC2 の `g1_speaker.py --serve` を起動するコマンド（ssh 込み）。
        None なら鳴らさずログに出すだけ。テストでは ssh の代わりに手元の偽物を渡す。

        `still_present` は「カメラが生きていて、まだ人が映っているか」。繰り返す前に毎回聞く。
        """
        self._command = list(command) if command else None
        self._phrase = phrase
        self._gap_s = repeat_gap_s
        self._still_present = still_present or (lambda: True)
        self._lock = threading.Lock()
        self._proc: subprocess.Popen[str] | None = None
        self._state = "off" if self._command is None else "starting"
        self._message = "ssh の宛先が無いので鳴らさない（ログのみ）" if self._command is None else ""
        self._present = False
        self._busy = False
        self._timer: threading.Timer | None = None
        self._closed = False
        self._last_played_at: float | None = None
        self._played = 0
        self._episode_played = 0

    # ---- 起動・停止 ---------------------------------------------------------

    def start(self) -> None:
        if self._command is None:
            print(f"[声] {self._message}")
            return
        print(f"[声] 起動: {' '.join(self._command)}")
        try:
            self._proc = subprocess.Popen(
                self._command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=None, text=True, bufsize=1)
        except OSError as exc:
            self._set("error", f"起動できません: {exc}")
            return
        threading.Thread(target=self._read_replies, name="voice-replies", daemon=True).start()

    def close(self) -> None:
        with self._lock:
            self._closed = True
            self._present = False
            if self._timer is not None:
                self._timer.cancel()
        proc = self._proc
        if proc is None:
            return
        # ⚠️ 標準入力を閉じれば PC2 側は終わる（g1_speaker.py の serve）。待っても
        #    終わらなければ ssh ごと止める
        try:
            if proc.stdin:
                proc.stdin.close()
        except OSError:
            pass
        try:
            proc.wait(timeout=6)    # 言いかけの台詞（数秒）を言い終えるのを待つ
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
        print("[声] 止めた")

    # ---- 検知から ----------------------------------------------------------

    def on_event(self, event: DetectionEvent) -> None:
        if event.kind == "検知":
            with self._lock:
                self._present = True
                self._episode_played = 0
            print(f"[声] 人が映った。映っている間「{self._phrase}」を繰り返す")
            self._try_play()
        elif event.kind == "消失":
            with self._lock:
                was, n = self._present, self._episode_played
                self._present = False
                if self._timer is not None:
                    self._timer.cancel()
                    self._timer = None
            if was:
                print(f"[声] 人が消えたので止める（{n} 回鳴らした）")

    def _try_play(self) -> bool:
        """映っていて、鳴らせる状態なら再生を頼む。頼んだら True。"""
        with self._lock:
            self._timer = None
            if self._closed or not self._present:
                return False
            if not self._still_present():
                # カメラが途切れた・映っていない。「消失」が来ないこともあるのでここで止める
                self._present = False
                print(f"[声] カメラが途切れたか人が映っていないので止める（{self._episode_played} 回鳴らした）")
                return False
            if self._command is None:
                self._episode_played += 1
                if self._episode_played == 1:
                    print(f"[声] （ログのみ）「{self._phrase}」")
                # 実際には鳴らさないが、繰り返しの周期は同じにする（ログの検証用）
                self._schedule_next_locked(self._gap_s + 3.0)
                return True
            if self._state != "ready" or self._busy:
                return False
            proc = self._proc
            try:
                assert proc is not None and proc.stdin is not None
                proc.stdin.write("play\n")
                proc.stdin.flush()
            except (OSError, AssertionError) as exc:
                self._state, self._message = "error", f"PC2 に届きません: {exc}"
                print(f"[声] {self._message}")
                return False
            self._busy = True
            self._episode_played += 1
            return True

    def _schedule_next_locked(self, delay_s: float) -> None:
        """`delay_s` 後にもう一度鳴らす。呼ぶ側が self._lock を持っていること。"""
        if self._closed or not self._present:
            return
        if self._timer is not None:
            self._timer.cancel()
        self._timer = threading.Timer(delay_s, self._try_play)
        self._timer.daemon = True
        self._timer.start()

    # ---- PC2 からの返事 ------------------------------------------------------

    def _read_replies(self) -> None:
        proc = self._proc
        assert proc is not None and proc.stdout is not None
        for line in proc.stdout:
            try:
                reply = json.loads(line)
            except ValueError:
                continue    # 1 行 1 JSON 以外は無視（ログは標準エラーに出ている）
            status = reply.get("status")
            with self._lock:
                if status == "ready":
                    self._state = "ready"
                    self._message = f"PC2 の音量 {reply.get('volume')}"
                elif status in ("played", "error"):
                    self._busy = False
                    if status == "played":
                        self._played += 1
                        self._last_played_at = time.time()
                    else:
                        self._message = str(reply.get("message", ""))
                    # 映っている間は繰り返す。失敗しても間を空けて試し続ける
                    self._schedule_next_locked(self._gap_s if status == "played"
                                               else max(self._gap_s, 3.0))
            if status == "error":
                print(f"[声] PC2 で失敗: {reply.get('message')}")
            elif status == "ready":
                print(f"[声] 準備できた（{self._message}）")
                # 準備が整う前に人が映っていたら、ここで鳴らし始める
                self._try_play()
        proc.stdout.close()
        code = proc.wait()
        if not self._closed:
            self._set("error", f"PC2 の再生プロセスが終わった（終了コード {code}）")

    def _set(self, state: str, message: str) -> None:
        with self._lock:
            self._state, self._message, self._busy = state, message, False
        print(f"[声] {message}")

    @property
    def status(self) -> dict[str, object]:
        with self._lock:
            return {
                "state": "speaking" if self._busy else self._state,
                "message": self._message,
                "phrase": self._phrase,
                "present": self._present,
                "played": self._played,
                "last_played_at": self._last_played_at,
                "repeat_gap_s": self._gap_s,
            }


def build_voice(cfg: dict, host: str | None,
                still_present: Callable[[], bool] | None = None) -> VoiceAnnouncer | None:
    """設定から作る。`voice.enabled: false` なら None（何もしない）。

    `host` は PC2 への ssh の宛先。`run.sh` が WAV と `g1_speaker.py` を PC2 に置いてから
    `--voice-host` で渡す。無ければログだけにする。
    """
    if not bool(cfg.get("enabled", True)):
        print("[声] 声かけは切ってある（voice.enabled: false）")
        return None
    phrase = str(cfg.get("phrase", ""))
    gap = float(cfg.get("repeat_gap_s", 1.0))
    if not host:
        return VoiceAnnouncer(None, phrase, gap, still_present)
    remote = str(cfg.get("remote_dir", "/home/unitree/g1_ui_speaker"))
    wav_name = str(cfg.get("wav", "voice.wav")).rsplit("/", 1)[-1]
    # ⚠️ BatchMode: パスワードを聞いて止まらないように。ServerAlive: 無線が切れたら気づく
    command = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
               "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=3", host,
               f"exec python3 -u {remote}/g1_speaker.py --serve --wav {remote}/{wav_name}"]
    return VoiceAnnouncer(command, phrase, gap, still_present)
