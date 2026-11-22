"""追従中のデバッグログ（JSON Lines）。追従の1回ごとに debug/follow-日時.jsonl を作る。

1行が1つの出来事。ev の種類:
- start : 追従の開始（モード、開始時の姿勢、制限）
- sample: 手の入力 inp（0〜1・-1〜1に直した値）、画面側の生の値 dbg、指令までの経過時間 dt、
          読んだ姿勢 cur、書いた目標 goal。次の sample の cur と goal を比べると、実機が追えているか分かる
- stop  : 追従の停止（理由 reason）
- error : 追従中に起きたエラー（理由 reason）
ログを書けなくても追従は止めない（最初の1回だけ標準エラーに出して、以後は書かない）。
古いログは KEEP 件だけ残して消す（follow-*.jsonl だけが対象）。
"""
import json
import sys
import time
from pathlib import Path

KEEP = 20          # 残すログの数
PRECISION = 2      # 姿勢・入力の小数桁


def rounded(value):
    """辞書・数値の小数桁をそろえる（ログを読みやすく小さくする）。数値以外はそのまま。"""
    if isinstance(value, dict):
        return {k: rounded(v) for k, v in value.items()}
    return round(value, PRECISION) if isinstance(value, float) else value


class FollowLog:
    def __init__(self, folder, now=time.time, keep=KEEP):
        self.folder, self.now, self.keep = Path(folder), now, keep
        self.file = None
        self.path = None

    def begin(self, **info):
        """新しいログを開く。開いていれば先に閉じる。開けなければ理由を出して、ログなしで続ける。"""
        self.end("次の追従を開始")
        try:
            self.folder.mkdir(exist_ok=True)
            files = sorted(self.folder.glob("follow-*.jsonl"))
            for path in files[:max(0, len(files) - (self.keep - 1))]:     # 新しく作る分を空けて KEEP 件にする
                path.unlink()
            stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(self.now())) + f"-{int(self.now() * 1000) % 1000:03d}"
            number = 0
            while True:                                           # 同じ時刻でも、別のファイルにする
                self.path = self.folder / f"follow-{stamp}{f'-{number}' if number else ''}.jsonl"
                try:
                    self.file = open(self.path, "x", buffering=1, encoding="utf-8")
                    break
                except FileExistsError:
                    number += 1
        except OSError as exc:
            self.file = None
            print(f"[followlog] ログを開けません（{exc}）。ログなしで続けます", file=sys.stderr, flush=True)
            return
        self.write("start", **info)

    def write(self, ev, **fields):
        if self.file is None:
            return
        try:
            line = json.dumps(dict(ev=ev, ts=round(self.now(), 3), **rounded(fields)), ensure_ascii=False, allow_nan=False)
            self.file.write(line + "\n")
        except (OSError, ValueError) as exc:
            print(f"[followlog] ログを書けません（{exc}）。以後は書きません", file=sys.stderr, flush=True)
            self.close()

    def end(self, reason):
        """停止の理由を書いて閉じる。開いていなければ何もしない。"""
        if self.file is None:
            return
        self.write("stop", reason=reason)
        self.close()

    def close(self):
        if self.file is not None:
            try:
                self.file.close()
            except OSError:
                pass
            self.file = None
