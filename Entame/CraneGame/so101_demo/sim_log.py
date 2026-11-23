"""/sim/camera の操作ログ。画面（ブラウザ）が 1 秒ほどごとにまとめて送る 1 フレームごとの記録を、debug フォルダへ JSONL で残す。

「グリッパーが閉じない」「操作しにくい」の原因を、あとから数字で調べるためのもの（認識した値・指令・シミュレーションの状態）。
映像そのものは送らない。1 回の送信は 1 つの記録（セッション）にまとめ、ページを開き直すと別のファイルになる。
"""
import json
import re
import threading
import time
from pathlib import Path

SESSION_RE = re.compile(r"^[a-z0-9]{6,24}$")
BATCH_RECORDS_MAX = 200               # 1 回の送信に入る記録の最大数
FILE_BYTES_MAX = 20 * 1024 * 1024     # 1 セッションのファイルの上限（超えたら受け付けない）
SESSIONS_MAX = 50                     # 1 回のサーバー起動で作るファイルの最大数


class SimLog:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.files = {}               # セッション ID → ファイル
        self.lock = threading.Lock()

    def append(self, body):
        """body は JSON {"session": "...", "records": [{...}, ...]}。書いた件数を返す。不正は ValueError。"""
        try:
            data = json.loads(body)
        except json.JSONDecodeError as exc:
            raise ValueError(f"JSON を読めません: {exc}") from exc
        if not isinstance(data, dict):
            raise ValueError("JSONオブジェクトが必要です")
        session, records = data.get("session"), data.get("records")
        if not isinstance(session, str) or not SESSION_RE.match(session):
            raise ValueError("session が不正です")
        if not isinstance(records, list) or not 0 < len(records) <= BATCH_RECORDS_MAX:
            raise ValueError(f"records は 1〜{BATCH_RECORDS_MAX} 件の配列にしてください")
        if not all(isinstance(r, dict) for r in records):
            raise ValueError("records の各要素はオブジェクトにしてください")
        lines = "".join(json.dumps(r, ensure_ascii=False, separators=(",", ":")) + "\n" for r in records)
        with self.lock:
            path = self.files.get(session)
            if path is None:
                if len(self.files) >= SESSIONS_MAX:
                    raise ValueError("ログのファイル数が上限に達しました。サーバーを再起動してください")
                self.folder.mkdir(parents=True, exist_ok=True)
                path = self.files[session] = self.folder / f"sim-camera-{time.strftime('%Y%m%d-%H%M%S')}-{session[:6]}.jsonl"
            if path.exists() and path.stat().st_size + len(lines.encode()) > FILE_BYTES_MAX:
                raise ValueError("ログが上限（20 MB）に達しました。ページを開き直してください")
            with path.open("a", encoding="utf-8") as f:
                f.write(lines)
        return len(records)
