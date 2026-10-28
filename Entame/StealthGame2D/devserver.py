#!/usr/bin/env python3
"""
ステージエディタの「保存」をブラウザのダウンロードではなく、
このディレクトリ配下の stages/ に直接書き込むための簡易開発用サーバー。

python3 -m http.server の代わりにこのスクリプトを起動する:
    python3 devserver.py

静的ファイル配信は http.server と同じ。
追加で POST /api/save-stage を提供し、editor.js の保存ボタンから
ステージJSONを stages/ に書き込み、必要なら manifest.json も更新する。
"""

import json
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
STAGES_DIR = BASE_DIR / "stages"
MANIFEST_PATH = STAGES_DIR / "manifest.json"

# ファイル名は日本語・英数字・記号を含めて50文字以内の "*.json" を許可する。
# パストラバーサル対策として、パス区切り文字（/ \）と制御文字、".." を含む名前は拒否する。
import re

MAX_FILENAME_LENGTH = 50
VALID_FILENAME_RE = re.compile(r'^[^\\/:*?"<>|\x00-\x1f]+\.json$')


def is_valid_filename(file_name):
    if not isinstance(file_name, str):
        return False
    if len(file_name) > MAX_FILENAME_LENGTH:
        return False
    if ".." in file_name:
        return False
    return bool(VALID_FILENAME_RE.match(file_name))


class DevRequestHandler(SimpleHTTPRequestHandler):
    def do_POST(self):
        if self.path == "/api/save-stage":
            self._handle_save_stage()
        else:
            self.send_error(404, "Not Found")

    def _handle_save_stage(self):
        try:
            length = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(length)
            payload = json.loads(body)
        except (ValueError, json.JSONDecodeError) as error:
            self._send_json(400, {"error": f"リクエストの読み取りに失敗しました: {error}"})
            return

        file_name = payload.get("fileName")
        stage = payload.get("stage")

        if not is_valid_filename(file_name):
            self._send_json(
                400,
                {"error": f"不正なファイル名です（50文字以内、拡張子.json、/ \\ を含まない）: {file_name!r}"},
            )
            return
        if not isinstance(stage, dict):
            self._send_json(400, {"error": "stage フィールドが不正です"})
            return

        try:
            STAGES_DIR.mkdir(exist_ok=True)
            stage_path = STAGES_DIR / file_name
            stage_path.write_text(json.dumps(stage, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

            manifest = self._read_manifest()
            manifest_changed = False
            if file_name not in manifest:
                manifest = [*manifest, file_name]
                manifest_changed = True
                MANIFEST_PATH.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        except OSError as error:
            self._send_json(500, {"error": f"ファイル書き込みに失敗しました: {error}"})
            return

        self._send_json(200, {"fileName": file_name, "manifestUpdated": manifest_changed})

    def _read_manifest(self):
        if not MANIFEST_PATH.exists():
            return []
        try:
            return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            return []

    def _send_json(self, status, data):
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    import os

    os.chdir(BASE_DIR)
    server = HTTPServer(("localhost", 8000), DevRequestHandler)
    print("[devserver] http://localhost:8000 で起動しました（保存API: POST /api/save-stage）")
    server.serve_forever()


if __name__ == "__main__":
    main()
