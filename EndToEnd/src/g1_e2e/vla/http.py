"""UIと機体の小さなJSON API。pickle、CORS、任意ファイル配信は使わない。"""

from __future__ import annotations

import hmac
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen


MAX_BODY = 2 * 1024 * 1024


class ApiClient:
    def __init__(self, base_url: str, *, token: str = "", timeout: float = 5):
        parts = urlsplit(base_url)
        if parts.scheme not in {"http", "https"} or not parts.hostname or parts.username or parts.password:
            raise ValueError("接続先にはhttp(s) URLを指定してください")
        self.base_url = base_url.rstrip("/")
        self.token, self.timeout = token, timeout

    def request(self, path: str, body: dict | None = None):
        headers = {"Accept": "application/json"}
        data = None
        if body is not None:
            headers["Content-Type"] = "application/json"
            data = json.dumps(body, allow_nan=False).encode()
        if self.token:
            headers["Authorization"] = "Bearer " + self.token
        try:
            with urlopen(
                Request(self.base_url + path, data=data, headers=headers), timeout=self.timeout
            ) as response:
                return json.loads(response.read(MAX_BODY))
        except HTTPError as error:
            try:
                reason = json.loads(error.read(MAX_BODY)).get("error", str(error))
            except (ValueError, AttributeError):
                reason = str(error)
            raise RuntimeError(reason) from error
        except (URLError, TimeoutError, OSError) as error:
            raise RuntimeError(f"G1に接続できません: {error}") from error


def serve(application, bind: str, port: int, *, token: str = "") -> ThreadingHTTPServer:
    """application.handle(method, path, body) -> (content_type, payload)。"""

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            self.dispatch("GET")

        def do_POST(self) -> None:
            self.dispatch("POST")

        def dispatch(self, method: str) -> None:
            try:
                self.connection.settimeout(5)
                host = urlsplit("//" + self.headers.get("Host", "")).hostname
                if bind in {"127.0.0.1", "localhost"} and host not in {"127.0.0.1", "localhost"}:
                    self.respond(403, "application/json", {"error": "localhostから接続してください"})
                    return
                if token and not hmac.compare_digest(
                    self.headers.get("Authorization", ""), "Bearer " + token
                ):
                    self.respond(401, "application/json", {"error": "認証に失敗しました"})
                    return
                # 別サイトのJavaScriptからローカルの腕制御APIを呼べないようにする。
                origin = self.headers.get("Origin")
                if origin and urlsplit(origin).netloc != self.headers.get("Host"):
                    self.respond(403, "application/json", {"error": "別のページからの操作は受け付けません"})
                    return
                body = None
                if method == "POST":
                    if self.headers.get_content_type() != "application/json":
                        raise ValueError("Content-Type: application/jsonが必要です")
                    size = int(self.headers.get("Content-Length", "0"))
                    if not 0 < size <= MAX_BODY:
                        raise ValueError("リクエストサイズが範囲外です")
                    body = json.loads(self.rfile.read(size))
                    if not isinstance(body, dict):
                        raise ValueError("JSON objectが必要です")
                content_type, payload = application.handle(method, urlsplit(self.path).path, body)
                self.respond(200, content_type, payload)
            except (ValueError, KeyError, TypeError) as error:
                self.respond(400, "application/json", {"error": str(error)})
            except FileNotFoundError:
                self.respond(404, "application/json", {"error": "ページがありません"})
            except Exception as error:
                self.respond(503, "application/json", {"error": str(error)})

        def respond(self, status: int, content_type: str, payload) -> None:
            if content_type == "application/json":
                data = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode()
            elif isinstance(payload, str):
                data = payload.encode()
            else:
                data = payload
            self.send_response(status)
            self.send_header(
                "Content-Type", content_type + ("; charset=utf-8" if content_type.startswith("text/") else "")
            )
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header(
                "Content-Security-Policy",
                "default-src 'self'; img-src 'self' data: blob:; frame-ancestors 'none'",
            )
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_args) -> None:
            pass

    server = ThreadingHTTPServer((bind, port), Handler)
    server.daemon_threads = True
    return server
