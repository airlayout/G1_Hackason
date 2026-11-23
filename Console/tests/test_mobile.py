"""静的配信・認証とブラウザを使わない入力/ライフサイクルの回帰検査。"""
import json
import shutil
import subprocess
import sys
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

CONSOLE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(CONSOLE))
from server import MockHelper, Monitor, make_handler
from g1console.auth import Auth


class StaticFilesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        helper = MockHelper()
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(helper, Monitor(helper)))
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = "http://127.0.0.1:%d" % cls.server.server_port

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()

    def test_only_named_ui_files_are_served_with_correct_types(self):
        for path, content_type in [("/", "text/html"), ("/index.html", "text/html"), ("/legacy.html", "text/html"),
                                   ("/mobile.css", "text/css"), ("/mobile.js?v=1", "application/javascript"), ("/ui-core.js", "application/javascript")]:
            with self.subTest(path=path), urllib.request.urlopen(self.base + path) as r:
                self.assertEqual(r.headers.get_content_type(), content_type)
                self.assertEqual(r.headers["Cache-Control"], "no-store")
                self.assertGreater(len(r.read()), 100)

    def test_unknown_and_parent_paths_never_expose_files(self):
        for path in ("/server.py", "/settings.json", "/web/../server.py", "/%2e%2e/server.py", "/web/mobile.js", "/missing.js"):
            with self.subTest(path=path), self.assertRaises(urllib.error.HTTPError) as caught:
                urllib.request.urlopen(self.base + path)
            self.assertEqual(caught.exception.code, 404)

    def test_static_files_share_the_existing_auth_gate(self):
        helper = MockHelper()
        auth = Auth("test-mobile-token", ["127.0.0.1"])
        srv = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(helper, Monitor(helper), auth=auth))
        thread = threading.Thread(target=srv.serve_forever, daemon=True)
        thread.start()
        try:
            base = "http://127.0.0.1:%d" % srv.server_port
            for path in ("/mobile.js", "/mobile.css", "/legacy.html"):
                with self.assertRaises(urllib.error.HTTPError) as caught:
                    urllib.request.urlopen(base + path)
                self.assertEqual(caught.exception.code, 401)
                request = urllib.request.Request(base + path, headers={"Authorization": "Bearer test-mobile-token"})
                with urllib.request.urlopen(request) as response:
                    self.assertEqual(response.status, 200)
        finally:
            srv.shutdown()
            srv.server_close()
            thread.join()


class MobileRuntimeTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which("node"), "Node.js is needed only for the development-time JS regression checks")
    def test_inputs_freshness_routes_and_late_claim_are_checked_without_a_browser(self):
        for name in ("mobile_runtime.cjs", "mobile_startup.cjs"):
            with self.subTest(script=name):
                result = subprocess.run([shutil.which("node"), str(CONSOLE / "tests" / name)],
                                        text=True, capture_output=True, timeout=20)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("PASS", result.stdout)


if __name__ == "__main__":
    unittest.main()
