"""外部（wlan0）へ開くときの認証・待ち受け。"""
import http.client
import json
import os
import stat
import sys
import tempfile
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import server  # noqa: E402
from g1console import auth as auth_mod  # noqa: E402
from g1console.auth import Auth, host_only, iface_ipv4, load_or_create_token, parse_bind  # noqa: E402
from g1console.camera_proc import MockCameraCtl  # noqa: E402
from g1console.lidar_proc import MockLidarCtl  # noqa: E402
from g1console.settings import Settings  # noqa: E402

TOKEN = "t0ken-for-tests_ABC"
EXTRA_HOST = "192.0.2.7"


class TokenFileTest(unittest.TestCase):
    def test_created_with_0600_and_reused(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "sub" / "token"
            first = load_or_create_token(path)
            self.assertGreaterEqual(len(first), 20)
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
            self.assertEqual(load_or_create_token(path), first)

    def test_empty_file_is_regenerated(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "token"
            path.write_text("\n")
            self.assertTrue(load_or_create_token(path))


class BindTest(unittest.TestCase):
    def test_parse_bind(self):
        self.assertEqual(parse_bind("192.168.0.5"), "192.168.0.5")
        for bad in ("0.0.0.0", "::", "abc", "", "192.168.0.256"):
            with self.assertRaises(ValueError):
                parse_bind(bad)

    def test_iface_ipv4_parses_ip_output(self):
        out = "3: wlan0    inet 192.168.0.23/24 brd 192.168.0.255 scope global dynamic wlan0\\ valid_lft 3600sec\n"
        fake = mock.Mock(return_value=mock.Mock(stdout=out))
        self.assertEqual(iface_ipv4("wlan0", run=fake), "192.168.0.23")
        self.assertEqual(fake.call_args[0][0], ["ip", "-4", "-o", "addr", "show", "dev", "wlan0"])

    def test_iface_without_address_or_bad_name(self):
        with self.assertRaises(ValueError):
            iface_ipv4("wlan0", run=mock.Mock(return_value=mock.Mock(stdout="")))
        for bad in ("-x", "a b", "", "wlan0;rm"):
            with self.assertRaises(ValueError):
                iface_ipv4(bad, run=mock.Mock())

    def test_iface_command_failure(self):
        with self.assertRaises(ValueError):
            iface_ipv4("wlan0", run=mock.Mock(side_effect=OSError("no ip")))

    def test_resolve_binds(self):
        self.assertEqual(server.resolve_binds([]), ["127.0.0.1"])
        iface = lambda name: "192.168.0.23"
        self.assertEqual(server.resolve_binds(["192.168.0.9"], "wlan0", iface_fn=iface),
                         ["127.0.0.1", "192.168.0.9", "192.168.0.23"])
        self.assertEqual(server.resolve_binds(["127.0.0.1", "192.168.0.23"], "wlan0", iface_fn=iface),
                         ["127.0.0.1", "192.168.0.23"])
        with self.assertRaises(ValueError):
            server.resolve_binds(["0.0.0.0"])


class AuthUnitTest(unittest.TestCase):
    def setUp(self):
        self.auth = Auth(TOKEN, [EXTRA_HOST, "Console.Local"])

    def test_empty_token_rejected(self):
        with self.assertRaises(ValueError):
            Auth("")

    def test_host_only(self):
        self.assertEqual(host_only("192.168.0.5:18790"), "192.168.0.5")
        self.assertEqual(host_only("LOCALHOST:1"), "localhost")
        self.assertEqual(host_only("[::1]:18790"), "::1")
        self.assertEqual(host_only(""), "")

    def test_check_host(self):
        for ok in ("127.0.0.1:18790", "localhost", EXTRA_HOST + ":18790", "console.local:18790"):
            self.assertTrue(self.auth.check_host(ok), ok)
        for bad in ("evil.example", "evil.example:18790", "", "192.0.2.8"):
            self.assertFalse(self.auth.check_host(bad), bad)

    def test_check_origin(self):
        host = EXTRA_HOST + ":18790"
        self.assertTrue(self.auth.check_origin(None, host))
        self.assertTrue(self.auth.check_origin("http://" + host, host))
        self.assertFalse(self.auth.check_origin("http://evil.example", host))
        self.assertFalse(self.auth.check_origin("http://" + EXTRA_HOST, host))  # ポートが違う

    def test_authorized_by_cookie_and_bearer(self):
        self.assertTrue(self.auth.authorized({"Cookie": "a=1; %s=%s" % (auth_mod.COOKIE_NAME, TOKEN)}))
        self.assertTrue(self.auth.authorized({"Authorization": "Bearer " + TOKEN}))
        self.assertFalse(self.auth.authorized({"Cookie": "%s=wrong" % auth_mod.COOKIE_NAME}))
        self.assertFalse(self.auth.authorized({"Authorization": "Bearer wrong"}))
        self.assertFalse(self.auth.authorized({"Authorization": TOKEN}))
        self.assertFalse(self.auth.authorized({}))

    def test_lockout_after_repeated_failures(self):
        for i in range(auth_mod.FAIL_LIMIT):
            self.assertFalse(self.auth.verify("bad", "10.0.0.1", now=100.0 + i))
        self.assertTrue(self.auth.throttled("10.0.0.1", now=105.0))
        self.assertFalse(self.auth.verify(TOKEN, "10.0.0.1", now=105.0))  # ロック中は正解でも拒否
        self.assertTrue(self.auth.verify(TOKEN, "10.0.0.2", now=105.0))  # 他のクライアントは影響なし
        self.assertTrue(self.auth.verify(TOKEN, "10.0.0.1", now=105.0 + auth_mod.LOCKOUT_S + 1))

    def test_old_failures_expire(self):
        for i in range(auth_mod.FAIL_LIMIT - 1):
            self.auth.verify("bad", "10.0.0.3", now=100.0 + i)
        self.assertFalse(self.auth.verify("bad", "10.0.0.3", now=100.0 + auth_mod.FAIL_WINDOW_S + 10))
        self.assertFalse(self.auth.throttled("10.0.0.3", now=100.0 + auth_mod.FAIL_WINDOW_S + 11))

    def test_success_clears_failures(self):
        for i in range(auth_mod.FAIL_LIMIT - 1):
            self.auth.verify("bad", "10.0.0.4", now=100.0 + i)
        self.assertTrue(self.auth.verify(TOKEN, "10.0.0.4", now=110.0))
        self.assertFalse(self.auth.verify("bad", "10.0.0.4", now=111.0))
        self.assertFalse(self.auth.throttled("10.0.0.4", now=112.0))


class AuthHttpTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        helper = server.MockHelper()
        monitor = server.Monitor(helper)
        cls.auth = Auth(TOKEN, [EXTRA_HOST])
        handler = server.make_handler(helper, monitor, None, Settings(None), MockCameraCtl(), MockLidarCtl(), auth=cls.auth)
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        cls.port = cls.server.server_address[1]
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()

    def req(self, method, path, headers=None, body=None, host=None):
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        hdrs = {"Host": host or "127.0.0.1:%d" % self.port, **(headers or {})}
        conn.request(method, path, body=body, headers=hdrs, **{})
        res = conn.getresponse()
        data = res.read()
        out = (res.status, dict(res.getheaders()), data)
        conn.close()
        return out

    def cookie(self):
        return {"Cookie": "%s=%s" % (auth_mod.COOKIE_NAME, TOKEN)}

    def test_api_requires_auth(self):
        for path in ("/api/status", "/api/settings", "/api/buttons", "/camera/std", "/lidar", "/api/lidar/status"):
            self.assertEqual(self.req("GET", path)[0], 401, path)
        for path in ("/api/mode", "/api/settings", "/api/lidar/start", "/api/camera/start", "/api/monitor"):
            self.assertEqual(self.req("POST", path, body=b"{}")[0], 401, path)

    def test_index_without_auth_shows_login(self):
        status, _, body = self.req("GET", "/")
        self.assertEqual(status, 401)
        self.assertIn("/login", body.decode())
        self.assertNotIn("モード切り替え", body.decode())  # 本体の画面は返さない

    def test_login_form_without_token(self):
        status, _, body = self.req("GET", "/login")
        self.assertEqual(status, 200)
        self.assertIn("<form", body.decode())

    def test_login_bad_token(self):
        status, headers, _ = self.req("GET", "/login?token=nope")
        self.assertEqual(status, 401)
        self.assertNotIn("Set-Cookie", headers)

    def test_login_good_token_sets_cookie_then_works(self):
        status, headers, _ = self.req("GET", "/login?token=" + TOKEN)
        self.assertEqual(status, 302)
        self.assertEqual(headers["Location"], "/")
        cookie = headers["Set-Cookie"]
        self.assertIn("HttpOnly", cookie)
        self.assertIn("SameSite=Strict", cookie)
        status, _, body = self.req("GET", "/api/status", headers={"Cookie": cookie.split(";")[0]})
        self.assertEqual(status, 200)
        self.assertIn("jetson", json.loads(body))

    def test_index_with_cookie(self):
        status, _, body = self.req("GET", "/", headers=self.cookie())
        self.assertEqual(status, 200)
        self.assertIn("モード切り替え", body.decode())

    def test_bearer_works(self):
        self.assertEqual(self.req("GET", "/api/status", headers={"Authorization": "Bearer " + TOKEN})[0], 200)

    def test_wrong_cookie_rejected(self):
        self.assertEqual(self.req("GET", "/api/status", headers={"Cookie": "%s=zzz" % auth_mod.COOKIE_NAME})[0], 401)

    def test_unknown_host_rejected_even_with_cookie(self):
        self.assertEqual(self.req("GET", "/api/status", headers=self.cookie(), host="evil.example")[0], 403)
        self.assertEqual(self.req("GET", "/login?token=" + TOKEN, host="evil.example")[0], 403)

    def test_allowed_extra_host(self):
        self.assertEqual(self.req("GET", "/api/status", headers=self.cookie(), host=EXTRA_HOST + ":18790")[0], 200)

    def test_cross_origin_post_rejected(self):
        hdrs = {**self.cookie(), "Origin": "http://evil.example"}
        self.assertEqual(self.req("POST", "/api/mode", headers=hdrs, body=b'{"id": 1}')[0], 403)

    def test_same_origin_post_allowed(self):
        hdrs = {**self.cookie(), "Origin": "http://127.0.0.1:%d" % self.port, "Content-Type": "application/json"}
        self.assertEqual(self.req("POST", "/api/monitor", headers=hdrs, body=b'{"paused": false}')[0], 200)

    def test_mode_change_blocked_without_auth_and_helper_untouched(self):
        helper = mock.Mock()
        handler = server.make_handler(helper, server.Monitor(helper), None, Settings(None), None, None, auth=self.auth)
        srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        try:
            conn = http.client.HTTPConnection("127.0.0.1", srv.server_address[1], timeout=5)
            conn.request("POST", "/api/mode", body=b'{"id": 1}')
            self.assertEqual(conn.getresponse().status, 401)
            conn.close()
        finally:
            srv.shutdown()
            srv.server_close()
        helper.call.assert_not_called()


class NoAuthHttpTest(unittest.TestCase):
    """--no-auth: トークンは求めないが、Host・Origin の検査は残る。"""

    @classmethod
    def setUpClass(cls):
        helper = server.MockHelper()
        cls.auth = Auth("", [EXTRA_HOST], required=False)
        handler = server.make_handler(helper, server.Monitor(helper), None, Settings(None), MockCameraCtl(), MockLidarCtl(),
                                      auth=cls.auth)
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        cls.port = cls.server.server_address[1]
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()

    def req(self, method, path, headers=None, body=None, host=None):
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        conn.request(method, path, body=body, headers={"Host": host or "127.0.0.1:%d" % self.port, **(headers or {})})
        res = conn.getresponse()
        out = (res.status, dict(res.getheaders()), res.read())
        conn.close()
        return out

    def test_empty_token_allowed_only_when_not_required(self):
        self.assertFalse(Auth("", required=False).required)
        with self.assertRaises(ValueError):
            Auth("", required=True)

    def test_api_and_index_open_without_token(self):
        self.assertEqual(self.req("GET", "/api/status")[0], 200)
        status, _, body = self.req("GET", "/")
        self.assertEqual(status, 200)
        self.assertIn("モード切り替え", body.decode())

    def test_login_redirects_and_sets_no_cookie(self):
        status, headers, _ = self.req("GET", "/login?token=anything")
        self.assertEqual(status, 302)
        self.assertEqual(headers["Location"], "/")
        self.assertNotIn("Set-Cookie", headers)

    def test_unknown_host_still_rejected(self):
        self.assertEqual(self.req("GET", "/api/status", host="evil.example")[0], 403)
        self.assertEqual(self.req("GET", "/api/status", host=EXTRA_HOST + ":18790")[0], 200)

    def test_cross_origin_post_still_rejected(self):
        self.assertEqual(self.req("POST", "/api/monitor", headers={"Origin": "http://evil.example"}, body=b"{}")[0], 403)


class NoAuthCommandLineTest(unittest.TestCase):
    """起動オプションの組み合わせ（ハブを起動せず、引数の扱いだけを見る）。"""

    def test_no_auth_and_require_token_conflict(self):
        with mock.patch.object(sys, "argv", ["server.py", "--mock", "--no-auth", "--require-token"]):
            with self.assertRaises(SystemExit) as cm:
                server.main()
        self.assertEqual(cm.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
