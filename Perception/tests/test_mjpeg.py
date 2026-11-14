import threading
import time
import unittest
import urllib.error
import urllib.request

from common.mjpeg import MjpegServer


class TestMjpegServer(unittest.TestCase):
    def setUp(self) -> None:
        self.server = MjpegServer(0, host="127.0.0.1")
        self.base = f"http://127.0.0.1:{self.server.port}"

    def tearDown(self) -> None:
        self.server.close()

    def test_page_and_snapshot(self) -> None:
        with urllib.request.urlopen(self.base + "/") as r:
            self.assertIn(b'src="/stream"', r.read())
        with self.assertRaises(urllib.error.HTTPError) as cm:
            urllib.request.urlopen(self.base + "/snapshot.jpg")
        self.assertEqual(cm.exception.code, 503)  # まだ 1 枚も無い
        self.server.publish(b"\xff\xd8jpeg1")
        with urllib.request.urlopen(self.base + "/snapshot.jpg") as r:
            self.assertEqual(r.read(), b"\xff\xd8jpeg1")

    def test_stream_counts_viewer_and_sends_latest(self) -> None:
        r = urllib.request.urlopen(self.base + "/stream")
        self.assertTrue(r.headers["Content-Type"].startswith("multipart/x-mixed-replace"))
        for _ in range(50):  # 見ている人として数えられるまで待つ
            if self.server.viewers == 1:
                break
            time.sleep(0.02)
        self.assertEqual(self.server.viewers, 1)
        threading.Timer(0.1, self.server.publish, args=(b"\xff\xd8jpeg2",)).start()
        head = r.read(len(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 7\r\n\r\n"))
        self.assertIn(b"Content-Length: 7", head)
        self.assertEqual(r.read(7), b"\xff\xd8jpeg2")
        r.close()

    def test_wait_next_times_out_without_new_frame(self) -> None:
        self.server.publish(b"a")
        seq, _ = self.server.wait_next(-1, timeout=0.1)
        self.assertIsNone(self.server.wait_next(seq, timeout=0.1))


if __name__ == "__main__":
    unittest.main()
