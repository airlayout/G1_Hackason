import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from g1console.camera_proc import MockCameraCtl, parse_holders, parse_status, summarize  # noqa: E402

HOLDERS = "/dev/video4\t9113\troot\t/unitree/module/video_hub_pc4/videohub_pc4 /dev/video4\n/dev/video0\t77\tunitree\tpython camera_stream.py\n"


class CameraProcTest(unittest.TestCase):
    def test_parse_holders(self):
        h = parse_holders(HOLDERS)
        self.assertEqual([x["pid"] for x in h], [9113, 77])
        self.assertEqual([x["own"] for x in h], [False, True])

    def test_parse_status(self):
        self.assertEqual(parse_status("[camera] 起動中 (pid=42)\n"), {"running": True, "pid": 42})
        self.assertEqual(parse_status("[camera] 停止中\n"), {"running": False, "pid": None})

    def test_foreign_only_others(self):
        s = summarize("[camera] 停止中", HOLDERS)
        self.assertEqual([x["pid"] for x in s["foreign"]], [9113])

    def test_mock_start_stop(self):
        m = MockCameraCtl()
        self.assertTrue(m.call("start")["running"])
        self.assertFalse(m.call("stop")["running"])
        with self.assertRaises(ValueError):
            m.call("restart")


if __name__ == "__main__":
    unittest.main()
