import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from g1console.camera_proc import SEP, MockCameraCtl, SshCameraCtl, parse_holders, parse_status, summarize  # noqa: E402

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


class SshCallTest(unittest.TestCase):
    def _ctl(self, out):
        ctl = SshCameraCtl(lambda: ("u@h", ""))
        ctl.scripts = []
        ctl._run = lambda script: (ctl.scripts.append(script), out)[1]
        return ctl

    def test_start_is_one_ssh_split_in_three(self):
        out = "[camera] 起動しました (pid=5)\n%s\n[camera] 起動中 (pid=5)\n%s\n/dev/video0\t5\tu\tpython -u camera_stream.py\n" % (SEP, SEP)
        ctl = self._ctl(out)
        r = ctl.call("start")
        self.assertEqual(len(ctl.scripts), 1)
        self.assertTrue(r["running"] and r["pid"] == 5 and "起動しました" in r["message"])
        self.assertEqual(len(r["holders"]), 1)

    def test_status_has_no_message(self):
        ctl = self._ctl("\n%s\n[camera] 停止中\n%s\n" % (SEP, SEP))
        r = ctl.call("status")
        self.assertFalse(r["running"])
        self.assertEqual(r["message"], "")
        self.assertNotIn("stop", ctl.scripts[0])

    def test_own_requires_python_camera_stream(self):
        h = parse_holders("/dev/video0\t9\tu\tvim camera_stream.py\n/dev/video0\t8\tu\t/usr/bin/python3 -u /h/camera_stream.py --camera x\n")
        self.assertEqual([x["own"] for x in h], [False, True])


if __name__ == "__main__":
    unittest.main()
