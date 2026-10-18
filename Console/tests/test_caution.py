"""Nav2 の起動・停止タスクと、動作未確認の注意ポップアップ（段階 7、R-20）。ロボットには触れない。
T7-1…T7-6 は docs/TEST_SPEC.md の番号に対応する。"""
import json
import os
import re
import stat
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from g1console.modes import FEATURES  # noqa: E402
from g1console.tasks import load_definitions  # noqa: E402

CONSOLE = Path(__file__).resolve().parent.parent
SCRIPT = CONSOLE / "jetson" / "nav2_down.sh"
HTML = (CONSOLE / "web" / "index.html").read_text(encoding="utf-8")
NAV2 = ("nav2-up", "nav2-up-dry", "nav2-down", "nav2-down-dry")
ROBOT_FREE = {"run-tests", "sample-countdown"}  # ロボットにも Jetson にも触れないので verified=true にしてある


def shipped():
    return load_definitions(CONSOLE / "tasks", CONSOLE)


class TaskDefinitionTest(unittest.TestCase):
    def _load(self, raw):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "x.json").write_text(json.dumps(raw), encoding="utf-8")
            return load_definitions(d, d)["x"]

    def test_t7_1_defaults_are_unverified_without_caution(self):
        d = self._load({"argv": ["echo"]})
        self.assertIs(d.verified, False)
        self.assertEqual(d.caution, ())

    def test_t7_1_verified_and_caution_are_validated(self):
        ok = self._load({"argv": ["echo"], "verified": True, "caution": ["a", "b"]})
        self.assertEqual((ok.verified, ok.caution), (True, ("a", "b")))
        bad = [{"verified": "yes"}, {"verified": 1}, {"caution": "text"}, {"caution": [1]}, {"caution": [""]},
               {"caution": ["x"] * 9}, {"caution": ["x" * 201]}]
        for extra in bad:
            with self.subTest(extra):
                with self.assertRaises(ValueError):
                    self._load({"argv": ["echo"], **extra})

    def test_t7_1_view_carries_verified_and_caution(self):
        from g1console.tasks import TaskRunner
        defs = shipped()
        view = {t["name"]: t for t in TaskRunner(defs, CONSOLE).list()["tasks"]}
        self.assertEqual(view["nav2-up"]["verified"], False)
        self.assertTrue(view["nav2-up"]["caution"])
        self.assertEqual(view["run-tests"]["verified"], True)


class ShippedNav2TasksTest(unittest.TestCase):
    def test_t7_2_four_nav2_tasks_exist_in_one_exclusive_group(self):
        defs = shipped()
        self.assertTrue(set(NAV2) <= set(defs))
        self.assertEqual({defs[n].group for n in NAV2}, {"nav2"})

    def test_t7_2_nav2_tasks_are_unverified_and_carry_caution(self):
        for n in NAV2:
            with self.subTest(n):
                self.assertIs(shipped()[n].verified, False)
                self.assertTrue(shipped()[n].caution)

    def test_t7_2_only_robot_free_tasks_are_marked_verified(self):
        self.assertEqual({n for n, d in shipped().items() if d.verified}, ROBOT_FREE)

    def test_t7_2_nav2_tasks_never_arm_or_enable(self):
        """発進ゲート（G1_ARM）・走行許可（enable_navigation）・sudo・姿勢検査の無効化は、定義に現れない。"""
        for n in NAV2:
            raw = (CONSOLE / "tasks" / (n + ".json")).read_text(encoding="utf-8")
            argv = shipped()[n].argv
            with self.subTest(n):
                for word in ("G1_ARM", "enable_navigation", "sudo"):
                    self.assertNotIn(word, " ".join(argv))
                for flag in ("--enable", "--force-posture", "--arm"):
                    self.assertNotIn(flag, argv)
                self.assertIn("行わない" if n == "nav2-up" else "", raw)

    def test_t7_2_commands_match_the_security_team_procedure(self):
        defs = shipped()
        self.assertEqual(defs["nav2-up"].argv, ("/home/unitree/g1_nav2/g1up.sh",))
        self.assertEqual(defs["nav2-up-dry"].argv, ("/home/unitree/g1_nav2/g1up.sh", "--dry-run"))
        self.assertEqual(defs["nav2-down"].argv, ("bash", "jetson/nav2_down.sh"))
        self.assertEqual(defs["nav2-down-dry"].argv, ("bash", "jetson/nav2_down.sh", "--dry-run"))
        self.assertTrue((CONSOLE / defs["nav2-down"].argv[1]).is_file())


class Nav2DownScriptTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.bin = self.dir / "bin"
        self.bin.mkdir()
        self.log = self.dir / "calls.log"
        self.env_file = self.dir / "g1-sdk-bridge"
        self.tools = self.dir / "tools"
        self.tools.mkdir()
        (self.tools / "send_slam_api.py").write_text(
            "import sys\nopen(%r, 'a').write('slam %%s\\n' %% sys.argv[1])\n" % str(self.log), encoding="utf-8")
        self._stub("pkill", 'echo "pkill $*" >> %s' % self.log)
        self._stub("sleep", "true")
        # ps は「ブリッジが --arm つきで動いている」ときだけその行を返す（STUB_ARM=1）
        self._stub("ps", 'if [ "${STUB_ARM:-0}" = 1 ]; then echo "python3 g1_sdk_bridge_real_server.py --arm"; fi')

    def tearDown(self):
        self.tmp.cleanup()

    def _stub(self, name, body):
        p = self.bin / name
        p.write_text("#!/bin/bash\n%s\n" % body, encoding="utf-8")
        p.chmod(p.stat().st_mode | stat.S_IXUSR)

    def run_script(self, *args, gate_file="G1_ARM=\n", arm_process=False):
        self.env_file.write_text(gate_file, encoding="utf-8")
        env = {**os.environ, "PATH": "%s:%s" % (self.bin, os.environ["PATH"]), "G1_BRIDGE_ENV": str(self.env_file),
               "G1_NAV2_TOOLS": str(self.tools), "G1_PYTHON": sys.executable, "STUB_ARM": "1" if arm_process else "0"}
        p = subprocess.run(["bash", str(SCRIPT), *args], env=env, capture_output=True, text=True, timeout=30)
        calls = self.log.read_text(encoding="utf-8").splitlines() if self.log.exists() else []
        return p.returncode, p.stdout + p.stderr, calls

    def test_t7_3_syntax_and_no_executable_sudo_or_gate_write(self):
        self.assertEqual(subprocess.run(["bash", "-n", str(SCRIPT)]).returncode, 0)
        for no, line in enumerate(SCRIPT.read_text(encoding="utf-8").splitlines(), 1):
            code = line.strip()
            if code.startswith("#") or code.startswith("echo"):
                continue
            with self.subTest(line=no):
                for word in ("sudo", "systemctl", "sed -i", "G1_ARM=--arm >", "> \"$BRIDGE_ENV\""):
                    self.assertNotIn(word, code)

    def test_t7_3_dry_run_stops_nothing(self):
        code, out, calls = self.run_script("--dry-run")
        self.assertEqual((code, calls), (0, []))
        self.assertIn("[dry] pkill", out)
        self.assertIn("send_slam_api.py 1901", out)

    def test_t7_3_stops_in_runbook_order_when_gate_is_closed(self):
        code, out, calls = self.run_script()
        self.assertEqual(code, 0, out)
        self.assertEqual(calls[0], "pkill -f ros2 bag recor[d]")  # ② 記録が先
        self.assertEqual(calls[-1], "slam 1901")  # ③ 内蔵 SLAM は最後
        self.assertTrue(any("ros2 launch g1_navigatio[n]" in c for c in calls))
        self.assertLess(max(i for i, c in enumerate(calls) if c.startswith("pkill")), calls.index("slam 1901"))
        self.assertIn("sudo systemctl stop g1-sdk-bridge", out)  # ④ は人が行う（表示だけ）

    def test_t7_3_gate_open_by_file_still_stops_but_fails_loudly(self):
        before = "G1_ARM=--arm\n"
        code, out, calls = self.run_script(gate_file=before)
        self.assertEqual(code, 3, out)
        self.assertIn("発進ゲートが開いている", out)
        self.assertIn("slam 1901", calls)
        self.assertEqual(self.env_file.read_text(encoding="utf-8"), before)  # ゲートの設定は書き換えない

    def test_t7_3_gate_open_by_running_bridge_is_detected(self):
        code, out, _ = self.run_script(arm_process=True)
        self.assertEqual(code, 3, out)

    def test_t7_3_unknown_argument_is_refused(self):
        code, _, calls = self.run_script("--force")
        self.assertEqual((code, calls), (2, []))


class WebCautionTest(unittest.TestCase):
    def _func(self, name):
        m = re.search(r"(?:async )?function %s\(.*?\n\}\n" % name, HTML, re.S)
        self.assertTrue(m, name)
        return m.group(0)

    def test_t7_4_dialog_has_the_parts_the_script_uses(self):
        self.assertIn('<dialog id="caution"', HTML)
        for i in ("ca-title", "ca-unver", "ca-note", "ca-lines", "ca-check", "ca-checkbox", "ca-ok", "ca-cancel"):
            self.assertIn('id="%s"' % i, HTML, i)
        self.assertIn('id="ca-cancel" type="submit" autofocus', HTML)  # 既定の選択は「やめる」

    def test_t7_4_dialog_text_is_never_parsed_as_html(self):
        body = self._func("askCaution") + self._func("guard")
        self.assertNotIn("innerHTML", body)
        self.assertNotIn("insertAdjacentHTML", body)
        self.assertIn("textContent", body)

    def test_t7_4_checkbox_gates_the_ok_button(self):
        body = self._func("askCaution")
        self.assertIn("ok.disabled=!!o.needCheck", body)
        self.assertIn("ok.disabled=!e.target.checked", body)
        self.assertIn("returnValue==='ok'", body)  # Esc・「やめる」は実行にならない

    def test_t7_4_unknown_feature_counts_as_unverified(self):
        self.assertIn("unverified=!f||!f.verified", self._func("guard"))

    def test_t7_5_every_robot_action_button_asks_first(self):
        wired = {"setMode": "guard('mode'", "speak": "guard('audio'", "setLed": "guard('audio'",
                 "camProc": "guard('camera'", "ldProc": "guard('lidar'", "moClaim": "guard('move'",
                 "moAction": "guard('action'"}
        for fn, call in wired.items():
            with self.subTest(fn):
                body = self._func(fn)
                self.assertIn(call, body)
                self.assertLess(body.index(call), body.index("fetch(") if "fetch(" in body else len(body))
        self.assertIn("guard('topics'", HTML)

    def test_t7_5_task_buttons_ask_for_unverified_or_cautioned_tasks(self):
        body = self._func("tkAct")
        self.assertIn("!t.verified||t.caution.length", body)
        self.assertIn("needCheck:t.caution.length>0", body)
        self.assertLess(body.index("askCaution"), body.index("tkApi('/api/tasks/'"))
        self.assertIn("tag('実機未確認','unver')", self._func("tkCard"))

    def test_t7_6_guarded_features_exist_and_are_unverified(self):
        by_id = {f["id"]: f for f in FEATURES}
        guarded = set(re.findall(r"guard\('(\w+)'", HTML))
        self.assertTrue(guarded <= set(by_id), guarded - set(by_id))
        for fid in guarded | {"nav2ctl"}:
            self.assertIs(by_id[fid]["verified"], False, fid)
            self.assertTrue(by_id[fid]["implemented"], fid)


if __name__ == "__main__":
    unittest.main()
