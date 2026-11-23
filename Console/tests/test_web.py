"""新UIの到達性と、既存の制御要素を失わないことの検査。ブラウザは使わない。"""
import re
import unittest
from html.parser import HTMLParser
from web_source import WEB, source

HTML = (WEB / "index.html").read_text(encoding="utf-8")
JS = source()


class Elements(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = []
        self.screens = []
        self.scripts = []

    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        if "id" in a:
            self.ids.append(a["id"])
        if "data-tab" in a:
            self.screens.append(a["data-tab"])
        if tag == "script" and "src" in a:
            self.scripts.append(a["src"])

    handle_startendtag = handle_starttag


class NavStructureTest(unittest.TestCase):
    def setUp(self):
        self.elements = Elements()
        self.elements.feed(HTML)

    def test_twelve_screens_and_a_single_id_for_every_control(self):
        expected = {"home", "ops", "motion", "action", "state", "joints", "audio", "camera", "map", "tasks", "dds", "settings"}
        self.assertEqual(set(self.elements.screens), expected)
        self.assertEqual(len(self.elements.ids), len(set(self.elements.ids)))
        for screen in expected:
            self.assertIn("tab-b-" + screen, self.elements.ids)

    def test_every_original_control_is_preserved(self):
        old = Elements()
        old.feed((WEB / "legacy.html").read_text(encoding="utf-8"))
        self.assertFalse(set(old.ids) - set(self.elements.ids))

    def test_routes_are_keyboard_focusable(self):
        for screen in self.elements.screens:
            self.assertIn('id="tab-%s" data-tab="%s" tabindex="-1"' % (screen, screen), HTML)
        self.assertIn("window.addEventListener('popstate'", JS)
        self.assertIn("window.addEventListener('hashchange'", JS)
        self.assertIn("history.pushState", JS)
        self.assertIn("||'home'", JS)

    def test_full_screen_status_and_preferences_have_a_home(self):
        state = HTML.index('id="tab-state"')
        status = HTML.index('id="statusbar"')
        joints = HTML.index('id="tab-joints"')
        self.assertLess(state, status)
        self.assertLess(status, joints)
        self.assertGreater(HTML.index('id="mon-toggle"'), HTML.index('id="tab-settings"'))

    def test_scripts_are_local_files_and_legacy_remains_available(self):
        self.assertEqual(self.elements.scripts, ["/ui-core.js", "/mobile.js"])
        for name in self.elements.scripts:
            self.assertTrue((WEB / name.lstrip("/")).is_file())
        self.assertIn('href="/legacy.html"', HTML)

    def test_actual_features_not_mock_placeholders(self):
        for i in ("st-rc", "pending", "planned-modes", "speed-btns", "vol-restore", "led-set", "tts-go", "cp-stop", "ld-density", "mp-save", "tk-log", "tp-sub", "tp-pub", "hub-stop"):
            self.assertIn(i, self.elements.ids)
        for sample in ("data-voice-prompt", "task-percent", "setting-domain", "data-destination"):
            self.assertNotIn(sample, HTML)

    def test_phone_inputs_and_simultaneous_axes(self):
        for name in ("forward", "backward", "left", "right", "turn-left", "turn-right"):
            self.assertIn('data-drive="%s"' % name, HTML)
        self.assertIn('name="viewport"', HTML)
        self.assertIn('id="pad-move"', HTML)
        self.assertIn('id="pad-turn"', HTML)
        self.assertIn("pointercancel", JS)
        self.assertIn("lostpointercapture", JS)


if __name__ == "__main__":
    unittest.main()
