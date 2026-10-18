"""web/index.html の画面構成の検査（T2b-1, T2b-2）。ブラウザは使わず、ソースの整合だけを見る。"""
import re
import unittest
from pathlib import Path

HTML = (Path(__file__).resolve().parent.parent / "web" / "index.html").read_text(encoding="utf-8")


def _groups():
    m = re.search(r"const GROUPS=\[(.*?)\];\nconst TABS", HTML, re.S)
    assert m, "GROUPS の定義が見つからない"
    return re.findall(r"\['(\w+)','([^']+)',\[([^\]]*)\]\]", m.group(1))


def _grouped_tabs():
    return [t for _, _, ts in _groups() for t in re.findall(r"'(\w+)'", ts)]


class NavStructureTest(unittest.TestCase):
    def test_five_main_nav_groups(self):
        self.assertEqual([g[1] for g in _groups()],
                         ["操作", "状態", "マップ・タスク", "センサ", "開発"])

    def test_every_screen_is_reachable_from_exactly_one_group(self):
        screens = re.findall(r'<div id="tab-(\w+)" data-tab="\1"', HTML)
        tabs = _grouped_tabs()
        self.assertEqual(sorted(screens), sorted(tabs))
        self.assertEqual(len(tabs), len(set(tabs)), "同じ画面が複数のグループにある")

    def test_every_screen_has_a_subtab_button(self):
        for t in _grouped_tabs():
            self.assertIn(f'id="tab-b-{t}"', HTML, t)

    def test_hash_is_still_the_tab_name(self):
        self.assertIn("history.replaceState(null,'','#'+t)", HTML)
        self.assertIn("setTab(location.hash.slice(1)||'ops')", HTML)

    def test_settings_and_dds_live_under_dev(self):
        dev = [g for g in _groups() if g[0] == "dev"][0]
        self.assertEqual(re.findall(r"'(\w+)'", dev[2]), ["dds", "settings"])

    def test_phone_layout_foundations_kept(self):
        self.assertIn('name="viewport"', HTML)
        self.assertIn("@media (max-width:899px)", HTML)
        self.assertIn("@media (pointer:coarse)", HTML)

    def test_status_is_shown_as_labelled_cards_not_sentences(self):
        # モード・FSM・標準モーションサービス・デバッグモード・脚・腕モータ・バッテリー・最終取得・レイテンシを、項目名つきのカードで分けて出す
        for label in ("現在のモード", "FSM ID", "標準モーションサービス", "デバッグモード", "脚", "腕モータ",
                      "バッテリー", "最終取得", "レイテンシ"):
            self.assertIn("<h3>%s" % label, HTML, label)
        for el in ("st-svc", "st-dbg", "st-legs", "st-arm", "st-last", "st-lat", "st-fsm", "bat-soc"):
            self.assertIn('id="%s"' % el, HTML, el)
        self.assertNotIn("標準モーションサービスが解放されています", HTML)

    def test_two_round_sticks_side_by_side_even_on_phones(self):
        # 左=前後左右、右=旋回。どちらも丸いスティックで、狭い画面でも折り返さず横に並べる
        left, right = HTML.index('id="pad-move"'), HTML.index('id="pad-turn"')
        self.assertLess(left, right)
        self.assertNotIn("pad wide", HTML)
        self.assertIn(".pads{display:flex;gap:16px;flex-wrap:nowrap", HTML)
        self.assertIn("width:min(170px,40vw);aspect-ratio:1", HTML)


if __name__ == "__main__":
    unittest.main()
