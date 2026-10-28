"""各自の学習の設定（button_press_rl/profiles.py、configs/profiles/）のテスト。"""

from __future__ import annotations

import dataclasses
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import yaml

from button_press_rl.profiles import ENV_KEYS, PROFILE_DIR, ProfileError, apply_profile, load_profile
from button_press_rl.rewards import RewardWeights


@dataclasses.dataclass
class _Rand:
    """DomainRandCfg の代わり（Isaac Lab 無しで試すため。項目と既定の値は configs/profiles/default.yaml と同じ）。"""

    offset_margin: float = 0.2
    kp_scale: tuple[float, float] = (0.9, 1.1)
    armature: tuple[float, float] = (0.001, 0.03)
    command_delay_prob: float = 0.5
    q_noise: float = 0.0005
    dq_noise: float = 0.01
    init_q_noise: float = 0.03
    button_stiffness: tuple[float, float] = (600.0, 1500.0)
    button_travel: tuple[float, float] = (0.0025, 0.0045)


def _cfg() -> SimpleNamespace:
    return SimpleNamespace(rewards=RewardWeights(), rand=_Rand(), episode_length_s=8.0, terminate_on_failure=False,
                           force_abort_scale=2.0, tip_gap_m=0.004, tip_lateral_m=0.002)


def _write(d: str, data: dict) -> str:
    p = Path(d) / "p.yaml"
    p.write_text(yaml.safe_dump(data, allow_unicode=True))
    return str(p)


class TestProfiles(unittest.TestCase):
    def test_default_matches_code(self) -> None:
        """default.yaml の値は、コードの既定の値と同じ（どちらかだけを変えると、ずれる）。"""
        prof = load_profile("default")
        self.assertEqual(prof["rewards"], RewardWeights().to_dict())
        rand = {f.name: getattr(_Rand(), f.name) for f in dataclasses.fields(_Rand)}
        self.assertEqual({k: tuple(v) if isinstance(v, list) else v for k, v in prof["rand"].items()}, rand)
        self.assertEqual(set(prof["env"]), set(ENV_KEYS))
        c = _cfg()
        self.assertEqual({k: getattr(c, k) for k in ENV_KEYS}, prof["env"])

    def test_partial_override(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            c = _cfg()
            apply_profile(c, _write(d, {"rewards": {"reach": 5.0}, "rand": {"kp_scale": [0.8, 1.2]},
                                        "env": {"episode_length_s": 10}}))
        self.assertEqual(c.rewards.reach, 5.0)
        self.assertEqual(c.rewards.depth, RewardWeights().depth)  # 書かなかった項目は既定のまま
        self.assertEqual(c.rand.kp_scale, (0.8, 1.2))
        self.assertEqual(c.episode_length_s, 10.0)
        self.assertIsInstance(c.episode_length_s, float)

    def test_typo_and_bad_values_are_errors(self) -> None:
        bad = [
            {"rewards": {"reech": 1.0}},                 # 打ち間違い
            {"reward": {"reach": 1.0}},                  # 節の名前の間違い
            {"rand": {"kp_scale": 1.0}},                 # 並びのはずが数
            {"rewards": {"reach": "big"}},               # 数のはずが文字
            {"env": {"terminate_on_failure": 1}},        # true / false のはずが数
            {"env": {"decimation": 5}},                  # 変えてはいけない項目
        ]
        for data in bad:
            with self.subTest(data=data), tempfile.TemporaryDirectory() as d:
                with self.assertRaises(ProfileError):
                    apply_profile(_cfg(), _write(d, data))

    def test_missing_profile(self) -> None:
        with self.assertRaises(ProfileError):
            load_profile("no_such_profile_xyz")
        self.assertTrue((PROFILE_DIR / "default.yaml").exists())


if __name__ == "__main__":
    unittest.main()
