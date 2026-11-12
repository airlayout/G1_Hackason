"""11-12 台本: --staged --script を付けた door_run_stock.main() を、偽の機体・偽の FAST-LIO の受け口で実時間で回す（約 25 秒）。

  DOOR_POLICY=<書き出した npz> python -m pytest G1_Hackason/Door/real/tests/test_door_script_loop.py -q

偽の機体の腕は動かない（角度は初めの姿勢のまま）ので、台本は 2b（構える）で時間切れになる。確かめるのは輪のつなぎ:
段 0 を飛ばして始まる・方策の歩きが出ない・左腕だけ台本の目標が出る（今の角度 ±δ の内）・時間切れで立ったまま止まる。
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import door_frame as dfr  # noqa: E402
import door_script as dscr  # noqa: E402
import test_door_staged_loop as tl  # noqa: E402
from test_door_real import needs_policy  # noqa: E402
from test_door_run_stock import POLICY  # noqa: E402


@needs_policy
def test_script_start1_skips_stage0_and_script_drives_left_arm_only(monkeypatch, tmp_path):
    import door_run_stock as drs

    params = tmp_path / "script.json"
    params.write_text(json.dumps({"raise_max_s": 3.0}), encoding="utf-8")
    st = dscr.script_stance(dscr.load_params(str(params)), 0.94, 0.064, 0.99, True).target
    orig = tl.FakeLidarSource.__init__

    def init(self, loco):  # 前の回の REF が残っている形: 初めから姿勢を返し、機体は構えの位置に立っている
        orig(self, loco)
        self.ref_t = time.monotonic() - 10.0
        self.true = dfr.PlanarPose(st.x, st.y, st.yaw)

    monkeypatch.setattr(tl.FakeLidarSource, "__init__", init)
    z, lidar, staged, seen = tl._run_staged(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (17.0, "r"), (21.0, "q")],
                                            extra=("--script", str(params), "--script-start", "1"))
    assert lidar.refs == []                      # 段 0 を飛ばした（REF を送り直していない）
    assert seen[0] == 1 and 0 not in seen and 2 in seen
    phases = np.array([drs.PHASES[int(v)] for v in z["phase"]])
    door = phases == "DOOR"
    rows = z["script"][door]
    in_script = np.isfinite(rows[:, 0])
    assert in_script.any()
    subs = {dscr.SUBS[int(v)] for v in rows[in_script, 0]}
    assert {"2a", "2b"} <= subs and "2c" not in subs   # 腕が動かないので構えで時間切れ
    assert not z["vel_want"][door][:, 0].any()         # 構えの位置に居るので歩かない（方策の歩きも出ない）
    arm = z["arm_sent"][door]
    p = dc.load_params(POLICY)
    q_arm = p.q_default[15:29]
    delta = np.asarray(p.arm_guard_delta if p.arm_guard_delta is not None else dc.ARM_GUARD[0], dtype=float)
    assert np.abs(arm[:, :7] - q_arm[:7]).max() > 0.02                    # 左腕は台本の目標へ動かそうとしている
    assert (np.abs(arm - q_arm) <= delta + 0.06).all()                    # 今の角度 ±δ（＋可動域の余裕）の内
    assert np.abs(arm[:, 7:] - arm[0, 7:]).max() < 1e-9                   # 右腕は動かさない
    k_post = int(np.argmax(phases == "POST"))
    assert phases[k_post - 1] == "DOOR" and staged[k_post - 1, 2] == 3    # 腕の出どころ 3＝台本
