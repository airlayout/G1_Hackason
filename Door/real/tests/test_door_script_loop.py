"""11-12 台本: --staged --script を付けた door_run_stock.main() を、偽の機体・偽の FAST-LIO の受け口で実時間で回す。

  DOOR_POLICY=<書き出した npz> python -m pytest G1_Hackason/Door/real/tests/test_door_script_loop.py -q

偽の機体の腕は動かない（角度は初めの姿勢のまま）ので、台本は 2b（構える）で止まる（手が進まない・時間切れ）。確かめるのは輪のつなぎ:
段 0 を飛ばして始まる・方策の歩きが出ない・左腕だけ台本の目標が出る（今の角度 ±δ の内）・止まったら立ったまま。
11-12 夜: 起こしたまま a → d を打ち直すと定数を読み直す・門番を通っていない定数は断る・d0 で段 0 から・脚が揺れている間は d を待つ。
"""
from __future__ import annotations

import contextlib
import io
import json
import sys
import time
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import door_frame as dfr  # noqa: E402
import door_script as dscr  # noqa: E402
import script_gate as sg  # noqa: E402
import test_door_staged_loop as tl  # noqa: E402
from test_door_real import needs_policy  # noqa: E402
from test_door_run_stock import POLICY  # noqa: E402

DOOR_S = "0.94,2.05,0.99,0.064"


def _params(tmp_path, gate: bool, **over) -> Path:
    src = json.loads((HERE / "script_params.json").read_text(encoding="utf-8"))
    src.update(**over)  # 偽の機体の腕は動かないので、2b は「手が 1.2 秒進まない」で止まる
    f = tmp_path / "script.json"
    f.write_text(json.dumps(src), encoding="utf-8")
    if gate:
        with contextlib.redirect_stdout(io.StringIO()):
            assert sg.main([str(f), "--door", DOOR_S, "--pelvis-z", "0.75"]) == 0
    return f


def _ref_already(monkeypatch, params: Path) -> None:
    """前の回の REF が残っている形: 初めから姿勢を返し、機体は構えの位置に立っている"""
    st = dscr.script_stance(dscr.load_params(str(params)), 0.94, 0.064, 0.99, True).target
    orig = tl.FakeLidarSource.__init__

    def init(self, loco):
        orig(self, loco)
        self.ref_t = time.monotonic() - 10.0
        self.true = dfr.PlanarPose(st.x, st.y, st.yaw)

    monkeypatch.setattr(tl.FakeLidarSource, "__init__", init)


def _spans(z) -> list:
    import ana_script_run as asr
    return asr.door_spans(z["phase"])


@needs_policy
def test_script_start1_skips_stage0_and_script_drives_left_arm_only(monkeypatch, tmp_path):
    import door_run_stock as drs

    params = _params(tmp_path, gate=False, raise_max_s=3.0)
    _ref_already(monkeypatch, params)
    z, lidar, staged, seen = tl._run_staged(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (17.0, "r"), (21.0, "q")],
                                            extra=("--script", str(params), "--script-start", "1", "--script-gate", "off"))
    assert lidar.refs == []                      # 段 0 を飛ばした（REF を送り直していない）
    assert seen[0] == 1 and 0 not in seen and 2 in seen
    phases = np.array([drs.PHASES[int(v)] for v in z["phase"]])
    door = phases == "DOOR"
    rows = z["script"][door]
    in_script = np.isfinite(rows[:, 0])
    assert in_script.any()
    subs = {dscr.SUBS[int(v)] for v in rows[in_script, 0]}
    assert {"2a", "2b"} <= subs and "2c" not in subs   # 腕が動かないので構えで止まる
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
    # 11-12 夜: 台本の段 1 以降は学習した方策を回さない（記録の方策の出力が NaN）
    assert np.isnan(z["a_pol"][door & np.isfinite(staged[:, 0]) & (staged[:, 0] >= 1)]).all()


@needs_policy
def test_gate_on_refuses_to_start_without_ok(monkeypatch, tmp_path):
    params = _params(tmp_path, gate=False)
    _ref_already(monkeypatch, params)
    with pytest.raises(SystemExit, match="門番"):
        tl._run_staged(monkeypatch, tmp_path, [(0.3, "q")], extra=("--script", str(params), "--script-start", "1"))


@needs_policy
def test_rearm_and_d_reloads_params_and_refuses_unchecked_edit(monkeypatch, tmp_path, capsys):
    """止まった後に a（構え直す）→ d を打つと、走らせ役を起こし直さずに定数を読み直す。門番を通っていない直しは断る"""
    import door_run_stock as drs

    params = _params(tmp_path, gate=True, note="1 回目")
    _ref_already(monkeypatch, params)

    def edit(note: str, gate: bool):
        def f():
            src = json.loads(params.read_text(encoding="utf-8"))
            src["note"] = note
            params.write_text(json.dumps(src), encoding="utf-8")
            if gate:
                with contextlib.redirect_stdout(io.StringIO()):
                    sg.main([str(params), "--door", DOOR_S, "--pelvis-z", "0.75"])
        return f

    rows = [(0.3, "a"), (3.8, "d"),                     # 1 回目（2b で止まる・約 5 秒）
            (11.0, edit("2 回目", True)), (12.0, "a"), (12.1, "d"),   # 門番を通した直し → 読み直して 2 回目（a ; d を続けて打つ）
            (23.0, edit("3 回目", False)), (24.0, "a"), (28.0, "d"),  # 門番を通していない直し → 断る
            (31.0, "r"), (35.0, "q")]
    z, lidar, staged, seen = tl._run_staged(monkeypatch, tmp_path, rows, extra=("--script", str(params), "--script-start", "1"),
                                            run_seconds="60")
    out = capsys.readouterr().out
    assert "台本の定数を読み直した" in out and "2 回目" in out
    assert "d を受け付けない: 台本の定数: 定数か寸法のファイルが門番を通った後に変わった" in out
    assert len(_spans(z)) == 2                          # 2 回走り、3 回目は始まらなかった
    assert lidar.refs == []
    assert out.count("[ana]") >= 2 and "止まった理由" in out  # 回ごとの要約が出る
    phases = [drs.PHASES[int(v)] for v in z["phase"]]
    assert phases[-1] in ("WAIT", "DAMP") or "ARMS_OUT" in phases


@needs_policy
def test_stage0_runs_once_then_is_skipped_and_d0_forces_it(monkeypatch, tmp_path, capsys):
    """--script-start 0: 1 回目は段 0（REF を送る）→ 2 回目の d は段 0 を飛ばす → d0 は段 0 から"""
    params = _params(tmp_path, gate=True)
    # 1 回目は段 0・段 1（歩き出しの遅れのある偽の脚で約 20〜26 秒）＋ 2b。a と d は続けて打つ（構えている間の d は待つ）
    rows = [(0.3, "a"), (3.8, "d"), (34.0, "a"), (34.1, "d"), (46.0, "a"), (46.1, "d0"), (62.0, "r"), (66.0, "q")]
    z, lidar, staged, seen = tl._run_staged(monkeypatch, tmp_path, rows, extra=("--script", str(params)), run_seconds="80")
    spans = _spans(z)
    assert len(spans) == 3, spans
    first = [int(staged[a, 0]) for a, _ in spans]
    assert first == [0, 1, 0], first                   # 段 0 → 段 1 から → 段 0 から
    assert len(lidar.refs) == 2                         # 段 0 の REF は 1 回目だけ（2 回目は飛ばした）
    # 3 回目（d0）は構えの位置（扉の 0.4 m 手前）で段 0 を始めたので、段 0 の立ち位置（0.8〜1.3 m）の外で正しく断る
    assert "段 0 の立ち位置の外" in capsys.readouterr().out


@needs_policy
def test_d_waits_while_a_leg_is_shaking_then_starts(monkeypatch, tmp_path, capsys):
    """11-12 の機体: 右足首 roll が断続的に揺れて d を十数回断られた → 揺れている間は待ち、止まったら始める"""
    params = _params(tmp_path, gate=True)
    _ref_already(monkeypatch, params)
    t0 = [time.monotonic()]
    orig_state = tl._State

    class ShakyState(orig_state):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            el = time.monotonic() - t0[0]
            if 3.5 < el < 7.0 and int(el * 5) % 2 == 0:   # 3.5〜7 秒は 0.2 秒おきに右足首 roll が 0.6 rad/s
                self.motor_state[11].dq = 0.6

    monkeypatch.setattr(tl, "_State", ShakyState)
    z, lidar, staged, seen = tl._run_staged(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (20.0, "r"), (24.0, "q")],
                                            extra=("--script", str(params), "--script-start", "1"))
    out = capsys.readouterr().out
    assert "d を待っている" in out
    spans = _spans(z)
    assert len(spans) == 1
    t_start = z["t"][spans[0][0]] - z["t"][0]
    assert 6.5 < t_start < 10.0, t_start               # 揺れが止まって 1 秒たまってから始めた（打ち直しは要らない）
