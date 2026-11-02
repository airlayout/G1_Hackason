"""analyze_stock.py（純正の脚の口の記録の解析）の試験。作り物の記録で、進んだ割合・扉の角度・観測の外れ・押す試しが戻るかを見る"""
from __future__ import annotations

import math
import sys
import types
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import analyze_stock as A  # noqa: E402
import door_run_stock as drs  # noqa: E402

OBS_DIM = 60
LAYOUT = {"actions": (0, 29), "dof_pos": (29, 58), "complete": (58, 59), "base_ang_vel": (59, 60)}


def _policy():
    return types.SimpleNamespace(layout=LAYOUT, rms_mean=np.zeros(OBS_DIM), rms_var=np.ones(OBS_DIM),
                                 c={"rms_eps": 1e-8, "rms_clip": 5.0})


def _fake_log(n_ready=50, n_door=250, n_post=50, n_probe=50, fwd_share=0.4, yaw=math.radians(30)):
    """扉の方策の 5 秒で前へ 0.3 m/s の指令。odometry では fwd_share 倍しか進まない（機体は世界の yaw を向いている）。
    扉は 35° まで開く。観測は脚と腰の dof_pos だけ学習の統計から 8σ 外れる。最後に押す試し（前へ 0.5 m/s・0.1 倍しか進まない）"""
    n = n_ready + n_door + n_post + n_probe
    t = np.arange(n) * 0.02
    names = ["READY"] * n_ready + ["DOOR"] * n_door + ["POST"] * n_post + ["PROBE"] * n_probe
    phase = np.array([drs.PHASES.index(s) for s in names])
    vx = np.r_[np.zeros(n_ready), np.full(n_door, 0.3), np.zeros(n_post), np.full(n_probe, 0.5)]
    walk = (vx > 0).astype(float)
    share = np.r_[np.zeros(n_ready), np.full(n_door, fwd_share), np.zeros(n_post), np.full(n_probe, 0.1)]
    fwd = np.r_[0.0, np.cumsum(vx[:-1] * share[:-1] * 0.02)]
    odom = np.c_[2.0 + fwd * math.cos(yaw), -1.0 + fwd * math.sin(yaw), np.full(n, yaw)]
    deg = np.r_[np.zeros(n_ready), np.linspace(0, 35, n_door), np.full(n_post + n_probe, 35)]
    obs = np.zeros((n, OBS_DIM))
    obs[:, 29:29 + 15] = 8.0
    q = np.zeros((n, 29))
    arm_sent = np.full((n, 14), 0.05)
    tau = np.zeros((n, 29))
    tau[:, 15] = 7.5
    return {"t": t, "phase": phase, "quat": np.tile([1.0, 0.0, 0.0, 0.0], (n, 1)), "period": np.full(n, 0.02),
            "door_angle_obs": np.c_[np.radians(deg), np.ones(n), np.full(n, 0.1)],
            "pose": np.c_[np.linspace(-1.0, -0.4, n), np.zeros(n), np.zeros(n)],
            "vel_want": np.c_[walk, vx, np.zeros(n), np.zeros(n)], "odom": odom,
            "vel_sent": np.arange(n) // 2, "vel_rpc_ms": np.full(n, 3.0), "vel_code": np.zeros(n),
            "q": q, "arm_sent": arm_sent, "tau": tau, "obs": obs,
            "meta": np.array(repr({"reason": "時間"}))}


def test_forward_share_is_measured_in_body_frame():
    # Arrange
    z = _fake_log(fwd_share=0.4, yaw=math.radians(30))

    # Act
    r = A.summarize(z)

    # Assert
    assert r["歩き"]["前"]["割合"] == pytest.approx(0.4, abs=0.01)
    assert abs(r["歩き"]["前"]["指令の積分"] - 0.3 * 249 * 0.02) < 0.01
    assert "左" not in r["歩き"] and "後ろ" not in r["歩き"]


def test_door_angle_rpc_arms_and_reason():
    r = A.summarize(_fake_log())
    assert r["扉"]["最大°"] == pytest.approx(35.0)
    assert r["扉"]["通り抜けた"] is False
    assert r["RPC"]["中央ms"] == pytest.approx(3.0) and r["RPC"]["失敗"] == 0
    assert r["腕"]["差最大rad"] == pytest.approx(0.05) and r["腕"]["トルク最大Nm"] == pytest.approx(7.5)
    assert r["止めた理由"] == "時間"


def test_obs_zscore_flags_only_the_lower_body_joint_angles():
    r = A.summarize(_fake_log(), _policy())
    obs = r["観測"]
    assert obs["dof_pos（脚と腰）"]["z99"] == pytest.approx(8.0) and obs["dof_pos（脚と腰）"]["頭打ち超え"] == 1.0
    assert obs["dof_pos（腕）"]["z99"] == 0.0 and obs["actions（脚と腰）"]["z99"] == 0.0


def test_probe_segment_reports_forward_share():
    r = A.summarize(_fake_log())
    (p,) = r["押す試し"]
    assert p["前の指令 m/s"] == 0.5 and p["割合"] == pytest.approx(0.1, abs=0.01)


def test_report_prints_without_policy(capsys):
    A.print_report(A.summarize(_fake_log()))
    out = capsys.readouterr().out
    assert "前" in out and "押す試し" in out and "止めた理由" in out
