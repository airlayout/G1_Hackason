"""11-03: 推定器（door_lidar_node.py）の REF の記録と、記録を流すときの REF の出し方の試験（機体なし・点群なし）。

  Navigation/.venv/bin/python -m pytest G1_Hackason/Door/real/tests/test_door_lidar_node.py -q

方策の回を記録の上で再現するには、実機で REF を受けた時刻に、同じ回から扉と地図を作り直す必要がある。
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import door_lidar_node as dln  # noqa: E402


class _FakeNode:
    """REF と回の順番だけを覚える"""

    def __init__(self) -> None:
        self.events: list = []

    def request_ref(self, t=None) -> None:
        self.events.append(("REF", t))

    def on_scan(self, stamp, xyz) -> None:
        self.events.append(("scan", stamp))


def _items(t_rec0: float, n: int, dt: float = 0.1):
    """(電文の時刻, 点, 記録の時刻)。電文の時刻は記録の時刻に 1000 を足しただけ（別の時計であることを確かめる）"""
    return [(t_rec0 + i * dt + 1000.0, np.zeros((1, 3)), t_rec0 + i * dt) for i in range(n)]


def test_request_ref_records_wall_time_by_default():
    node = dln.Node(0.04, +1, "127.0.0.1", 47391)
    before = time.time()
    node.request_ref()
    assert before <= node.ref_log[-1] <= time.time()


def test_save_writes_ref_wall_even_without_rows(tmp_path):
    """扉を測れなかった（推定の行が 0）回でも、REF を受けた時刻は残す"""
    node = dln.Node(0.04, +1, "127.0.0.1", 47392)
    node.request_ref(123.5)
    node.request_ref(140.25)
    out = tmp_path / "lidar_node.npz"
    node.save(out)
    d = np.load(out)
    assert d["ref_wall"].tolist() == [123.5, 140.25]
    assert "stamp" not in d


def test_run_replay_fires_ref_before_first_scan_at_or_after_ref_time():
    node = _FakeNode()
    items = _items(50.0, 8)
    dln.run_replay(node, items, refs=[50.25, 50.65], relative=False, stop=threading.Event())
    kinds = [k for k, _ in node.events]
    assert kinds == ["scan", "scan", "scan", "REF", "scan", "scan", "scan", "scan", "REF", "scan"]
    # REF には、その次に流す回の記録の時刻を渡す（ref_wall は生の点群の記録と同じ時計にそろえる）
    assert node.events[3] == ("REF", pytest.approx(50.3))


def test_run_replay_relative_refs_count_from_first_record_time():
    node = _FakeNode()
    dln.run_replay(node, _items(100.0, 3), refs=[0.0], relative=True, stop=threading.Event())
    assert [k for k, _ in node.events] == ["REF", "scan", "scan", "scan"]


def test_run_replay_refs_before_recording_fire_at_first_scan():
    """記録を始める前に REF を受けていた（地図を作る回が記録に無い）ときも、最初の回で出して止まらない"""
    node = _FakeNode()
    dln.run_replay(node, _items(100.0, 2), refs=[90.0, 95.0], relative=False, stop=threading.Event())
    assert [k for k, _ in node.events] == ["REF", "REF", "scan", "scan"]


def test_run_replay_stops_when_asked():
    node = _FakeNode()
    stop = threading.Event()
    stop.set()
    dln.run_replay(node, _items(0.0, 5), refs=[0.0], relative=True, stop=stop)
    assert node.events == []


def _fake_bag(monkeypatch, recv_s):
    """crop_lidar_bag（Mac の g1_mapping を使う）を、受け取った時刻だけの偽物に差し替える"""
    import types
    fake = types.SimpleNamespace(iter_lidar=lambda _s: [(int(round(t * 1e9)), 0, b"") for t in recv_s],
                                 scan_xyz=lambda _p: np.zeros((1, 3)))
    monkeypatch.setitem(sys.modules, "crop_lidar_bag", fake)


def test_replay_scans_rec_clock_stamps_match_real_node(monkeypatch):
    """--rec-clock: 電文の時刻を実機と同じ「PC2 で受け取った時刻 − 55 ms」で付ける（実機の lidar_node_*.npz と時刻で突き合わせる）"""
    _fake_bag(monkeypatch, [100.0, 100.1])
    out = list(dln.replay_scans(Path("x"), speed=1000.0, rec_clock=True))
    assert [s for s, _, _ in out] == pytest.approx([100.0 - dln.SCAN_DELAY_S, 100.1 - dln.SCAN_DELAY_S])
    assert [t for _, _, t in out] == pytest.approx([100.0, 100.1])


def test_replay_scans_default_stamps_follow_now(monkeypatch):
    """既定は今の時計（door_run_real.py が電文の年齢を見るため）。間隔は記録のまま"""
    _fake_bag(monkeypatch, [100.0, 100.1])
    before = time.time()
    out = list(dln.replay_scans(Path("x"), speed=1000.0))
    assert out[0][0] >= before - dln.SCAN_DELAY_S - 0.01
    assert out[1][0] - out[0][0] == pytest.approx(0.1)
    assert [t for _, _, t in out] == pytest.approx([100.0, 100.1])


def test_load_ref_wall_reads_sorted_times(tmp_path):
    p = tmp_path / "n.npz"
    np.savez(p, ref_wall=np.array([30.0, 10.0]))
    assert dln.load_ref_wall(p) == [10.0, 30.0]


def test_load_ref_wall_rejects_old_records(tmp_path):
    """11-03 より前の記録には REF の時刻が無い。黙って最初の回で REF を出さない"""
    p = tmp_path / "old.npz"
    np.savez(p, stamp=np.array([1.0]))
    with pytest.raises(SystemExit, match="ref_wall"):
        dln.load_ref_wall(p)
