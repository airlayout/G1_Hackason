"""11-05: 画像だけの生徒を実機で回す部品（door_vision_core・door_vision_encoder・door_vision_source・door_vision_node・
door_run_stock.py の --door-source vision）の試験。機体なしで動く。

  DOOR_VISION_POLICY=<export_vision_student.py の npz> python -m pytest G1_Hackason/Door/real/tests/test_door_vision.py -q
  （既定は logs_real/policy/vision/ の最初の npz。画像の網の npz は同じ名前の _encoder.npz）

torch の生徒・網との一致は export_vision_student.py が書き出しの時に確かめる（OMEN）。ここでは numpy の側の約束と輪を見る。
"""
from __future__ import annotations

import math
import os
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import door_vision_core as dvc  # noqa: E402
import door_vision_encoder as dve  # noqa: E402
import door_vision_source as dvs  # noqa: E402


def _find_policy() -> str:
    env = os.environ.get("DOOR_VISION_POLICY")
    if env:
        return env
    found = sorted(p for p in (HERE / "logs_real/policy/vision").glob("*.npz") if not p.name.endswith("_encoder.npz"))
    return str(found[0]) if found else ""


VPOLICY = _find_policy()
VENCODER = VPOLICY[:-4] + "_encoder.npz" if VPOLICY else ""
needs_vpolicy = pytest.mark.skipif(not (VPOLICY and os.path.exists(VPOLICY)), reason="画像の生徒の書き出しが無い")
needs_encoder = pytest.mark.skipif(not (VENCODER and os.path.exists(VENCODER)), reason="画像の網の書き出しが無い")


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _quat_y(a: float) -> np.ndarray:
    return np.array([math.cos(a / 2), 0.0, math.sin(a / 2), 0.0])


# ---- 頭の重力 ----

def test_head_gravity_matches_sim_convention():
    """直立・腰 0 で (sin θ, 0, −cos θ)（door_vision_feat.head_gravity と同じ）。腰の yaw では変わらない"""
    th = math.radians(51.0)
    g = dvc.head_gravity(np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3), th)
    assert np.allclose(g, [math.sin(th), 0.0, -math.cos(th)])
    g_yaw = dvc.head_gravity(np.array([1.0, 0.0, 0.0, 0.0]), np.array([0.4, 0.0, 0.0]), th)
    assert np.allclose(g_yaw, g)


def test_head_gravity_adds_pelvis_and_waist_pitch():
    """pelvis の前傾・腰の pitch は俯角に足される（どちらも y 軸まわり）"""
    th, a, w = math.radians(45.0), math.radians(5.0), math.radians(3.0)
    g = dvc.head_gravity(_quat_y(a), np.array([0.0, 0.0, w]), th)
    tot = th + a + w
    assert np.allclose(g, [math.sin(tot), 0.0, -math.cos(tot)])


def test_head_gravity_roll_shows_in_y():
    """pelvis が右へ傾く（x 軸まわり +）と頭の座標で y に出る。鏡写しは y を反転する"""
    r = math.radians(10.0)
    g = dvc.head_gravity(np.array([math.cos(r / 2), math.sin(r / 2), 0.0, 0.0]), np.zeros(3), 0.0)
    assert np.allclose(g, [0.0, -math.sin(r), -math.cos(r)])


# ---- 電文 ----

def test_pack_unpack_roundtrip():
    f = np.linspace(-1, 1, 128)
    m = dvs.unpack(dvs.pack(7, f, 10.5, 10.6, 12.5))
    assert m is not None
    seq, feat, t_cap, t_done, ms = m
    assert seq == 7 and np.allclose(feat, f, atol=1e-6) and (t_cap, t_done) == (10.5, 10.6) and abs(ms - 12.5) < 1e-4


def test_unpack_rejects_wrong_shapes():
    data = dvs.pack(1, np.zeros(128), 0.0, 0.0, 0.0)
    assert dvs.unpack(data[:-4]) is None
    assert dvs.unpack(b"XXXX" + data[4:]) is None
    assert dvs.unpack(b"") is None


def test_feat_source_reports_stale_and_old():
    port = _free_port()
    src = dvs.VisionFeatSource(128, port)
    try:
        assert "まだ来ない" in src.why_not(time.monotonic())
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        now = time.monotonic()
        tx.sendto(dvs.pack(1, np.ones(64), now, now, 1.0), ("127.0.0.1", port))  # 特徴の数が違う
        tx.sendto(dvs.pack(2, np.ones(128), now, now, 1.0), ("127.0.0.1", port))
        for _ in range(50):
            if src.latest() is not None:
                break
            time.sleep(0.02)
        assert src.n_bad == 1 and src.latest()[0] == 2
        assert src.why_not(time.monotonic()) == ""
        assert "来ない" in src.why_not(time.monotonic() + dvs.STALE_S + 0.1)
        tx.sendto(dvs.pack(3, np.ones(128), now - 2.0, now, 1.0), ("127.0.0.1", port))  # 2 秒前に撮った
        time.sleep(0.1)
        assert "古い" in src.why_not(time.monotonic())
        tx.close()
    finally:
        src.close()


# ---- 画像の網（numpy） ----

def _naive_conv(x, w, b, stride, pad):
    c, h, wd = x.shape
    o, _, kh, kw = w.shape
    xp = np.pad(x, ((0, 0), (pad, pad), (pad, pad)))
    ho, wo = (h + 2 * pad - kh) // stride + 1, (wd + 2 * pad - kw) // stride + 1
    y = np.zeros((o, ho, wo))
    for i in range(ho):
        for j in range(wo):
            patch = xp[:, i * stride:i * stride + kh, j * stride:j * stride + kw]
            y[:, i, j] = (w * patch[None]).sum(axis=(1, 2, 3)) + b
    return y


@pytest.mark.parametrize("stride,pad,k", [(1, 1, 3), (2, 1, 3), (2, 3, 7), (2, 0, 1)])
def test_conv2d_matches_naive(stride, pad, k):
    rng = np.random.default_rng(0)
    x = rng.normal(size=(3, 9, 11))
    w = rng.normal(size=(4, 3, k, k))
    b = rng.normal(size=4)
    assert np.allclose(dve.conv2d(x, w, b, stride, pad), _naive_conv(x, w, b, stride, pad))


def test_maxpool_pads_with_minus_inf():
    x = -np.ones((1, 4, 4))
    x[0, 0, 0] = 5.0
    y = dve.maxpool3x3s2(x)
    assert y.shape == (1, 2, 2) and y[0, 0, 0] == 5.0 and (y.ravel()[1:] == -1.0).all()


def test_decode_small_bgr_rejects_non_16_9():
    cv2 = pytest.importorskip("cv2")
    ok, jpg = cv2.imencode(".jpg", np.zeros((480, 640, 3), np.uint8))
    with pytest.raises(ValueError, match="16:9"):
        dve.decode_small_bgr(jpg.tobytes())
    ok, jpg = cv2.imencode(".jpg", np.full((1080, 1920, 3), 128, np.uint8))
    assert dve.decode_small_bgr(jpg.tobytes()).shape == (dve.IMG_H, dve.IMG_W, 3)


# ---- 書き出しを使う ----

@needs_vpolicy
def test_insert_door_writes_only_door_terms():
    p = dc.load_params(VPOLICY)
    vp = dvc.load_vision(VPOLICY, p)
    obs = np.arange(p.obs_dim, dtype=float)
    d17 = np.linspace(0.1, 1.7, 17)
    out = dvc.insert_door(vp, obs, d17)
    door = np.zeros(p.obs_dim, bool)
    for t in dvc.DOOR_TERMS:
        a, b = p.layout[t]
        door[a:b] = True
    assert np.array_equal(out[~door], obs[~door])
    a, _ = p.layout["door_angle_lidar"]
    assert np.allclose(out[a:a + 3], [d17[0], 1.0, vp.angle_age])
    assert np.allclose(dvc.door_from_obs(vp, out), d17)


@needs_vpolicy
def test_head_gravity_mirror_flips_y():
    p = dc.load_params(VPOLICY)
    vp = dvc.load_vision(VPOLICY, p)
    assert np.allclose(dvc.mirror_head_gravity(vp, np.array([0.5, 0.2, -0.8]), True), [0.5, -0.2, -0.8])
    assert np.allclose(dvc.mirror_head_gravity(vp, np.array([0.5, 0.2, -0.8]), False), [0.5, 0.2, -0.8])


@needs_vpolicy
def test_vision_actor_step_is_finite_and_uses_feature():
    """dc.step に policy を渡すと扉の値が観測に入り、特徴が変われば行動も変わる"""
    p = dc.load_params(VPOLICY)
    vp = dvc.load_vision(VPOLICY, p)
    inp = dc.Inputs(q=p.q_default.copy(), dq=np.zeros(29), gyro=np.zeros(3), gravity=np.array([0.0, 0.0, -1.0]),
                    door_angle=np.zeros(3), door_frame_loc=np.zeros(9), door_geometry=np.zeros(7))
    hg = dvc.mirror_head_gravity(vp, dvc.head_gravity(np.array([1.0, 0, 0, 0]), np.zeros(3), math.radians(51)), p.mirror)
    outs = []
    for seed in (0, 1):
        feat = np.random.default_rng(seed).normal(size=vp.feat_dim)
        s = dc.initial_state(p)
        for _ in range(5):
            s, out = dc.step(p, s, inp, mode="door", policy=dvc.VisionActor(vp, feat, hg))
        assert np.isfinite(out.a_pol).all() and np.isfinite(out.q_target).all()
        a, _ = p.layout["door_angle_lidar"]
        assert out.obs[a + 1] == 1.0  # 「測れた」は 1
        outs.append(out.a_pol)
    assert np.abs(outs[0] - outs[1]).max() > 1e-6


def _with_tracker(vp, hidden: int = 16, out_scale: float = 0.0, seed: int = 0):
    """vp に乱数の追う段を付けた写し（out_scale=0 なら出口が 0＝始めの d3 と同じ）"""
    import dataclasses

    rng = np.random.default_rng(seed)
    n_in = vp.lstm[0][0].shape[1]
    trk = (rng.normal(size=(4 * hidden, n_in)) * 0.05, rng.normal(size=(4 * hidden, hidden)) * 0.1,
           rng.normal(size=4 * hidden) * 0.1, rng.normal(size=4 * hidden) * 0.1)
    out = (rng.normal(size=(dvc.DOOR_DIM, hidden)) * out_scale, np.zeros(dvc.DOOR_DIM))
    return dataclasses.replace(vp, tracker=trk, tracker_out=out, door_tol=np.full(dvc.DOOR_DIM, 0.05))


def _vision_inputs(p, vp):
    inp = dc.Inputs(q=p.q_default.copy(), dq=np.zeros(29), gyro=np.zeros(3), gravity=np.array([0.0, 0.0, -1.0]),
                    door_angle=np.zeros(3), door_frame_loc=np.zeros(9), door_geometry=np.zeros(7))
    hg = dvc.mirror_head_gravity(vp, dvc.head_gravity(np.array([1.0, 0, 0, 0]), np.zeros(3), math.radians(51)), p.mirror)
    return inp, hg


@needs_vpolicy
def test_tracker_with_zero_exit_matches_no_tracker():
    """追う段の出口が 0 なら、追う段なしと同じ扉の値・行動（d2 → d3 の始め）。d3 以降の重みは追う段を外して比べる"""
    import dataclasses

    p = dc.load_params(VPOLICY)
    vp = dataclasses.replace(dvc.load_vision(VPOLICY, p), tracker=None, tracker_out=None, door_tol=None)
    vt = _with_tracker(vp)
    inp, hg = _vision_inputs(p, vp)
    feat = np.random.default_rng(2).normal(size=vp.feat_dim)
    s0, s1 = dc.initial_state(p), dc.initial_state(p)
    for _ in range(4):
        s0, o0 = dc.step(p, s0, inp, mode="door", policy=dvc.VisionActor(vp, feat, hg))
        s1, o1 = dc.step(p, s1, inp, mode="door", policy=dvc.VisionActor(vt, feat, hg))
    assert np.allclose(o0.obs, o1.obs) and np.allclose(o0.a_pol, o1.a_pol)
    assert s0.policy_state is None and s1.policy_state is not None and np.abs(s1.policy_state[0]).max() > 0


@needs_vpolicy
def test_tracker_state_advances_only_in_door_and_resets():
    """追う段の記憶は "door" の周期だけ進み（"hold" は止める）、reset_actor で消え、当て直した値が観測に入る"""
    p = dc.load_params(VPOLICY)
    vp = dvc.load_vision(VPOLICY, p)
    vt = _with_tracker(vp, out_scale=0.5)
    inp, hg = _vision_inputs(p, vp)
    feat = np.random.default_rng(3).normal(size=vp.feat_dim)
    s = dc.initial_state(p)
    s, _ = dc.step(p, s, inp, mode="hold", policy=dvc.VisionActor(vt, feat, hg))
    assert s.policy_state is None
    s, out = dc.step(p, s, inp, mode="door", policy=dvc.VisionActor(vt, feat, hg))
    head = dvc.door_values(vp, out.obs, feat, hg)
    assert np.abs(dvc.door_from_obs(vp, out.obs) - head).max() > 1e-6  # 当て直した値が入っている
    kept = s.policy_state
    s, _ = dc.step(p, s, inp, mode="hold", policy=dvc.VisionActor(vt, feat, hg))
    assert s.policy_state is kept
    s = dc.reset_actor(p, s)
    assert s.policy_state is None


@needs_vpolicy
def test_camera_pitch_from_export():
    p = dc.load_params(VPOLICY)
    vp = dvc.load_vision(VPOLICY, p)
    assert abs(dvc.camera_pitch_deg(vp) - 51.0) < 0.1


# ---- カメラ側のプロセス（フォルダの JPEG で空回し） ----

@needs_encoder
def test_node_sends_features_from_jpeg_dir(tmp_path):
    cv2 = pytest.importorskip("cv2")
    import door_vision_node as dvn

    jd = tmp_path / "jpg"
    jd.mkdir()
    for k in range(3):
        img = np.full((1080, 1920, 3), 60 * k, np.uint8)
        cv2.imwrite(str(jd / f"{k}.jpg"), img)
    port = _free_port()
    src = dvs.VisionFeatSource(128, port)
    try:
        rc = dvn.main(["--encoder", VENCODER, "--backend", "numpy", "--jpeg-dir", str(jd), "--seconds", "1.5",
                       "--port", str(port), "--log-dir", str(tmp_path / "logs"), "--full-every", "0.5"])
        assert rc == 0
        assert src.n_rx >= 8 and src.n_bad == 0
    finally:
        src.close()
    out = sorted((tmp_path / "logs").glob("vision_*.npz"))
    assert len(out) == 1
    z = np.load(out[0])
    assert z["rgb"].shape[1:] == (dve.IMG_H, dve.IMG_W, 3) and z["feat"].shape[1] == 128
    assert (z["t_done"] >= z["t_capture"]).all()
    assert len(list((tmp_path / "logs").glob("vision_*_jpeg/*.jpg"))) >= 2


# ---- 走らせ役（偽の機体） ----

class _FeatSender:
    """偽の door_vision_node: 10 Hz で特徴を送る。stop_at 秒で止める"""

    def __init__(self, port: int, stop_at: float = 1e9) -> None:
        self.port, self.stop_at = port, stop_at
        self._stop = threading.Event()
        self.t0 = time.monotonic()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self) -> None:
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rng = np.random.default_rng(0)
        k = 0
        while not self._stop.is_set() and time.monotonic() - self.t0 < self.stop_at:
            now = time.monotonic()
            tx.sendto(dvs.pack(k, rng.normal(size=128) * 0.5, now - 0.05, now, 20.0), ("127.0.0.1", self.port))
            k += 1
            time.sleep(0.1)
        tx.close()

    def close(self) -> None:
        self._stop.set()


def _run_vision(monkeypatch, tmp_path, script, stop_feat_at=1e9, run_seconds="0.5", pitch_args=("--head-pitch-deg", "48")):
    import test_door_run_stock as tdrs

    monkeypatch.setattr(tdrs, "POLICY", VPOLICY)
    port = _free_port()
    tx = _FeatSender(port, stop_feat_at)
    try:
        return tdrs._run(monkeypatch, tmp_path, script, run_seconds=run_seconds,
                         extra_args=("--door-source", "vision", "--vision-port", str(port), *pitch_args))
    finally:
        tx.close()


@needs_vpolicy
def test_vision_full_flow_logs_door_from_image(monkeypatch, tmp_path):
    """a → d → 画像の生徒で 0.5 秒 → POST。記録に特徴の番号・古さ・頭の重力・当てた扉の値が毎周期そろう"""
    rc, seen, phases, z, arms, loco, t0 = _run_vision(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (4.8, "r"), (8.2, "q")])
    assert rc == 0
    assert seen == ["WAIT", "ARMS_IN", "READY", "DOOR", "POST", "ARMS_OUT", "WAIT"]
    door = [k for k, ph in enumerate(phases) if ph == "DOOR"]
    assert 20 <= len(door) <= 30
    for k in ("vis_seq", "vis_age", "vis_hg", "vis_door", "obs", "cmd"):
        assert np.isfinite(z[k][door]).all(), k
    assert (z["vis_age"][door] < dvs.AGE_MAX_S).all()
    th = math.radians(48.0)  # 偽の機体は直立・腰 0 → 頭の重力は (sin θ, 0, −cos θ)（鏡写しで y は 0 のまま）
    assert np.allclose(z["vis_hg"][door][0], [math.sin(th), 0.0, -math.cos(th)], atol=1e-6)
    wait = [k for k, ph in enumerate(phases) if ph == "WAIT"]
    assert np.isnan(z["vis_door"][wait]).all()


@needs_vpolicy
def test_vision_cam_tilt_sets_head_pitch(monkeypatch, tmp_path):
    """11-05 夜: --cam-tilt-deg（カメラの加速度計の床からの俯角）を渡すと、起動時の姿勢から俯角を決める（--head-pitch-deg より優先）。
    偽の機体は直立・腰 0 なので torso からの俯角＝床からの俯角"""
    rc, seen, phases, z, arms, loco, t0 = _run_vision(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (4.8, "r"), (8.2, "q")],
                                                      pitch_args=("--head-pitch-deg", "48", "--cam-tilt-deg", "53.25"))
    assert rc == 0
    door = [k for k, ph in enumerate(phases) if ph == "DOOR"]
    th = math.radians(53.25)
    assert np.allclose(z["vis_hg"][door][0], [math.sin(th), 0.0, -math.cos(th)], atol=1e-6)


@needs_vpolicy
def test_vision_refuses_head_pitch_outside_training(monkeypatch, tmp_path):
    """11-06 朝: 頭が垂れて床から 84°（学習は 30〜70°）なら、腕も歩きも送らずに起動しない"""
    rc, _, _, _, arms, loco, _ = _run_vision(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d")],
                                            pitch_args=("--cam-tilt-deg", "84.18"))
    assert rc == 1 and arms == [] and loco.calls == []


@needs_vpolicy
def test_vision_stops_when_features_stop(monkeypatch, tmp_path):
    """DOOR の途中で特徴が途切れたら 0.5 秒あまりで立ったまま止まる（脱力しない）"""
    rc, seen, phases, z, arms, loco, t0 = _run_vision(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (6.0, "r"), (9.5, "q")],
                                                      stop_feat_at=4.3, run_seconds="5.0")
    assert seen[:5] == ["WAIT", "ARMS_IN", "READY", "DOOR", "POST"]
    t_post = float(z["t"][phases.index("POST")]) - t0
    assert 4.5 < t_post < 5.2
    assert loco.damps() == []


@needs_vpolicy
def test_vision_ready_refuses_door_without_features(monkeypatch, tmp_path):
    """特徴が来ていなければ d を受け付けない"""
    rc, seen, phases, z, arms, loco, t0 = _run_vision(monkeypatch, tmp_path, [(0.3, "a"), (3.8, "d"), (4.5, ""), (5.5, "r"), (9.0, "q")],
                                                      stop_feat_at=0.0)
    assert "DOOR" not in seen
