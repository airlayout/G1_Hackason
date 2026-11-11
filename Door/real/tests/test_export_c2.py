"""export_c2_1110.replace_head の試験: e2b の npz の扉の頭の欄だけが c2 に替わり、ほかと元の dict は変わらない。"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import export_c2_1110 as ex  # noqa: E402


def arrays_like_e2b() -> dict:
    rng = np.random.default_rng(0)
    a = {"vis_head0_w": rng.normal(size=(256, 137)), "vis_head0_b": rng.normal(size=256),
         "vis_head1_w": rng.normal(size=(256, 256)), "vis_head1_b": rng.normal(size=256),
         "vis_head2_w": rng.normal(size=(17, 256)), "vis_head2_b": rng.normal(size=17),
         "vis_head_out_mean": rng.normal(size=17), "vis_head_out_std": rng.random(17),
         "vis_lstm0_w_ih": rng.normal(size=(8, 4)), "vis_actor0_w": rng.normal(size=(3, 3))}
    a["meta_json"] = np.array(json.dumps({"vision": {"feat_dim": 128, "encoder_onnx": "door_vision_e2b_encoder.onnx"}, "x": 1}))
    return a


def head_like(a: dict, shift: float) -> dict:
    h = {}
    for tname, nname in ex.HEAD_LAYERS:
        h[f"{tname}.weight"] = a[f"{nname}_w"] + shift
        h[f"{tname}.bias"] = a[f"{nname}_b"] + shift
    h["out_mean"], h["out_std"] = a["vis_head_out_mean"] + shift, a["vis_head_out_std"] + shift
    return h


def test_replace_head_swaps_only_head_and_keeps_original() -> None:
    # Arrange
    a = arrays_like_e2b()
    before = {k: v.copy() for k, v in a.items()}
    h = head_like(a, 1.0)
    # Act
    out = ex.replace_head(a, h, "c2.pt", "door_vision_c2_encoder.onnx")
    # Assert
    np.testing.assert_allclose(out["vis_head1_w"], before["vis_head1_w"] + 1.0)
    np.testing.assert_allclose(out["vis_head_out_std"], before["vis_head_out_std"] + 1.0)
    np.testing.assert_array_equal(out["vis_lstm0_w_ih"], before["vis_lstm0_w_ih"])  # 生徒の欄はそのまま
    for k, v in before.items():  # 元の dict は変わらない
        np.testing.assert_array_equal(a[k], v)
    meta = json.loads(str(out["meta_json"]))
    assert meta["vision"]["encoder_onnx"] == "door_vision_c2_encoder.onnx" and meta["vision"]["feat_dim"] == 128
    assert meta["vision"]["door_head_from"] == "c2.pt" and meta["x"] == 1


def test_replace_head_refuses_shape_mismatch() -> None:
    a = arrays_like_e2b()
    h = head_like(a, 0.0)
    h["net.2.weight"] = np.zeros((128, 256))
    with pytest.raises(ValueError):
        ex.replace_head(a, h, "c2.pt", "x.onnx")
