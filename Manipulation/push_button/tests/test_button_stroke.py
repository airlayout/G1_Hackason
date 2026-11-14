"""小ストロークの入力と、物理モデルの押下・復帰を検証する。"""

import os
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sim"))
from trajectory import validate_button_stroke


@pytest.mark.parametrize("stroke", [0.0014, 0.0151, float("nan"), float("inf")])
def test_stroke_rejects_unsupported_and_nonfinite_travel(stroke):
    with pytest.raises(ValueError, match="1.5～15mm"):
        validate_button_stroke(stroke)


def test_both_buttons_stop_at_short_travel_and_return_after_release():
    path = os.environ.get("G1_MUJOCO_MODEL")
    if not path:
        pytest.skip("G1_MUJOCO_MODEL に外部のMenagerieモデルを指定してください")
    import mujoco
    from run_mujoco import DEFAULT_TIP_OFFSET, build_model

    model, stand = build_model(Path(path), 0.36, -0.25, 1.0, 0.0015,
                              DEFAULT_TIP_OFFSET, fixed_base=True)
    data = mujoco.MjData(model)
    data.qpos[:29], data.ctrl[:] = stand, stand
    joints = [model.joint(f"button_slide_{direction}") for direction in ("up", "down")]
    assert all(np.array_equal(joint.range, [0, 0.0015]) for joint in joints)
    qadr, dadr = [joint.qposadr[0] for joint in joints], [joint.dofadr[0] for joint in joints]
    maximum = np.zeros(2)
    # 0.2秒で2Nまで力を増やす。保持中も2msごとの物理変位で終端を検証する。
    for index in range(400):
        data.qfrc_applied[dadr] = 2.0 * min(1.0, index / 100)
        mujoco.mj_step(model, data)
        maximum = np.maximum(maximum, data.qpos[qadr])
    assert np.all(maximum >= 0.0015)
    assert np.all(maximum <= 0.0016)
    assert np.allclose(data.qpos[qadr], 0.0015, atol=0.00002)
    data.qfrc_applied[:] = 0
    for _ in range(400):
        mujoco.mj_step(model, data)
    assert np.all(np.abs(data.qpos[qadr]) < 0.00001)
