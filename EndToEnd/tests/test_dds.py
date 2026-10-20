"""公式SDKのCDR/CRC、PR関節順、PD合力、通信途絶をIsaacなしで検証する。"""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from g1_e2e.contracts import JointCommand, Observation
from g1_e2e.data.frames import ImuFrame
from g1_e2e.data.geometry import mount_matrix
from g1_e2e.dds.bridge import CommandMailbox
from g1_e2e.dds.codec import LowLevelCodec


@pytest.fixture
def codec():
    pytest.importorskip("cyclonedds", reason="DDS extraが未インストールです")
    if not (Path(__file__).resolve().parents[1] / "assets/cache/dds/sdk").is_dir():
        pytest.skip("setup.sh --with-ddsでSDKを取得してください")
    return LowLevelCodec()


@pytest.fixture
def observation():
    return Observation(
        0,
        0,
        1.234,
        np.arange(29, dtype=np.float32) / 10,
        np.ones(29, np.float32) * 0.4,
        np.arange(29, dtype=np.float32),
        np.array([0, 0, np.sin(0.3), np.cos(0.3)], np.float32),
        np.array([0.1, 0.2, 0.3], np.float32),
        np.array([0, 0, -1], np.float32),
    )


def message(codec):
    return codec.encode_command(
        JointCommand(
            np.ones(29, np.float32) * 1.5, np.ones(29, np.float32) * -0.1, np.ones(29, np.float32) * 0.5
        ),
        np.ones(29) * 20,
        np.ones(29) * 2,
    )


def test_sdk_crc_zero_payload_known_values(codec):
    assert codec.sdk.crc.Crc(codec.sdk.new_command()) == 4262932383
    assert codec.sdk.crc.Crc(codec.sdk.new_state()) == 3595284554


def test_state_sdk_cdr_roundtrip_and_units(codec, observation):
    state = codec.encode_state(observation, [1, 2, 9.81])
    decoded = codec.sdk.LowState.deserialize(state.serialize())
    assert decoded.tick == 1234 and decoded.mode_pr == 0 and decoded.mode_machine == 5
    assert codec.sdk.crc.Crc(decoded) == decoded.crc
    np.testing.assert_allclose([m.q for m in decoded.motor_state[:29]], observation.joint_position_rad)
    np.testing.assert_allclose([m.tau_est for m in decoded.motor_state[:29]], observation.joint_torque_nm)
    np.testing.assert_allclose(decoded.imu_state.quaternion, observation.orientation_xyzw[[3, 0, 1, 2]])
    np.testing.assert_allclose(decoded.imu_state.rpy, [0, 0, 0.6], atol=1e-6)
    np.testing.assert_allclose(decoded.imu_state.accelerometer, [1, 2, 9.81], rtol=1e-6)
    assert [m.mode for m in decoded.motor_state] == [1] * 29 + [0] * 6
    assert all(m.vol == 0 and m.temperature == [0, 0] for m in decoded.motor_state)


def test_command_sdk_cdr_to_physics_pd_and_saturation(codec, observation):
    source = message(codec)
    decoded = codec.sdk.LowCmd.deserialize(source.serialize())
    targets = codec.decode_command(decoded)
    effort = targets.effort_command(observation, np.ones(29) * 9)
    expected = np.clip(0.5 + 20 * (1.5 - observation.joint_position_rad) + 2 * (-0.1 - 0.4), -9, 9)
    np.testing.assert_allclose(effort.feedforward_torque_nm, expected, atol=1e-6)
    assert not targets.q.flags.writeable


def test_motor_off_and_sdk_stop_sentinels(codec):
    source = message(codec)
    source.motor_cmd[0].mode = 0
    source.motor_cmd[1].q = 2.146e9
    source.motor_cmd[2].dq = 16000
    source.crc = codec.sdk.crc.Crc(source)
    decoded = codec.decode_command(source)
    assert decoded.kp[0] == decoded.kd[0] == decoded.tau[0] == 0
    assert decoded.kp[1] == 0 and decoded.kd[1] == 2 and decoded.tau[1] == 0.5
    assert decoded.kd[2] == 0 and decoded.kp[2] == 20


@pytest.mark.parametrize("kind", ["crc", "ab", "machine", "extra", "mode", "nan", "negative_gain", "huge_q"])
def test_unsupported_commands_rejected(codec, kind):
    source = message(codec)
    if kind == "ab":
        source.mode_pr = 1
    elif kind == "machine":
        source.mode_machine = 9
    elif kind == "extra":
        source.motor_cmd[29].mode = 1
    elif kind == "mode":
        source.motor_cmd[1].mode = 2
    elif kind == "nan":
        source.motor_cmd[1].q = float("nan")
    elif kind == "negative_gain":
        source.motor_cmd[1].kp = -1
    elif kind == "huge_q":
        source.motor_cmd[1].q = 1000
    source.crc = codec.sdk.crc.Crc(source) ^ int(kind == "crc")
    with pytest.raises(ValueError):
        codec.decode_command(source)


def test_invalid_command_does_not_refresh_wall_clock_watchdog(codec):
    mailbox = CommandMailbox(codec, timeout_s=0.1)
    assert mailbox.current(now=0) is None
    source = message(codec)
    assert mailbox.receive(source, now=10)
    source.motor_cmd[0].q = 42
    assert not mailbox.receive(source, now=10.09)
    assert mailbox.current(now=10.099) is not None
    assert mailbox.current(now=10.101) is None
    assert mailbox.stats() == {"accepted": 1, "rejected": {"CRC": 1}}
    targets = mailbox.damping()
    assert np.all(targets.kp == 0) and np.all(targets.tau == 0) and np.all(targets.kd > 0)


def test_mounted_imu_and_secondary_cdr_roundtrip(codec, observation):
    frame = ImuFrame(
        observation.episode_id,
        observation.simulation_time_s,
        np.array([0, 0, np.sqrt(0.5), np.sqrt(0.5)], np.float32),
        np.array([0, 0, 2], np.float32),
        np.array([0, 4, 9.81], np.float32),
        np.array([0, 0, -1], np.float32),
        mount_matrix((1, 0, 0), (0, 0, 90)),
        mount_matrix((1, 0, 0), (0, 0, 90)),
        True,
    )
    state = codec.encode_state(observation, imu=frame)
    np.testing.assert_allclose(state.imu_state.accelerometer, frame.specific_force_sensor_m_s2, atol=1e-6)
    np.testing.assert_allclose(state.imu_state.gyroscope, frame.angular_velocity_sensor_rad_s, atol=1e-6)
    np.testing.assert_allclose(state.imu_state.quaternion, frame.orientation_xyzw[[3, 0, 1, 2]], atol=1e-6)
    assert codec.sdk.crc.Crc(state) == state.crc
    encoded = codec.encode_imu(frame)
    decoded = codec.sdk.IMUState.deserialize(encoded.serialize())
    np.testing.assert_allclose(decoded.accelerometer, state.imu_state.accelerometer)
    with pytest.raises(ValueError, match="時刻"):
        codec.encode_state(observation, imu=replace(frame, time_s=frame.time_s + 0.02))
