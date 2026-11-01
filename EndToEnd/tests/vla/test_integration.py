"""HTTP経由のUI→推論→腕制御と、推論中の停止を検証する。実機・ブラウザは使わない。"""

import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np
import pytest
import yaml

from g1_e2e.vla.app import Application
from g1_e2e.vla.config import ROOT, load_config
from g1_e2e.vla.demo import DemoBridge, HoldPolicy
from g1_e2e.vla.http import ApiClient, serve
from g1_e2e.vla.policy import create_adapter
from g1_e2e.vla.session import ArmSession


def wait_until(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("待機条件を満たしませんでした")


@pytest.fixture(params=["pi05", "groot"])
def system(request):
    config = load_config(ROOT / f"configs/vla/{request.param}.json")
    if config["backend"] == "groot" and not (ROOT / config["urdf"]).exists():
        pytest.skip("公式URDFがありません")
    bridge = DemoBridge(config)
    robot_server = serve(bridge, "127.0.0.1", 0, token="test-token")
    robot_thread = threading.Thread(target=robot_server.serve_forever, daemon=True)
    robot_thread.start()
    robot_client = ApiClient(f"http://127.0.0.1:{robot_server.server_port}", token="test-token")
    names = yaml.safe_load((ROOT / "configs/robot/g1_29dof.yaml").read_text())["joint_names"]
    session = ArmSession(robot_client, HoldPolicy(), create_adapter(config, names), config)
    ui_server = serve(Application(session, demo=True), "127.0.0.1", 0)
    ui_thread = threading.Thread(target=ui_server.serve_forever, daemon=True)
    ui_thread.start()
    client = ApiClient(f"http://127.0.0.1:{ui_server.server_port}")
    wait_until(lambda: session.status()["connected"])
    yield bridge, session, client, robot_client
    session.close()
    ui_server.shutdown()
    robot_server.shutdown()
    ui_server.server_close()
    robot_server.server_close()
    bridge.close()


def test_http_instruction_to_arm_and_stop(system):
    bridge, session, client, _ = system
    status = client.request("/api/status")
    assert status["demo"] and status["connected"]
    assert status["backend"] == session.config["backend"]
    assert status["model_label"] == session.config["model_label"]
    assert not status["robot"]["execute"]
    client.request("/api/task", {"instruction": "Hold position."})
    wait_until(lambda: client.request("/api/status")["steps"] >= 1)
    assert bridge.control.phase == "running"
    client.request("/api/stop", {})
    wait_until(lambda: bridge.control.phase == "idle")
    assert bridge.control.session is None
    assert not session.status()["error"]


def test_stop_during_slow_inference_never_starts_robot_after_result(system):
    bridge, session, client, _ = system
    entered, release = threading.Event(), threading.Event()

    class SlowPolicy(HoldPolicy):
        def predict(self, observation):
            entered.set()
            assert release.wait(3)
            return super().predict(observation)

    session.policy = SlowPolicy()
    client.request("/api/task", {"instruction": "Hold position."})
    assert entered.wait(1)
    before = time.monotonic()
    client.request("/api/stop", {})
    assert time.monotonic() - before < 0.5
    release.set()
    wait_until(lambda: not session.status()["busy"])
    assert bridge.control.weight == 0 and bridge.control.session is None
    assert session.status()["steps"] == 0


def test_out_of_bounds_policy_stops_instead_of_clipping_silently(system):
    bridge, session, client, _ = system

    class BadPolicy(HoldPolicy):
        def predict(self, _observation):
            return np.full((8, 14), 10)

    session.policy = BadPolicy()
    client.request("/api/task", {"instruction": "Move."})
    wait_until(lambda: session.status()["phase"] == "error")
    wait_until(lambda: bridge.control.weight == 0)
    assert session.status()["steps"] == 0


def test_stop_while_start_response_is_pending_releases_late_session(system):
    bridge, session, client, robot_client = system
    entered, release = threading.Event(), threading.Event()

    class DelayedStartClient:
        def request(self, path, body=None):
            if path == "/api/arm/start":
                entered.set()
                assert release.wait(3)
            return robot_client.request(path, body)

    session.client = DelayedStartClient()
    client.request("/api/task", {"instruction": "Hold position."})
    assert entered.wait(1)
    client.request("/api/stop", {})
    release.set()
    wait_until(lambda: not session.status()["busy"])
    wait_until(lambda: bridge.control.weight == 0)
    assert bridge.control.session is None
    assert session.status()["steps"] == 0


def test_robot_authentication_is_required(system):
    _, _, _, robot = system
    unauthenticated = ApiClient(robot.base_url)
    with pytest.raises(RuntimeError, match="認証"):
        unauthenticated.request("/api/arm/start", {})


def test_ui_rejects_cross_origin_commands_and_serves_only_known_files(system):
    _, _, client, _ = system
    request = Request(
        client.base_url + "/api/stop",
        data=b"{}",
        headers={"Origin": "https://example.com", "Content-Type": "application/json"},
    )
    with pytest.raises(HTTPError) as error:
        urlopen(request)
    assert error.value.code == 403
    with urlopen(client.base_url + "/") as response:
        assert b'lang="ja"' in response.read()
    with pytest.raises(RuntimeError, match="ページ"):
        client.request("/../configs/vla/groot.json")
    request = Request(
        client.base_url + "/api/stop",
        data=b"{}",
        headers={"Host": "attacker.example", "Content-Type": "application/json"},
    )
    with pytest.raises(HTTPError) as error:
        urlopen(request)
    assert error.value.code == 403


def test_sdk_command_contains_only_arms_and_weight():
    pytest.importorskip("cyclonedds")
    from g1_e2e.dds.sdk import load_sdk
    from g1_e2e.vla.robot import UnitreeArm

    sdk = load_sdk()
    commands = []

    class Publisher:
        def Write(self, message):
            commands.append(message)
            return True

    arm = UnitreeArm.__new__(UnitreeArm)
    arm.new_command, arm.crc, arm.publisher = sdk.new_command, sdk.crc, Publisher()
    arm.bridge = type("Bridge", (), {"config": load_config()})()
    q = np.linspace(-0.1, 0.1, 14)
    arm.publish(q, 0.7)
    message = sdk.LowCmd.deserialize(commands[0].serialize())
    assert message.mode_pr == 0 and message.mode_machine == 5
    np.testing.assert_allclose([m.q for m in message.motor_cmd[15:29]], q)
    assert message.motor_cmd[29].q == pytest.approx(0.7)
    assert all(m.mode == 0 and m.kp == 0 and m.kd == 0 for m in message.motor_cmd[:15])
    assert all(m.mode == 0 and m.q == 0 for m in message.motor_cmd[30:])
    assert sdk.crc.Crc(message) == message.crc
