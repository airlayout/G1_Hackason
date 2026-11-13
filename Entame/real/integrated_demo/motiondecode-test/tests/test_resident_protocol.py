from __future__ import annotations

import socket
import threading
import time

from resident_protocol import DryRunBackend, JsonUnixServer, ResidentController, unix_request


def test_state_machine_unknown_busy_and_reconnect(tmp_path):
    path = tmp_path / "worker.sock"
    backend = DryRunBackend(.08)
    controller = ResidentController(backend, {"surprise"})
    controller.start()
    server = JsonUnixServer(path, controller)
    thread = threading.Thread(target=server.serve, daemon=True); thread.start()
    deadline = time.monotonic() + 1
    while not path.exists() and time.monotonic() < deadline: time.sleep(.005)

    assert unix_request(path, {"operation": "status"})["state"] == "READY"
    assert unix_request(path, {"reaction": "unknown"})["accepted"] is False
    first = {}
    running = threading.Thread(target=lambda: first.update(
        unix_request(path, {"reaction": "surprise"})))
    running.start(); time.sleep(.02)
    busy = unix_request(path, {"reaction": "surprise"})
    assert busy == {"accepted": False, "reason": "worker busy", "state": "EXECUTING"}
    running.join(1)
    assert first["status"] == "pass"
    # A new connection works after the first client disconnects.
    assert unix_request(path, {"operation": "status"})["state"] == "READY"
    unix_request(path, {"operation": "stop"}); thread.join(1)


class FaultBackend(DryRunBackend):
    def execute(self, reaction, request, received):
        raise RuntimeError("backend failed")


def test_fault_is_latched():
    controller = ResidentController(FaultBackend(), {"surprise"}); controller.start()
    response = controller.handle({"reaction": "surprise"})
    assert response["state"] == "FAULT"
    assert controller.handle({"reaction": "surprise"})["accepted"] is False


def test_preflight_is_read_only_and_failure_does_not_fault_worker():
    class PreflightBackend(DryRunBackend):
        def __init__(self):
            super().__init__(); self.safe = True

        def preflight(self):
            if not self.safe:
                raise RuntimeError("robot not stable")
            return {"passed": True}

    backend = PreflightBackend()
    controller = ResidentController(backend, {"surprise"}); controller.start()
    assert controller.handle({"operation": "preflight"}) == {
        "accepted": True, "state": "READY", "passed": True,
    }
    assert backend.executions == 0
    backend.safe = False
    response = controller.handle({"operation": "preflight"})
    assert response["passed"] is False
    assert response["reason"] == "robot not stable"
    assert controller.state == "READY"
    assert backend.executions == 0


def test_every_reaction_holds_waist_and_leaves_legs_ungained():
    import json
    from pathlib import Path
    import numpy as np
    from hold_core import command_fields
    from reaction_profiles import align_precomputed_reaction, precompute_named_reaction
    from run_real_reaction import CONTROLLED
    import resident_worker
    assert CONTROLLED == list(range(12, 29))
    fields = command_fields(np.zeros(29), 1.0, 0, CONTROLLED)
    assert all(fields["kp"][i] == 0. and fields["kd"][i] == 0. for i in range(12))
    root = Path(__file__).resolve().parents[1]
    q0 = np.asarray(json.loads((root / "output/fast_game_reactions_20260916/"
                                "found_replacement_target_retry4/fresh_q0.json").read_text())["all_q"])
    for name, (profile, arms, waist, fast) in resident_worker.REACTION_SPECS.items():
        assert waist == 0.0, name
        asset = precompute_named_reaction(profile, arms, waist, fast_reaction=fast)
        paths, _, _, _ = align_precomputed_reaction(asset, q0)
        for phase, path in paths.items():
            assert np.allclose(np.asarray(path)[:, :3], q0[12:15]), (name, phase)


def test_surprise_keeps_full_size_at_the_speed_of_the_other_reactions():
    import numpy as np
    from reaction_profiles import precompute_named_reaction
    from play_g1_arms import DT
    import resident_worker
    profile, arms, waist, fast = resident_worker.REACTION_SPECS["surprise"]
    asset = precompute_named_reaction(profile, arms, waist, fast_reaction=fast)
    assert arms == 1.0 and fast is True
    assert asset["durations"] == {"entry": .1, "clip": 1.5, "return": 1.0}
    delta = np.asarray(asset["scaled_delta"])
    clip_speed = np.abs(np.diff(delta[:, 3:], axis=0)).max() * (len(delta) - 1) / 1.5
    return_speed = 1.875 * np.abs(delta[-1]).max() / 1.0
    assert max(clip_speed, return_speed) <= 2.8
