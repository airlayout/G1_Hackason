from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

from Voice.onboard_voice import VoiceController
from Voice.segmenter import Segment
from Voice.task_router import TaskRouter, load_catalog
from Voice.tool_client import ToolClient
from Voice.tool_runtime import CommandBackend, ToolRuntime, load_tools
from Voice.tool_server import ToolServer

ROOT = Path(__file__).resolve().parents[2]


class WaveControlTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.directory = Path(self.tmp.name)
        self.specs = load_tools(ROOT / "Voice/configs/tools.wave.json")
        self.catalog = load_catalog(ROOT / "Voice/configs/tasks.wave.json")

    def test_registry_rejects_other_presets_and_extra_arguments(self) -> None:
        backend = Mock()
        runtime = ToolRuntime(self.specs, {"command": backend})
        for arguments in ({"preset": "clap"}, {}, {"preset": "high wave", "action_id": 0}):
            self.assertEqual(runtime.call("sdk_arm_action", arguments, execute=True).status, "invalid_request")
        self.assertEqual(runtime.call("sdk_arm_action", {"preset": "high wave"}).status, "dry_run")
        backend.assert_not_called()

    def test_relative_command_cwd_uses_config_location(self) -> None:
        spec = self.specs[0]
        self.assertEqual(Path(spec.config["cwd"]), ROOT)
        backend = CommandBackend()
        self.addCleanup(backend.close)
        # 同じ登録済みラッパーを、実行フラグだけ外して別の cwd から試す。
        from dataclasses import replace
        item = replace(spec, config={**spec.config, "argv": spec.config["argv"][:-1]})
        previous = Path.cwd()
        try:
            os.chdir(self.directory)
            with patch.dict(os.environ, {"VOICE_SDK_PYTHON": sys.executable}):
                result = backend(item, {"preset": "high wave"})
        finally:
            os.chdir(previous)
        self.assertTrue(result.success, result.data)
        self.assertEqual(json.loads(result.data["output"])["status"], "dry_run")

    def test_voice_to_ipc_to_command_invokes_sdk_preset(self) -> None:
        # SDK の境界だけを子プロセス内で置換し、実際の IPC・argv・ラッパーを通す。
        sdk = self.directory / "fake-sdk"
        sdk.mkdir()
        sources = {
            "core/channel.py": '''import json, os
def ChannelFactoryInitialize(domain, interface):
    with open(os.environ["WAVE_TEST_MARKER"], "a") as f:
        f.write(json.dumps({"domain": domain, "interface": interface}) + "\\n")
''',
            "g1/loco/g1_loco_client.py": '''class LocoClient:
    def SetTimeout(self, timeout): pass
    def Init(self): pass
    def GetFsmId(self): return 0, 500
''',
            "g1/arm/g1_arm_action_client.py": '''import json, os
class G1ArmActionClient:
    def SetTimeout(self, timeout): pass
    def Init(self): pass
    def ExecuteAction(self, action):
        with open(os.environ["WAVE_TEST_MARKER"], "a") as f:
            f.write(json.dumps({"action_id": action}) + "\\n")
        return 0
''',
        }
        for relative, source in sources.items():
            path = sdk / "unitree_sdk2py" / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            for parent in path.parents:
                if parent == sdk:
                    break
                (parent / "__init__.py").touch()
            path.write_text(source)
        marker = self.directory / "sdk-events.jsonl"
        backend = CommandBackend()
        runtime = ToolRuntime(self.specs, {"command": backend})
        server = ToolServer(self.directory / "tools.sock", runtime, execute=True)
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.02})
        thread.start()
        try:
            encoder = Mock(model_name="fake-audio-model", revision="test")
            encoder.encode_audio.return_value = np.array([1.0, 0.0])
            references = np.array([[1.0, 0.0] if task.action else [0.0, 1.0]
                                   for task in self.catalog.tasks for _ in range(task.reference_count)])
            controller = VoiceController(encoder, TaskRouter(self.catalog, references), runtime,
                                         self.directory / "runs", ToolClient(server.path, timeout_s=23))
            with patch.dict(os.environ, {"VOICE_SDK_PYTHON": sys.executable, "VOICE_SDK_INTERFACE": "eth0",
                                         "PYTHONPATH": str(sdk), "WAVE_TEST_MARKER": str(marker)}):
                result = controller.process(Segment(np.full(6400, 0.1, dtype=np.float32), time.monotonic()))
                self.assertEqual(result["decision"]["task_id"], "wave_hand")
                self.assertTrue(result["tool_result"]["success"], result)
                child = json.loads(result["tool_result"]["data"]["output"])
                self.assertEqual(child["status"], "accepted")
                self.assertFalse(child["motion_completed"])
                self.assertEqual([json.loads(line) for line in marker.read_text().splitlines()],
                                 [{"domain": 0, "interface": "eth0"}, {"action_id": 26}])
                # 見送り用の音声は同じ経路でも SDK を再度呼ばない。
                before = marker.read_text()
                encoder.encode_audio.return_value = np.array([0.0, 1.0])
                rejected = controller.process(Segment(np.full(6400, 0.1, dtype=np.float32), time.monotonic()))
                self.assertEqual(rejected["status"], "rejected")
                self.assertEqual(marker.read_text(), before)
        finally:
            server.shutdown()
            thread.join(2)
            server.server_close()
            backend.close()
