from __future__ import annotations

import ast
import re
import subprocess
import unittest
from pathlib import Path

from Voice.supervision import HeartbeatWire, ONBOARD_HEARTBEAT_PORT
from Voice.tool_runtime import load_tools

ROOT = Path(__file__).resolve().parents[2]


class NavigationContractTests(unittest.TestCase):
    def test_patrol_tool_uses_existing_service_and_confirms_running(self) -> None:
        tools = load_tools(ROOT / "Voice/configs/tools.json")
        start = next(tool for tool in tools if tool.name == "patrol_start")
        source = (ROOT / "Navigation/nav2_stable/g1_ws/src/g1_navigation/scripts/patrol_node.py").read_text()
        self.assertIn(f'"{start.config["service"]}"', source)
        self.assertEqual(start.config["success_field"], "success")
        self.assertEqual(start.config["confirm"]["allowed_values"], ["RUNNING"])

    def test_cpp_heartbeat_wire_and_python_sender_agree(self) -> None:
        header = (ROOT / "Navigation/nav2_stable/g1_sdk_bridge_cpp/include/g1_sdk_bridge/heartbeat.hpp").read_text()
        self.assertIn("kHeartbeatWireSize = 29", header)
        self.assertIn("kHeartbeatMagic = 0x47314842", header)
        self.assertEqual(len(HeartbeatWire().next()), 29)
        original_port = int(re.search(r"kDefaultHeartbeatPort = (\d+)", header)[1])
        self.assertNotEqual(ONBOARD_HEARTBEAT_PORT, original_port)

    def test_launch_keeps_operator_defaults_and_required_monitoring(self) -> None:
        path = ROOT / "Navigation/nav2_stable/g1_ws/src/g1_navigation/launch/navigation.launch.py"
        tree = ast.parse(path.read_text())
        defaults = {}
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "DeclareLaunchArgument":
                for kw in node.keywords:
                    if kw.arg == "default_value" and isinstance(kw.value, ast.Constant):
                        defaults[node.args[0].value] = kw.value.value
        self.assertEqual(defaults["heartbeat_required"], "true")
        self.assertEqual(defaults["heartbeat_port"], "47311")
        self.assertEqual(defaults["heartbeat_bind_address"], "0.0.0.0")

    def test_invalid_supervision_mode_exits_before_startup(self) -> None:
        directory = ROOT / "Navigation/nav2_stable/deploy/pc2_humble"
        for script, args in (("g1up.sh", ["--supervision", "invalid"]),
                             ("start_nav.sh", ["0 0 0", "0", "1", "", "", "0.5", "invalid"])):
            result = subprocess.run(["bash", str(directory / script), *args], capture_output=True, text=True, timeout=2)
            self.assertEqual(result.returncode, 2)
            self.assertIn("operator / onboard", result.stderr + result.stdout)
