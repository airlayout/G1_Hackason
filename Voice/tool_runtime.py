"""音声やモデルから独立した、名前・引数スキーマ付きのツール実行基盤。"""

from __future__ import annotations

import json
import math
import os
import re
import signal
import subprocess
import tempfile
import threading
import time
from dataclasses import asdict, dataclass, field
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Callable

from jsonschema import Draft202012Validator


@dataclass(frozen=True)
class ToolSpec:
    name: str
    description: str
    backend: str
    config: dict[str, Any]
    input_schema: dict[str, Any]
    timeout_s: float = 5.0
    exclusive_group: str = "motion"


@dataclass(frozen=True)
class ToolResult:
    tool: str
    success: bool
    status: str
    message: str
    data: dict[str, Any] = field(default_factory=dict)


def load_tools(path: Path) -> tuple[ToolSpec, ...]:
    data = json.loads(path.expanduser().read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("tools"), list):
        raise ValueError("ツール設定には tools 配列が必要です")
    specs = []
    names = set()
    for row in data["tools"]:
        if not isinstance(row, dict):
            raise ValueError("ツール定義はオブジェクトにしてください")
        name = row.get("name")
        if not isinstance(name, str) or not re.fullmatch(r"[a-z][a-z0-9_]*", name) or name in names:
            raise ValueError(f"不正または重複したツール名です: {name}")
        schema = row.get("input_schema", {"type": "object", "additionalProperties": False})
        Draft202012Validator.check_schema(schema)
        if schema.get("type") != "object" or schema.get("additionalProperties") is not False:
            raise ValueError(f"{name}: 引数は object 型とし、additionalProperties: false を指定してください")
        timeout = row.get("timeout_s", 5.0)
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or not 0 < timeout <= 300:
            raise ValueError(f"{name}: timeout_s は 0 より大きく 300 以下にしてください")
        backend, config = row.get("backend"), row.get("config", {})
        if not isinstance(backend, str) or not backend or not isinstance(config, dict):
            raise ValueError(f"{name}: backend と config を確認してください")
        group = row.get("exclusive_group", "motion")
        if not isinstance(group, str) or not group:
            raise ValueError(f"{name}: exclusive_group を指定してください")
        specs.append(ToolSpec(name, str(row.get("description", name)), backend, config, schema, float(timeout), group))
        names.add(name)
    return tuple(specs)


Backend = Callable[[ToolSpec, dict[str, Any]], ToolResult]


@contextmanager
def claim_backend(lock: Any):
    if not lock.acquire(blocking=False):
        raise RuntimeError("実行 backend が使用中です。指示を待ち行列に積まずに見送りました")
    try:
        yield
    finally:
        lock.release()


class ToolRuntime:
    def __init__(self, specs: tuple[ToolSpec, ...], backends: dict[str, Backend]) -> None:
        self.specs = {spec.name: spec for spec in specs}
        if len(self.specs) != len(specs):
            raise ValueError("ツール名が重複しています")
        self.backends = backends
        self.locks = {spec.exclusive_group: threading.Lock() for spec in specs}
        for spec in specs:
            if spec.backend not in backends:
                raise ValueError(f"未登録の backend です: {spec.backend}")

    def describe(self) -> list[dict[str, Any]]:
        return [asdict(spec) for spec in self.specs.values()]

    def validate(self, name: str, arguments: dict[str, Any]) -> ToolSpec:
        if not isinstance(name, str) or name not in self.specs:
            raise ValueError(f"未登録のツールです: {name}")
        if not isinstance(arguments, dict):
            raise ValueError("ツール引数はオブジェクトにしてください")
        errors = list(Draft202012Validator(self.specs[name].input_schema).iter_errors(arguments))
        if errors:
            raise ValueError(f"{name}: 引数がスキーマに合いません: {errors[0].message}")
        # JSON の NaN / Infinity 拡張は IPC にもコマンドにも渡さない。
        json.dumps(arguments, allow_nan=False)
        return self.specs[name]

    def call(self, name: str, arguments: dict[str, Any], *, execute: bool = False) -> ToolResult:
        try:
            spec = self.validate(name, arguments)
        except ValueError as exc:
            return ToolResult(name, False, "invalid_request", str(exc))
        if not execute:
            return ToolResult(name, True, "dry_run", "ツールを実行せず、登録と引数を確認しました", {"arguments": arguments})
        lock = self.locks[spec.exclusive_group]
        if not lock.acquire(blocking=False):
            return ToolResult(name, False, "busy", "同じ実行グループのツールが動作中です")
        try:
            return self.backends[spec.backend](spec, arguments)
        except Exception as exc:
            return ToolResult(name, False, "error", str(exc))
        finally:
            lock.release()


def command_backend(spec: ToolSpec, arguments: dict[str, Any], *, observe=None) -> ToolResult:
    """管理者が登録した argv を shell=False で実行する。音声からコマンドは作らない。"""
    template = spec.config.get("argv")
    if not isinstance(template, list) or not template or not all(isinstance(s, str) and s for s in template):
        raise ValueError("command backend には空でない argv 配列が必要です")
    argv = []
    for token in template:
        match = re.fullmatch(r"\{([a-zA-Z_][a-zA-Z0-9_]*)\}", token)
        value = arguments[match[1]] if match else token
        if not isinstance(value, (str, int, float, bool)):
            raise ValueError("argv の引数置換には文字列・数値・真偽値を指定してください")
        argv.append(str(value))
    argv[0] = os.path.expanduser(argv[0])
    cwd = Path(spec.config.get("cwd", ".")).expanduser().absolute()
    # 出力は一時ファイルへ流し、読み返す量を制限する。
    with tempfile.TemporaryFile() as output:
        process = subprocess.Popen(argv, cwd=cwd, stdout=output, stderr=output, start_new_session=True)
        if observe:
            observe(process, True)
        timed_out = False
        try:
            code = process.wait(timeout=spec.timeout_s)
        except subprocess.TimeoutExpired:
            timed_out = True
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=2.0)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
            # 親だけ終了して同じ process group の子が残った場合も回収する。
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            code = process.returncode
        size = output.tell()
        output.seek(max(0, size - 4096))
        text = output.read().decode("utf-8", errors="replace")
        if observe:
            observe(process, False)
    return ToolResult(spec.name, code == 0 and not timed_out, "timeout" if timed_out else "process_exited",
                      "プロセスが終了しました", {"returncode": code, "output": text})


class CommandBackend:
    def __init__(self) -> None:
        self.processes: dict[int, subprocess.Popen] = {}
        self.lock = threading.Lock()
        self.stopping = False

    def _observe(self, process: subprocess.Popen, running: bool) -> None:
        with self.lock:
            if running and self.stopping:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
                raise RuntimeError("ツールサーバーは終了処理中です")
            if running:
                self.processes[process.pid] = process
            else:
                self.processes.pop(process.pid, None)

    def __call__(self, spec: ToolSpec, arguments: dict[str, Any]) -> ToolResult:
        return command_backend(spec, arguments, observe=self._observe)

    def close(self) -> None:
        with self.lock:
            self.stopping = True
            processes = list(self.processes.values())
        for process in processes:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for process in processes:
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()


class ROSServiceBackend:
    """ROS の Python 環境だけで使う。推論側からの rclpy import は不要。"""
    def __init__(self) -> None:
        self.node = None
        self.lock = threading.Lock()

    def prepare(self) -> None:
        import rclpy
        if self.node is None:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node("g1_voice_tools")

    def __call__(self, spec: ToolSpec, arguments: dict[str, Any]) -> ToolResult:
        import rclpy
        from rosidl_runtime_py.convert import message_to_ordereddict
        from rosidl_runtime_py.set_message import set_message_fields
        from rosidl_runtime_py.utilities import get_message, get_service

        with claim_backend(self.lock):
            self.prepare()
            service_type = get_service(spec.config["service_type"])
            client = self.node.create_client(service_type, spec.config["service"])
            deadline = time.monotonic() + spec.timeout_s
            try:
                if not client.wait_for_service(timeout_sec=spec.timeout_s):
                    return ToolResult(spec.name, False, "unavailable", "ROS サービスが見つかりません")
                request = service_type.Request()
                set_message_fields(request, {**spec.config.get("request", {}), **arguments})
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=max(0.0, deadline - time.monotonic()))
                if not future.done():
                    future.cancel()
                    return ToolResult(spec.name, False, "timeout", "ROS サービスの応答が期限内に届きませんでした")
                response = dict(message_to_ordereddict(future.result()))
                success_field = spec.config.get("success_field")
                success = True if success_field is None else response.get(success_field) is True
                confirm = spec.config.get("confirm")
                if success and confirm:
                    observed = []
                    def on_status(message: Any) -> None:
                        try:
                            data = json.loads(message.data)
                            observed.append(data[confirm["json_field"]])
                        except (ValueError, KeyError, TypeError, AttributeError):
                            pass
                    subscription = self.node.create_subscription(
                        get_message(confirm["message_type"]), confirm["topic"], on_status, 1)
                    try:
                        while time.monotonic() < deadline:
                            if observed and observed[-1] in confirm["allowed_values"]:
                                response["confirmed_state"] = observed[-1]
                                break
                            rclpy.spin_once(self.node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
                        else:
                            return ToolResult(spec.name, False, "unconfirmed", "要求は受理されましたが状態を確認できませんでした", response)
                    finally:
                        self.node.destroy_subscription(subscription)
                return ToolResult(spec.name, success, "completed" if success else "rejected",
                                  str(response.get("message", "ROS サービスから応答を受信しました")), response)
            finally:
                self.node.destroy_client(client)

    def close(self) -> None:
        with self.lock:
            if self.node is not None:
                self.node.destroy_node()


def default_backends() -> dict[str, Backend]:
    return {"command": CommandBackend(), "ros_service": ROSServiceBackend()}
