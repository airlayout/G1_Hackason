"""観測受信とVLA推論を分離する。停止はモデルの応答を待たずにG1へ伝える。"""

from __future__ import annotations

from collections import deque
import threading
import time


class ArmSession:
    def __init__(self, client, policy, adapter, config: dict):
        self.client, self.policy, self.adapter, self.config = client, policy, adapter, config
        self.lock = threading.RLock()
        self.done = threading.Event()
        self.cancel = threading.Event()
        self.worker = None
        self.phase = "idle"
        self.error = ""
        self.robot_error = "接続中"
        self.observation = None
        self.observed_at = 0.0
        self.inference_ms = None
        self.steps = 0
        self.instruction = ""
        self.events = deque(maxlen=30)
        self.reader = threading.Thread(target=self.poll, name="robot-observation", daemon=True)
        self.reader.start()

    def event(self, text: str) -> None:
        self.events.append({"time": time.strftime("%H:%M:%S"), "text": text})

    def poll(self) -> None:
        while not self.done.is_set():
            try:
                observation = self.client.request("/api/observation")
                with self.lock:
                    self.adapter.record(observation)
                    self.observation = observation
                    self.observed_at = time.monotonic()
                    self.robot_error = ""
            except Exception as error:
                with self.lock:
                    self.robot_error = str(error)
            self.done.wait(0.1)

    def latest(self) -> dict:
        with self.lock:
            if self.observation is None or time.monotonic() - self.observed_at > 1:
                raise RuntimeError(self.robot_error or "G1の観測が途絶しました")
            return self.observation.copy()

    def start(self, instruction: str) -> None:
        if not isinstance(instruction, str) or not instruction.strip() or len(instruction) > 512:
            raise ValueError("指示は1..512文字で入力してください")
        with self.lock:
            if self.worker is not None and self.worker.is_alive():
                raise ValueError("前の実行が終了するまでお待ちください")
            self.latest()
            self.cancel = threading.Event()
            self.instruction = instruction.strip()
            self.phase, self.error, self.steps = "loading", "", 0
            self.event(self.instruction)
            self.worker = threading.Thread(
                target=self.run, args=(self.instruction, self.cancel), name="vla-inference", daemon=True
            )
            self.worker.start()

    def run(self, instruction: str, cancel: threading.Event) -> None:
        session = None
        start_attempted = False
        started = time.monotonic()
        try:
            self.policy.load()
            started = time.monotonic()
            with self.lock:
                if cancel.is_set():
                    return
                self.phase = "running"
                self.event("推論開始")
            while not cancel.is_set() and time.monotonic() - started < self.config["max_task_s"]:
                observation = self.latest()
                with self.lock:
                    inputs = self.adapter.build(observation, instruction)
                before = time.monotonic()
                positions = self.policy.predict(inputs)
                with self.lock:
                    self.inference_ms = round((time.monotonic() - before) * 1000)
                # ネットワーク待機中もUIの停止処理にロックを渡せるようにする。
                if cancel.is_set():
                    return
                if session is None:
                    start_attempted = True
                    response = self.client.request("/api/arm/start", {})
                    session = response["session"]
                    with self.lock:
                        self.event("実送信" if response["execute"] else "送信なしで実行")
                if cancel.is_set():
                    return
                self.client.request(
                    "/api/arm/chunk",
                    {
                        "session": session,
                        "observation_id": observation["observation_id"],
                        "positions": positions.tolist(),
                        "action_hz": self.config["action_hz"],
                    },
                )
                with self.lock:
                    self.steps += 1
                if cancel.wait(len(positions) / self.config["action_hz"]):
                    return
                # G1側でwatchdog停止していたら、古いsessionで再開しない。
                robot = self.client.request("/api/status")
                if robot["phase"] != "running":
                    raise RuntimeError(robot.get("reason") or "G1側で実行が停止しました")
        except Exception as error:
            with self.lock:
                if not cancel.is_set():
                    self.error = str(error)
                    self.event(str(error))
        finally:
            if start_attempted:
                try:
                    self.client.request("/api/arm/stop", {})
                except Exception as error:
                    with self.lock:
                        self.error = f"停止の応答なし（G1側watchdogで停止）: {error}"
            with self.lock:
                self.phase = "error" if self.error else "idle"
                self.event("終了")

    def stop(self) -> None:
        with self.lock:
            self.cancel.set()
            self.phase = "stopping" if self.worker and self.worker.is_alive() else "idle"
            self.event("停止")
        # 推論が止まらなくても、機体には停止を先に伝える。
        try:
            self.client.request("/api/arm/stop", {})
        except Exception as error:
            with self.lock:
                self.error = str(error)
            raise

    def status(self) -> dict:
        with self.lock:
            robot = None
            if self.observation is not None:
                robot = {k: v for k, v in self.observation.items() if k != "jpeg"}
            return {
                "phase": self.phase,
                "error": self.error,
                "robot_error": self.robot_error,
                "robot": robot,
                "inference_ms": self.inference_ms,
                "steps": self.steps,
                "instruction": self.instruction,
                "events": list(self.events),
                "model": self.config["model_id"],
                "model_label": self.config["model_label"],
                "backend": self.config["backend"],
                "training_task": self.config.get("training_task", ""),
                "max_task_s": self.config["max_task_s"],
                "busy": self.worker is not None and self.worker.is_alive(),
                "connected": bool(self.observation) and time.monotonic() - self.observed_at < 1,
            }

    def close(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
        self.done.set()
        self.reader.join(timeout=6)
