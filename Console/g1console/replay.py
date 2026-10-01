"""実機ログの間引き版（tests/fixtures/g1_live_sample.jsonl）から、ヘルパーのテレメトリ辞書を再生する。

各トピックは「経過時間以前で最新の 1 件」を階段状に返し、ログの長さで循環する。
出力の形は MockHelper._telemetry() / remote_helper.py と同じ。
"""
import json
import time
from pathlib import Path

DEFAULT_FIXTURE = Path(__file__).resolve().parent.parent / "tests" / "fixtures" / "g1_live_sample.jsonl"
JOINTS = 29
SYSTEM = {"load": [1.2, 1.0, 0.8], "mem_total_mb": 15388, "mem_avail_mb": 12700, "uptime_s": 4000.0,
          "cpus": 8, "disk_free_gb": 1800.0, "temps": {"CPU-therm": 58.9, "GPU-therm": 54.9, "tj-therm": 58.8}}
STRING_TOPICS = ("rt/rtc/state", "rt/arm/action/state", "rt/public_network_status", "rt/gpt_state")


class Replay:
    def __init__(self, path=DEFAULT_FIXTURE):
        self._by = {}
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            rec = json.loads(line)
            self._by.setdefault(rec["topic"], []).append((rec["t"], rec["v"]))
        if not self._by:
            raise ValueError("フィクスチャが空: %s" % path)
        self.length_s = max(v[-1][0] for v in self._by.values()) + 1.0
        self._t0 = time.time()

    def _at(self, topic, t):
        rows = self._by.get(topic)
        if not rows:
            return None
        cur = rows[0][1]
        for ts, v in rows:
            if ts > t:
                break
            cur = v
        return cur

    def telemetry(self, now=None):
        t = ((now if now is not None else time.time()) - self._t0) % self.length_s
        g = lambda name: self._at(name, t)
        low, cmd, bms, odom = g("rt/lowstate"), g("rt/lowcmd"), g("rt/lf/bmsstate"), g("rt/odommodestate")
        imu2, mb = g("rt/secondary_imu"), g("rt/lf/mainboardstate")
        if not (low and cmd and bms and odom):
            return None
        imu = low["imu_state"]
        strings = {n: {"value": self._json(g(n)), "age_s": 0.5} for n in STRING_TOPICS if g(n)}
        return {
            "battery": {"soc": bms["soc"], "current": bms["current"], "voltage": bms["bmsvoltage"],
                        "temperature": bms["temperature"], "age_s": 0.1},
            "imu": {"rpy": imu["rpy"], "gyro": imu["gyroscope"], "accel": imu["accelerometer"],
                    "temperature": imu["temperature"], "age_s": 0.1},
            "joints": {"age_s": 0.1, "mode_machine": low["mode_machine"], "items": [
                {"q": m["q"], "dq": m["dq"], "tau": m["tau_est"], "temperature": m["temperature"],
                 "lost": m["motorstate"]} for m in low["motor_state"][:JOINTS]]},
            "odom": {"position": odom["position"], "velocity": odom["velocity"], "yaw_speed": odom["yaw_speed"],
                     "mode": odom["mode"], "gait_type": odom["gait_type"], "body_height": odom["body_height"],
                     "error_code": odom["error_code"], "age_s": 0.1},
            "lowcmd": {"age_s": 0.01, "mode_machine": cmd["mode_machine"], "items": [
                {k: m[k] for k in ("q", "dq", "tau", "kp", "kd", "mode")} for m in cmd["motor_cmd"][:JOINTS]]},
            "imu2": imu2 and {"rpy": imu2["rpy"], "gyro": imu2["gyroscope"], "accel": imu2["accelerometer"],
                              "age_s": 0.1},
            "mainboard": mb and {k: mb[k] for k in ("fan_state", "state", "temperature", "value")} | {"age_s": 0.05},
            "estop": None, "remote": None,
            "strings": strings,
            "ages": {"rt/lowstate": 0.0, "rt/lowcmd": 0.0, "rt/lf/bmsstate": 0.0,
                     "rt/odommodestate": 0.0, "rt/secondary_imu": 0.0, "rt/servicestate": None},
            "system": SYSTEM,
        }

    @staticmethod
    def _json(v):
        try:
            return json.loads(v["data"])
        except (KeyError, TypeError, ValueError):
            return v
