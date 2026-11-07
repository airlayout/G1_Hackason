#!/usr/bin/env python3
"""PC2 の余力と受信を測る診断（読み取り専用）。Python 3.8 互換、標準ライブラリ中心。

測るもの: ① PC2 の機器情報と入っているモジュール ② PC2 から Pepper(9559) への到達
③ rt/lowstate と LiDAR 点群の受信（何も無い状態 / 負荷をかけた状態）④ YOLO の処理速度。

しないこと: G1 への指令（RPC・publish）、ファイルの書き込み。受信（DataReader）だけ行う。
使い方は docs/pc2-measurement.md。SSH の標準入力で流し込める（PC2 にファイルを置かない）。
"""
import argparse
import importlib.util
import json
import math
import os
import platform
import select
import shutil
import socket
import statistics
import subprocess
import sys
import time

sys.dont_write_bytecode = True

DDS_XML = """<CycloneDDS><Domain Id="0"><General><Interfaces>
<NetworkInterface name="eth0"/></Interfaces></General>
<Tracing><Verbosity>none</Verbosity></Tracing></Domain></CycloneDDS>"""
DEFAULT_SDK_PATH = "/home/unitree/unitree_sdk2_python"
LOWSTATE = "rt/lowstate"
LIDAR = "rt/utlidar/cloud_livox_mid360"
STALE_MS = {LOWSTATE: 100, LIDAR: 1000}  # lowstate の 100 ms は g1-pc2-doctor.py と同じ基準
READER_DEPTH = {LOWSTATE: 1024, LIDAR: 8}
MAX_SECONDS = 60
ABORT_GAP_MS = 500  # 負荷中に lowstate がこれ以上途切れたら、負荷を止めて測定を打ち切る（仮）
WORKER_READY_TIMEOUT_S = 180
MODULES = ("numpy", "cv2", "torch", "onnxruntime", "ultralytics", "tensorrt",
           "cyclonedds", "flask", "fastapi", "aiohttp", "websockets")
TOOLS = ("nvidia-smi", "tegrastats")

# 灰色の 1 枚絵を繰り返し推論する。検出物が無いぶん、実画像より速く出る可能性がある。
YOLO_WORKER = r"""
import json, sys, time
import numpy as np
from ultralytics import YOLO
weights, seconds = sys.argv[1], float(sys.argv[2])
model = YOLO(weights)
frame = np.full((480, 640, 3), 127, dtype=np.uint8)
model.predict(frame, verbose=False)
print("READY", flush=True)
times = []
end = time.monotonic() + seconds
while time.monotonic() < end:
    t0 = time.monotonic()
    model.predict(frame, verbose=False)
    times.append((time.monotonic() - t0) * 1000.0)
times.sort()
n = len(times)
print(json.dumps({"frames": n, "fps": n / seconds,
                  "ms_median": times[n // 2] if n else None,
                  "ms_p95": times[min(n - 1, int(n * 0.95))] if n else None}))
"""


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def read_text(path):
    try:
        with open(path, errors="replace") as handle:
            return handle.read().replace("\x00", "").strip()
    except OSError:
        return None


def has_module(name):
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):
        return False


def sysinfo():
    meminfo = {}
    for line in (read_text("/proc/meminfo") or "").splitlines():
        key, _, rest = line.partition(":")
        if key in ("MemTotal", "MemAvailable"):
            meminfo[key] = rest.strip()
    temps = {}
    base = "/sys/class/thermal"
    if os.path.isdir(base):
        for name in sorted(os.listdir(base)):
            raw = read_text(os.path.join(base, name, "temp")) if name.startswith("thermal_zone") else None
            if raw and raw.lstrip("-").isdigit():
                temps[read_text(os.path.join(base, name, "type")) or name] = int(raw) / 1000.0
    return {
        "python": sys.version.split()[0], "machine": platform.machine(), "system": platform.system(),
        "device_model": read_text("/proc/device-tree/model"), "cpu_count": os.cpu_count(),
        "loadavg": os.getloadavg() if hasattr(os, "getloadavg") else None,
        "memory": meminfo, "temperatures_c": temps,
        "modules": {name: has_module(name) for name in MODULES},
        "tools": {name: shutil.which(name) for name in TOOLS},
    }


def reach_pepper(ip, port, tries=5, timeout=2.0):
    results = []
    for _ in range(tries):
        started = time.monotonic()
        try:
            with socket.create_connection((ip, port), timeout=timeout):
                results.append(round((time.monotonic() - started) * 1000.0, 1))
        except OSError as exc:
            results.append(str(exc))
    connected = [r for r in results if isinstance(r, float)]
    return {"ip": ip, "port": port, "connect_ms": results, "ok": len(connected) == tries,
            "note": "TCP 接続だけ確認。NAOqi の応答と映像は含まない"}


class DdsSampler:
    """受信専用。writer も RPC クライアントも作らない。"""

    def __init__(self, sdk_path):
        if not os.path.isfile(os.path.join(sdk_path, "unitree_sdk2py", "__init__.py")):
            raise RuntimeError("unitree_sdk2py が見つかりません: " + sdk_path)
        sys.path.insert(0, sdk_path)
        from cyclonedds.domain import Domain, DomainParticipant
        from cyclonedds.qos import Policy, Qos
        from cyclonedds.sub import DataReader
        from cyclonedds.topic import Topic
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        self._domain = Domain(0, DDS_XML)
        self._participant = DomainParticipant(0)
        types = {LOWSTATE: LowState_, LIDAR: PointCloud2_}
        self.readers = {}
        for name, typ in types.items():
            qos = Qos(Policy.Reliability.BestEffort, Policy.Durability.Volatile,
                      Policy.History.KeepLast(READER_DEPTH[name]))
            self.readers[name] = DataReader(self._participant, Topic(self._participant, name, typ), qos)

    def sample(self, seconds, guard=None):
        """seconds 秒受信して要約を返す。guard(arrivals) が True を返したら打ち切る。"""
        from cyclonedds.internal import InvalidSample
        arrivals = {name: [] for name in self.readers}
        last = {}
        started_ns = time.monotonic_ns()
        end = time.monotonic() + seconds
        aborted = None
        while time.monotonic() < end:
            for name, reader in self.readers.items():
                for sample in reader.take(1024):
                    if isinstance(sample, InvalidSample):
                        continue
                    arrivals[name].append(time.monotonic_ns())
                    if name == LIDAR:
                        last[name] = {"frame_id": sample.header.frame_id,
                                      "points": int(sample.width) * int(sample.height)}
            if guard is not None:
                aborted = guard(arrivals, started_ns)
                if aborted:
                    break
            time.sleep(0.0005)
        return {"aborted": aborted, "topics": {n: summarize(n, a, started_ns, last.get(n))
                                               for n, a in arrivals.items()}}


def summarize(name, arrivals_ns, started_ns, extra):
    now_ns = time.monotonic_ns()
    elapsed = (now_ns - started_ns) / 1e9
    gaps = [(b - a) / 1e6 for a, b in zip(arrivals_ns, arrivals_ns[1:])]
    out = {"samples": len(arrivals_ns), "rate_hz": len(arrivals_ns) / elapsed if elapsed else None,
           "first_sample_delay_ms": (arrivals_ns[0] - started_ns) / 1e6 if arrivals_ns else None,
           "ending_gap_ms": (now_ns - arrivals_ns[-1]) / 1e6 if arrivals_ns else None,
           "gap_median_ms": statistics.median(gaps) if gaps else None,
           "gap_p95_ms": percentile(gaps, 0.95), "gap_max_ms": max(gaps) if gaps else None,
           "stale_threshold_ms": STALE_MS[name], "stale_events": sum(g > STALE_MS[name] for g in gaps)}
    out.update(extra or {})
    return out


def lowstate_guard(arrivals, started_ns):
    """負荷中に lowstate が途切れたら理由の文字列を返す（負荷を止めるため）。"""
    now_ns = time.monotonic_ns()
    seen = arrivals[LOWSTATE]
    silent_ms = (now_ns - (seen[-1] if seen else started_ns)) / 1e6
    if silent_ms > ABORT_GAP_MS:
        return "lowstate が %.0f ms 途切れた（上限 %d ms）" % (silent_ms, ABORT_GAP_MS)
    return None


class Load:
    """試験用の負荷。cpu:N は N 個の空回りプロセス、yolo:PATH は YOLO の連続推論。"""

    def __init__(self, spec, seconds):
        kind, _, arg = spec.partition(":")
        if kind not in ("cpu", "yolo") or not arg:
            raise ValueError("--load は cpu:N か yolo:重みのパス")
        self.kind, self.arg, self.seconds, self.procs = kind, arg, seconds, []

    def start(self):
        if self.kind == "cpu":
            limit = max(1, (os.cpu_count() or 2) - 1)  # 1 コアは残す
            count = min(int(self.arg), limit)
            for _ in range(count):
                self.procs.append(subprocess.Popen([sys.executable, "-c", "while True: pass"]))
            return {"kind": "cpu", "procs": count, "cap": limit}
        proc = subprocess.Popen([sys.executable, "-c", YOLO_WORKER, self.arg, str(self.seconds + 2)],
                                stdout=subprocess.PIPE, universal_newlines=True)
        self.procs.append(proc)
        ready, _, _ = select.select([proc.stdout], [], [], WORKER_READY_TIMEOUT_S)
        if not ready or proc.stdout.readline().strip() != "READY":
            raise RuntimeError("YOLO の準備ができません（ultralytics と重みのパスを確認）")
        return {"kind": "yolo", "weights": self.arg}

    def stop(self):
        result = None
        for proc in self.procs:
            if self.kind == "yolo":
                try:
                    out, _ = proc.communicate(timeout=self.seconds + 30)
                    result = json.loads(out.strip().splitlines()[-1]) if out.strip() else None
                except (subprocess.TimeoutExpired, ValueError):
                    pass
            proc.terminate()
        for proc in self.procs:
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
        return result


def compare(idle, loaded):
    out = {}
    for name in (LOWSTATE, LIDAR):
        a, b = idle["topics"][name], loaded["topics"][name]
        out[name] = {"rate_hz": [a["rate_hz"], b["rate_hz"]], "gap_max_ms": [a["gap_max_ms"], b["gap_max_ms"]],
                     "stale_events": [a["stale_events"], b["stale_events"]]}
    return out


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dds", action="store_true", help="lowstate と LiDAR の受信を測る")
    parser.add_argument("--seconds", type=float, default=10, help="1 回の受信時間（秒）")
    parser.add_argument("--sdk-path", default=os.environ.get("G1_SDK_PATH", DEFAULT_SDK_PATH))
    parser.add_argument("--pepper-ip", help="Pepper への TCP 到達（9559）を確認")
    parser.add_argument("--pepper-port", type=int, default=9559)
    parser.add_argument("--load", help="負荷をかけて受信を測る: cpu:N または yolo:重みのパス（--dds が要る）")
    args = parser.parse_args(argv)
    if not 0 < args.seconds <= MAX_SECONDS:
        parser.error("--seconds は 0 より大きく %d 以下" % MAX_SECONDS)
    if args.load and not args.dds:
        parser.error("--load には --dds が要る")
    return args


def main(argv=None):
    args = parse_args(argv)
    report = {"sysinfo": sysinfo()}
    if args.pepper_ip:
        report["pepper"] = reach_pepper(args.pepper_ip, args.pepper_port)
    if args.dds:
        sampler = DdsSampler(args.sdk_path)
        report["idle"] = sampler.sample(args.seconds)
        if args.load:
            load = Load(args.load, args.seconds)
            try:
                report["load_started"] = load.start()
                report["loaded"] = sampler.sample(args.seconds, guard=lowstate_guard)
            finally:
                report["load_result"] = load.stop()
            report["compare_idle_vs_load"] = compare(report["idle"], report["loaded"])
    print(json.dumps(report, indent=2, ensure_ascii=False, default=str))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, ValueError) as exc:
        print("診断に失敗:", exc, file=sys.stderr)
        raise SystemExit(2)
