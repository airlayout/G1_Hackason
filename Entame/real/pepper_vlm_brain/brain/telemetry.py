"""Optional system-wide NVIDIA measurements, without dependency installation."""
import subprocess
from threading import Thread
from time import perf_counter


class GpuSampler:
    def __init__(self, enabled=False):
        self.enabled = enabled
        self.samples = []
        self.error = None
        self.process = None
        self.thread = None

    def _read(self):
        for line in self.process.stdout:
            try:
                utilization, used, total = [float(v.strip()) for v in line.strip().split(",")]
                self.samples.append({"time": perf_counter(), "utilization_percent": utilization,
                                     "memory_used_mib": used, "memory_total_mib": total})
            except ValueError:
                self.error = "Invalid nvidia-smi output"

    def __enter__(self):
        if self.enabled:
            try:
                self.process = subprocess.Popen(
                    ["nvidia-smi", "--id=0", "--query-gpu=utilization.gpu,memory.used,memory.total",
                     "--format=csv,noheader,nounits", "--loop-ms=500"],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
                self.thread = Thread(target=self._read, daemon=True)
                self.thread.start()
            except OSError as exc:
                self.error = str(exc)
        return self

    def summarize(self, begin, end):
        samples = [s for s in self.samples if begin <= s["time"] <= end]
        if not samples:
            return {"scope": "system GPU, not just this process", "sample_count": 0, "error": self.error}
        return {"scope": "system GPU, not just this process", "sample_count": len(samples),
                "utilization_mean_percent": sum(s["utilization_percent"] for s in samples) / len(samples),
                "utilization_peak_percent": max(s["utilization_percent"] for s in samples),
                "memory_peak_mib": max(s["memory_used_mib"] for s in samples)}

    def __exit__(self, *args):
        if self.process is not None:
            self.process.terminate()
            self.process.wait(timeout=5)
            self.thread.join(timeout=1)
            self.process.stdout.close()
