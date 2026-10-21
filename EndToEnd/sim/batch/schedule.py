"""物理tickを唯一の時計として、制御・RGB・LiDARの取得周期を共有する。"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CaptureSchedule:
    physics_dt_s: float
    control_ticks: int
    camera_ticks: int
    scan_ticks: int

    def __post_init__(self):
        if not math.isfinite(self.physics_dt_s) or self.physics_dt_s <= 0:
            raise ValueError("physics_dt_sは有限の正数です")
        ticks = (self.control_ticks, self.camera_ticks, self.scan_ticks)
        if any(type(value) is not int or value < 1 for value in ticks):
            raise ValueError("制御・センサーのtick周期は正の整数です")
        if self.scan_ticks % self.control_ticks:
            raise ValueError("LiDAR周期は制御周期の整数倍にしてください")

    @classmethod
    def from_configs(cls, environment, sensors):
        camera, scan = sensors.periods_in_steps(environment.physics_dt)
        control = round(environment.control_dt / environment.physics_dt)
        return cls(environment.physics_dt, control, camera, scan)

    @property
    def block_steps(self):
        return self.scan_ticks // self.control_ticks

    def validate_steps(self, steps):
        if steps < self.block_steps or steps % self.block_steps:
            raise ValueError(f"stepsは{self.block_steps}以上の{self.block_steps}の倍数です")

    def frame_counts(self, steps):
        self.validate_steps(steps)
        ticks = steps * self.control_ticks
        return {
            "camera": (ticks - 1) // self.camera_ticks + 1,
            "lidar": ticks // self.scan_ticks,
            "imu_each": ticks + 1,
        }
