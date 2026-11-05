"""頭カメラ（D435i）の床からの俯角とロールを、カメラ自身の加速度計から出す（読むだけ。機体には何も送らない）。

2026-11-05: 手順 9.4 の 3（床の AprilTag で俯角を測る）の代わり。D435i の IMU は PC2 の usbhid が
IIO（/sys/bus/iio/devices/iio:device*/name が accel_3d・gyro_3d）として出しているので、sysfs を読むだけで取れる
（videohub がカラーを握っていても読める）。カメラと同じ基板なので、ライダーの IMU と違い取付の差を測らなくてよい。

軸の読み方: 生の y＝画像の下、生の z＝カメラの後ろ（librealsense が D435i で z を反転するのと同じ）。
止まっている間の加速度計は上を指すので、俯角 = atan2(a_z, −a_y)（水平で 0、真下で 90）、ロール = atan2(a_x, √(a_y²+a_z²))。
11-05 の実測: 10 秒 88 回で 53.20°・ロール 0.05°・|a| は 1 g の +0.3%（1 回ずつは ±2.2° 振れる＝平均で使う）。
ジャイロが STILL_GYRO を越えた回は捨てる（歩いている・揺れている間は加速度が混ざる）。

使い方（PC2）: python3 cam_tilt.py [秒（既定 10）]
出力: [cam_tilt] 俯角（床から） 53.20°（sd 2.21°・n 88）ロール 0.05°・|a| 1g の +0.3%
"""
import glob
import math
import os
import statistics
import sys
import time
from typing import List, Optional, Sequence, Tuple

IIO = "/sys/bus/iio/devices"
D435I_USB = "8086:0B3A"  # sysfs のパスに出る D435i の USB の番号
STILL_GYRO = 0.05  # [rad/s] これより回っていたら捨てる
PERIOD = 0.1  # [s] HID の既定の周期（in_accel_sampling_frequency = 10）


def find_iio(name: str) -> Optional[str]:
    """D435i の IIO の口（name が accel_3d／gyro_3d）を探す。番号は起動ごとに変わりうる"""
    for d in sorted(glob.glob(os.path.join(IIO, "iio:device*"))):
        try:
            with open(os.path.join(d, "name")) as f:
                if f.read().strip() != name:
                    continue
        except OSError:
            continue
        if D435I_USB.lower() in os.path.realpath(d).lower():
            return d
    return None


def tilt_from_accel(ax: float, ay: float, az: float) -> Tuple[float, float]:
    """生の加速度（m/s²）から (床からの俯角°, ロール°)。水平で前を向くと (0, 0)、真下を向くと俯角 90"""
    pitch = math.degrees(math.atan2(az, -ay))
    roll = math.degrees(math.atan2(ax, math.hypot(ay, az)))
    return pitch, roll


def summarize(acc: Sequence[Sequence[float]]) -> Tuple[float, float, float, float]:
    """止まっている回の加速度から (俯角°, 1 回ずつの俯角の sd°, ロール°, |a| の 1 g 比 %)。平均の加速度で角度を出す"""
    mean = [statistics.mean(a[i] for a in acc) for i in range(3)]
    pitch, roll = tilt_from_accel(*mean)
    sd = statistics.pstdev([tilt_from_accel(*a)[0] for a in acc]) if len(acc) > 1 else 0.0
    norm = math.sqrt(sum(v * v for v in mean))
    return pitch, sd, roll, (norm / 9.80665 - 1.0) * 100.0


def read_vec(d: str, kind: str, scale: float) -> List[float]:
    out = []
    for k in "xyz":
        with open(os.path.join(d, "in_%s_%s_raw" % (kind, k))) as f:
            out.append(float(f.read()) * scale)
    return out


def main() -> int:
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    da, dg = find_iio("accel_3d"), find_iio("gyro_3d")
    if da is None:
        print("[cam_tilt] D435i の加速度計（IIO の accel_3d）が無い。lsusb に 8086:0b3a があるか（頭カメラの Type-C）を見る")
        return 1
    with open(os.path.join(da, "in_accel_scale")) as f:
        sa = float(f.read())
    sg = None
    if dg is not None:
        with open(os.path.join(dg, "in_anglvel_scale")) as f:
            sg = float(f.read())
    acc: List[List[float]] = []
    dropped = 0
    t_end = time.time() + secs
    while time.time() < t_end:
        a = read_vec(da, "accel", sa)
        if sg is not None and math.sqrt(sum(v * v for v in read_vec(dg, "anglvel", sg))) > STILL_GYRO:
            dropped += 1
        else:
            acc.append(a)
        time.sleep(PERIOD)
    if not acc:
        print("[cam_tilt] 止まっている回が無い（%d 回とも揺れていた）。機体を止めて測り直す" % dropped)
        return 1
    pitch, sd, roll, dn = summarize(acc)
    print("[cam_tilt] 俯角（床から） %.2f°（sd %.2f°・n %d・揺れて捨てた %d）ロール %.2f°・|a| 1g の %+.1f%%"
          % (pitch, sd, len(acc), dropped, roll, dn))
    if abs(dn) > 3.0:
        print("[cam_tilt] ⚠️ |a| が 1 g から 3%% 以上ずれている＝動いている間に測った疑い。止まってから測り直す")
    return 0


if __name__ == "__main__":
    sys.exit(main())
