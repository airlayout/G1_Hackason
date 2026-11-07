"""腕を動かす前の事前確認（実機）。Button_Press/J1-gen のボタン押し（real/auto_press.py の run_prechecks）と同じ項目・しきい値。

J1-gen の関数は引数の形がボタン押し用（--target など）なので、項目ごとの部品（motion_controller_check・color_changed・Check）を
import し、しきい値も同じ値を使う。どれか 1 つでも通らなければ、腕に何も送らずに終わる。

| 項目 | 基準 | 出典 |
|---|---|---|
| モーションコントローラ | 稼働中（arm_sdk の受け手がいる） | auto_press.py |
| 立っているか | 骨盤の傾き 5.0° 以下、腰 yaw/roll/pitch 各 3.0° 以下 | auto_press.py（傾きは 2026-11-01 に 3.0 → 5.0、origin/Dev/ButtonPress） |
| 頭カメラ | 2 枚のカラーの差 0.05 以上（画像が止まっていない） | auto_press.py |
| モータの mode | 腰・腕すべて 1（ゼロトルクでない） | auto_press.py |

腕を動かしている間は、J1-gen の ArmCommander が腰のずれ 2.9°（arm.yaml の waist_max_deviation_rad）で自動で中止して戻る。
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "Button_Press" / "J1-gen" / "real"))

from auto_press import Check, color_changed, motion_controller_check  # noqa: E402
from common.head_tilt import pelvis_tilt_deg  # noqa: E402
from common.robot_model import JOINT_NAMES, LEFT_ARM_IDX, RIGHT_ARM_IDX, WAIST_IDX  # noqa: E402

# J1-gen の auto_press.py と同じ値（傾きは origin/Dev/ButtonPress の最新。main はまだ 3.0）
MAX_PELVIS_TILT_DEG = 5.0
MAX_WAIST_DEG = 3.0
MIN_COLOR_CHANGE = 0.05


def run_prechecks(st: Any, src: Any, mock: bool = False) -> list[Check]:
    """st は lowstate（JointState）、src は頭カメラ（RgbdZmqSource、開いたもの）。

    mock=True（模擬ロボット、lo）: モーションコントローラは問い合わせず、頭カメラは届くかだけ見る（模擬の画像は止まっているのが正しい。
    J1-gen は模擬ロボットでは --skip-prechecks で事前確認ごと飛ばす）
    """
    checks = []
    if not mock:
        checks.append(motion_controller_check())
    else:
        checks.append(Check("モーションコントローラ", True, "模擬ロボット（lo）なので問い合わせない"))

    tilt = pelvis_tilt_deg(st.imu_quat)
    waist = np.degrees(st.q[list(WAIST_IDX)])
    stand_ok = tilt <= MAX_PELVIS_TILT_DEG and float(np.max(np.abs(waist))) <= MAX_WAIST_DEG
    checks.append(Check("立っているか", stand_ok, f"骨盤の傾き {tilt:.1f}°（上限 {MAX_PELVIS_TILT_DEG}°）、腰 yaw/roll/pitch "
                        + ", ".join(f"{v:+.1f}" for v in waist) + f"°（上限 {MAX_WAIST_DEG}°）"))

    a = src.read_rgbd()
    time.sleep(0.5)
    b = src.read_rgbd() if a is not None else None
    if a is None or b is None:
        checks.append(Check("頭カメラ", False, "画像が届かない（PC2 の配信サーバ）"))
    else:
        change = color_changed(a.color_bgr, b.color_bgr)
        msg = f"カラーの変化 {change:.2f}"
        if mock:
            checks.append(Check("頭カメラ", True, msg + "（模擬ロボットなので、届くかだけ見る）"))
        else:
            if change < MIN_COLOR_CHANGE:
                msg += "。カラーが止まっている（古い画像を返している。G1 の再起動で直った例がある）"
            checks.append(Check("頭カメラ", change >= MIN_COLOR_CHANGE, msg))

    off = [JOINT_NAMES[i] for i in list(WAIST_IDX) + list(LEFT_ARM_IDX) + list(RIGHT_ARM_IDX) if st.motor_mode[i] != 1]
    checks.append(Check("モータの mode", not off, "腰・腕すべて 1" if not off else
                        f"1 でない: {', '.join(off)}（ゼロトルク。リモコンでダンピングに入れてから立たせる）"))
    return checks
