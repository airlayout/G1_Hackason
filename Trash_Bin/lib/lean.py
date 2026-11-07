"""ゴミ箱を押すときの腕の指令（Button_Press/J1-gen の ArmCommander を継承）。sim / real 共通。

足しているのは 2 つ:
- 押し込んだまま保持する間に、何回かに分けて画像を撮る（follow）。今の流れで使っている
- 腰を前に傾けて押す（lean_waist_pitch_deg）。**今は使っていない**: 安全のため腰は曲げず、ゴミ箱を台に載せる方針にした
  （2026-11-07）。腰を傾ければ届く範囲は広がる（10° で約 3 cm 深く届く）ので、仕組みだけ残している

以下は腰を傾ける仕組みの説明。
ArmCommander は arm_sdk の経路で腰の 3 関節に「開始時の角度を保つ」指令を送っているので、その目標をゆっくり動かして傾ける。

なぜ ArmCommander の中でやるのか: 指令を終えると weight が 0 に戻り、腰は内蔵コントローラの姿勢（直立）へ戻る。
傾けた姿勢で押すには、傾ける → 押す → 戻る → 腰を戻す を、1 回の指令の区間（with 文）の中で行う必要がある。

順番:
  start（weight 0→1、J1-gen のまま）→ 腰を傾ける → …押す…（J1-gen）→ 腰を戻す → 安全終了（weight 1→0、J1-gen のまま）
  途中で中止したとき: J1-gen の「来た経路を戻る」→ 腰を戻す → weight 1→0

安全の確認:
  - 傾ける速さは max_speed_deg_s 以下（smoothstep で、最大の速さがこの値になる長さにする）
  - J1-gen の「腰が開始時から waist_max_deviation_rad（2.9°）ずれたら中止」は、傾けている間は「今の目標の角度」を基準にする
  - 傾け終わって落ち着いたあと、実測が目標から max_error_deg 以上ずれていたら中止する（腰が付いてこない）
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Any

import numpy as np

from common.arm import ArmCommander, WaistDeviationError
from common.robot_model import WAIST_IDX

WAIST_PITCH = 14  # motor 番号（waist_yaw 12、waist_roll 13、waist_pitch 14）


class LeaningCommander(ArmCommander):
    """開始の直後に腰を lean_rad まで前に傾け、終了の直前に開始時の角度へ戻す ArmCommander。"""

    def __init__(self, *args: Any, lean_rad: float = 0.0, max_speed_deg_s: float = 5.0,
                 settle_s: float = 1.0, max_error_deg: float = 2.0, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.lean_rad = float(lean_rad)
        self.max_speed = np.radians(float(max_speed_deg_s))
        self.settle_s = float(settle_s)
        self.max_error = np.radians(float(max_error_deg))
        self._pitch_start: float | None = None  # 開始時の腰の pitch（戻す先）
        self._leaned = False
        self._dry_lean = False  # dry-run で、傾けたつもりにしているか
        # 押し込んだまま保持する間に、hold_frames 回に分けて呼ぶ関数（画像を撮る。lib/bin_task.py の BinPipeline が入れる）
        self.hold_capture: Callable[[int], None] | None = None
        self.hold_frames = 3

    # ---- 押し込んだまま保持する間に画像を撮る ----------------------------------------

    def follow(self, waypoints_arm: Sequence[np.ndarray], label: str = "軌道",
               stop_when: Callable[[np.ndarray, np.ndarray], bool] | None = None, **kwargs: Any) -> int:
        """J1-gen の follow。ただし「押し込んだまま保持」は hold_frames 回に分けて送り、区切りごとに hold_capture を呼ぶ。
        kwargs（J1-gen の新しい版の expect_motion など）はそのまま渡す。

        頭カメラの配信は 1 秒に 5 枚なので、押し込みが止まった瞬間の 1 枚だけだと、フラップが開き切る前の画像になりうる。
        区切りの間（画像を撮る数十 ms）は指令を送らないが、腕は最後の指令を保つ。2 つ目の区切りからは Enter を待たない
        （腕を伸ばしたまま待つと体が傾いていく。2026-10-31 の J1-gen の記録）。
        """
        wps = list(waypoints_arm)
        k = self.hold_frames
        if label != "押し込んだまま保持" or self.hold_capture is None or len(wps) < k:
            return super().follow(wps, label, stop_when, **kwargs)
        confirm, sent = self.confirm, 0
        try:
            for i in range(k):
                sent += super().follow(wps[i * len(wps) // k:(i + 1) * len(wps) // k], label, stop_when, **kwargs) or 0
                self.confirm = False
                try:
                    self.hold_capture(i)
                except Exception as e:  # noqa: BLE001  画像が撮れなくても、腕の動きは止めない
                    print(f"[lean] 保持中の画像を撮れなかった（{type(e).__name__}: {e}）")
        finally:
            self.confirm = confirm
        return sent

    # ---- 腰を動かす ----------------------------------------------------------------

    def _move_waist_pitch(self, target: float, label: str) -> None:
        """腰の pitch の目標を target へゆっくり動かし、落ち着くまで待って、付いてきたかを確かめる。"""
        k = list(WAIST_IDX).index(WAIST_PITCH)
        p0 = float(self._cmd.q[WAIST_PITCH])
        dist = abs(target - p0)
        if dist < 1e-6:
            return
        # smoothstep の最大の速さは平均の 1.5 倍なので、長さ = 1.5 × 距離 ÷ 上限の速さ
        duration = max(1.0, 1.5 * dist / self.max_speed)
        self._confirm(f"{label}（{np.degrees(p0):+.1f}° → {np.degrees(target):+.1f}°、{duration:.1f} 秒）")
        print(f"[lean] {label}: 腰の pitch {np.degrees(p0):+.1f}° → {np.degrees(target):+.1f}°（{duration:.1f} 秒）")
        n = max(1, int(np.ceil(duration / self.dt)))
        for i in range(1, n + 1):
            s = i / n
            a = s * s * (3 - 2 * s)
            p = p0 + (target - p0) * a
            self._cmd.q[WAIST_PITCH] = p
            # J1-gen の腰のずれの確認は、今の目標を基準にする（傾けている途中で中止にならないように）
            self._waist_start[k] = p
            self._send_tick()
        for _ in range(int(self.settle_s / self.dt)):
            self._send_tick()
        if self.backend.dry_run:
            return
        meas = float(self._fresh_state().q[WAIST_PITCH])
        err = abs(meas - target)
        print(f"[lean] 腰の pitch の実測 {np.degrees(meas):+.1f}°（目標との差 {np.degrees(err):.1f}°）")
        if err > self.max_error:
            raise WaistDeviationError(f"腰が指令に付いてこない（目標 {np.degrees(target):+.1f}°、"
                                      f"実測 {np.degrees(meas):+.1f}°、差 {np.degrees(err):.1f}°）。中止する")

    def lean(self) -> None:
        """腰を開始時の角度 + lean_rad まで傾ける。"""
        if self.lean_rad == 0.0:
            return
        if self.backend.dry_run:
            # dry-run は指令を送らないので、腰は傾かない。安全確認の基準は動かさず、計画の確かめ（measured_q）だけ
            # 傾けたつもりの角度で行う（基準を動かすと、実測が付いてこないので「腰がずれた」で中止になる）
            self._dry_lean = True
            print(f"[lean] dry-run なので腰は動かさない（計画は腰 {np.degrees(self.lean_rad):+.1f}° のつもりで確かめる）")
            return
        self._pitch_start = float(self._cmd.q[WAIST_PITCH])
        self._leaned = True  # 途中で中止しても戻すように、動かし始める前に立てる
        self._move_waist_pitch(self._pitch_start + self.lean_rad, "腰を前に傾ける")

    def unlean(self) -> None:
        """腰を開始時の角度へ戻す。"""
        if not self._leaned or self._pitch_start is None:
            return
        self._move_waist_pitch(self._pitch_start, "腰を戻す")
        self._leaned = False

    def measured_q(self) -> np.ndarray:
        """lowstate の最新の関節角。dry-run で傾けたつもりのときは、腰の pitch に lean_rad を足す
        （J1-gen は押す手前で、この値の腰の角度で計画を立て直す）。"""
        q = super().measured_q()
        if self._dry_lean:
            q[WAIST_PITCH] += self.lean_rad
        return q

    # ---- J1-gen の開始・終了に組み込む ----------------------------------------------------

    def start(self) -> None:
        super().start()
        self.lean()

    def safe_stop(self, retreat: bool = False) -> None:
        if self._stopped or self._stopping:
            return
        if self._started and self._leaned:
            try:
                if retreat:
                    self._retreat_to_start()  # 先に腕を開始姿勢へ戻す（J1-gen と同じ）
                    retreat = False
                self.unlean()
            except Exception as e:  # noqa: BLE001  腰を戻せなくても、weight は下げる（内蔵コントローラが直立へ戻す）
                print(f"[lean] ⚠️ 腰を戻せなかった（{type(e).__name__}: {e}）。このまま weight を下げる")
        super().safe_stop(retreat=retreat)
