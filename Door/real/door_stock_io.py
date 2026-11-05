"""11-02: 扉の方策を純正の脚で動かすための部品（door_run_stock.py が使う）。numpy と unitree_sdk2py だけ（PC2 の Python 3.8）。

純正の歩行（LocoClient）には歩きの指令だけを渡し、腕は rt/arm_sdk で借りる（通常運転 501 のときだけ効く。10-27 の遠隔操作で確認）。
  腕     rt/arm_sdk に腕 14 関節（SDK 15〜28）と借りる重み（motor_cmd[29].q、0〜1）を送る。腰と脚は純正のまま
  歩き   別の糸から SetVelocity（有効 0.5 秒）を 20 回/秒。制御の輪（50 Hz）は RPC の応答を待たない。
         送り手が止まっても純正は 0.5 秒で止まる。Move() と StopMove() は SetVelocity の戻り値を捨てるので使わない
         （Navigation/real/loco_driver.py の 09-17 の教訓。拒否が見えない）
  脚の観測 学習の生徒は観測の actions の先頭 15 で HOMIE の脚と腰の行動を見ていた（MASK=0）。純正では、HOMIE を計算だけ回した
         出力（影の HOMIE。脚には送らない）を入れるのが既定。関節の角度から逆算した値・0 にも替えられる
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, replace
from typing import Callable, Optional, Tuple

import numpy as np

import door_core as dc

ARM_IDX = tuple(range(dc.NUM_HOMIE, dc.NUM_DOF))  # SDK 15〜28（左腕 7・右腕 7）
WEIGHT_IDX = 29  # rt/arm_sdk の借りる重みの置き場所（motor_cmd[29].q）
WAIST_IDX = (12, 13, 14)  # 腰 yaw・roll・pitch。arm_sdk の重みは腰も純正から取り上げる（11-03 実機）
WALKING_FSM_IDS = (500, 501)  # 歩ける FSM（loco_driver.py と同じ。802 の立位ロックでは指令が通っても足が動かない）
VEL_RATE_HZ = 20.0
VEL_DURATION_S = 0.5  # 1 回の SetVelocity が効く時間
STOP_REPEATS = 3  # 止めるときに速度 0 を送る回数
LEG_ACTIONS = ("shadow", "from_q", "zero")


def stock_velocity(p: dc.Params, cmd: np.ndarray) -> Tuple[bool, float, float, float]:
    """生徒の指令 cmd（[vx, vy, wz, …]・実の単位・±0.5 で切った後）を純正へ渡す (歩くか, vx, vy, wz) にする。

    歩く／立つは学習の HOMIE と同じ線: 観測の尺度 [2vx, 2vy, 0.5wz] の大きさが homie_switch_threshold を越えたら歩く
    """
    b = dc.homie_command_obs(p, cmd)
    if not float(np.linalg.norm(b[0:3])) > p.c["homie_switch_threshold"]:
        return False, 0.0, 0.0, 0.0
    return True, float(cmd[0]), float(cmd[1]), float(cmd[2])


def with_leg_actions(p: dc.Params, s: dc.CoreState, q: np.ndarray, mode: str) -> dc.CoreState:
    """次の周期の生徒の観測に入る脚と腰の行動（whole_prev の先頭 15）を mode で入れ替えた状態を返す。

    shadow: HOMIE を計算だけ回した出力のまま（door_core.step の値）／from_q: 今の関節の角度から逆算した (q − 既定) ÷ action_scale／
    zero: 0。HOMIE 自身の履歴（homie_a_prev）は変えない
    """
    if mode == "shadow":
        return s
    whole = s.whole_prev.copy()
    legs = slice(0, dc.NUM_HOMIE)
    if mode == "from_q":
        lim = p.c["action_clip"]
        whole[legs] = np.clip((q[legs] - p.q_default[legs]) / p.c["action_scale"], -lim, lim)
    elif mode == "zero":
        whole[legs] = 0.0
    else:
        raise ValueError(f"leg_actions={mode}（{LEG_ACTIONS} のどれか）")
    return replace(s, whole_prev=whole)


def move_toward(cur: np.ndarray, goal: np.ndarray, max_step: float) -> np.ndarray:
    """cur から goal へ、関節ごとに 1 周期 max_step まで近づけた値"""
    return cur + np.clip(goal - cur, -max_step, max_step)


class ArmSdkWriter:
    """rt/arm_sdk に腕 14 関節と借りる重みを送る（armsdk_probe.py と同じ作法）。ChannelFactoryInitialize の後に作る"""

    def __init__(self, mode_machine: int) -> None:
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = mode_machine
        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub.Init()

    def hold_waist(self, q_waist: np.ndarray, kp_waist: np.ndarray, kd_waist: np.ndarray) -> None:
        """腰 3 関節を q_waist で保つ。以後の write に毎回入る。
        ⚠️ 11-03 実機: arm_sdk の重みは腰も純正から取り上げる。腰を書かないと重み 1 で腰のトルクが 0 になり、胴が振られて純正が足踏みを続けた"""
        for k, j in enumerate(WAIST_IDX):
            m = self.msg.motor_cmd[j]
            m.mode = 1
            m.q, m.dq, m.tau = float(q_waist[k]), 0.0, 0.0
            m.kp, m.kd = float(kp_waist[k]), float(kd_waist[k])

    def write(self, q_arm: np.ndarray, kp_arm: np.ndarray, kd_arm: np.ndarray, weight: float) -> None:
        for k, j in enumerate(ARM_IDX):  # 腕 14 関節だけ。他は既定値のまま（無指令）
            m = self.msg.motor_cmd[j]
            m.mode = 1
            m.q, m.dq, m.tau = float(q_arm[k]), 0.0, 0.0
            m.kp, m.kd = float(kp_arm[k]), float(kd_arm[k])
        self.msg.motor_cmd[WEIGHT_IDX].q = float(min(1.0, max(0.0, weight)))
        self.msg.crc = self.crc.Crc(self.msg)
        self.pub.Write(self.msg)


def make_loco_client(rpc_timeout_s: float):
    """純正の歩行の RPC の口。ChannelFactoryInitialize の後に呼ぶ"""
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

    client = LocoClient()
    client.SetTimeout(rpc_timeout_s)
    client.Init()
    return client


def fsm_id(client) -> Optional[int]:
    """今の FSM。読めなければ None"""
    code, fsm = client.GetFsmId()
    return int(fsm) if code == 0 and fsm is not None else None


@dataclass(frozen=True)
class VelStatus:
    sent: int  # 送った RPC の数
    failed_in_row: int  # 続けて失敗した数（code が 0 でない・例外）
    last_code: Optional[int]
    last_rpc_ms: float
    want: Tuple[bool, float, float, float]  # 今送っている (歩くか, vx, vy, wz)
    damped: bool
    last_error: str
    age_s: float = 0.0  # この状態の古さ [s]。同じプロセスの送り手は常に 0、中継（loco_relay.LocoRelay）は最後に届いてからの秒


class VelocitySender:
    """純正へ歩きの指令を 20 回/秒で送る糸。制御の輪は set()・stop()・damp() で置くだけ（RPC を待たない）。

    歩く間は SetVelocity(vx, vy, wz, 0.5 秒)、歩くのをやめた・止めたときは SetVelocity(0, 0, 0, 0.5 秒) を STOP_REPEATS 回、
    それ以外は何も送らない（純正は最後の指令が切れて立つ）。damp() は脱力（Damp）を 1 回送り、以後は何も送らない
    """

    def __init__(self, client, rate_hz: float = VEL_RATE_HZ, duration_s: float = VEL_DURATION_S,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._client = client
        self._period = 1.0 / rate_hz
        self._duration = duration_s
        self._clock = clock
        self._lock = threading.Lock()
        self._want: Tuple[bool, float, float, float] = (False, 0.0, 0.0, 0.0)
        self._stop_left = 0
        self._damp_req = False
        self._damped = False
        self._sent = 0
        self._failed = 0
        self._code: Optional[int] = None
        self._rpc_ms = 0.0
        self._error = ""
        self._quit = threading.Event()
        self._thread = threading.Thread(target=self._run, name="stock-velocity", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def set(self, walk: bool, vx: float, vy: float, wz: float) -> None:
        with self._lock:
            if self._damped or self._damp_req:
                return
            if walk:
                self._want = (True, vx, vy, wz)
            elif self._want[0]:
                self._want = (False, 0.0, 0.0, 0.0)
                self._stop_left = STOP_REPEATS

    def stop(self) -> None:
        with self._lock:
            self._want = (False, 0.0, 0.0, 0.0)
            if not (self._damped or self._damp_req):
                self._stop_left = STOP_REPEATS

    def damp(self) -> None:
        with self._lock:
            self._want = (False, 0.0, 0.0, 0.0)
            self._stop_left = 0
            self._damp_req = not self._damped

    def status(self) -> VelStatus:
        with self._lock:
            return VelStatus(self._sent, self._failed, self._code, self._rpc_ms, self._want, self._damped, self._error)

    def close(self, timeout_s: float = 1.0) -> None:
        """止める指令（と脱力の求め）を送り切ってから糸を終える"""
        self.stop()
        t_end = self._clock() + timeout_s
        while self._clock() < t_end and self._thread.is_alive():
            with self._lock:
                if self._stop_left == 0 and not self._damp_req:
                    break
            time.sleep(0.01)
        self._quit.set()
        if self._thread.is_alive():
            self._thread.join(timeout_s)

    def _next_call(self):
        """この回に送るもの: ("damp", ()) / ("vel", (vx, vy, wz)) / None"""
        with self._lock:
            if self._damp_req:
                self._damp_req = False
                return "damp", ()
            walk, vx, vy, wz = self._want
            if walk:
                return "vel", (vx, vy, wz)
            if self._stop_left > 0:
                self._stop_left -= 1
                return "vel", (0.0, 0.0, 0.0)
            return None

    def _run(self) -> None:
        next_t = self._clock()
        while not self._quit.is_set():
            call = self._next_call()
            if call is not None:
                kind, args = call
                t0 = self._clock()
                code, err = None, ""
                try:
                    if kind == "damp":
                        self._client.Damp()  # SDK の Damp は戻り値を返さない
                        code = 0
                    else:
                        code = self._client.SetVelocity(*args, self._duration)
                except Exception as e:  # noqa: BLE001  糸を止めない。失敗は数えて輪の側が止める
                    code, err = -1, f"{type(e).__name__}: {e}"
                with self._lock:
                    self._sent += 1
                    self._failed = 0 if code == 0 else self._failed + 1
                    self._code = code
                    self._rpc_ms = (self._clock() - t0) * 1000.0
                    self._error = err or self._error
                    if kind == "damp":
                        self._damped = True
            next_t += self._period
            slack = next_t - self._clock()
            if slack > 0:
                self._quit.wait(slack)
            else:
                next_t = self._clock()
