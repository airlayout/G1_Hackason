"""11-10 手順 L2b（計画書 docs/plan/2026-11-09-g1-door-staged-rgbd.md 9.4・9.8 の 2）: 段 2 の扉の角度を、
レバーを下げた時からの左の手のひらの動き（順運動学 g1_arm_fk.py ∘ pelvis の姿勢）で出す。numpy だけ。

なぜ: 段 2 の終わり（扉 3°）はレバーの位置で手の前進 4.6 cm にあたり、扉の位置の誤差（2〜6 cm）と同じ大きさ。
扉の位置そのものでなく「レバーを下げた時からの差」を使えば、扉の位置の片寄りは差で消える。

流れ（実機・シムの評価とも同じ）:
  1. 閉じている間、閉じたレバーの当てる点を set_lever_ref で渡す（実機は段 2 の始めに止まって測った値）
  2. 毎歩 step(t, 手のひら, 手とレバーの力) を呼ぶ。基準の引き金（ref_mode）が立った歩の手のひらを基準にする。それまでは
     閉じた扉の値 [0, 1, 0.1] を返す
       "contact"（既定）: 手のひらが閉じたレバーの当てる点から水平 0.10 m 以内・力 5 N 以上が 5 歩（手が触れた）
       "press": 0100 と同じ（上に加えて 0.02 m 下）。シムの記録（11-10・229 台）では、下がりが 2 cm に届かないまま
                扉が 3° 開いた回が 10 台あり、そのあいだ角度は 0 のまま。contact は 2 台で、外れはほぼ同じ
     ⚠️ 脚を放す引き金（0100）は press のまま。ここで替えるのは角度の基準だけ
  3. 立った後は、基準からの手のひらの動きを扉の角度にして [角度 rad, 1, 古さ/1.0 s] を返す（door_angle_lidar と同じ形）
扉の角度の出し方（method）:
  "hinge"   蝶番の軸のまわりに、基準の手のひらから今の手のひらまで回った角度（板に沿って滑っても変わらない）
  "advance" 押す向き（閉じた板の法線）への前進量 ÷ 蝶番から基準の手のひらまでの板に沿った長さ（計画書の「手の前進量」）
扉の座標は door_frame.py と同じ（原点 = 開口の中心の床、+x = 押して入る向き、把手が左なら蝶番は y = −幅/2）。
lost_s を渡すと、手が離れたら（力がそのあいだ線を下回る）最後の角度を保ち、測れない扱いにする（既定は切ってある）。
段 2 で使うのは 3° を最初に越えた時まで
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

import door_frame as dfr
import g1_arm_fk as fk

# 0100（arm_stage.py の PRESS_DEFAULTS）と同じ線
PRESS_RADIUS_M = 0.10
PRESS_DROP_M = 0.02
PRESS_FORCE_N = 5.0
PRESS_HOLD = 5
# 方策へ渡す古さ（door_lidar_source と同じ尺度: 古さ / MAX_AGE_S）。順運動学は毎歩なので、遅れは姿勢の受け取りの分の目安
LATENCY_S = 0.1
MAX_AGE_S = 1.0
# 基準の後、力がこの間ずっと線を下回ったら手が離れたとみなす（lost_s に渡した時だけ）。⚠️ 既定は切ってある: シムの記録（11-10・229 台）では
# 0.3 s で 10 台が本当の 3° の前に止まり（大半は 0° のまま。開く前に手がレバーから一度離れて触れ直した）、早すぎは減らなかった。
# シムの力は手とレバーの接触だけ。実機の腕のトルクから出す力（L2c）がそろったら決め直す
LOST_S = 0.3
METHODS = ("hinge", "advance")
REF_MODES = ("contact", "press")


@dataclass(frozen=True)
class DoorLine:
    """床の上の閉じた扉: 蝶番の軸の位置・蝶番 → 戸先の向き・押して開く向き（どれも扉の座標か世界の xy）"""
    hinge_xy: np.ndarray
    along: np.ndarray
    push: np.ndarray

    @property
    def open_sign(self) -> float:
        """開く向きの回り（左回り +1・右回り −1）。along を push の側へ回す向き"""
        return float(np.sign(self.along[0] * self.push[1] - self.along[1] * self.push[0]))


def door_line(width: float, lever_left: bool = True) -> DoorLine:
    """扉の座標（door_frame.py の決まり）での閉じた扉。蝶番の軸は閉じた板の中央の面（x = 0）に置く"""
    lr = 1.0 if lever_left else -1.0
    return DoorLine(np.array([0.0, -lr * width / 2]), np.array([0.0, lr]), np.array([1.0, 0.0]))


def door_angle_signed(line: DoorLine, ref: np.ndarray, palm: np.ndarray, method: str = "hinge") -> float:
    """基準の手のひら ref から今の手のひら palm までの動きを扉の角度 [rad] にする（閉じる向きは負のまま）"""
    if method not in METHODS:
        raise ValueError(f"method は {METHODS} のどれか: {method}")
    v0 = np.asarray(ref, dtype=float)[:2] - line.hinge_xy
    v1 = np.asarray(palm, dtype=float)[:2] - line.hinge_xy
    if method == "hinge":
        cross = v0[0] * v1[1] - v0[1] * v1[0]
        return line.open_sign * math.atan2(cross, float(v0 @ v1))
    radius = abs(float(v0 @ line.along))
    if radius < 1e-6:
        raise ValueError("基準の手のひらが蝶番の軸の上にある（前進量から角度にできない）")
    return math.asin(max(-1.0, min(1.0, float((v1 - v0) @ line.push) / radius)))


def door_angle(line: DoorLine, ref: np.ndarray, palm: np.ndarray, method: str = "hinge") -> float:
    """door_angle_signed を 0 で切ったもの（扉は枠より閉じない）"""
    return max(0.0, door_angle_signed(line, ref, palm, method))


class PressTrigger:
    """「レバーを下げた」の引き金（0100 と同じ）: 手のひらが閉じたレバーの当てる点から水平 radius 以内・drop 以上下・
    力 min_force 以上の歩が hold 歩つづいたら立つ（一度立ったら立ったまま）。drop=None なら上下は見ない（手が触れた）。
    first_palm は、立った続きの最初の歩の手のひら（角度の基準に使う。hold 歩待つ間の動きを取りこぼさない）"""

    def __init__(self, radius: float = PRESS_RADIUS_M, drop: Optional[float] = PRESS_DROP_M,
                 min_force: float = PRESS_FORCE_N, hold: int = PRESS_HOLD) -> None:
        if not (radius > 0 and (drop is None or drop >= 0) and min_force >= 0 and hold >= 1):
            raise ValueError(f"引き金の線がおかしい: radius={radius} drop={drop} min_force={min_force} hold={hold}")
        self.radius, self.drop, self.min_force, self.hold = radius, drop, min_force, hold
        self.ref: Optional[np.ndarray] = None
        self.count = 0
        self.fired = False
        self.first_palm: Optional[np.ndarray] = None

    def set_ref(self, ref: np.ndarray) -> None:
        """閉じたレバーの当てる点（実機は段 2 の始めに止まって測った値）"""
        r = np.asarray(ref, dtype=float)
        if not np.all(np.isfinite(r)):
            raise ValueError(f"閉じたレバーの当てる点が有限でない: {r}")
        self.ref = r.copy()

    def step(self, palm: np.ndarray, force: float, ref: Optional[np.ndarray] = None) -> bool:
        if ref is not None:
            self.set_ref(ref)
        if self.fired:
            return True
        p = np.asarray(palm, dtype=float)
        if self.ref is None or not np.all(np.isfinite(p)) or not math.isfinite(force):
            self.count = 0
            return False
        low = self.drop is None or p[2] <= self.ref[2] - self.drop
        pressing = np.linalg.norm(p[:2] - self.ref[:2]) <= self.radius and low and force >= self.min_force
        if pressing and self.count == 0:
            self.first_palm = p.copy()
        self.count = self.count + 1 if pressing else 0
        self.fired = self.count >= self.hold
        return self.fired


class HandDoorAngle:
    """段 2 の扉の角度の観測（door_angle_lidar の形 [角度, 測れたか, 古さ/1.0 s]）を手のひらから作る。

    lost_s を渡すと、基準の後、力が lost_s のあいだずっと線を下回ったら「手が離れた」とみなし（既定は切ってある。LOST_S の注記）、以後は最後の角度を保って
    測れない（1 → 0）にし、古さを増やす（door_lidar_source と同じ形。触れ直しても戻さない: 板の別の所に触れると
    蝶番のまわりの角度が数度ずれる）。手のひらか力が有限でない歩も、その歩だけ測れない扱い"""

    def __init__(self, line: DoorLine, method: str = "hinge", ref_mode: str = "contact",
                 lost_s: Optional[float] = None, **trigger_kw) -> None:
        if method not in METHODS:
            raise ValueError(f"method は {METHODS} のどれか: {method}")
        if ref_mode not in REF_MODES:
            raise ValueError(f"ref_mode は {REF_MODES} のどれか: {ref_mode}")
        if ref_mode == "contact":
            trigger_kw.setdefault("drop", None)
        self.line, self.method, self.ref_mode, self.lost_s = line, method, ref_mode, lost_s
        self.trigger = PressTrigger(**trigger_kw)
        self.palm_ref: Optional[np.ndarray] = None
        self.release_t: Optional[float] = None
        self.angle = 0.0
        self.angle_signed = 0.0
        self.angle_t: Optional[float] = None
        self.contact_t: Optional[float] = None
        self.lost = False
        self.max_deg = 0.0

    @property
    def released(self) -> bool:
        return self.release_t is not None

    def set_lever_ref(self, ref: np.ndarray) -> None:
        """閉じたレバーの当てる点。基準が決まった後に呼んでも何も変わらない"""
        self.trigger.set_ref(ref)

    def _held(self, t: float) -> np.ndarray:
        age = min(MAX_AGE_S, max(LATENCY_S, t - (self.angle_t if self.angle_t is not None else t)))
        return np.array([self.angle, 0.0, age / MAX_AGE_S])

    def step(self, t: float, palm: np.ndarray, force: float) -> np.ndarray:
        if not self.released:
            if not self.trigger.step(palm, force):
                return dfr.door_closed_angle_obs()
            self.release_t = self.contact_t = t
            self.palm_ref = self.trigger.first_palm.copy()
        p = np.asarray(palm, dtype=float)
        if self.lost or not np.all(np.isfinite(p)) or not math.isfinite(force):
            return self._held(t)
        if force >= self.trigger.min_force:
            self.contact_t = t
        elif self.lost_s is not None and t - self.contact_t > self.lost_s:
            self.lost = True
            return self._held(t)
        self.angle_signed = door_angle_signed(self.line, self.palm_ref, p, self.method)
        self.angle = max(0.0, self.angle_signed)
        self.angle_t = t
        self.max_deg = max(self.max_deg, math.degrees(self.angle))
        return np.array([self.angle, 1.0, LATENCY_S / MAX_AGE_S])


def _quat_xyzw_to_mat(q: np.ndarray) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def palm_world(chain: fk.ArmFK, q: np.ndarray, root_pos: np.ndarray, root_quat_xyzw: np.ndarray, side: str = "left") -> np.ndarray:
    """シムの口: pelvis の位置と向き（四元数 xyzw）∘ 順運動学の手のひら"""
    return np.asarray(root_pos, dtype=float) + _quat_xyzw_to_mat(root_quat_xyzw) @ fk.palm_in_pelvis(chain, q, side)


def palm_in_door_frame(chain: fk.ArmFK, q: np.ndarray, pose: dfr.PlanarPose, imu_quat_wxyz: np.ndarray,
                       pelvis_z: float, side: str = "left") -> np.ndarray:
    """実機の口: 扉の座標の床の姿勢（段 0 の扉 ∘ FAST-LIO）・IMU の傾き（roll・pitch）・pelvis の高さ ∘ 順運動学の手のひら"""
    roll, pitch = dfr.roll_pitch_from_quat_wxyz(imu_quat_wxyz)
    R = dfr.rot_z(pose.yaw) @ dfr.tilt_matrix(roll, pitch)
    return np.array([pose.x, pose.y, pelvis_z]) + R @ fk.palm_in_pelvis(chain, q, side)
