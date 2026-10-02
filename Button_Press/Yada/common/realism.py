"""実機に近づける乱し（評価セット realistic）。シミュレーターに依存しない部分。

configs/contest.yaml の realism の範囲から、試行ごとの値を乱数の種で決める（Realism）。
センサの乱し（深度・カラー・関節のノイズ、画像の遅れ）、体の揺れ、カメラの取り付けの誤差、立ち位置のずれを扱う。
MuJoCo（contest/robots/mujoco_robot.py）と模擬 G1（sim/mujoco/g1_sim_server.py）の両方がこれを使う。
"""

from __future__ import annotations

import copy
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np

# 乱しの乱数の種の 2 つ目の値（基本の条件の乱数とは別の流れにする。同じ seed なら盤の配置は basic と同じになる）
REALISM_STREAM = 1
# 関節のセンサのノイズなど、毎周期の乱数の 2 つ目の値
NOISE_STREAM = 2


@dataclass(frozen=True)
class Realism:
    depth_sigma_at_1m: float
    depth_hole_ratio: float
    depth_edge_hole_prob: float
    depth_edge_jump_m: float
    color_noise_sigma: float
    color_gain: float
    camera_xyz_offset: np.ndarray  # (3,) [m]（torso_link 基準の取り付け位置に足す）
    camera_rpy_offset: np.ndarray  # (3,) [rad]
    image_latency_s: float
    image_fps: float
    command_latency_s: float
    q_sigma: float
    dq_sigma: float
    kp_scale: np.ndarray  # (29,)
    armature: float
    friction_nm: float
    sway_amp: np.ndarray  # (6,) x, y, z [m], roll, pitch, yaw [rad]
    sway_freq_hz: np.ndarray  # (6, 2) 軸ごとに 2 つの正弦波
    sway_phase: np.ndarray  # (6, 2)
    stance_xy: np.ndarray  # (2,) [m]（乗り場をロボットに対してずらす量）
    stance_yaw: float  # [rad]
    seed: int = 0
    extra: dict[str, Any] = field(default_factory=dict)

    def summary(self) -> dict[str, Any]:
        """記録用（結果の JSON に入れる）。"""
        return {
            "depth_sigma_at_1m_mm": round(self.depth_sigma_at_1m * 1000, 2),
            "depth_hole_ratio": round(self.depth_hole_ratio, 4),
            "color_gain": round(self.color_gain, 3),
            "camera_xyz_offset_mm": (self.camera_xyz_offset * 1000).round(2).tolist(),
            "camera_rpy_offset_deg": np.degrees(self.camera_rpy_offset).round(2).tolist(),
            "image_latency_ms": round(self.image_latency_s * 1000, 1),
            "image_fps": round(self.image_fps, 1),
            "command_latency_ms": round(self.command_latency_s * 1000, 1),
            "armature": round(self.armature, 4),
            "friction_nm": round(self.friction_nm, 3),
            "sway_amp_mm_deg": [*(self.sway_amp[:3] * 1000).round(2).tolist(),
                                *np.degrees(self.sway_amp[3:]).round(2).tolist()],
            "stance_xy_mm": (self.stance_xy * 1000).round(1).tolist(),
            "stance_yaw_deg": round(float(np.degrees(self.stance_yaw)), 2),
        }


def sample_realism(seed: int, cfg: dict[str, Any]) -> Realism:
    """configs/contest.yaml の realism から、種 seed の乱しを決める。"""
    rng = np.random.default_rng([int(seed), REALISM_STREAM])

    def u(v: Any, size: int | None = None) -> Any:
        """[最小, 最大] なら一様分布、数 1 つなら ± その値の一様分布。"""
        if isinstance(v, (list, tuple)):
            return rng.uniform(float(v[0]), float(v[1]), size)
        return rng.uniform(-float(v), float(v), size)

    # 引く順番を変えると、同じ種でも値が変わるので注意（足すときは最後に足す）
    d, c, m, lat, js, mo, sw, st = (cfg[k] for k in
                                    ("depth", "color", "camera_mount", "latency", "joint_sensor", "motors", "sway",
                                     "stance"))
    amp = np.concatenate([u(sw["amp_xy_m"], 2), [u(sw["amp_z_m"])], np.radians(u(sw["amp_rot_deg"], 3))])
    return Realism(
        depth_sigma_at_1m=float(u(d["sigma_at_1m_m"])),
        depth_hole_ratio=float(u(d["hole_ratio"])),
        depth_edge_hole_prob=float(d["edge_hole_prob"]),
        depth_edge_jump_m=float(d["edge_jump_m"]),
        color_noise_sigma=float(u(c["noise_sigma"])),
        color_gain=float(u(c["gain"])),
        camera_xyz_offset=u(m["xyz_m"], 3),
        camera_rpy_offset=np.radians(u(m["rpy_deg"], 3)),
        image_latency_s=float(u(lat["image_s"])),
        image_fps=float(u(lat["image_fps"])),
        command_latency_s=float(u(lat["command_s"])),
        q_sigma=float(js["q_sigma_rad"]),
        dq_sigma=float(js["dq_sigma_rad_s"]),
        kp_scale=u(mo["kp_scale"], 29),
        armature=float(u(mo["armature"])),
        friction_nm=float(u(mo["friction_nm"])),
        sway_amp=amp,
        sway_freq_hz=u(sw["freq_hz"], (6, 2)),
        sway_phase=rng.uniform(0.0, 2.0 * np.pi, (6, 2)),
        stance_xy=u(st["xy_m"], 2),
        stance_yaw=float(np.radians(u(st["yaw_deg"]))),
        seed=int(seed),
    )


def sway_offset(r: Realism, t: float) -> np.ndarray:
    """時刻 t の体の揺れ (x, y, z [m], roll, pitch, yaw [rad])。軸ごとに 2 つの正弦波の和（振幅は合わせて sway_amp 以内）。"""
    w = 2.0 * np.pi * r.sway_freq_hz * float(t)
    return r.sway_amp * 0.5 * np.sin(w + r.sway_phase).sum(axis=1)


def perturbed_robot_cfg(robot_cfg: dict[str, Any], r: Realism) -> dict[str, Any]:
    """頭カメラの取り付け位置と向きをずらした robot.yaml（シミュレーションのカメラにだけ使う）。"""
    out = copy.deepcopy(robot_cfg)
    cam = out["head_camera"]
    cam["xyz"] = (np.asarray(cam["xyz"], dtype=float) + r.camera_xyz_offset).tolist()
    cam["rpy"] = (np.asarray(cam["rpy"], dtype=float) + r.camera_rpy_offset).tolist()
    return out


def _smooth_noise(rng: np.random.Generator, shape: tuple[int, int], cell: int) -> np.ndarray:
    """標準偏差 1 くらいの、cell 画素ほどの大きさでつながったノイズ（粗い格子の乱数を引き伸ばす）。"""
    h, w = shape
    coarse = rng.standard_normal((h // cell + 2, w // cell + 2))
    yy = np.linspace(0, coarse.shape[0] - 1.001, h)
    xx = np.linspace(0, coarse.shape[1] - 1.001, w)
    y0, x0 = yy.astype(int), xx.astype(int)
    fy, fx = (yy - y0)[:, None], (xx - x0)[None, :]
    a = coarse[y0][:, x0]
    b = coarse[y0][:, x0 + 1]
    c = coarse[y0 + 1][:, x0]
    d = coarse[y0 + 1][:, x0 + 1]
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy


class SensorModel:
    """センサの乱し（深度・カラー・関節）。乱数は試行の種から決まる（同じ種なら同じ乱れ方）。"""

    def __init__(self, r: Realism):
        self.r = r
        self.rng = np.random.default_rng([r.seed, NOISE_STREAM])

    def depth(self, depth_m: np.ndarray) -> np.ndarray:
        """深度 [m]（0 = 測れない）にノイズと穴を足し、1 mm 単位に丸める。"""
        z = depth_m.astype(np.float64)
        # RealSense が測れる範囲の外（近すぎる・遠すぎる・何も無い）は 0
        valid = (z > 0.1) & (z < 10.0)
        sigma = self.r.depth_sigma_at_1m * z ** 2
        n = 0.7 * _smooth_noise(self.rng, z.shape, 8) + 0.3 * self.rng.standard_normal(z.shape)
        out = z + sigma * n
        # 穴: ところどころ、かたまりで測れなくなる
        if self.r.depth_hole_ratio > 0:
            blob = _smooth_noise(self.rng, z.shape, 12)
            out[blob > np.quantile(blob, 1.0 - self.r.depth_hole_ratio)] = 0.0
        # 物の縁: 隣との深度の差が大きいところは、確率で測れなくなる
        jump = np.zeros_like(z, dtype=bool)
        jump[:, 1:] |= np.abs(np.diff(z, axis=1)) > self.r.depth_edge_jump_m
        jump[1:, :] |= np.abs(np.diff(z, axis=0)) > self.r.depth_edge_jump_m
        out[jump & (self.rng.random(z.shape) < self.r.depth_edge_hole_prob)] = 0.0
        out[~valid] = 0.0
        return (np.round(np.clip(out, 0.0, None) * 1000.0) / 1000.0).astype(np.float32)

    def color(self, rgb: np.ndarray) -> np.ndarray:
        x = rgb.astype(np.float32) * self.r.color_gain + self.rng.normal(0.0, self.r.color_noise_sigma, rgb.shape)
        return np.clip(x + 0.5, 0, 255).astype(np.uint8)

    def joints(self, q: np.ndarray, dq: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return (q + self.rng.normal(0.0, self.r.q_sigma, q.shape),
                dq + self.rng.normal(0.0, self.r.dq_sigma, dq.shape))


class ImageDelay:
    """画像の遅れとコマ数。撮った画像を時刻付きでためておき、「今 − 遅れ」までに撮った最新の 1 枚を返す。"""

    def __init__(self, latency_s: float, fps: float, keep_s: float = 1.0):
        self.latency = float(latency_s)
        self.period = 1.0 / float(fps)
        self._buf: deque[tuple[float, Any]] = deque()
        self._keep = float(keep_s)
        self._next_capture = 0.0

    def due(self, t: float) -> bool:
        """時刻 t に画像を撮るべきか（コマ数に合わせる）。"""
        return t + 1e-9 >= self._next_capture

    def push(self, t: float, frame: Any) -> None:
        self._buf.append((t, frame))
        self._next_capture = max(self._next_capture + self.period, t)
        while self._buf and self._buf[0][0] < t - self._keep:
            self._buf.popleft()

    def get(self, t: float) -> tuple[float, Any]:
        """(撮った時刻, 画像)。まだ遅れの分の画像が無ければ、いちばん古いもの。"""
        best = self._buf[0]
        for item in self._buf:
            if item[0] <= t - self.latency:
                best = item
        return best


# ---- 弱点のレポートと、乱しを 1 種類ずつ入れる評価（contest/report.py、evaluate.py --ablation）のため --------

# 乱しの種類と、それに入る Realism の値の名前。順番はレポートに出す順
ABLATION_GROUPS: dict[str, tuple[str, ...]] = {
    "depth": ("depth_sigma_at_1m", "depth_hole_ratio", "depth_edge_hole_prob"),
    "color": ("color_noise_sigma", "color_gain"),
    "camera_mount": ("camera_xyz_offset", "camera_rpy_offset"),
    "latency": ("image_latency_s", "image_fps", "command_latency_s"),
    "joint_sensor": ("q_sigma", "dq_sigma"),
    "motors": ("kp_scale", "armature", "friction_nm"),
    "sway": ("sway_amp",),
    "stance": ("stance_xy", "stance_yaw"),
}
GROUP_LABELS: dict[str, str] = {
    "none": "乱しなし",
    "all": "すべての乱し",
    "depth": "深度のノイズ・穴",
    "color": "カラーのノイズ・明るさ",
    "camera_mount": "カメラの取り付けの誤差",
    "latency": "画像と指令の遅れ",
    "joint_sensor": "関節のセンサのノイズ",
    "motors": "モータ（PD のばらつき・armature・摩擦）",
    "sway": "体の揺れ",
    "stance": "立ち位置のずれ",
}


def neutral(r: Realism) -> Realism:
    """乱しをすべて理想の値（乱しなし）にした Realism。乱しの値の乱数の流れは変えない。"""
    import dataclasses

    return dataclasses.replace(
        r, depth_sigma_at_1m=0.0, depth_hole_ratio=0.0, depth_edge_hole_prob=0.0, color_noise_sigma=0.0,
        color_gain=1.0, camera_xyz_offset=np.zeros(3), camera_rpy_offset=np.zeros(3), image_latency_s=0.0,
        image_fps=1000.0, command_latency_s=0.0, q_sigma=0.0, dq_sigma=0.0, kp_scale=np.ones(29), armature=0.0,
        friction_nm=0.0, sway_amp=np.zeros(6), stance_xy=np.zeros(2), stance_yaw=0.0)


def only(r: Realism, group: str) -> Realism:
    """乱しの種類 group だけを残した Realism（"none" は乱しなし、"all" はすべて）。"""
    import dataclasses

    if group == "all":
        return r
    base = neutral(r)
    if group == "none":
        return base
    if group not in ABLATION_GROUPS:
        raise ValueError(f"乱しの種類が無い: {group}（あるのは {list(ABLATION_GROUPS)}）")
    return dataclasses.replace(base, **{f: getattr(r, f) for f in ABLATION_GROUPS[group]})


# レポートで使う、条件の数値（名前: (説明, 乱しの種類)）。大きいほど「きつい」向きにそろえる
FEATURES: dict[str, tuple[str, str]] = {
    "depth_sigma_at_1m_mm": ("深度のノイズ（1 m で）[mm]", "depth"),
    "depth_hole_ratio": ("深度の穴の割合", "depth"),
    "color_noise_sigma": ("カラーのノイズ", "color"),
    "color_gain_dev": ("カラーの明るさの倍率と 1 の差", "color"),
    "camera_xyz_offset_mm": ("カメラの取り付けの位置の誤差 [mm]", "camera_mount"),
    "camera_rpy_offset_deg": ("カメラの取り付けの角度の誤差 [°]", "camera_mount"),
    "image_latency_ms": ("画像の遅れ [ms]", "latency"),
    "image_period_ms": ("画像の間隔（1 / コマ数）[ms]", "latency"),
    "command_latency_ms": ("指令の遅れ [ms]", "latency"),
    "kp_scale_dev": ("PD の強さの倍率と 1 の差（関節の中で最大）", "motors"),
    "armature": ("armature", "motors"),
    "friction_nm": ("関節の摩擦 [Nm]", "motors"),
    "sway_xy_mm": ("体の揺れ（前後・左右）[mm]", "sway"),
    "sway_rot_deg": ("体の揺れ（傾き）[°]", "sway"),
    "stance_xy_mm": ("立ち位置の位置のずれ [mm]", "stance"),
    "stance_yaw_deg": ("立ち位置の向きのずれの大きさ [°]", "stance"),
}


def features(r: Realism) -> dict[str, float]:
    """Realism を、レポートで集計する数値にする（FEATURES の名前）。"""
    return {
        "depth_sigma_at_1m_mm": r.depth_sigma_at_1m * 1000.0,
        "depth_hole_ratio": r.depth_hole_ratio,
        "color_noise_sigma": r.color_noise_sigma,
        "color_gain_dev": abs(r.color_gain - 1.0),
        "camera_xyz_offset_mm": float(np.linalg.norm(r.camera_xyz_offset)) * 1000.0,
        "camera_rpy_offset_deg": float(np.degrees(np.linalg.norm(r.camera_rpy_offset))),
        "image_latency_ms": r.image_latency_s * 1000.0,
        "image_period_ms": 1000.0 / r.image_fps,
        "command_latency_ms": r.command_latency_s * 1000.0,
        "kp_scale_dev": float(np.max(np.abs(r.kp_scale - 1.0))),
        "armature": r.armature,
        "friction_nm": r.friction_nm,
        "sway_xy_mm": float(np.linalg.norm(r.sway_amp[:2])) * 1000.0,
        "sway_rot_deg": float(np.degrees(np.linalg.norm(r.sway_amp[3:]))),
        "stance_xy_mm": float(np.linalg.norm(r.stance_xy)) * 1000.0,
        "stance_yaw_deg": abs(float(np.degrees(r.stance_yaw))),
    }
