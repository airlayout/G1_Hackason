"""押している間に体（pelvis）がどれだけ動いたかを、記録した lowstate から求める。

実機の G1 は、腕を伸ばしたり押した反動を受けたりすると、足側（Unitree のバランス制御）が腰の位置を直す。
シミュレーションは腰を固定しているので、この動きは見えない。対象の位置は押す前に pelvis 座標で測っているので、
そのあとに腰が動くと、そのぶん狙いがずれる。

求め方: 両足が床に着いて滑らないとして、脚の関節角（lowstate の q0〜q11。pelvis から
足首まで）の FK で「足から見た pelvis の位置と向き」を出し、基準の時刻（対象を測ったとき）からの変化を見る。
左右の足のそれぞれから求めて平均する。左右の足の間の位置関係が変わったら、足を踏み替えた（この求め方の前提が
崩れた）とみなす。IMU の四元数からは、重力に対する傾きの変化だけを別に出す（脚の FK の答え合わせ）。

出すもの（すべて基準の時刻の pelvis 座標。x 前、y 左、z 上）:
- pelvis の移動 [mm] と回転 [deg]
- 狙いのずれ [mm]: 基準の時刻に測った対象の点が、今の pelvis 座標でどこへ動いて見えるか（今の座標 − 測った座標）。
  腕は pelvis 座標で計画どおりに動くので、指先は部屋から見てこのぶん逆向きにずれる
- 左右の足の間の位置関係の変化 [mm]（大きければ足を踏み替えた）
- IMU の傾きの変化 [deg]
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .config import resolve_repo_path
from .robot_model import JOINT_NAMES, NUM_MOTORS

FEET = ("left_ankle_roll_link", "right_ankle_roll_link")


class LegKinematics:
    """脚の FK（URDF + Pinocchio）。足首（ankle_roll_link）から見た pelvis の位置と向きを出す。"""

    def __init__(self, robot_cfg: dict[str, Any]) -> None:
        import pinocchio as pin

        self._pin = pin
        urdf = resolve_repo_path(robot_cfg["model"]["urdf_path"])
        self.model = pin.buildModelFromUrdf(str(urdf))
        names = [self.model.names[i] for i in range(1, self.model.njoints)]
        if names != list(JOINT_NAMES) or self.model.nq != NUM_MOTORS:
            raise ValueError(f"URDF の関節の並びが motor 番号順と違う: {names}")
        self.data = self.model.createData()
        self.feet = [self.model.getFrameId(n) for n in FEET]

    def pelvis_in_feet(self, q: np.ndarray) -> list[np.ndarray]:
        """左右の足首から見た pelvis の姿勢（4×4 の同次変換）。"""
        pin = self._pin
        pin.framesForwardKinematics(self.model, self.data, np.asarray(q, dtype=float))
        return [np.linalg.inv(self.data.oMf[f].homogeneous) for f in self.feet]


def gravity_in_body(quat_wxyz: np.ndarray) -> np.ndarray:
    """IMU の四元数（w, x, y, z）から、体の座標で見た下向き（重力の向き）の単位ベクトル。"""
    w, x, y, z = np.asarray(quat_wxyz, dtype=float) / np.linalg.norm(quat_wxyz)
    # 回転行列の 3 行目（世界の z 軸を体の座標で見たもの）の符号を反転
    return -np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])


def rotation_vector(r: np.ndarray) -> np.ndarray:
    """回転行列 → 回転ベクトル [rad]（向き = 回転軸、長さ = 角度）。"""
    c = np.clip((np.trace(r) - 1) / 2, -1.0, 1.0)
    a = float(np.arccos(c))
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    if a < 1e-9:
        return v / 2
    return v * a / (2 * np.sin(a))


@dataclass
class BodyMotion:
    t: np.ndarray  # (N,) 時刻 [s]
    ref: int  # 基準の行
    move_mm: np.ndarray  # (N, 3) pelvis の移動
    rot_deg: np.ndarray  # (N, 3) pelvis の回転（回転ベクトル）
    aim_mm: np.ndarray | None  # (N, 3) 狙いのずれ（対象の点が無ければ None）
    feet_mm: np.ndarray  # (N,) 左右の足の間の位置関係の変化
    imu_tilt_deg: np.ndarray  # (N,) IMU の傾きの変化
    labels: list[str]  # (N,) 行ごとの段階の名前（無ければ空）
    point: np.ndarray | None  # 対象の点（pelvis 座標、基準の時刻）

    def peak(self, x: np.ndarray, after_ref: bool = True) -> int:
        """大きさが最大の行（基準の行より後）。"""
        n = np.linalg.norm(x, axis=1) if x.ndim == 2 else np.abs(x)
        start = self.ref if after_ref else 0
        return start + int(np.argmax(n[start:]))

    def summary(self) -> dict[str, Any]:
        def at(i: int) -> dict[str, Any]:
            return {"t_s": round(float(self.t[i] - self.t[self.ref]), 3), "label": self.labels[i]}

        i_move = self.peak(self.move_mm)
        i_rot = self.peak(self.rot_deg)
        i_feet = self.peak(self.feet_mm)
        i_tilt = self.peak(self.imu_tilt_deg)
        out: dict[str, Any] = {
            "rows": len(self.t),
            "ref_label": self.labels[self.ref],
            "move_max_mm": np.round(self.move_mm[i_move], 1).tolist(), "move_max_at": at(i_move),
            "rot_max_deg": np.round(self.rot_deg[i_rot], 2).tolist(), "rot_max_at": at(i_rot),
            "feet_max_mm": round(float(self.feet_mm[i_feet]), 1), "feet_max_at": at(i_feet),
            "imu_tilt_max_deg": round(float(self.imu_tilt_deg[i_tilt]), 2), "imu_tilt_max_at": at(i_tilt),
            "leg_tilt_max_deg": round(float(np.max(np.linalg.norm(self.rot_deg[self.ref:, :2], axis=1))), 2),
        }
        if self.aim_mm is not None:
            i_aim = self.peak(self.aim_mm)
            out.update(point=np.round(self.point, 4).tolist(), aim_max_mm=np.round(self.aim_mm[i_aim], 1).tolist(),
                       aim_max_at=at(i_aim))
        return out

    def text(self, feet_warn_mm: float = 10.0) -> str:
        s = self.summary()
        f = lambda v: "[" + ", ".join(f"{x:+.1f}" for x in v) + "]"  # noqa: E731
        n = lambda v: float(np.linalg.norm(v))  # noqa: E731
        lines = [
            f"基準: {s['ref_label'] or '最初の行'}（{s['rows']} 行）。座標は基準の時刻の pelvis（x 前、y 左、z 上）",
            f"pelvis の移動の最大: {n(s['move_max_mm']):.1f} mm {f(s['move_max_mm'])}"
            f"（{s['move_max_at']['t_s']:+.2f} s、{s['move_max_at']['label']}）",
            f"pelvis の回転の最大: {n(s['rot_max_deg']):.2f}° {f(s['rot_max_deg'])}"
            f"（{s['rot_max_at']['t_s']:+.2f} s、{s['rot_max_at']['label']}）",
        ]
        if "aim_max_mm" in s:
            lines.append(f"狙いのずれの最大: {n(s['aim_max_mm']):.1f} mm {f(s['aim_max_mm'])}"
                         f"（{s['aim_max_at']['t_s']:+.2f} s、{s['aim_max_at']['label']}。対象の点 {s['point']}）")
        lines.append(f"傾きの変化の最大: IMU {s['imu_tilt_max_deg']:.2f}° / 脚の FK {s['leg_tilt_max_deg']:.2f}°"
                     "（大きく違えば、足が床から浮いたか滑った）")
        feet = f"左右の足の間の位置関係の変化の最大: {s['feet_max_mm']:.1f} mm"
        if s["feet_max_mm"] > feet_warn_mm:
            feet += f" ⚠️ {feet_warn_mm:.0f} mm を超えた（足を踏み替えた。そのあとの値は当てにならない）"
        lines.append(feet)
        return "\n".join(lines)


def analyze(legs: LegKinematics, t: np.ndarray, q: np.ndarray, imu: np.ndarray, ref: int = 0,
            point: np.ndarray | None = None, labels: list[str] | None = None) -> BodyMotion:
    """t (N,)、q (N, 29)、imu (N, 4: w, x, y, z) から、基準の行 ref からの体の動きを求める。"""
    n = len(t)
    poses = [legs.pelvis_in_feet(q[i]) for i in range(n)]
    ref_pose = poses[ref]
    move = np.zeros((n, 3))
    rot = np.zeros((n, 3))
    aim = np.zeros((n, 3)) if point is not None else None
    feet = np.zeros(n)
    # 右足から見た左足（足を踏み替えると変わる）
    gap_ref = ref_pose[1] @ np.linalg.inv(ref_pose[0])
    for i, pose in enumerate(poses):
        # 基準の pelvis から見た今の pelvis（左右の足のそれぞれから求めて平均）
        rel = [np.linalg.inv(ref_pose[k]) @ pose[k] for k in range(2)]
        move[i] = np.mean([r[:3, 3] for r in rel], axis=0) * 1000
        rot[i] = np.degrees(np.mean([rotation_vector(r[:3, :3]) for r in rel], axis=0))
        if aim is not None:
            p = np.append(np.asarray(point, dtype=float), 1.0)
            aim[i] = np.mean([(np.linalg.inv(r) @ p)[:3] - p[:3] for r in rel], axis=0) * 1000
        gap = pose[1] @ np.linalg.inv(pose[0])
        feet[i] = np.linalg.norm(gap[:3, 3] - gap_ref[:3, 3]) * 1000
    g = np.array([gravity_in_body(x) for x in imu])
    tilt = np.degrees(np.arccos(np.clip(g @ g[ref], -1.0, 1.0)))
    return BodyMotion(t=np.asarray(t, dtype=float), ref=ref, move_mm=move, rot_deg=rot, aim_mm=aim, feet_mm=feet,
                      imu_tilt_deg=tilt, labels=labels or [""] * n, point=None if point is None else np.asarray(point))


def read_lowstate_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str]]:
    """lowstate.csv（common/recording.py の LOWSTATE_HEADER）か、エージェントの記録（body_log.csv。phase の列が
    ある）を読む。t、q (N, 29)、IMU (N, 4)、段階の名前を返す。"""
    with open(path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"行が無い: {path}")
    t = np.array([float(r["t"]) for r in rows])
    q = np.array([[float(r[f"q{i}"]) for i in range(NUM_MOTORS)] for r in rows])
    imu = np.array([[float(r[k]) for k in ("qw", "qx", "qy", "qz")] for r in rows])
    labels = [r.get("phase", "") or "" for r in rows]
    return t, q, imu, labels


def write_csv(path: Path, m: BodyMotion) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "label", "move_x_mm", "move_y_mm", "move_z_mm", "rot_x_deg", "rot_y_deg", "rot_z_deg",
                    "aim_x_mm", "aim_y_mm", "aim_z_mm", "feet_mm", "imu_tilt_deg"])
        for i in range(len(m.t)):
            aim = m.aim_mm[i] if m.aim_mm is not None else [np.nan] * 3
            w.writerow([f"{m.t[i] - m.t[m.ref]:.4f}", m.labels[i]] + [f"{v:.2f}" for v in m.move_mm[i]]
                       + [f"{v:.3f}" for v in m.rot_deg[i]] + [f"{v:.2f}" for v in aim]
                       + [f"{m.feet_mm[i]:.2f}", f"{m.imu_tilt_deg[i]:.3f}"])


# 全体をつなぐスクリプト（common/pipeline.py）の記録で、行に付ける段階の名前（events.jsonl の出来事から）
TARGET_EVENTS = ("target_depth", "target_manual")


def stage_label(stage: str) -> str:
    """stage の出来事（common/press.py の on_stage）を、そのあとに腕がしていることの名前にする。"""
    if stage == "approach":
        return "手前で計算し直す"
    if stage.endswith("_replanned"):  # approach_replanned、retry_2_replanned …
        return "押し込み"
    return {"end": "保持・戻り", "retreated": "押したあとの確認"}.get(stage, stage)


def run_labels(events: list[dict[str, Any]], t: np.ndarray) -> list[str]:
    """行ごとに、腕がしていることの名前（対象を測るまでは「対象を測る」、そのあとは「手前へ移動」、以降は stage から）。"""
    marks = sorted((float(e["t"]), stage_label(e["stage"]) if e["event"] == "stage" else "手前へ移動")
                   for e in events if e["event"] == "stage" or e["event"] in TARGET_EVENTS)
    out, k, cur = [], 0, "対象を測る"
    for ti in t:
        while k < len(marks) and marks[k][0] <= ti:
            cur = marks[k][1]
            k += 1
        out.append(cur)
    return out


def analyze_run_dir(run_dir: Path, legs: LegKinematics, point: np.ndarray | None = None) -> BodyMotion:
    """全体をつなぐスクリプトの記録（lowstate.csv と events.jsonl）を読む。基準は対象を測った時刻の直前の行、
    対象の点は記録の値（point を渡せばそちら）。"""
    import json

    run_dir = Path(run_dir)
    t, q, imu, _ = read_lowstate_csv(run_dir / "lowstate.csv")
    events = []
    if (run_dir / "events.jsonl").exists():
        with open(run_dir / "events.jsonl", encoding="utf-8") as f:
            events = [json.loads(line) for line in f if line.strip()]
    ref = 0
    found = [e for e in events if e["event"] in TARGET_EVENTS]
    if found:
        before = np.nonzero(t <= float(found[-1]["t"]))[0]
        ref = int(before[-1]) if len(before) else 0
        if point is None:
            point = np.asarray(found[-1]["target"], dtype=float)
    return analyze(legs, t, q, imu, ref=ref, point=point, labels=run_labels(events, t))
