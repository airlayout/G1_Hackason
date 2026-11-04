"""エレベーターの記録（body_log）から、押し込みの間に指先がボタンを押した力を出し直す（ラボ PC で。ロボットには何も送らない）。

G1 の腕には力のセンサーが無いので、lowstate の推定トルク（tau_est）の、押す前からの変化から推定する
（重力のモデルで計算した重力のトルクを引いた残りを使う。common/push_force.py。エージェントの中と同じ手順）。エージェントも終わったときに同じまとめを
段階のメッセージと auto の記録（agent.press_forces）に残すが、ここでは行ごとの値も出す。

    P=G1_HuggingFace/venv/bin/python
    J1GEN_BODY_LOG=_local/button_press/body_log ...（real/auto_press.py は自動で付ける）
    $P Button_Press/J1-gen/real/press_force.py _local/button_press/body_log/<日時>_<up|down>

表示するもの:
- 腕が止まっている間（look の段階）の推定トルクのばらつき（右腕 7 関節の標準偏差）。推定の力がこれに比べて
  十分大きいかを見る目安（実機の値は 2026-11-04 の時点で未確認）
- 押し込みの回ごとの、押す向きの力の最大（そのときの横の力）、保持中の平均・最大

結果は記録のフォルダの press_force.csv にも書く（t、段階、押す向きの力 [N]。グラフにするとき用）。
2026-11-04 より前の記録には推定トルクの列が無いので、出せない。
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.arm.gravity import GravityModel  # noqa: E402
from common.config import load_config  # noqa: E402
from common.kinematics import ArmKinematics  # noqa: E402
from common.push_force import replay, still_noise  # noqa: E402
from common.robot_model import NUM_MOTORS, RIGHT_ARM_IDX  # noqa: E402


def read_body_log(path: Path) -> tuple[np.ndarray, list[str], np.ndarray, np.ndarray, np.ndarray | None]:
    """body_log.csv の t、段階、関節角 (N, 29)、IMU (N, 4)、推定トルク (N, 29)（列が無い・空欄なら None）。"""
    with open(path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"行が無い: {path}")
    t = np.array([float(r["t"]) for r in rows])
    phases = [r.get("phase", "") or "" for r in rows]
    q = np.array([[float(r[f"q{i}"]) for i in range(NUM_MOTORS)] for r in rows])
    imu = np.array([[float(r[k]) for k in ("qw", "qx", "qy", "qz")] for r in rows])
    if "tau0" not in rows[0] or any(r["tau0"] == "" for r in rows):
        return t, phases, q, imu, None
    tau = np.array([[float(r[f"tau{i}"]) for i in range(NUM_MOTORS)] for r in rows])
    return t, phases, q, imu, tau


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder", type=Path, help="エージェントの記録のフォルダ（body_log.csv と meta.json があるところ）")
    a = ap.parse_args()

    log, meta = a.folder / "body_log.csv", a.folder / "meta.json"
    if not log.exists():
        print(f"[press_force] {a.folder} に body_log.csv が無い")
        return 1
    t, phases, q, imu, tau = read_body_log(log)
    if tau is None:
        print("[press_force] 推定トルクの列が無い（2026-11-04 より前の記録か、推定トルクを渡さない環境）")
        return 1
    if not meta.exists() or "push_dir" not in json.loads(meta.read_text()):
        print("[press_force] meta.json に押す向き（push_dir）が無い（計画まで行かなかった回か、2026-11-04 より前の記録）")
        return 1
    push_dir = np.asarray(json.loads(meta.read_text())["push_dir"], dtype=float)

    arm = list(RIGHT_ARM_IDX)
    look = [i for i, p in enumerate(phases) if p == "look"]
    if len(look) >= 10:
        noise = still_noise(tau[look][:, arm])
        print(f"[press_force] 腕が止まっている間（look、{len(look)} 周期）の推定トルクのばらつき（右腕、標準偏差）: "
              + " ".join(f"{v:.3f}" for v in noise) + " Nm")

    kin = ArmKinematics(load_config("robot.yaml"), "right", load_config("press.yaml")["ik"])
    force, summaries = replay(kin, GravityModel(load_config("robot.yaml")), phases, q, tau, imu, push_dir)
    if not summaries:
        print("[press_force] 押し込みの段階（press）の行が無い")
    for s in summaries:
        print(f"[press_force] {s.attempt} 回目: {s.text()}（基準は押す前の {s.baseline_samples} 周期）")

    out = a.folder / "press_force.csv"
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "phase", "push_force_n"])
        for ti, ph, v in zip(t, phases, force):
            w.writerow([f"{ti:.4f}", ph, "" if np.isnan(v) else f"{v:.2f}"])
    print(f"[press_force] 行ごとの値: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
