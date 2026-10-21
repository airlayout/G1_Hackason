"""Yada の評価環境（Button_Press/Yada/contest）で動かす、J1-gen のエージェント。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/Yada/contest/evaluate.py --agent Button_Press/J1-gen/contest_agent --seeds smoke
    $P Button_Press/Yada/contest/evaluate.py --agent Button_Press/J1-gen/contest_agent --seed 1 --view
    J1GEN_BUTTON_DETECTOR=yolo $P Button_Press/Yada/contest/evaluate.py --agent Button_Press/J1-gen/contest_agent

ボタンを探す方法は configs/elevator.yaml の detector（classic / yolo）。環境変数 J1GEN_BUTTON_DETECTOR で上書きできる
（yolo は sim/train_button_yolo.py で学習した重みを使う）。

中身は common/elevator_press.py（見つける・計画・押す）。ここは評価環境の決まり（Agent / Observation / Action）に
つなぐだけ。評価環境は Button_Press/Yada を import の検索先に入れるので、`common` は Yada のパッケージになる。
J1-gen の部品は、Yada の j1gen_bridge で別名（j1gen_common）として読み込む。
シミュレーターの正解の値（ボタンの座標など）は使わない。

環境変数 J1GEN_BODY_LOG にフォルダを渡すと、毎周期の関節角・IMU・段階をそこに記録する（実機で、押している間に
足側のバランス制御が腰をどれだけ動かしたかを見るため。読み方は real/body_motion.py）:

    J1GEN_BODY_LOG=_local/button_press/body_log $P Button_Press/Yada/contest/run_dds.py --agent ... （実機の引数）
    $P Button_Press/J1-gen/real/body_motion.py _local/button_press/body_log/<日時>_<up|down>

同じフォルダに、押す前（ボタンを見つけたとき）と点灯の確認のときの画像も保存する（lit_00_before.png、
lit_01_check.png … と、枠・明るさを描いた *_annotated.jpg、lit_log.json）。手が枠に被さって点灯の確認が
狂っていないか（点灯していないのに「点灯した」、点灯したのに「していない」）を、あとで見るため。
"""

from __future__ import annotations

import csv
import json
import os
import time
from pathlib import Path

import numpy as np

from common.j1gen_bridge import j1gen, j1gen_config
from contest.interface import UPPER_BODY_IDX, Action, Agent, Observation, TaskInfo


class J1genElevatorAgent(Agent):
    def __init__(self) -> None:
        elevator = j1gen_config("elevator.yaml")
        if os.environ.get("J1GEN_BUTTON_DETECTOR"):
            elevator["detector"] = os.environ["J1GEN_BUTTON_DETECTOR"]
        self.ctl = j1gen("elevator_press").ElevatorPressController.from_config(
            j1gen_config("robot.yaml"), j1gen_config("press.yaml"), j1gen_config("arm.yaml"), elevator)
        print(f"[j1gen-agent] ボタンを探す方法: {elevator['detector']}")

    def reset(self, task: TaskInfo) -> None:
        self.ctl.reset(task.target, task.control_dt, task.upper_kp)
        self.body_log = BodyLog.open(task.target)
        self.ctl.on_snapshot = self.body_log.save_snapshot if self.body_log is not None else None

    def act(self, obs: Observation) -> Action:
        q29, done = self.ctl.step(obs.rgb, obs.depth, obs.K, obs.q, obs.t, obs.image_t, obs.imu_quat)
        if self.body_log is not None:
            self.body_log.write(obs, self.ctl)
        return Action(q_target=q29[list(UPPER_BODY_IDX)], done=done)


class BodyLog:
    """毎周期の関節角（29）・IMU・段階を body_log.csv に書く（列は common/body_motion.read_lowstate_csv が読む形）。
    計画ができたら、押す点（pelvis 座標）を meta.json に書く。"""

    def __init__(self, folder: Path) -> None:
        folder.mkdir(parents=True, exist_ok=False)
        self.dir = folder
        self._f = open(folder / "body_log.csv", "w", newline="", encoding="utf-8", buffering=1)
        self._w = csv.writer(self._f)
        self._w.writerow(["t", "phase"] + [f"q{i}" for i in range(29)] + ["qw", "qx", "qy", "qz"])
        self._planned = False
        print(f"[j1gen-agent] 体の動きの記録: {folder}")

    @classmethod
    def open(cls, target: str) -> "BodyLog | None":
        root = os.environ.get("J1GEN_BODY_LOG")
        if not root:
            return None
        return cls(Path(root) / f"{time.strftime('%Y%m%d_%H%M%S')}_{target}")

    def save_snapshot(self, kind: str, rgb: np.ndarray, info: dict) -> None:
        """押す前（kind="before"）・点灯の確認（"check"）の画像を、元の画像と、枠・値を描いた画像で保存し、
        内容を lit_log.json に足す。元の画像は、あとで点灯判定の回帰テストや学習に使える。"""
        import cv2  # 実機でも入っている（評価環境が使う）。記録を使わないときは読み込まない

        self._shots = getattr(self, "_shots", 0)
        stem = f"lit_{self._shots:02d}_{kind}"
        self._shots += 1
        bgr = cv2.cvtColor(np.asarray(rgb, dtype=np.uint8), cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(self.dir / f"{stem}.png"), bgr)
        vis = bgr.copy()
        x1, y1, x2, y2 = info["bbox"]
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 1)
        if info.get("found_box") and info.get("moved"):  # 見つけ直して枠が動いたとき、元の枠を青で
            a1, b1, a2, b2 = info["found_box"]
            cv2.rectangle(vis, (a1, b1), (a2, b2), (255, 128, 0), 1)
        label = f"{kind} {info['target']} level={info['level']:.0f}"
        if "rise" in info:
            label += f" rise={info['rise']:+.0f} {'LIT' if info['lit'] else 'not lit'}"
        cv2.putText(vis, label, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        cv2.imwrite(str(self.dir / f"{stem}_annotated.jpg"), vis)
        path = self.dir / "lit_log.json"
        rows = json.loads(path.read_text()) if path.exists() else []
        rows.append({"image": f"{stem}.png", "kind": kind, **info})
        path.write_text(json.dumps(rows, ensure_ascii=False, indent=1, default=float))

    def write(self, obs: Observation, ctl: object) -> None:
        phase = getattr(ctl, "phase", "")
        self._w.writerow([f"{obs.t:.4f}", phase] + [f"{v:.6f}" for v in obs.q]
                         + [f"{v:.6f}" for v in np.asarray(obs.imu_quat, dtype=float)])
        plan = getattr(ctl, "plan_", None)
        if plan is not None and not self._planned:
            self._planned = True
            (self.dir / "meta.json").write_text(json.dumps(
                {"target_point": np.asarray(plan.target_point).tolist(), "planned_t": obs.t,
                 "phase": phase}, ensure_ascii=False, indent=2))


def make_agent() -> Agent:
    return J1genElevatorAgent()
