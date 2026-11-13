"""Yada の評価環境（Button_Press/Yada/contest）で動かす、J1-gen のエージェント。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/Yada/contest/evaluate.py --agent Button_Press/J1-gen/contest_agent --seeds smoke
    $P Button_Press/Yada/contest/evaluate.py --agent Button_Press/J1-gen/contest_agent --seed 1 --view
    J1GEN_BUTTON_DETECTOR=yolo $P Button_Press/Yada/contest/evaluate.py --agent Button_Press/J1-gen/contest_agent

ボタンを探す方法は configs/elevator.yaml の detector（classic / yolo）。環境変数 J1GEN_BUTTON_DETECTOR で上書きできる。
押す組は geometry.button_set（general = 柱の上の 2 つ / wheelchair = 下の 2 つ）。環境変数 J1GEN_BUTTON_SET で上書きできる
（yolo は sim/train_button_yolo.py で学習した重みを使う）。

中身は common/elevator_press.py（見つける・計画・押す）。ここは評価環境の決まり（Agent / Observation / Action）に
つなぐだけ。評価環境は Button_Press/Yada を import の検索先に入れるので、`common` は Yada のパッケージになる。
J1-gen の部品は、Yada の j1gen_bridge で別名（j1gen_common）として読み込む。
シミュレーターの正解の値（ボタンの座標など）は使わない。

環境変数 J1GEN_BODY_LOG にフォルダを渡すと、毎周期の関節角・IMU・段階・推定トルク・推定した押す力をそこに記録する（実機で、押している間に
足側のバランス制御が腰をどれだけ動かしたかを見るため。読み方は real/body_motion.py）:

    J1GEN_BODY_LOG=_local/button_press/body_log $P Button_Press/Yada/contest/run_dds.py --agent ... （実機の引数）
    $P Button_Press/J1-gen/real/body_motion.py _local/button_press/body_log/<日時>_<up|down>

環境変数 J1GEN_ARM に right / left / auto を渡すと、押す腕を変える（既定は configs/elevator.yaml の press.arm）。
auto は、ボタンを見つけたあとに計画が立つ腕を選ぶ（記録の arm は選んだ腕）。
環境変数 J1GEN_WAIST_SCAN=1 を渡したときだけ、見つからないときに腰を回して探す（press.waist_scan。評価環境で確かめる用。
elevator.yaml の enabled はこの口では使わない。実機では未確認なので、real/auto_press.py は必ず 0 を渡す）。
環境変数 J1GEN_MAX_JOINT_SPEED に rad/s を渡すと、計画で許す関節の速さ（既定は arm.yaml の 0.5）をその回だけ変える。

環境変数 J1GEN_PRESS_KEEP_MM・J1GEN_STOP_LAG_MM・J1GEN_PRESS_DEPTH_MM に mm を渡すと、押す力の設定（elevator.yaml の
contact_stop.keep_m・lag_m と planner.press_depth_m）をその回だけ変える（real/auto_press.py --keep-mm など）。

環境変数 J1GEN_AIR_BUTTONS に「x,y」（pelvis から前・左右 [m]）を渡すと、カメラでボタンを探さず、その空中に車いす用の
▲▼ があるものとして同じ動きをする（壁の無いところで腕を伸ばす動きだけを試す。real/auto_press.py --air。押し直しはしない）。

環境変数 J1GEN_RESULT_JSON にファイルの名前を渡すと、終わったときに結果（点灯した・確かめられない・中止とその理由）と
段階ごとのメッセージを書く（real/auto_press.py が渡し、自分の記録 _local/button_press/auto/*.json に取り込む）。

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
        if os.environ.get("J1GEN_BUTTON_SET"):
            elevator["geometry"]["button_set"] = os.environ["J1GEN_BUTTON_SET"]
        if os.environ.get("J1GEN_ARM"):  # 押す腕 right / left / auto（real/auto_press.py --arm）
            elevator["press"]["arm"] = os.environ["J1GEN_ARM"]
        # 腰を回して探す（press.waist_scan）は、J1GEN_WAIST_SCAN=1 を渡したときだけオン。yaml の enabled はここでは使わない
        # （実機ではまだ腰を動かす確認をしていない。yaml を誤ってオンにしても、この口を通る実機・評価環境では動かない。
        # real/auto_press.py は必ず 0 を渡す）
        if elevator["press"].get("waist_scan") is not None:
            elevator["press"]["waist_scan"]["enabled"] = os.environ.get("J1GEN_WAIST_SCAN") == "1"
        print(f"[j1gen-agent] 押す腕: {elevator['press'].get('arm', 'right')}")
        if os.environ.get("J1GEN_BUTTON_WEIGHTS"):  # 候補の重みを、既定を変えずに評価するとき
            elevator["yolo"]["weights"] = os.environ["J1GEN_BUTTON_WEIGHTS"]
        if os.environ.get("J1GEN_MAX_JOINT_SPEED"):  # 計画で許す関節の速さ [rad/s]（real/auto_press.py --max-joint-speed）
            elevator["press"]["max_joint_speed_rad_s"] = float(os.environ["J1GEN_MAX_JOINT_SPEED"])
            print(f"[j1gen-agent] ⚠️ 計画で許す関節の速さ: {elevator['press']['max_joint_speed_rad_s']} rad/s（既定は arm.yaml）")
        # 押す力（real/auto_press.py --keep-mm / --stop-lag-mm / --depth-mm）。指令した指先が実際の指先より先に出た分 × 腕の硬さが力
        cs = elevator["press"]["contact_stop"]
        for name, key, d in (("J1GEN_PRESS_KEEP_MM", "keep_m", cs), ("J1GEN_STOP_LAG_MM", "lag_m", cs),
                             ("J1GEN_PRESS_DEPTH_MM", "press_depth_m", elevator["press"]["planner"])):
            if os.environ.get(name):
                d[key] = float(os.environ[name]) / 1000
                print(f"[j1gen-agent] ⚠️ 押す力の設定 {key}: {d[key] * 1000:.0f} mm（既定は elevator.yaml）")
        cs["retry_lag_m"] = max(float(cs["retry_lag_m"]), float(cs["lag_m"]))  # 押し直しが 1 回目より弱くならないように
        if os.environ.get("J1GEN_HAND_MASK") is not None:  # 実機の黒い手は 0（real/auto_press.py。common/lamp_watch.py）
            elevator["press"]["lit_watch"]["hand_mask"] = os.environ["J1GEN_HAND_MASK"] not in ("0", "", "false")
            print(f"[j1gen-agent] ランプを見るとき手の画素を除く: {elevator['press']['lit_watch']['hand_mask']}")
        if os.environ.get("J1GEN_NO_RETRY"):  # 押す力を確かめるとき、強めの押し直しで点いたのと区別する（real/auto_press.py --no-retry）
            elevator["press"]["max_retries"] = 0
            print("[j1gen-agent] ⚠️ 押し直しをしない")
        air = os.environ.get("J1GEN_AIR_BUTTONS")
        if air:
            elevator["press"]["max_retries"] = 0  # 空中なので点灯しない。押し直して深くしない
            elevator["press"]["lit_check"]["prelit_level"] = None  # 枠は画像の真ん中の仮の四角。明るい物が写っても止めない
            elevator["geometry"]["button_set"] = "wheelchair"  # 高さは車いす用。指をまっすぐ向ける設定を使う
        self.ctl = j1gen("elevator_press").ElevatorPressController.from_config(
            j1gen_config("robot.yaml"), j1gen_config("press.yaml"), j1gen_config("arm.yaml"), elevator)
        if air:
            x, y = (float(v) for v in air.split(","))
            self.ctl.finder = j1gen("elevator_buttons").AirButtonFinder(x, y)
            print(f"[j1gen-agent] ⚠️ 空中のボタン（カメラで探さない）: 前 {x * 100:.0f} cm、左右 {y * 100:+.0f} cm")
        weights = f"（重み: {elevator['yolo']['weights']}）" if elevator["detector"] == "yolo" else ""
        print(f"[j1gen-agent] ボタンを探す方法: {elevator['detector']}{weights}、押す組: {elevator['geometry'].get('button_set', 'general')}")

    def reset(self, task: TaskInfo) -> None:
        self.ctl.reset(task.target, task.control_dt, task.upper_kp)
        self.body_log = BodyLog.open(task.target)
        self.ctl.on_snapshot = self.body_log.save_snapshot if self.body_log is not None else None
        self._result_written = False

    def act(self, obs: Observation) -> Action:
        # 推定トルク（Observation.tau_est）は 2026-11-04 に評価環境へ足した。足す前の評価環境でも動くよう getattr で読む
        q29, done = self.ctl.step(obs.rgb, obs.depth, obs.K, obs.q, obs.t, obs.image_t, obs.imu_quat,
                                  getattr(obs, "tau_est", None))
        if self.body_log is not None:
            self.body_log.write(obs, self.ctl)
        if done:
            self._write_result()
        return Action(q_target=q29[list(UPPER_BODY_IDX)], done=done)

    def close(self) -> None:
        self._write_result()  # 終わる前に止まった（Ctrl+C・通信が切れた）ときも、そこまでの段階を残す

    def _write_result(self) -> None:
        path = os.environ.get("J1GEN_RESULT_JSON")
        if not path or getattr(self, "_result_written", True):
            return
        self._result_written = True
        self.ctl.finish_force()  # 押し込み・保持の途中で終わった（点灯した瞬間に終わる評価環境、Ctrl+C）ときの分
        write_result(Path(path), self.ctl, self.body_log.dir if self.body_log is not None else None)


def write_result(path: Path, ctl: object, body_log_dir: Path | None) -> None:
    """結果（点灯した・確かめられない・中止とその理由）と、段階ごとのメッセージを JSON に書く（real/auto_press.py が
    J1GEN_RESULT_JSON で渡し、自分の記録に取り込む）。2026-11-03 は、計画まで行かずに終わった回の理由が画面にしか出ず、
    あとから分からなかった。"""
    log = getattr(ctl, "log", [])
    path.write_text(json.dumps({
        "outcome": getattr(ctl, "outcome", "") or "（結果が出る前に終わった）",
        "phase": getattr(ctl, "phase", ""),
        "notes": [{"phase": s.phase, "message": s.message} for s in log],
        "body_log": str(body_log_dir) if body_log_dir is not None else None,
        # 押し込みの回ごとの、推定トルクから推定した押す力 [N]（common/push_force.py）。推定トルクが無ければ空
        "press_forces": getattr(ctl, "press_forces", []),
        # 押している間・戻る間に見たランプ（一度でも点灯したか）と、乗り場の扉が開いたか（記録だけ）。common/lamp_watch.py
        "lamp_watch": ctl.lamp.summary() if getattr(ctl, "lamp", None) is not None else None,
        "door_watch": ctl.door.summary() if getattr(ctl, "door", None) is not None else None,
    }, ensure_ascii=False, indent=2, default=float))


class BodyLog:
    """毎周期の関節角（29）・IMU・段階を body_log.csv に書く（列は common/body_motion.read_lowstate_csv が読む形）。
    2026-11-04 から、推定トルク tau0..tau28 と、押し込み・保持の間に推定した押す向きの力 push_force_n も書く
    （無ければ空欄。読み方は real/press_force.py）。
    計画ができたら、押す点（pelvis 座標）を meta.json に書く。"""

    def __init__(self, folder: Path) -> None:
        folder.mkdir(parents=True, exist_ok=False)
        self.dir = folder
        self._f = open(folder / "body_log.csv", "w", newline="", encoding="utf-8", buffering=1)
        self._w = csv.writer(self._f)
        self._w.writerow(["t", "phase"] + [f"q{i}" for i in range(29)] + ["qw", "qx", "qy", "qz"]
                         + [f"tau{i}" for i in range(29)] + ["push_force_n"])
        self._planned = False
        print(f"[j1gen-agent] 体の動きの記録: {folder}")

    @classmethod
    def open(cls, target: str) -> "BodyLog | None":
        root = os.environ.get("J1GEN_BODY_LOG")
        if not root:
            return None
        base = Path(root) / f"{time.strftime('%Y%m%d_%H%M%S')}_{target}"
        # 並列の試行が同じ秒に始まると名前がぶつかる（2026-11-13、評価環境の 4 並列で FileExistsError）。
        # ぶつかったら _2, _3 … を付ける（mkdir が先に取った方を勝たせるので、プロセスの間でも重ならない）
        for n in range(1, 100):
            try:
                return cls(base if n == 1 else base.with_name(f"{base.name}_{n}"))
            except FileExistsError:
                continue
        raise FileExistsError(f"体の動きの記録のフォルダを作れない（同じ名前が多すぎる）: {base}")

    def save_snapshot(self, kind: str, rgb: np.ndarray, info: dict) -> None:
        """押す前（kind="before"）・点灯の確認（"check"）の画像を、元の画像と、枠・値を描いた画像で保存し、
        内容を lit_log.json に足す。元の画像は、あとで点灯判定の回帰テストや学習に使える。
        押している間・戻る間に見た画像（"watch"、初めて点灯を見たものは "seen"）は数が多いので、"watch" は
        枠・値・手の写る円を描いた JPEG だけにする（手がランプをどれだけ隠すかを、実機の画像で確かめるため）。"""
        import cv2  # 実機でも入っている（評価環境が使う）。記録を使わないときは読み込まない

        self._shots = getattr(self, "_shots", 0)
        stem = f"lit_{self._shots:02d}_{kind}"
        self._shots += 1
        bgr = cv2.cvtColor(np.asarray(rgb, dtype=np.uint8), cv2.COLOR_RGB2BGR)
        if kind != "watch":
            cv2.imwrite(str(self.dir / f"{stem}.png"), bgr)
        vis = bgr.copy()
        for u, v, r in info.get("hand", []):  # 手・前腕が写るとみなした範囲（ランプの明るさから除いた画素）
            cv2.circle(vis, (int(round(u)), int(round(v))), max(int(round(r)), 1), (255, 0, 255), 1)
        x1, y1, x2, y2 = info["bbox"]
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 1)
        if info.get("found_box") and info.get("moved"):  # 見つけ直して枠が動いたとき、元の枠を青で
            a1, b1, a2, b2 = info["found_box"]
            cv2.rectangle(vis, (a1, b1), (a2, b2), (255, 128, 0), 1)
        label = f"{kind} {info['target']} level={info['level']:.0f}"
        if "rise" in info:
            label += f" rise={info['rise']:+.0f} {'LIT' if info['lit'] else 'not lit'}"
        if "visible" in info:
            label += f" {info['phase']} visible={info['visible'] * 100:.0f}%"
        cv2.putText(vis, label, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        cv2.imwrite(str(self.dir / f"{stem}_annotated.jpg"), vis)
        path = self.dir / "lit_log.json"
        rows = json.loads(path.read_text()) if path.exists() else []
        rows.append({"image": f"{stem}.png" if kind != "watch" else f"{stem}_annotated.jpg", "kind": kind,
                     **{k: v for k, v in info.items() if k != "hand"}})  # 手の円は画像にだけ描く（毎回書き直すので軽くする）
        path.write_text(json.dumps(rows, ensure_ascii=False, indent=1, default=float))

    def write(self, obs: Observation, ctl: object) -> None:
        phase = getattr(ctl, "phase", "")
        tau = getattr(obs, "tau_est", None)
        force = getattr(getattr(ctl, "force", None), "last_n", None)
        self._w.writerow([f"{obs.t:.4f}", phase] + [f"{v:.6f}" for v in obs.q]
                         + [f"{v:.6f}" for v in np.asarray(obs.imu_quat, dtype=float)]
                         + ([f"{v:.4f}" for v in tau] if tau is not None else [""] * 29)
                         + [f"{force:.2f}" if force is not None else ""])
        plan = getattr(ctl, "plan_", None)
        if plan is not None and not self._planned:
            self._planned = True
            (self.dir / "meta.json").write_text(json.dumps(
                {"target_point": np.asarray(plan.target_point).tolist(), "planned_t": obs.t,
                 "push_dir": np.asarray(plan.push_dir).tolist(),  # 押す力を出し直すとき（real/press_force.py）
                 "arm": getattr(ctl, "side", "right"),  # 押した腕（real/press_force.py が使う）
                 "phase": phase}, ensure_ascii=False, indent=2))


def make_agent() -> Agent:
    return J1genElevatorAgent()
