"""模擬 G1（MuJoCo）: エレベーター乗り場の G1 を、実機と同じ口（DDS + ZMQ のカメラ）で見せる評価環境。

    P=~/miniconda3/envs/lerobot/bin/python
    $P Button_Press/Yada/sim/mujoco/g1_sim_server.py --seed 3            # 試行の条件は seed で決まる
    $P Button_Press/Yada/sim/mujoco/g1_sim_server.py --seed 3 --view     # 画面で見る
    $P Button_Press/Yada/sim/mujoco/g1_sim_server.py --seed 3 --keep-running   # 判定のあとも止めない（開発用）

各チームは、実機用のコードを「つなぎ先」だけ変えて動かす（コードは変えない）:
- DDS: ネットワークの口 lo、domain 1（実機は有線の口、domain 0）。J1-gen の実機用のスクリプトなら
  `--network-interface lo` を付ける（common/dds.py が domain を 1 にする）
- カメラ: 127.0.0.1:5556（深度付き）/ 5555（RGB 互換）。J1-gen なら `--camera-config camera_sim.yaml`

実機と同じ口:
- rt/lowstate を送る（100 Hz）。reserve[0] に模擬ロボットの目印（J1-gen の dds.SIM_MARKER）を入れる
- rt/arm_sdk（上半身。weight でブレンド）と rt/lowcmd（全関節）を受け取る（J1-gen の SimBackend の近似）
- 下半身の LocoClient（RPC の "sport" サービス）に応答する。ただし腰（pelvis）を固定しているので、歩く指令は
  受け付けるだけで動かない（記録はする）
- 頭カメラの RGB + 深度を、PC2 の RGB-D サーバと同じ形式で配信する

評価環境として足したもの:
- 試行の条件（ボタン盤の位置、押すボタン、指示の文）を seed で決める（contest/task.py。Python の API と同じ）
- 指示は _local/button_press_yada/sim_server/task.json に書き、画面にも表示する
- 判定: 最初の指令（arm_sdk / lowcmd / loco）が届いた時刻から計り、ボタンが点灯するか制限時間が来たら、
  結果を _local/button_press_yada/sim_server/result_seed<N>.json に書く（success / wrong / timeout）
- 判定のあとは、少し待ってから止まる（--keep-running なら止まらない）

実機と混ざらないように、通信はループバックだけに固定している（変える引数は無い）。
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
import time
from pathlib import Path
from types import FrameType
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from common.config import REPO_ROOT, load_config  # noqa: E402
from common.j1gen_bridge import j1gen, j1gen_config  # noqa: E402
from common.scene_spec import build_hall_scene  # noqa: E402
from contest.task import make_trial  # noqa: E402
from mujoco_hall import HallMujoco, build_hall_model  # noqa: E402

OUT_DIR = REPO_ROOT / "_local" / "button_press_yada" / "sim_server"
CAMERA_HOST = "127.0.0.1"  # ループバックだけで待ち受ける
# 指令がこの秒数届かなければ、内蔵コントローラが元の姿勢を保持する状態に戻す（J1-gen の模擬ロボットと同じ）
COMMAND_TIMEOUT_S = 0.5
# 最初の指令を待つ時間の上限 [秒]（これを過ぎても指令が来なければ、no_command として終わる）
WAIT_FIRST_COMMAND_S = 300.0
# 判定のあと、止まるまで待つ時間 [秒]（つないでいる側が、腕を安全に戻して終われるように）
LINGER_S = 5.0

SimBackend = j1gen("arm.backend_sim").SimBackend
JointCommand = j1gen("arm.types").JointCommand
dds_consts = j1gen("dds")
rgbd_protocol = j1gen("rgbd_protocol")
robot_model = j1gen("robot_model")


class HallSimBackend(SimBackend):
    """J1-gen の SimBackend（arm_sdk の weight のブレンドと、内蔵コントローラの保持の近似）を、乗り場のシーンで使う。

    open() だけを差し替える（シーンを、机とボトルの代わりに乗り場にする）。それ以外は J1-gen と同じ。
    """

    def __init__(self, scene_cfg: dict[str, Any], robot_cfg: dict[str, Any], sim_cfg: dict[str, Any],
                 hold_kp: np.ndarray, hold_kd: np.ndarray) -> None:
        super().__init__(robot_cfg, sim_cfg, hold_kp, hold_kd, uses_weight=True)
        self._scene_cfg = scene_cfg
        self.scene = build_hall_scene(scene_cfg)
        self.hall: HallMujoco | None = None

    def open(self) -> None:
        import mujoco

        self._mj = mujoco
        self.model = build_hall_model(self.scene, self._robot_cfg, self._scene_cfg.get("fingertip"))
        # 以下は J1-gen の SimBackend.open() と同じ（Kd 項を陰的に解く、名前で関節を引く、開始時の姿勢）
        self.model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.data = mujoco.MjData(self.model)
        names = robot_model.JOINT_NAMES
        self._qadr = np.array([self.model.joint(n).qposadr[0] for n in names])
        self._vadr = np.array([self.model.joint(n).dofadr[0] for n in names])
        self._act = np.array([self.model.actuator(n).id for n in names])
        q0 = np.asarray(self._sim_cfg.get("initial_q", [0.0] * robot_model.NUM_MOTORS), dtype=float)
        self.data.qpos[self._qadr] = q0
        mujoco.mj_forward(self.model, self.data)
        self._hold_q = q0.copy()
        self._gravity_comp = bool(self._sim_cfg.get("gravity_compensation", False))
        self._realtime = bool(self._sim_cfg.get("realtime", False))
        self.hall = HallMujoco(self.model, self.data, self.scene)
        print(f"[sim_g1] MuJoCo 開始（arm_sdk の重力補償={'あり' if self._gravity_comp else 'なし'}）")
        if self._sim_cfg.get("viewer"):
            self._open_viewer()


def msg_to_command(msg: Any, use_weight: bool) -> Any:
    """LowCmd_ → JointCommand（mode=1 の関節だけ。weight は motor_cmd[29].q）。J1-gen の模擬ロボットと同じ。"""
    c = JointCommand()
    for i in range(robot_model.NUM_MOTORS):
        m = msg.motor_cmd[i]
        if m.mode == 1:
            c.q[i], c.kp[i], c.kd[i], c.tau[i] = m.q, m.kp, m.kd, m.tau
    c.weight = float(msg.motor_cmd[robot_model.ARM_SDK_WEIGHT_IDX].q) if use_weight else 1.0
    return c


def make_loco_server(on_request: Any) -> Any:
    """LocoClient（unitree_sdk2py.g1.loco）の相手をする RPC サーバ（サービス名 "sport"）。

    腰を固定しているので体は動かさない。受け取った依頼を on_request(api_id, parameter) に渡し、成功（0）を返す。
    """
    from unitree_sdk2py.g1.loco import g1_loco_api as api
    from unitree_sdk2py.rpc.server import Server

    class LocoSimServer(Server):
        def __init__(self) -> None:
            super().__init__(api.LOCO_SERVICE_NAME)
            self.fsm_id = 0

        def Init(self) -> None:
            self._SetApiVersion(api.LOCO_API_VERSION)
            for name in dir(api):
                if name.startswith("ROBOT_API_ID_LOCO_"):
                    self._RegistHandler(getattr(api, name), self._make_handler(getattr(api, name)), False)

        def _make_handler(self, api_id: int):
            def handler(parameter: str) -> tuple[int, str]:
                on_request(api_id, parameter)
                if api_id == api.ROBOT_API_ID_LOCO_SET_FSM_ID:
                    self.fsm_id = int(json.loads(parameter or "{}").get("data", self.fsm_id))
                if api_id == api.ROBOT_API_ID_LOCO_GET_FSM_ID:
                    return 0, json.dumps({"data": self.fsm_id})
                if api_id in (api.ROBOT_API_ID_LOCO_GET_FSM_MODE, api.ROBOT_API_ID_LOCO_GET_BALANCE_MODE):
                    return 0, json.dumps({"data": 0})
                return 0, ""

            return handler

    s = LocoSimServer()
    s.Init()
    s.Start(False)
    return s


class SimG1Server:
    def __init__(self, seed: int, view: bool, keep_running: bool, rgbd_port: int, rgb_port: int,
                 camera_fps: float, control_hz: float = 100.0) -> None:
        self.contest_cfg = load_config("contest.yaml")
        self.trial = make_trial(seed, self.contest_cfg)
        self.robot_cfg = j1gen_config("robot.yaml")
        arm_cfg = j1gen_config("arm.yaml")
        q0 = np.zeros(robot_model.NUM_MOTORS)
        for idx, deg in self.trial.scene_cfg.get("initial_pose_deg", {}).items():
            q0[int(idx)] = np.radians(float(deg))
        sim_cfg = dict(arm_cfg["sim"])
        sim_cfg.update(realtime=True, viewer=view, initial_q=list(q0),
                       gravity_compensation=bool(self.contest_cfg.get("arm_sdk_gravity_compensation", False)))
        self.backend = HallSimBackend(self.trial.scene_cfg, self.robot_cfg, sim_cfg,
                                      np.asarray(arm_cfg["lowcmd"]["hold_kp"], float),
                                      np.asarray(arm_cfg["lowcmd"]["hold_kd"], float))
        self.keep_running = keep_running
        self.dt = 1.0 / control_hz
        self.camera_period = 1.0 / camera_fps
        self.rgbd_port, self.rgb_port = rgbd_port, rgb_port
        self.time_limit = float(self.contest_cfg["time_limit_s"])
        self._lock = threading.Lock()
        self._pending: tuple[str, Any] | None = None
        self._stop = False
        self._t_first_cmd: float | None = None
        self.counts = {"arm_sdk": 0, "lowcmd": 0, "loco": 0, "lowstate": 0, "frames": 0}
        self.loco_requests: list[dict[str, Any]] = []

    # ---- 受け取り（DDS のスレッドから呼ばれる） ------------------------------

    def _mark_first_command(self) -> None:
        if self._t_first_cmd is None:
            self._t_first_cmd = time.monotonic()
            print("[sim_g1] 最初の指令を受け取った。ここから時間を計る")

    def _on_cmd(self, topic: str, msg: Any) -> None:
        cmd = msg_to_command(msg, use_weight=topic == "arm_sdk")
        with self._lock:
            self._pending = (topic, cmd)
            self.counts[topic] += 1
            self._mark_first_command()

    def _on_loco(self, api_id: int, parameter: str) -> None:
        with self._lock:
            self.counts["loco"] += 1
            if len(self.loco_requests) < 1000:
                self.loco_requests.append({"api_id": api_id, "parameter": parameter, "t": time.monotonic()})
            self._mark_first_command()

    # ---- 準備 -----------------------------------------------------------------

    def _open_dds(self) -> None:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        # 実機と混ざらないよう、口と domain は固定（引数で変えられない）
        ChannelFactoryInitialize(dds_consts.SIM_DOMAIN_ID, dds_consts.SIM_INTERFACE)
        self._pub = ChannelPublisher(dds_consts.TOPIC_LOWSTATE, LowState_)
        self._pub.Init()
        self._sub_arm = ChannelSubscriber("rt/arm_sdk", LowCmd_)
        self._sub_arm.Init(lambda m: self._on_cmd("arm_sdk", m), 10)
        self._sub_low = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self._sub_low.Init(lambda m: self._on_cmd("lowcmd", m), 10)
        self._loco = make_loco_server(self._on_loco)
        self._msg = unitree_hg_msg_dds__LowState_()
        self._crc = CRC()

    def _publish_state(self, tick: int) -> None:
        st = self.backend.read_state()
        m = self._msg
        for i in range(robot_model.NUM_MOTORS):
            m.motor_state[i].q = float(st.q[i])
            m.motor_state[i].dq = float(st.dq[i])
            m.motor_state[i].mode = 1
        m.mode_machine = 5
        m.imu_state.quaternion = [1.0, 0.0, 0.0, 0.0]
        m.reserve = [dds_consts.SIM_MARKER, 0, 0, 0]
        m.tick = tick
        m.crc = self._crc.Crc(m)
        self._pub.Write(m)
        self.counts["lowstate"] += 1

    def _open_camera(self) -> None:
        import mujoco
        import zmq

        # 描画は別のスレッドで、描画用の MjData に関節角を写して行う（J1-gen の模擬ロボットと同じ）
        self._render_data = mujoco.MjData(self.backend.model)
        self._render_qpos = self.backend.data.qpos.copy()
        self._zctx = zmq.Context()
        self._zrgbd = self._zctx.socket(zmq.PUB)
        self._zrgb = self._zctx.socket(zmq.PUB)
        for s, port in ((self._zrgbd, self.rgbd_port), (self._zrgb, self.rgb_port)):
            s.setsockopt(zmq.SNDHWM, 2)
            s.setsockopt(zmq.LINGER, 0)
            s.bind(f"tcp://{CAMERA_HOST}:{port}")

    def _camera_loop(self) -> None:
        import mujoco
        import zmq

        camera = j1gen("sim_camera").SimHeadCamera(self.backend.model, self._render_data, self.robot_cfg)
        try:
            while not self._stop:
                t = time.monotonic()
                with self._lock:
                    self._render_data.qpos[:] = self._render_qpos
                mujoco.mj_forward(self.backend.model, self._render_data)
                f = camera.render()
                try:
                    self._zrgbd.send(rgbd_protocol.encode_rgbd(f), zmq.NOBLOCK)
                    self._zrgb.send_string(rgbd_protocol.encode_legacy_rgb(f.color_bgr[:, :, ::-1].copy(), f.camera,
                                                                           f.timestamp), zmq.NOBLOCK)
                except zmq.Again:
                    pass
                self.counts["frames"] += 1
                rest = self.camera_period - (time.monotonic() - t)
                if rest > 0:
                    time.sleep(rest)
        finally:
            camera.close()

    def _write_task(self) -> None:
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        task = {"seed": self.trial.seed, "target": self.trial.target, "instruction": self.trial.instruction,
                "time_limit_s": self.time_limit, "dds": {"interface": dds_consts.SIM_INTERFACE,
                                                         "domain_id": dds_consts.SIM_DOMAIN_ID},
                "camera": {"host": CAMERA_HOST, "rgbd_port": self.rgbd_port, "rgb_port": self.rgb_port}}
        (OUT_DIR / "task.json").write_text(json.dumps(task, ensure_ascii=False, indent=2))

    # ---- 本体 -----------------------------------------------------------------

    def stop(self, signum: int = 0, frame: FrameType | None = None) -> None:
        self._stop = True

    def run(self) -> dict[str, Any]:
        be = self.backend
        be.open()
        hall = be.hall
        assert hall is not None
        self._open_dds()
        self._open_camera()
        self._write_task()
        print(f"[sim_g1] 準備できた: seed {self.trial.seed}、指示「{self.trial.instruction}」（target = {self.trial.target}）")
        print(f"[sim_g1] DDS は {dds_consts.SIM_INTERFACE} / domain {dds_consts.SIM_DOMAIN_ID}、"
              f"カメラは {CAMERA_HOST}:{self.rgbd_port}（深度付き）/ {self.rgb_port}（RGB 互換）。指示は {OUT_DIR / 'task.json'}")
        result: dict[str, Any] = {"seed": self.trial.seed, "target": self.trial.target,
                                  "instruction": self.trial.instruction, "sim": "mujoco_dds", "outcome": None,
                                  "time_s": None, "max_contact_force_n": 0.0}
        tick = 0
        last_cmd_t: float | None = None
        t_start = time.monotonic()
        t_judged: float | None = None
        cam_thread = threading.Thread(target=self._camera_loop, daemon=True)
        cam_thread.start()
        next_log = time.monotonic() + 5.0
        try:
            while not self._stop:
                now = time.monotonic()
                with self._lock:
                    pending, self._pending = self._pending, None
                if pending is not None:
                    last_cmd_t = now
                    topic, cmd = pending
                    be.uses_weight = topic == "arm_sdk"
                    be.send(cmd)
                elif last_cmd_t is not None and now - last_cmd_t > COMMAND_TIMEOUT_S:
                    # 指令が途絶えた: 内蔵コントローラが開始時の姿勢を保持する状態へ戻す
                    be.uses_weight = True
                    be.send(JointCommand())
                    last_cmd_t = None
                be.tick(self.dt)
                lit = [n for n in hall.update()]
                with self._lock:
                    self._render_qpos[:] = be.data.qpos
                tick += 1
                self._publish_state(tick)

                # 判定（最初に決まったものだけ）
                if result["outcome"] is None:
                    result["max_contact_force_n"] = max(result["max_contact_force_n"], hall.max_robot_contact_force())
                    on = [b.name for b in hall.scene.buttons if hall.lit(b.name)]
                    t0 = self._t_first_cmd
                    if on:
                        result["outcome"] = "success" if on == [self.trial.target] else "wrong"
                        result["time_s"] = round(now - (t0 if t0 is not None else t_start), 3)
                    elif t0 is not None and now - t0 > self.time_limit:
                        result["outcome"] = "timeout"
                    elif t0 is None and now - t_start > WAIT_FIRST_COMMAND_S:
                        result["outcome"] = "no_command"
                    if result["outcome"] is not None:
                        t_judged = now
                        self._finish(result)
                elif not self.keep_running and t_judged is not None and now - t_judged > LINGER_S:
                    break
                if lit:
                    print(f"[sim_g1] {lit} が点灯した")
                if now >= next_log:
                    print(f"[sim_g1] {self.counts}")
                    next_log = now + 5.0
        finally:
            self._stop = True
            cam_thread.join(timeout=5.0)
            self._zrgbd.close()
            self._zrgb.close()
            self._zctx.term()
            be.close()
            if result["outcome"] is None:
                result["outcome"] = "aborted"
                self._finish(result)
            print(f"[sim_g1] 終了: {self.counts}")
        return result

    def _finish(self, result: dict[str, Any]) -> None:
        result["max_contact_force_n"] = round(float(result["max_contact_force_n"]), 2)
        result["counts"] = dict(self.counts)
        result["loco_requests"] = len(self.loco_requests)
        path = OUT_DIR / f"result_seed{self.trial.seed}.json"
        path.write_text(json.dumps(result, ensure_ascii=False, indent=2))
        print(f"[sim_g1] 判定: {result['outcome']}（時間 {result['time_s']} s、接触力の最大 "
              f"{result['max_contact_force_n']} N）→ {path}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--seed", type=int, default=0, help="試行の条件を決める乱数の種（contest/seeds.yaml）")
    p.add_argument("--view", action="store_true", help="画面で見る")
    p.add_argument("--keep-running", action="store_true", help="判定のあとも止めない（開発用）")
    p.add_argument("--rgbd-port", type=int, default=5556)
    p.add_argument("--rgb-port", type=int, default=5555)
    p.add_argument("--camera-fps", type=float, default=15.0)
    args = p.parse_args()
    server = SimG1Server(args.seed, args.view, args.keep_running, args.rgbd_port, args.rgb_port, args.camera_fps)
    signal.signal(signal.SIGINT, server.stop)
    signal.signal(signal.SIGTERM, server.stop)
    res = server.run()
    return 0 if res["outcome"] in ("success", "wrong", "timeout", "no_command") else 1


if __name__ == "__main__":
    sys.exit(main())
