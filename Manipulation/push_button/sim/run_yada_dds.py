#!/usr/bin/env python3
"""模擬G1だけにDDS + ZMQで接続する。実機用の入口と設定は分離する。"""

import argparse
import json
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

from yada_paths import AGENT, find_yada_root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yada-root")
    args = parser.parse_args()
    root = find_yada_root(args.yada_root)
    sys.path.insert(0, str(root))
    import numpy as np
    from common.config import load_config
    from contest.interface import Observation, load_agent
    from contest.robots.real_g1 import RealG1Robot, StateTimeoutError
    from contest.runner import run_episode

    task_file = Path(os.environ.get("YADA_TASK_FILE", root.parents[1] /
                                   "_local/button_press_yada/sim_server/task.json"))
    task = json.loads(task_file.read_text())
    if task["dds"] != {"interface": "lo", "domain_id": 1} or task["camera"]["host"] != "127.0.0.1":
        parser.error("この入口は lo / domain 1 / 127.0.0.1 の模擬G1専用です")
    cfg = {**load_config("contest.yaml"), "time_limit_s": float(task["time_limit_s"])}
    result_file = task_file.with_name(f"result_seed{task['seed']}.json")

    class ScoringFinished(Exception):
        def __init__(self, verdict):
            self.verdict = verdict

    class SimDdsRobot(RealG1Robot):
        base_enabled = False

        def advance(self, dt):
            info = super().advance(dt)
            # APIのランナーと同じく、採点完了時に終了する。判定をエージェントへ渡さない。
            if result_file.is_file():
                try:
                    verdict = json.loads(result_file.read_text())
                except json.JSONDecodeError:
                    return info  # サーバが書き込んでいる最中なら次の周期で読む。
                raise ScoringFinished(verdict)
            return info

        def observe(self, t):
            st = self._fresh_state()
            if not st.sim_marker:
                raise RuntimeError("模擬G1の目印がありません")
            with self._frame_lock:
                frame = self._frame
            if frame is None:
                raise RuntimeError("模擬頭カメラのRGB-Dがありません")
            intr = frame.intrinsics
            K = np.array([[intr.fx, 0, intr.cx], [0, intr.fy, intr.cy], [0, 0, 1]])
            # 元のRealG1Robotはimage_tを埋めない。同じフレームを3回計測しないよう時刻を補う。
            if not hasattr(self, "image_epoch"):
                self.image_epoch = time.time()
                self.clock_started = time.monotonic()
            image_t = frame.timestamp - self.image_epoch
            # APIの試行時刻は物理刻み、DDSは実時間。推論に要した時間も経過時間に含める。
            observation_t = time.monotonic() - self.clock_started
            return Observation(frame.color_bgr[:, :, ::-1].copy(), frame.depth_m(), K,
                               st.q.copy(), st.dq.copy(), observation_t, st.imu_quat.copy(), image_t)

    agent = load_agent(AGENT)
    robot = SimDdsRobot(cfg, "lo", "127.0.0.1", int(task["camera"]["rgbd_port"]), use_loco=False)
    try:
        trial = SimpleNamespace(seed=task["seed"], target=task["target"], instruction=task["instruction"])
        result = run_episode(robot, agent, trial, cfg, verbose=True)
        if result.error:
            print(result.error, file=sys.stderr)
            return 2
        print(f"クライアント結果: {result.outcome}。正式な採点は模擬G1のresult_seed*.jsonを参照")
    except ScoringFinished as finished:
        print(f"模擬G1の判定: {finished.verdict['outcome']}。制御重みを解除して終了します")
        return 0 if finished.verdict["outcome"] == "success" else 1
    except StateTimeoutError:
        # 採点後にサーバが終了した場合だけ正常終了にする。
        if not result_file.exists():
            raise
        print(f"模擬G1の判定: {json.loads(result_file.read_text())['outcome']}")
    finally:
        try:
            robot.close()
        finally:
            agent.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
