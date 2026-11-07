"""頭カメラでゴミ箱を見つけ、立つ位置（押す動作を確かめた位置）まで歩く（実機）。ラボ PC で実行する。

    P=G1_HuggingFace/venv/bin/python
    # dry-run（既定）: 1 回測って、どう歩くかを表示するだけ。歩かない
    $P Trash_Bin/real/approach_bin.py
    # 歩く（1 歩ごとに Enter。q で中止）
    $P Trash_Bin/real/approach_bin.py --execute
    # 着いたら、そのまま押す: real/push_bin.py --execute を続けて実行する

前提: ロボットはゴミ箱のおおよそ正面、1 m 以内に立っている（頭カメラにゴミ箱の上面が写る範囲。体の前 0.3〜1.2 m）。
そこまでは今は人がリモコンで連れて来る（Navigation が実機で使えるようになったら、地図の地点へ行く形に替える）。

やり方は lib/approach.py: 「頭カメラの深度で測る → 少し歩く（1 回 15 cm まで）→ 止まって測り直す」を、立つ位置との
ずれが前後左右 2 cm・向き 3° 以内になるまで繰り返す。腕には何も送らない（lowstate は腰の角度を読むためだけに使う）。

歩く指令の安全:
- LocoClient.Move(..., continous_move=True) は使わない（864000 秒歩き続ける指令で、スクリプトが落ちると止まらない）。
  SetVelocity(…, duration=0.5 秒) を 0.2 秒ごとに送り直すので、送るのが止まれば 0.5 秒以内にロボットが自分で止まる
- 速さの上限（configs/bin_scene.yaml の approach。前 0.15 m/s・横 0.10 m/s・旋回 0.3 rad/s、後ろは 0.05 m/s まで）
- 骨盤が 5° 以上傾いている（倒れかけ・座っている）なら歩かない。ゴミ箱が 2 回続けて見えなければ止まる
- 終わるとき・Ctrl+C・エラーのときは必ず StopMove

⚠️ 人がリモコンを持つ（緊急時は L2+B）。G1 の安全な歩行速度は未確認（Navigation の loco_driver.py と同じく仮の値）。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "Button_Press" / "J1-gen"))
sys.path.insert(0, str(REPO / "Trash_Bin" / "lib"))
sys.path.insert(0, str(REPO / "Trash_Bin" / "sim"))  # 設定の読み込み（bin_scene.load_scene_config）だけ使う

from approach import ApproachConfig, ApproachStep, approach_step, measured_motion, walk_summary  # noqa: E402
from bin_geometry import BinPose, pose_from_config  # noqa: E402
from bin_scene import load_scene_config  # noqa: E402
from common.arm import make_backend  # noqa: E402
from common.camera_cfg import add_camera_args, apply_head_camera_mount, load_camera_config  # noqa: E402
from common.camera_geometry import HeadCameraTransform  # noqa: E402
from common.config import load_config, resolve_repo_path  # noqa: E402
from common.dds import peer_label  # noqa: E402
from common.run_logger import RunLogger  # noqa: E402
from locate_bin import LocateError, draw_detection, locate_bin  # noqa: E402

MAX_PELVIS_TILT_DEG = 5.0
RESEND_S = 0.2     # 歩く指令を送り直す間隔
CMD_DURATION_S = 0.5  # 1 回の指令で歩く長さ（送り直しが止まれば、この時間で止まる）


def pelvis_tilt_deg(imu_quat) -> float:  # noqa: ANN001
    """骨盤の傾き（鉛直からの角度）[deg]。IMU が無ければ 0。"""
    if imu_quat is None:
        return 0.0
    w, x, y, z = (float(v) for v in imu_quat)
    up_z = 1 - 2 * (x * x + y * y)  # 骨盤の z 軸の、鉛直成分
    return float(np.degrees(np.arccos(np.clip(up_z, -1.0, 1.0))))


class Walker:
    """LocoClient の薄い包み。dry-run なら何も呼ばない。"""

    def __init__(self, execute: bool) -> None:
        self.client = None
        if execute:
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

            self.client = LocoClient()
            self.client.SetTimeout(3.0)
            self.client.Init()

    def walk(self, step: ApproachStep) -> None:
        """step の速さで duration_s 秒歩いて止まる。指令は CMD_DURATION_S の長さで、RESEND_S ごとに送り直す。"""
        if self.client is None:
            return
        t_end = time.monotonic() + step.duration_s
        try:
            while True:
                rest = t_end - time.monotonic()
                if rest <= 0:
                    break
                self.client.SetVelocity(float(step.vx), float(step.vy), float(step.vyaw), min(CMD_DURATION_S, rest))
                time.sleep(min(RESEND_S, rest))
        finally:
            self.stop()

    def stop(self) -> None:
        if self.client is not None:
            self.client.StopMove()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--network-interface", help="G1 につないでいる NIC（既定は J1-gen の configs/arm.yaml）")
    add_camera_args(p)
    p.add_argument("--scene", help="ゴミ箱の設定（既定は Trash_Bin/configs/bin_scene.yaml）")
    p.add_argument("--execute", action="store_true", help="実際に歩く（付けなければ dry-run。1 回測って表示するだけ）")
    p.add_argument("--no-confirm", action="store_true", help="1 歩ごとの Enter 待ちを省く")
    p.add_argument("--label", default="", help="記録のフォルダ名に付ける言葉")
    args = p.parse_args()

    scene = load_scene_config(args.scene)
    target = pose_from_config(scene)
    acfg = ApproachConfig.from_dict(scene.get("approach"))
    robot, arm_cfg = load_config("robot.yaml"), load_config("arm.yaml")
    cam_cfg = load_camera_config(args, quiet=True)
    apply_head_camera_mount(robot, cam_cfg)
    if args.network_interface:
        arm_cfg["network_interface"] = args.network_interface
    iface = arm_cfg["network_interface"]
    loc = load_config("localize.yaml")
    transform = HeadCameraTransform(robot, np.asarray(loc["calibration"]["offset_pelvis_m"], dtype=float))
    print(f"[approach] ネットワークの口: {iface or '（未設定）'}、相手: {peer_label(iface) if iface else '?'}、"
          f"{'歩く（--execute）' if args.execute else 'dry-run（歩かない。1 回測って表示するだけ）'}")

    # lowstate を読むためだけに使う（dry_run=True なので腕には何も送らない）
    backend = make_backend(arm_cfg, robot, dry_run=True, path="arm_sdk")
    logger = RunLogger(resolve_repo_path("_local/trash_bin/runs"), args.label or "approach",
                       {"kind": "real", "mode": "approach", "execute": args.execute, "network_interface": iface,
                        "scene": scene})
    src = None
    walker = None
    code = 1
    try:
        backend.open()
        from common.camera_rgbd import RgbdZmqSource

        c = load_camera_config(args)["rgbd"]
        src = RgbdZmqSource(c["server_address"], int(c["port"]), int(c["timeout_ms"]))
        src.open()
        walker = Walker(args.execute)
        prior: BinPose | None = None
        lost = 0
        last: tuple[BinPose, np.ndarray] | None = None  # 前回測った置き方と、そのあと指令した量（実際に動いた量と比べる）
        walked: list[tuple[np.ndarray, np.ndarray]] = []
        for k in range(1, acfg.max_steps + 1):
            frame, st = None, None
            for _ in range(3):  # 最初の 1 枚は届くまで少しかかることがある
                frame, st = src.read_rgbd(), backend.read_state()
                if frame is not None and st is not None:
                    break
            if frame is None or st is None:
                raise RuntimeError("カメラのフレームか lowstate が届かない（rgbd_server と接続を確認）")
            tilt = pelvis_tilt_deg(getattr(st, "imu_quat", None))
            if tilt > MAX_PELVIS_TILT_DEG:
                raise RuntimeError(f"骨盤が {tilt:.1f}° 傾いている（上限 {MAX_PELVIS_TILT_DEG}°）。立っていないので歩かない")
            q_waist = st.q[12:15].copy()
            try:
                det = locate_bin(frame, transform, q_waist, scene, prior=prior)
            except LocateError as e:
                lost += 1
                print(f"[approach] {k} 回目: 見つからない（{e}）")
                logger.save_frame(f"lost{k}", frame, waist_q=q_waist)
                if lost >= 2 or not args.execute:
                    raise
                prior = None
                continue
            lost = 0
            if last is not None:
                moved = measured_motion(last[0], det.pose)
                walked.append((last[1], moved))
                logger.event("walk_measured", commanded=last[1], measured=moved)
                print(f"[approach]   前回の 1 歩: 指令 前 {last[1][0] * 100:+.1f} cm・左 {last[1][1] * 100:+.1f} cm・"
                      f"{np.degrees(last[1][2]):+.1f}° → 測った 前 {moved[0] * 100:+.1f} cm・左 {moved[1] * 100:+.1f} cm・"
                      f"{np.degrees(moved[2]):+.1f}°")
            last = None
            step = approach_step(det.pose, target, acfg)
            print(f"[approach] {k} 回目: {det.summary()}")
            print(f"[approach]   {step.summary()}")
            logger.event("approach_step", k=k, center_xy=det.pose.center_xy, yaw_deg=float(np.degrees(det.pose.yaw)),
                         error=step.error, vx=step.vx, vy=step.vy, vyaw=step.vyaw, duration_s=step.duration_s,
                         done=step.done)
            logger.save_image(f"step{k}", draw_detection(frame, transform, q_waist, scene, det))
            if step.done:
                print("[approach] 立つ位置に着いた。続けて押すなら: real/push_bin.py --locate-only で確かめてから --execute")
                code = 0
                break
            if not args.execute:
                print("[approach] dry-run なのでここで終わる（--execute で歩く）")
                code = 0
                break
            if not args.no_confirm:
                ans = input("[confirm] この指令で歩く → Enter / q で中止: ")
                if ans.strip().lower() == "q":
                    print("[approach] 中止")
                    break
            walker.walk(step)
            time.sleep(acfg.settle_s)
            last = (det.pose, np.array([step.vx, step.vy, step.vyaw]) * step.duration_s)
            # 次に測るときは、歩いた（つもりの）分だけ前回の置き方を動かし、探す範囲の中心にする
            dx, dy, dyaw = step.vx * step.duration_s, step.vy * step.duration_s, step.vyaw * step.duration_s
            cs, sn = np.cos(dyaw), np.sin(dyaw)
            prior = BinPose(center_xy=np.array([[cs, sn], [-sn, cs]]) @ (det.pose.center_xy - [dx, dy]),
                            yaw=float(det.pose.yaw - dyaw))
        else:
            print(f"[approach] ❌ {acfg.max_steps} 回歩いても着かなかった")
    except KeyboardInterrupt:
        print("[approach] Ctrl+C。止める")
    except (LocateError, RuntimeError, ValueError) as e:
        print(f"[approach] ❌ {e}")
        code = 2
    finally:
        if walker is not None:
            walker.stop()
        if args.execute:
            # 小さく歩けたか（指令した量と、測った量の比）。10 日の実機で、歩く速さ・時間の決め方を直す材料にする
            print("[approach] 歩いた量（指令 と 頭カメラで測った量）:")
            for line in walk_summary(walked):
                print(f"[approach]   {line}")
        backend.close()
        if src is not None:
            src.close()
        logger.close(code=code, message="")
    print(f"[approach] 記録: {logger.dir}")
    return code


if __name__ == "__main__":
    sys.exit(main())
