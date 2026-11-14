"""腕の定型動作を実行する（sim / arm_sdk 共通）。real/run_action.py から呼ぶ。

流れ: 腕の指令部分を開始（現在の姿勢のまま weight を 0→1）→ 各段へ動いて保つ → 開始姿勢へ戻る → 終了（weight を 1→0）。
歩行コントローラは止めない（rt/arm_sdk）。ReleaseMode は呼ばない。
"""

from __future__ import annotations

import argparse
from typing import Any

import numpy as np

from common.arm import (
    ArmCommander,
    NoMotionError,
    StateTimeoutError,
    StopRequested,
    UnsafeTargetError,
    WaistDeviationError,
    joint_limits,
    make_backend,
)
from common.arm.gravity import GravityModel, needs_gravity_model
from common.config import load_config

from .poses import ACTIONS, Action

# 腰の上限の既定 [deg]。2026-11-13 の実機で、point は上限 5° では止まり、8° で最後まで動いた（腰 pitch の最大 +6.7°）。
# point_level は 10° で最後まで動いた（腰の kp 60 のとき）
DEFAULT_WAIST_MAX_DEG = 8.0
# 腰を開始時の角度に保つ PD の既定（この動作だけ。arm.yaml の arm_sdk.waist_hold = 60 / 1.5 は変えない）。
# 2026-11-13 の実機: point_level の腰 pitch の最大は kp 60 で +9.2°、kp 120 / kd 2.5 で +3.9〜4.1°（震え無し）
DEFAULT_WAIST_KP = 120.0
DEFAULT_WAIST_KD = 2.5
# 腰の PD の上限（real/waist_turn_check.py と同じ。2026-11-13 に arm_sdk で kp 300 / kd 5 を実機で送った記録がある）
MAX_WAIST_KP = 300.0
MAX_WAIST_KD = 5.0


def hold_pose(arm: ArmCommander, seconds: float) -> None:
    """今の姿勢を seconds 秒保つ（同じ点を 1 周期に 1 点ずつ送る。動いたかは見ない）。"""
    n = int(round(seconds / arm.dt))
    if n > 0:
        q = arm.commanded_arm_q
        arm.follow([q] * n, label="保持", expect_motion=False)


def run_action(action: Action, args: argparse.Namespace, path: str, dry_run: bool, confirm: bool,
               overrides: dict[str, Any] | None = None) -> int:
    arm_cfg = load_config(args.arm_config)
    robot_cfg = load_config(args.robot_config)
    if args.arm:
        arm_cfg["arm"] = args.arm
    if args.gravity_scale is not None:
        arm_cfg["gravity_compensation"]["scale"] = args.gravity_scale
    # 腰の角度の上限は、この動作の実行時だけ変える（configs/arm.yaml は変えない。エレベーター押しなど他の動作は 2.9° のまま）
    arm_cfg["safety"]["waist_max_deviation_rad"] = float(np.radians(args.waist_max_deg))
    # 腰を開始時の角度に保つ PD も、この動作の実行時だけ変える（arm_sdk のときだけ効く）
    hold = arm_cfg["arm_sdk"]["waist_hold"]
    hold["kp"] = float(args.waist_kp)
    hold["kd"] = float(args.waist_kd)
    for k, v in (overrides or {}).items():
        arm_cfg[k] = v

    lower, upper = joint_limits(robot_cfg)
    backend = make_backend(arm_cfg, robot_cfg, dry_run=dry_run, path=path)
    try:
        backend.open()
        gravity = GravityModel(robot_cfg) if needs_gravity_model(arm_cfg) else None
        with ArmCommander(backend, arm_cfg, lower, upper, confirm=confirm, gravity=gravity) as arm:
            q_start = arm.commanded_arm_q
            sign = np.ones(7)
            if arm_cfg["arm"] == "left":
                # 左腕は、右腕の姿勢の左右を反転して使う
                from common.robot_model import ARM_MIRROR_JOINTS

                sign[list(ARM_MIRROR_JOINTS)] = -1.0
            print(f"[action] {action.name}: {action.description}")
            print(f"[action] 腰のずれの上限: {args.waist_max_deg:.1f}°")
            if path == "arm_sdk":
                print(f"[action] 腰の保持: kp {hold['kp']:.0f} / kd {hold['kd']:.1f}")
            if action.caution:
                print(f"[action] 注意: {action.caution}")
            for step in action.steps:
                q_goal = np.radians(np.array(step.q_deg) * sign)
                arm.move_to(q_goal, duration=step.duration, label=step.label)
                if step.hold_s > 0:
                    print(f"[action] {step.hold_s:.1f} 秒保つ")
                    hold_pose(arm, step.hold_s)
            arm.move_to(q_start, label="開始姿勢へ戻る")
        if arm.stop_reason:
            print(f"[action] 中止: {arm.stop_reason}")
            return 130
        print("[action] 完了")
        return 0
    except UnsafeTargetError as e:
        print(f"[action] 目標を拒否した（送信していない）: {e}")
        return 2
    except ValueError as e:
        print(f"[action] 設定の誤り: {e}")
        return 2
    except NoMotionError as e:
        print(f"[action] ❌ {e}")
        return 3
    except WaistDeviationError as e:
        print(f"[action] ❌ {e}")
        return 5
    except StateTimeoutError as e:
        print(f"[action] ❌ {e}")
        return 4
    except StopRequested as e:
        print(f"[action] 中止: {e}")
        return 130
    except RuntimeError as e:
        print(f"[action] ❌ {e}")
        return 4
    finally:
        backend.close()


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--action", choices=sorted(ACTIONS), required=True, help="動作の名前")
    p.add_argument("--path", choices=["sim", "arm_sdk"], required=True,
                   help="sim（MuJoCo）か arm_sdk（実機。歩行コントローラを止めない）")
    p.add_argument("--arm-config", default="arm.yaml", help="腕の設定（configs/ 基準）")
    p.add_argument("--robot-config", default="robot.yaml", help="機体の設定（configs/ 基準）")
    p.add_argument("--arm", choices=["left", "right"], help="動かす腕（既定は設定ファイルの値）")
    p.add_argument("--gravity-scale", type=float,
                   help="腕の重力補償の倍率 0.0〜1.0（実機は 0.5 を勧める。arm.yaml の記録）")
    p.add_argument("--waist-max-deg", type=float, default=DEFAULT_WAIST_MAX_DEG,
                   help=f"腰の角度が開始時からこれ以上ずれたら中止する [deg]（既定 {DEFAULT_WAIST_MAX_DEG:.0f}。"
                        "この動作だけの値で、arm.yaml の既定（約 2.9）は変えない。point_level は kp 60 で 10 が要った）")
    p.add_argument("--waist-kp", type=float, default=DEFAULT_WAIST_KP,
                   help=f"腰を開始時の角度に保つ PD の kp（既定 {DEFAULT_WAIST_KP:.0f}。この動作だけの値で、"
                        f"arm.yaml の arm_sdk.waist_hold = 60 は変えない。{MAX_WAIST_KP:.0f} まで）")
    p.add_argument("--waist-kd", type=float, default=DEFAULT_WAIST_KD,
                   help=f"腰を保つ PD の kd（既定 {DEFAULT_WAIST_KD}。{MAX_WAIST_KD:.0f} まで）")
    p.add_argument("--network-interface", help="G1 につないでいる NIC（既定は設定ファイル）")
    p.add_argument("--execute", action="store_true", help="実際に送信する（付けなければ dry-run）")
    p.add_argument("--no-confirm", action="store_true", help="各段階の Enter 待ちを省く（--execute 時の既定は確認あり）")
    args = p.parse_args(argv)
    if not (0.0 < args.waist_kp <= MAX_WAIST_KP):
        p.error(f"--waist-kp は 0 より大きく {MAX_WAIST_KP:.0f} 以下: {args.waist_kp}")
    if not (0.0 <= args.waist_kd <= MAX_WAIST_KD):
        p.error(f"--waist-kd は 0 以上 {MAX_WAIST_KD:.0f} 以下: {args.waist_kd}")
    if not (0.0 < args.waist_max_deg <= 15.0):
        p.error(f"--waist-max-deg は 0 より大きく 15 以下（腰 pitch の可動域 ±30° の半分）: {args.waist_max_deg}")

    overrides: dict[str, Any] = {}
    if args.network_interface:
        overrides["network_interface"] = args.network_interface
    if args.path == "arm_sdk" and args.gravity_scale is None:
        args.gravity_scale = 0.5
    dry_run = not args.execute
    confirm = args.execute and not args.no_confirm
    mode = "dry-run（何も送らない）" if dry_run else ("実送信・確認モード" if confirm else "実送信・確認なし")
    print(f"[action] {mode}  path={args.path}  action={args.action}")
    return run_action(ACTIONS[args.action], args, args.path, dry_run, confirm, overrides)
