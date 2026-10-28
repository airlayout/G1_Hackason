"""事前確認 → 計画 → 押す、を Enter なしで最後まで流す（ボトル / エレベーターの呼びボタン）。ラボ PC で実行する。

    # まず dry-run（既定）: 事前確認のあと、押す流れを送らずに動かす。何も送らない
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/auto_press.py --target bottle
    # 送る（人がリモコンを持ち、危なければ L2+B）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/auto_press.py --target bottle --execute
    # エレベーターの呼びボタン（▼）。ラボは車いす用の下のボタン・YOLO
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/auto_press.py --target down --button-set wheelchair \\
        --detector yolo --execute

事前確認（押すたびに毎回。見るだけで、そのつど変わりうることだけ。1 つでもだめなら何も送らずに終わる。終了コード 10）:
1. モーションコントローラが稼働中か（arm_sdk の受け手。倒れた・ダンピングにしたあとは止まっている）
2. 立っているか（lowstate の IMU の骨盤の傾き、腰の角度）
3. 頭カメラのカラーが止まっていないか（2 枚の差。2026-10-21 は途中で videohub のカラーが止まった）
4. ボタンのときだけ: 腰・腕のモータの mode が 1 か（プログラムを終えるとゼロトルクに戻ることがある。
   ボトルは press_bottle.py の ArmCommander が送る前に確かめるので、ここでは見ない）

その日に 1 回でよい前提（このスクリプトでは見ない。REAL_DAY_PROCEDURE.md の「Enter なしで最後まで流す」の前提）:
接続（check_connection.py --rgbd。mode_machine・lowstate の頻度）、頭カメラの向き（check_head_tilt.py）、
ボトルなら机の箱（make_table_boxes.py。机を動かしたら作り直す）、ボタンなら camera.yaml の head_camera_mount を使っていないこと
（エレベーターのエージェントは robot.yaml の向きのまま）。

押す（事前確認がすべて通ったとき）:
- ボトル: real/press_bottle.py --path arm_sdk（--execute なら --execute --no-confirm）。段階ごとの Enter 待ちをしない
- ボタン: Button_Press/Yada/contest/run_dds.py --agent Button_Press/J1-gen/contest_agent --no-loco（--execute でなければ --dry-run）
どちらも、計画が立たない（届かない・当たる・見つからない）ときは腕を動かさずに終わる。
Ctrl+C を押すと、押す側がその場で止めて来た経路を逆にたどって戻る（もう一度 Ctrl+C でその場で weight を下げる）。
このスクリプトは、押す側が終わるまで待つ。結果（事前確認・コマンド・終了コード）は _local/button_press/auto/ に JSON で残す。
`--` のあとの引数は、押す側のコマンドにそのまま渡す（例: `-- --seed-pose press_bottle`）。

模擬ロボット（sim/sim_robot_server.py）で流れを試すとき（モーションコントローラが無く、描画のカラーは変化しないので、事前確認を飛ばす）:

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/auto_press.py --target bottle --network-interface lo \\
        --camera-config camera_sim.yaml --skip-prechecks --execute -- --detector color --sim-scene-obstacles --seed-pose ""
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.camera_cfg import add_camera_args, load_camera_config  # noqa: E402
from common.config import FEATURE_DIR, REPO_ROOT, load_config  # noqa: E402
from common.robot_model import JOINT_NAMES, LEFT_ARM_IDX, RIGHT_ARM_IDX, WAIST_IDX  # noqa: E402

EXIT_PRECHECK = 10

# 事前確認のしきい値
MAX_PELVIS_TILT_DEG = 3.0
MAX_WAIST_DEG = 3.0
MIN_COLOR_CHANGE = 0.05  # 2 枚のカラーの差（画素の平均）。止まっていると 0.00、動いていれば 1 前後（2026-10-21）


@dataclass
class Check:
    name: str
    ok: bool
    message: str


def color_changed(a: np.ndarray, b: np.ndarray) -> float:
    """2 枚のカラーの差（画素の平均の絶対値）。"""
    return float(np.mean(np.abs(a.astype(float) - b.astype(float))))


def run_prechecks(args: argparse.Namespace, iface: str) -> list[Check]:
    """押すたびに見ること（そのつど変わりうることだけ）。"""
    from common.camera_rgbd import RgbdZmqSource
    from common.dds import LowStateReader
    from common.head_tilt import pelvis_tilt_deg

    reader = LowStateReader(int(load_config("arm.yaml").get("domain_id", 0)), iface)
    try:
        reader.open()
        st = reader.wait(10.0)
    except (TimeoutError, ValueError) as e:
        return [Check("lowstate", False, str(e))]
    checks = [motion_controller_check()]

    tilt = pelvis_tilt_deg(st.imu_quat)
    waist = np.degrees(st.q[list(WAIST_IDX)])
    stand_ok = tilt <= MAX_PELVIS_TILT_DEG and float(np.max(np.abs(waist))) <= MAX_WAIST_DEG
    checks.append(Check("立っているか", stand_ok, f"骨盤の傾き {tilt:.1f}°、腰 yaw/roll/pitch "
                        + ", ".join(f"{v:+.1f}" for v in waist) + "°"))

    cam = load_camera_config(args, quiet=True)["rgbd"]
    with RgbdZmqSource(cam["server_address"], int(cam["port"]), int(cam["timeout_ms"])) as src:
        a = src.read_rgbd()
        time.sleep(0.5)
        b = src.read_rgbd() if a is not None else None
    if a is None or b is None:
        checks.append(Check("頭カメラ", False, f"画像が届かない（{cam['server_address']}:{cam['port']}。PC2 の配信サーバ）"))
    else:
        change = color_changed(a.color_bgr, b.color_bgr)
        msg = f"カラーの変化 {change:.2f}"
        if change < MIN_COLOR_CHANGE:
            msg += "。カラーが止まっている（videohub が古い画像を返している。G1 の再起動で直った例がある）"
        checks.append(Check("頭カメラ", change >= MIN_COLOR_CHANGE, msg))

    if args.target != "bottle":
        st = reader.latest() or st
        off = [JOINT_NAMES[i] for i in list(WAIST_IDX) + list(LEFT_ARM_IDX) + list(RIGHT_ARM_IDX) if st.motor_mode[i] != 1]
        checks.append(Check("モータの mode", not off, "腰・腕すべて 1" if not off else
                            f"1 でない: {', '.join(off)}（ゼロトルク。リモコンでダンピングに入れてから立たせる）"))
    return checks


def motion_controller_check() -> Check:
    """モーションコントローラが稼働中か（MotionSwitcherClient.CheckMode で問い合わせるだけ。切り替えない）。"""
    try:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

        msc = MotionSwitcherClient()
        msc.SetTimeout(2.0)
        msc.Init()
    except Exception as e:  # noqa: BLE001
        return Check("モーションコントローラ", False, f"問い合わせられない（{type(e).__name__}: {e}）")
    for _ in range(3):  # RPC は散発的に失敗するので繰り返す（g1-starter-kit の mode_check.py。応答が無ければ最大 約 7 秒）
        try:
            status, result = msc.CheckMode()
            if status == 0 and result is not None:
                name = result.get("name") or ""
                if name:
                    return Check("モーションコントローラ", True, f"'{name}' が稼働中（arm_sdk の経路）")
                return Check("モーションコントローラ", False, "停止（デバッグ状態）。arm_sdk の受け手がいない。リモコンで立たせ直す")
        except Exception:  # noqa: BLE001
            pass
        time.sleep(0.3)
    return Check("モーションコントローラ", False, "応答が無い（有線接続・電源）")


def press_command(args: argparse.Namespace, iface: str) -> tuple[list[str], dict[str, str]]:
    """押す側のコマンドと、足す環境変数。args.extra（`--` のあと）は最後に足す。"""
    py = sys.executable
    env: dict[str, str] = {}
    if args.target == "bottle":
        cmd = [py, str(FEATURE_DIR / "real" / "press_bottle.py"), "--path", "arm_sdk", "--network-interface", iface,
               "--gravity-scale", str(args.gravity_scale), "--label", args.label or "auto",
               "--camera-config", args.camera_config]
        if args.host:
            cmd += ["--host", args.host]
        if args.execute:
            cmd += ["--execute", "--no-confirm"]
        return cmd + list(getattr(args, "extra", [])), env
    cam = load_camera_config(args, quiet=True)["rgbd"]
    cmd = [py, str(REPO_ROOT / "Button_Press" / "Yada" / "contest" / "run_dds.py"),
           "--agent", str(FEATURE_DIR / "contest_agent"), "--network-interface", iface,
           "--camera-host", str(cam["server_address"]), "--rgbd-port", str(cam["port"]),
           "--target", args.target, "--instruction", args.instruction or args.target, "--no-loco"]
    if not args.execute:
        cmd.append("--dry-run")
    env["J1GEN_BODY_LOG"] = str(REPO_ROOT / "_local" / "button_press" / "body_log")
    if args.button_set:
        env["J1GEN_BUTTON_SET"] = args.button_set
    if args.detector:
        env["J1GEN_BUTTON_DETECTOR"] = args.detector
    return cmd + list(getattr(args, "extra", [])), env


def run_until_done(cmd: list[str], env: dict[str, str]) -> int:
    """押す側を動かし、終わるまで待つ。Ctrl+C は押す側にも届く（安全に止めて戻る）ので、こちらは待ち続ける。"""
    proc = subprocess.Popen(cmd, env={**os.environ, **env})
    while True:
        try:
            return proc.wait()
        except KeyboardInterrupt:
            print("\n[auto] Ctrl+C: 押す側が止めて戻るのを待つ（もう一度 Ctrl+C で、押す側がその場で weight を下げる）")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--target", choices=["bottle", "up", "down"], required=True,
                   help="bottle（机の上のボトル）/ up・down（エレベーターの呼びボタンの ▲・▼）")
    p.add_argument("--execute", action="store_true", help="送る（付けなければ dry-run。事前確認と計画だけで何も送らない）")
    p.add_argument("--network-interface", help="G1 につないでいる NIC（既定は configs/arm.yaml）")
    p.add_argument("--label", default="", help="ボトル: 記録のフォルダ名に付ける言葉（既定 auto）")
    p.add_argument("--gravity-scale", type=float, default=0.5, help="ボトル: 腕の重力補償の倍率（2026-10-14 に決めた 0.5）")
    p.add_argument("--button-set", choices=["general", "wheelchair"], help="ボタン: 押す組（既定は elevator.yaml）")
    p.add_argument("--detector", choices=["classic", "yolo"], help="ボタン: 見つけ方（既定は elevator.yaml）")
    p.add_argument("--instruction", default="", help="ボタン: 指示の文")
    p.add_argument("--skip-prechecks", action="store_true", help="事前確認を飛ばす（模擬ロボットで試すときだけ）")
    add_camera_args(p)
    argv = sys.argv[1:]
    extra = argv[argv.index("--") + 1:] if "--" in argv else []
    args = p.parse_args(argv[:argv.index("--")] if "--" in argv else argv)
    args.extra = extra

    iface = args.network_interface or load_config("arm.yaml")["network_interface"]
    mode = "送る（--execute）" if args.execute else "dry-run（送らない）"
    print(f"[auto] 対象 {args.target}、{mode}、ネットワークの口 {iface}")
    out = REPO_ROOT / "_local" / "button_press" / "auto"
    out.mkdir(parents=True, exist_ok=True)
    record: dict[str, Any] = {"started_at": time.strftime("%Y-%m-%d %H:%M:%S"), "argv": sys.argv, "target": args.target,
                              "execute": args.execute}
    path = out / f"{time.strftime('%Y%m%d_%H%M%S')}_{args.target}.json"

    if args.skip_prechecks:
        print("[auto] ⚠️ 事前確認を飛ばす（--skip-prechecks）")
        checks: list[Check] = []
    else:
        print("[auto] 事前確認（見るだけ。何も送らない）")
        checks = run_prechecks(args, iface)
        for c in checks:
            print(f"[auto]   {'✅' if c.ok else '❌'} {c.name}: {c.message}")
    record["prechecks"] = [asdict(c) for c in checks]
    if not all(c.ok for c in checks):
        record["exit_code"] = EXIT_PRECHECK
        path.write_text(json.dumps(record, indent=2, ensure_ascii=False))
        print(f"[auto] ❌ 事前確認で止めた（何も送っていない）。記録: {path}")
        return EXIT_PRECHECK

    cmd, env = press_command(args, iface)
    record["command"], record["env"] = cmd, env
    print("[auto] 押す: " + " ".join(f"{k}={v}" for k, v in env.items()) + (" " if env else "") + " ".join(cmd))
    if args.execute:
        print("[auto] ⚠️ 送る。リモコンを持った人が立ち会っていること（緊急停止は L2+B）")
    code = run_until_done(cmd, env)
    record["exit_code"] = code
    record["finished_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    path.write_text(json.dumps(record, indent=2, ensure_ascii=False))
    print(f"[auto] 終了コード {code}。記録: {path}")
    return code


if __name__ == "__main__":
    sys.exit(main())
