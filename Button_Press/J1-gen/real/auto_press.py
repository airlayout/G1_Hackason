"""事前確認 → 計画 → 押す、を Enter なしで最後まで流す（ボトル / エレベーターの呼びボタン）。ラボ PC で実行する。

    # まず dry-run（既定）: 事前確認のあと、押す流れを送らずに動かす。何も送らない
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/auto_press.py --target bottle
    # 送る（人がリモコンを持ち、危なければ L2+B）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/auto_press.py --target bottle --execute
    # エレベーターの呼びボタン（▼）。ラボは車いす用の下のボタン・YOLO
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/auto_press.py --target down --button-set wheelchair \\
        --detector yolo --execute
    # 壁もボタンも無いところで、エレベーターと同じ腕の動きだけ（空中の車いす用 ▼。腕を伸ばしたときの体の崩れを見る）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/auto_press.py --target down --button-set wheelchair \\
        --air 0.48 -0.14 --execute
    # 左腕で押す（ボタンが体の左に来るように立つ。空中なら左右を + にする）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/auto_press.py --target down --button-set wheelchair \\
        --arm left --air 0.48 0.14

事前確認（押すたびに毎回。見るだけで、そのつど変わりうることだけ。1 つでもだめなら何も送らずに終わる。終了コード 10）:
1. モーションコントローラが稼働中か（arm_sdk の受け手。倒れた・ダンピングにしたあとは止まっている）
2. 立っているか（lowstate の IMU の骨盤の傾き、腰の角度）
3. 頭カメラのカラーが止まっていないか（2 枚の差。2026-10-21 は途中で videohub のカラーが止まった）
4. 頭カメラの向き: 同じ 2 枚の深度の床・机の面から pitch を求め、位置の計算に使う向き（ボタンは URDF、ボトルは
   head_camera_mount があればその値）から 10° より大きくずれていないか。首の向きはつなぎ直すたびにずれることがあった
   （2026-11-03、61.2°・64.4°。直すと 51〜54°）ので、その日に 1 回の前提から毎回の確認に移した。
   水平な面が写らず測れないときは止めない（注意だけ）。--skip-head-tilt で飛ばせる
5. ボタンのときだけ: 腰・腕のモータの mode が 1 か（プログラムを終えるとゼロトルクに戻ることがある。
   ボトルは press_bottle.py の ArmCommander が送る前に確かめるので、ここでは見ない）

その日に 1 回でよい前提（このスクリプトでは見ない。REAL_DAY_PROCEDURE.md の「Enter なしで最後まで流す」の前提）:
接続（check_connection.py --rgbd。mode_machine・lowstate の頻度）、
ボトルなら机の箱（make_table_boxes.py。机を動かしたら作り直す）、ボタンなら camera.yaml の head_camera_mount を使っていないこと
（エレベーターのエージェントは robot.yaml の向きのまま）。頭カメラの向きは、その日の最初に check_head_tilt.py でも確かめる
（床が広く写る所で、首を直す前後を測るため）。

押す（事前確認がすべて通ったとき）:
- ボトル: real/press_bottle.py --path arm_sdk（--execute なら --execute --no-confirm）。段階ごとの Enter 待ちをしない
- ボタン: Button_Press/Yada/contest/run_dds.py --agent Button_Press/J1-gen/contest_agent --no-loco（--execute でなければ --dry-run）
どちらも、計画が立たない（届かない・当たる・見つからない）ときは腕を動かさずに終わる。
Ctrl+C を押すと、押す側がその場で止めて来た経路を逆にたどって戻る（もう一度 Ctrl+C でその場で weight を下げる）。
このスクリプトは、押す側が終わるまで待つ。結果（事前確認・コマンド・終了コード）は _local/button_press/auto/ に JSON で残す。
ボタンのときは、エージェントの結果（点灯した・確かめられない・中止とその理由、段階ごとのメッセージ、body_log のフォルダ）も
"agent" に入れる（終了コードは中止でも 0 なので、これを見る）。
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
MAX_PELVIS_TILT_DEG = 5.0
MAX_WAIST_DEG = 3.0
MIN_COLOR_CHANGE = 0.05  # 2 枚のカラーの差（画素の平均）。止まっていると 0.00、動いていれば 1 前後（2026-10-21）
# 頭カメラの向き（深度の床の面から求めた pitch）と、位置の計算に使う向きの差の上限 [deg]。2026-11-03: 51〜54°
# （URDF 47.6° から +3〜+7°）では推定の位置は 0〜5 mm で合い点灯した。首がずれた 61.2°・64.4°（+14〜+17°）を止める
MAX_HEAD_PITCH_DIFF_DEG = 10.0


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
        if not args.skip_head_tilt:
            checks.append(head_tilt_check([a, b], st.q[list(WAIST_IDX)], st.imu_quat, expected_head_pitch(args)))

    if args.target != "bottle":
        st = reader.latest() or st
        off = [JOINT_NAMES[i] for i in list(WAIST_IDX) + list(LEFT_ARM_IDX) + list(RIGHT_ARM_IDX) if st.motor_mode[i] != 1]
        checks.append(Check("モータの mode", not off, "腰・腕すべて 1" if not off else
                            f"1 でない: {', '.join(off)}（ゼロトルク。リモコンでダンピングに入れてから立たせる）"))
    return checks


def expected_head_pitch(args: argparse.Namespace) -> tuple[float, str]:
    """位置の計算に使う頭カメラの pitch [deg] と、その出どころ。ボトルはカメラの設定の head_camera_mount があればその値
    （press_bottle.py が使う）、ボタンは robot.yaml（URDF）の値（エレベーターのエージェントは上書きを使わない）。"""
    if args.target == "bottle":
        mount = load_camera_config(args, quiet=True).get("head_camera_mount")
        if mount and mount.get("rpy_deg") is not None:
            return float(mount["rpy_deg"][1]), f"{args.camera_config} の head_camera_mount"
    return float(np.degrees(load_config("robot.yaml")["head_camera"]["rpy"][1])), "URDF"


def head_tilt_check(frames: list[Any], q_waist: np.ndarray, imu_quat: np.ndarray | None,
                    expected: tuple[float, str]) -> Check:
    """頭カメラの向き（深度の床・机の面から求めた pitch。common/head_tilt.py）が、位置の計算に使う向きと大きく違わないか。

    首の向きは、つなぎ直すたびにずれることがある（2026-11-03、ラボで 61.2°、エレベーターで 64.4°。直すと 51〜54°）。
    ずれたまま押すと、ボタン・ボトルの位置が大きくずれる。水平な面が写らず測れないときは止めない（注意だけ出す。
    エレベーターの前では床が画像の 1 割ほどしか写らないことがある）。
    """
    from common.camera_geometry import HeadCameraTransform
    from common.head_tilt import estimate_head_tilt, judge

    robot_cfg = load_config("robot.yaml")  # 面から向きを解くのに使うのは、首より上の取り付け（pitch・roll はここで求める）
    tf = HeadCameraTransform(robot_cfg)
    planes = [p for f in frames for p in estimate_head_tilt(f, tf, robot_cfg, q_waist, imu_quat)]
    pitch_ref, source = expected
    res = judge(planes, pitch_ref)
    if res["pitch_deg"] is None:
        return Check("頭カメラの向き", True, "⚠️ 水平な面（床・机）が写らず測れない（止めない）。"
                     "首がずれていないか、床が広く写る向きで real/check_head_tilt.py を実行して確かめる")
    diff = res["pitch_deg"] - pitch_ref
    msg = (f"pitch {res['pitch_deg']:.1f}°（{source} {pitch_ref:.1f}° から {diff:+.1f}°、面 {res['n_planes']} 枚"
           + (f"、ばらつき {res['spread_deg']:.1f}°" if res["n_planes"] > 1 else "") + "）")
    ok = abs(diff) <= MAX_HEAD_PITCH_DIFF_DEG
    if not ok:
        msg += (f"。{MAX_HEAD_PITCH_DIFF_DEG:.0f}° より大きくずれている（首の向きがずれた）。首を直して "
                "real/check_head_tilt.py で測り直す")
    return Check("頭カメラの向き", ok, msg)


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
        if getattr(args, "arm", None):
            cmd += ["--arm", args.arm]
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
    if getattr(args, "arm", None):
        env["J1GEN_ARM"] = args.arm
    if args.button_set:
        env["J1GEN_BUTTON_SET"] = args.button_set
    if args.detector:
        env["J1GEN_BUTTON_DETECTOR"] = args.detector
    if getattr(args, "max_joint_speed", None):
        env["J1GEN_MAX_JOINT_SPEED"] = str(args.max_joint_speed)
    for name, v in (("J1GEN_PRESS_KEEP_MM", "keep_mm"), ("J1GEN_STOP_LAG_MM", "stop_lag_mm"), ("J1GEN_PRESS_DEPTH_MM", "depth_mm")):
        if getattr(args, v, None) is not None:
            env[name] = str(getattr(args, v))
    if getattr(args, "no_retry", False):
        env["J1GEN_NO_RETRY"] = "1"
    if getattr(args, "air", None):
        env["J1GEN_AIR_BUTTONS"] = f"{args.air[0]},{args.air[1]}"
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
    p.add_argument("--arm", choices=["left", "right"],
                   help="押す腕（既定は ボタン: elevator.yaml の press.arm / ボトル: pipeline.yaml）。左腕で押すときは、"
                        "ボタンが体の左に来るように立つ（--air なら Y を + にする）")
    p.add_argument("--button-set", choices=["general", "wheelchair"], help="ボタン: 押す組（既定は elevator.yaml）")
    p.add_argument("--detector", choices=["classic", "yolo"], help="ボタン: 見つけ方（既定は elevator.yaml）")
    p.add_argument("--instruction", default="", help="ボタン: 指示の文")
    p.add_argument("--max-joint-speed", type=float, metavar="RAD_S",
                   help="ボタン: 計画で許す関節の速さ [rad/s]（既定は arm.yaml の 0.5。腕が伸びきる近くで「関節が 1 周期に … 動く」と"
                        "断られるとき。送るときの頭打ちは run_dds の安全フィルタの 1.0）")
    p.add_argument("--keep-mm", type=float,
                   help="ボタン: 指先が止まったあと、実際の指先より何 mm 先を押し続けるか（既定は elevator.yaml の 2。大きいほど保持の力が強い）")
    p.add_argument("--stop-lag-mm", type=float,
                   help="ボタン: 指令が実際の指先より何 mm 先に出たら押し込みを止めるか（既定は elevator.yaml の 10。大きいほど止まる前の力が強い）")
    p.add_argument("--depth-mm", type=float,
                   help="ボタン: 表面から何 mm 奥まで押し込む計画にするか（既定は elevator.yaml の 15。--stop-lag-mm より大きくする）")
    p.add_argument("--no-retry", action="store_true",
                   help="ボタン: 点灯していなくても押し直さない（押す力を確かめるとき。押し直しは 1 回目より強い）")
    p.add_argument("--air", type=float, nargs=2, metavar=("X", "Y"),
                   help="ボタン: カメラで探さず、pelvis から前 X・左右 Y [m]（− が右）の空中に車いす用の ▲▼ があるものとして動かす")
    p.add_argument("--skip-prechecks", action="store_true", help="事前確認を飛ばす（模擬ロボットで試すときだけ）")
    p.add_argument("--skip-head-tilt", action="store_true",
                   help="事前確認のうち、頭カメラの向きだけを飛ばす（床でない斜めの面を床と取り違えて止まるとき。"
                        "real/check_head_tilt.py で別に確かめてから）")
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
    agent_result = path.with_name(path.stem + "_agent.json")
    if args.target != "bottle":
        env["J1GEN_RESULT_JSON"] = str(agent_result)  # エージェントが結果と中止の理由を書く（contest_agent/agent.py）
    record["command"], record["env"] = cmd, env
    print("[auto] 押す: " + " ".join(f"{k}={v}" for k, v in env.items()) + (" " if env else "") + " ".join(cmd))
    if args.execute:
        print("[auto] ⚠️ 送る。リモコンを持った人が立ち会っていること（緊急停止は L2+B）")
    code = run_until_done(cmd, env)
    record["exit_code"] = code
    record["finished_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    if args.target != "bottle":
        record["agent"] = take_agent_result(agent_result)
    path.write_text(json.dumps(record, indent=2, ensure_ascii=False))
    if record.get("agent"):
        print(f"[auto] エージェントの結果: {record['agent']['outcome']}")
    print(f"[auto] 終了コード {code}。記録: {path}")
    return code


def take_agent_result(path: Path) -> dict[str, Any] | None:
    """エージェントが書いた結果（contest_agent/agent.py の write_result）を読み、ファイルは消す（auto の記録に入れるので）。
    書かれていない（押す側が途中で落ちた）・読めないときは None。"""
    try:
        res = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    path.unlink(missing_ok=True)
    return res


if __name__ == "__main__":
    sys.exit(main())
