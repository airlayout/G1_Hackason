"""人がリモコンで柱の手前（約 0.6〜1.5 m）に置いたあと、頭カメラだけで押せる位置まで寄る（common/approach.py）。腕は動かさない。

    # 送らずに、観測と「次にどう動くか」だけを見る（既定。1 回観測して終わる）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/approach.py --target up --button-set wheelchair --detector yolo
    # 実機で寄る（G1 は立って歩ける状態・腕は下ろす・周りに人がいない・リモコンを持った人がそばにいること）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/approach.py --target up --button-set wheelchair --detector yolo --execute
    # 左腕も候補にする（計画が立った腕を出す）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/approach.py --target up --button-set wheelchair --arms right left --execute
    # 前方の Web カメラも使う（較正済みのときだけ効く。無い・映像が来ない・未較正なら、注意を出して頭カメラだけで寄る）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/approach.py --target up --button-set wheelchair --detector yolo --webcam --execute

押せる位置に着いたら（ready）、出てきた腕で real/auto_press.py を実行する（このスクリプトは押さない）。
--execute のときは、歩き出す前に Enter を待つ。歩いている途中の Ctrl+C でも StopMove を送る。
--voice local（PC2 の上）で、歩き始め・着いた・中止のときに喋る（configs/voice.yaml の approach_*）。
結果（観測・判断・1 回ごとの動き）は _local/button_press/approach/<時刻>/result.json に残す。
設定は configs/approach.yaml（face・loco・approach）と elevator.yaml（押す組・勧める立つ位置・▲▼ の見つけ方）。
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.approach import ApproachResult, Approacher, Frame, HeadPerception, PlanFeasibility, WebcamAssist  # noqa: E402
from common.camera_cfg import add_camera_args, load_camera_config  # noqa: E402
from common.column_face import FaceFinder  # noqa: E402
from common.config import REPO_ROOT, load_config  # noqa: E402
from common.elevator_buttons import ButtonFinder  # noqa: E402
from common.loco import LegStillness, LocoMover  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--target", choices=["up", "down"], required=True, help="押すボタン（計画を確かめるのに使う）")
    p.add_argument("--arms", nargs="+", choices=["right", "left"], help="計画を試す腕（既定は approach.yaml）")
    p.add_argument("--button-set", choices=["general", "wheelchair"], help="押す組（既定は elevator.yaml）")
    p.add_argument("--detector", choices=["classic", "yolo"], help="▲▼ の見つけ方（既定は elevator.yaml）")
    p.add_argument("--execute", action="store_true", help="歩く（付けなければ 1 回観測して、次の動きを表示するだけ）")
    p.add_argument("--webcam", action="store_true",
                   help="前方の Web カメラも使う（configs/webcam.yaml。使えなければ頭カメラだけ）")
    p.add_argument("--network-interface", help="G1 につないでいる NIC（既定は configs/arm.yaml）")
    p.add_argument("--no-lowstate", action="store_true",
                   help="lowstate を読まない（--execute なしのときだけ。腰 0°・骨盤は水平・腕は下ろしたままと仮定）")
    p.add_argument("--voice", choices=["local", "ssh"],
                   help="--execute のとき、歩き始め・着いた・中止で G1 に喋らせる（configs/voice.yaml の approach_*。"
                        "local = PC2 の上で動かすとき、ssh = ノート PC から。先に real/voice_phrases.py make・send）")
    p.add_argument("--no-confirm", action="store_true",
                   help="--execute のとき、歩き始める前の Enter を待たない（人がリモコンを持ってそばにいるときだけ）")
    p.add_argument("--result-json", type=Path,
                   help="結果（result.json）の保存先（親フォルダは作る。既定は _local/button_press/approach/<時刻>/result.json）")
    add_camera_args(p)
    args = p.parse_args()
    if args.execute and args.no_lowstate:
        p.error("--execute には lowstate が要る（脚が止まったかを見る）")

    cfg = load_config("approach.yaml")
    app = copy.deepcopy(cfg["approach"])
    if args.arms:
        app["arms"] = args.arms
    elevator = load_config("elevator.yaml")
    if args.button_set:
        elevator["geometry"]["button_set"] = args.button_set
    if args.detector:
        elevator["detector"] = args.detector
    robot_cfg = load_config("robot.yaml")

    reader = None
    if not args.no_lowstate:
        from common.dds import LowStateReader, init_dds

        iface = args.network_interface or load_config("arm.yaml")["network_interface"]
        init_dds(0, iface)
        reader = LowStateReader(0, iface)
        reader.open()
        reader.wait(10.0)
    else:
        print("[approach] ⚠️ lowstate を読まない: 腰は 0°、骨盤は水平、腕は下ろしたままと仮定する")

    from common.camera_rgbd import RgbdZmqSource

    cam = load_camera_config(args)["rgbd"]
    source = RgbdZmqSource(cam["server_address"], int(cam["port"]), int(cam["timeout_ms"]))

    def get_frame() -> Frame | None:
        f = source.read_rgbd()
        if f is None:
            return None
        st = reader.latest() if reader is not None else None
        i = f.intrinsics
        K = np.array([[i.fx, 0, i.cx], [0, i.fy, i.cy], [0, 0, 1.0]])
        q = st.q.copy() if st is not None else np.zeros(29)
        return Frame(f.color_bgr[:, :, ::-1].copy(), f.depth_m(), K, q, st.imu_quat if st is not None else None)

    webcam = webcam_stream = None
    if args.webcam:
        from common.elevator_buttons import make_candidate_detector
        from common.webcam_buttons import WebcamModel, WebcamNotReady, WebcamStream

        wcfg = load_config("webcam.yaml")
        wcfg["buttons"]["button_set"] = elevator["geometry"].get("button_set", wcfg["buttons"]["button_set"])
        try:
            wcam = WebcamModel.from_config(wcfg)
            webcam_stream = WebcamStream(wcfg["stream"].get("host") or cam["server_address"], wcfg)
            webcam_stream.open()
            if webcam_stream.read() is None:
                raise WebcamNotReady("Web カメラの映像が届かない（Mapping の配信 innomaker_camera_server.py、5555 番）")
            webcam = WebcamAssist(webcam_stream.read, make_candidate_detector(elevator), wcam, wcfg)
            print("[approach] 前方の Web カメラも使う")
        except WebcamNotReady as e:
            print(f"[approach] ⚠️ Web カメラは使わない（頭カメラだけで寄る）: {e}")
            if webcam_stream is not None:
                webcam_stream.close()
                webcam_stream = None
    perception = HeadPerception(get_frame, ButtonFinder.from_config(robot_cfg, elevator),
                                FaceFinder.from_config(robot_cfg, cfg["face"]), elevator, cfg["face"], app, webcam)
    feasibility = PlanFeasibility.from_config(robot_cfg, load_config("press.yaml"), load_config("arm.yaml"), elevator,
                                              list(app["arms"]), args.target)
    backend = stillness = None
    if args.execute:
        from common.loco import SdkLocoBackend

        backend = SdkLocoBackend()
        stillness = LegStillness(reader, cfg["loco"]["still"])
    mover = LocoMover(backend, stillness, cfg["loco"], go=args.execute)
    print(f"[approach] 押す組: {elevator['geometry'].get('button_set', 'general')}、見つけ方: {elevator['detector']}、"
          f"腕の候補: {app['arms']}、{'歩く' if args.execute else '歩かない（1 回観測するだけ）'}")
    if args.execute and args.no_confirm:
        print("[approach] ⚠️ --no-confirm: Enter を待たずに歩き始める（リモコンを持った人がそばにいること。危なければ L2+B）")
    elif args.execute:
        input("[approach] 腕を下ろした・周りに人がいない・リモコンを持った人がそばにいることを確かめて Enter（やめるなら Ctrl+C）: ")

    # 声（歩くときだけ。dry-run で「近づきます」と言うと、歩くと誤解するので喋らない）
    voice = None
    if args.voice and args.execute:
        from common.announce import Announcer

        voice = Announcer.from_config(load_config("voice.yaml"), args.voice)
        voice.say("approach_start")

    approacher = Approacher(perception, feasibility, mover, app)
    interrupted = False
    try:
        with source:
            result = approacher.run()
    except KeyboardInterrupt:
        # 途中で止めても、それまでの観測と判断を残す（11/14、Ctrl+C で止めた回の result.json が残らなかった）。
        # 歩いている途中なら、LocoMover が StopMove を送っている
        interrupted = True
        result = ApproachResult("aborted", "Ctrl+C で止めた", None, approacher.steps)
        print("[approach] ❌ Ctrl+C で止めた")
    finally:
        if webcam_stream is not None:
            webcam_stream.close()

    if voice is not None:
        voice.say("approach_ready" if result.outcome == "ready" else "approach_gave_up")
        # 言い終わるまで待つ（続けて auto_press が「ボタンを押します」と言うので、重ならないように）
        voice.close()
    if result.outcome == "ready":
        print(f"[approach] 次: real/auto_press.py --target {args.target} --arm {result.arm}"
              + (f" --button-set {args.button_set}" if args.button_set else "")
              + (f" --detector {args.detector}" if args.detector else ""))
    if args.result_json is not None:
        path = args.result_json
    else:
        path = REPO_ROOT / "_local" / "button_press" / "approach" / time.strftime("%Y%m%d_%H%M%S") / "result.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    saved_args = {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}
    path.write_text(json.dumps({"args": saved_args, "outcome": result.outcome, "reason": result.reason, "arm": result.arm,
                                "interrupted": interrupted, "steps": result.steps,
                                "voice": voice.said if voice is not None else None},
                               indent=2, ensure_ascii=False, default=float))
    print(f"[approach] 保存: {path}")
    if interrupted:
        return 130
    return 0 if result.outcome in ("ready", "dry_run") else 1


if __name__ == "__main__":
    sys.exit(main())
