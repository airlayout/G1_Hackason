"""全体をつなぐスクリプト（実機）: 検出 → 目標 → IK → 手前の姿勢 → 押し込み → 戻る（タスク6）。ラボ PC で実行する。

    # dry-run（既定）: 検出・計画・指令の計算まで行い、送信しない。記録は残す
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/press_bottle.py --path arm_sdk
    # 実際に送る（確認モードで各段階 Enter）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/press_bottle.py --path arm_sdk --execute
    # 深度が使えないとき: 教えた姿勢の中指の先 + 定規で測ったずれを対象にする
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/press_bottle.py --path arm_sdk --target manual --taught-pose touch_bottle --offset 0 0 0

流れと記録は common/pipeline.py。記録は _local/button_press/runs/<日時>_<ラベル>/（dry-run でも残す）。
開始時に、ネットワークの口と、相手が実機か模擬ロボット（口が lo）かを表示し、食い違っていれば中止する。

⚠️ 実機で送る前に: 人がリモコンを持つ（緊急時は L2+B）。検出するときは腕をカメラの視野から外す。
   プランB（--path lowcmd）は座った状態か吊り下げで。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.arm import make_backend  # noqa: E402
from common.config import load_config  # noqa: E402
from common.dds import peer_label  # noqa: E402
from common.localize import make_detector  # noqa: E402
from common.pipeline import PressPipeline  # noqa: E402
from common.pipeline_cli import add_common_args, load_pipeline_config, make_logger  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--path", choices=["arm_sdk", "lowcmd"], required=True, help="プランA（arm_sdk）/ プランB（lowcmd）")
    p.add_argument("--network-interface", help="G1 につないでいる NIC（既定は configs/arm.yaml。lo なら模擬ロボット）")
    p.add_argument("--camera-config", default="camera.yaml", help="カメラの接続先（模擬ロボットは camera_sim.yaml）")
    p.add_argument("--no-camera", action="store_true", help="カメラを使わない（manual のときだけ）")
    p.add_argument("--execute", action="store_true", help="実際に送信する（付けなければ dry-run）")
    p.add_argument("--no-confirm", action="store_true", help="各段階の Enter 待ちを省く")
    add_common_args(p)
    args = p.parse_args()

    cfg = load_pipeline_config(args)
    if args.network_interface:
        cfg.arm["network_interface"] = args.network_interface
    iface = cfg.arm["network_interface"]
    print(f"[press_bottle] ネットワークの口: {iface or '（未設定）'}、相手: {peer_label(iface) if iface else '?'}、"
          f"経路 {args.path}、{'送信する（--execute）' if args.execute else 'dry-run（送信しない）'}")
    if args.no_camera and cfg.pipeline["target"]["source"] == "depth":
        print("[press_bottle] ❌ --no-camera は --target manual のときだけ使える")
        return 2

    src = None
    grab = None
    if not args.no_camera:
        from common.camera_rgbd import RgbdZmqSource

        c = load_config(args.camera_config)["rgbd"]
        src = RgbdZmqSource(c["server_address"], int(c["port"]), int(c["timeout_ms"]))
        src.open()
        grab = src.read_rgbd
    detector = make_detector(cfg.localize) if cfg.pipeline["target"]["source"] == "depth" else None
    backend = make_backend(cfg.arm, cfg.robot, dry_run=not args.execute, path=args.path)
    logger = make_logger(cfg, args.label, {"kind": "real", "path": args.path, "network_interface": iface,
                                           "peer": peer_label(iface) if iface else None,
                                           "execute": args.execute, "camera_config": args.camera_config})
    try:
        backend.open()
        pipe = PressPipeline(cfg, backend, logger, grab, detector, confirm=args.execute and not args.no_confirm)
        res = pipe.run()
    except ValueError as e:  # NIC 名が空、など
        print(f"[press_bottle] 設定の誤り: {e}")
        logger.close(code=2, message=str(e))
        return 2
    finally:
        backend.close()
        if src is not None:
            src.close()
    print(f"[press_bottle] 記録: {logger.dir}")
    return res.code


if __name__ == "__main__":
    sys.exit(main())
