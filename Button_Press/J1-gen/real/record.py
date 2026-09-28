"""収録ツール: 頭カメラの RGB（と深度）と lowstate を時刻付きで保存する（タスク5）。ラボ PC で実行する。

    # 深度付き + lowstate を、受信したフレームすべて保存（Ctrl+C で止める）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label bottle
    # ボタン撮影: 1 秒に 1 枚だけ保存（間引き保存）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label button --mode interval --interval-s 1
    # ボタン撮影: Enter を押したときだけ 1 枚保存
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label button --mode enter
    # 深度が使えないとき: RGB だけ（run_g1_server.py --camera の 5555）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label bottle --rgb-only

保存先: _local/button_press/recordings/<日時>_<ラベル>/（形式は common/recording.py の先頭のコメント）。
lowstate は DDS で直接受信する（NIC は configs/arm.yaml の network_interface か --network-interface）。
--no-lowstate で lowstate を記録しない（G1 につながっていないとき）。

⚠️ 実機で撮ったデータは取り直せない。終わったら必ずバックアップする（終了時に表示される手順）。
"""

from __future__ import annotations

import argparse
import shutil
import signal
import sys
import threading
import time
from pathlib import Path
from types import FrameType

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import load_config, resolve_repo_path  # noqa: E402
from common.recorder import Recorder  # noqa: E402
from common.recording import RecordingWriter  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--label", default="", help="フォルダ名に付ける言葉（例: bottle、button_1f）")
    p.add_argument("--mode", choices=["all", "interval", "enter"], default="all")
    p.add_argument("--interval-s", type=float, help="interval モードの間隔 [秒]（既定は設定ファイル）")
    p.add_argument("--rgb-only", action="store_true", help="深度を使わず、RGB 互換ストリーム（5555）だけを保存する")
    p.add_argument("--no-lowstate", action="store_true", help="lowstate を記録しない")
    p.add_argument("--network-interface", help="lowstate を読む NIC（既定は configs/arm.yaml）")
    p.add_argument("--address", help="カメラの接続先（既定は configs/camera.yaml）")
    p.add_argument("--port", type=int, help="カメラのポート（既定は configs/camera.yaml）")
    p.add_argument("--duration-s", type=float, help="この秒数で止める")
    p.add_argument("--max-frames", type=int, help="この枚数を保存したら止める")
    p.add_argument("--note", default="", help="メモ（meta.json に残す。置き場所・照明など）")
    args = p.parse_args()

    cfg = load_config("record.yaml")
    cam_cfg = load_config("camera.yaml")
    root = resolve_repo_path(cfg["output_dir"])
    root.mkdir(parents=True, exist_ok=True)
    free_gb = shutil.disk_usage(root).free / 1e9
    print(f"[record] 保存先 {root}（空き {free_gb:.0f} GB）")
    if free_gb < float(cfg["warn_free_gb"]):
        print(f"[record] ⚠️ ディスクの空きが {cfg['warn_free_gb']} GB より少ない")

    # カメラ
    if args.rgb_only:
        from common.perception_bridge import perception

        c = cam_cfg["legacy_rgb"]
        src = perception("camera").ZmqFrameSource(args.address or c["server_address"], args.port or int(c["port"]),
                                                   c["camera_name"], int(c["timeout_ms"]))

        def get_frame():  # type: ignore[no-untyped-def]
            img = src.read()
            return None if img is None else (img, None)
    else:
        from common.camera_rgbd import RgbdZmqSource

        c = cam_cfg["rgbd"]
        src = RgbdZmqSource(args.address or c["server_address"], args.port or int(c["port"]), int(c["timeout_ms"]))

        def get_frame():  # type: ignore[no-untyped-def]
            f = src.read_rgbd()
            return None if f is None else (f.color_bgr, f)
    print(f"[record] カメラ: {'RGB だけ' if args.rgb_only else '深度付き'}"
          f"（tcp://{args.address or c['server_address']}:{args.port or c['port']}）")

    # lowstate
    reader = None
    if not args.no_lowstate:
        from common.dds import LowStateReader

        iface = args.network_interface or load_config("arm.yaml")["network_interface"]
        reader = LowStateReader(0, iface)
        reader.open()
        st = reader.wait(10.0)
        print(f"[record] lowstate を受信（mode_machine={st.mode_machine}）")

    meta = {"label": args.label, "note": args.note, "mode": args.mode, "rgb_only": args.rgb_only,
            "argv": sys.argv, "camera": cam_cfg, "lowstate": not args.no_lowstate,
            "robot": load_config("robot.yaml"), "localize": load_config("localize.yaml")}
    writer = RecordingWriter(root, args.label, meta, cfg["color_format"], int(cfg["jpeg_quality"]))
    print(f"[record] 収録フォルダ: {writer.dir}")

    flag = threading.Event()
    if args.mode == "enter":
        def wait_enter() -> None:
            while True:
                try:
                    input()
                except EOFError:
                    return
                flag.set()
        threading.Thread(target=wait_enter, daemon=True).start()
        print("[record] Enter を押すと 1 枚保存する。Ctrl+C で終わる")

    def trigger() -> bool:
        if flag.is_set():
            flag.clear()
            return True
        return False

    rec = Recorder(writer, get_frame, reader, args.mode,
                   args.interval_s if args.interval_s is not None else float(cfg["interval_s"]),
                   float(cfg["lowstate_hz"]), trigger)

    def on_signal(signum: int, frame: FrameType | None) -> None:
        print("\n[record] 止める（保存を閉じる）")
        rec.stop()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    last_print = [0.0]

    def on_saved(i: int, t: float) -> None:
        if args.mode == "enter" or t - last_print[0] >= 5.0:
            print(f"[record] {i} 枚保存（{t:.0f} 秒）")
            last_print[0] = t

    src.open()
    try:
        n = rec.run(max_frames=args.max_frames, duration_s=args.duration_s, on_saved=on_saved)
    finally:
        src.close()
        writer.close()
    size_mb = sum(f.stat().st_size for f in writer.dir.rglob("*") if f.is_file()) / 1e6
    print(f"[record] 終了: {n} 枚、lowstate {writer.n_lowstate} 行、{size_mb:.0f} MB（受信 {rec.n_received} 枚、"
          f"タイムアウト {rec.n_timeouts} 回）")
    print(f"[record] ⚠️ 実機のデータは取り直せない。バックアップする（例: 外付けドライブへ）:")
    print(f"[record]   cp -r {writer.dir} <バックアップ先>/")
    return 0 if n > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
