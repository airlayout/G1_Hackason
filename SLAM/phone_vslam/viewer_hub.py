"""表示の口を 1 つに固定する（ユーザーの依頼 11-17「ポート固定にして」）。

表示のサーバー（rerun の --serve-web）を start_mediamtx.sh が 1 つだけ常に動かし、口は固定:
  ブラウザ   http://<OMEN>:9090/?url=ws://<OMEN>:9877
  道具から   127.0.0.1:9876（SDK の TCP）
較正・単眼 SLAM・ステレオ SLAM・保存した結果の表示は、どれもここへ送るだけ（自分では口を開かない）。
道具を替えても起こし直しても、ブラウザは開いたままでよい（新しい記録が来ると表示がそちらに替わる）。

前は道具ごとに serve_web で口を開いていた（9090・9092・9093）。道具を起こし直すとブラウザは古い口に
つながったままになり、再読み込みが要った。serve_web の前に送ると止まる rerun の不具合（rerun-io/rerun#8417）
もこの形なら起きない。
"""

from __future__ import annotations

import os
import time

WEB_PORT = 9090
WS_PORT = 9877
SDK_ADDR = os.environ.get("PHONE_VSLAM_HUB", "127.0.0.1:9876")
# 2 台以上の軌跡の色（cam1 赤・cam2 青・…）。生の表示（multicam_viz）と保存した結果（view_result）で揃える
CAM_COLORS = [(230, 60, 60), (40, 140, 255), (60, 190, 90), (240, 160, 30)]


def cam_color(c: int) -> tuple[int, int, int]:
    return CAM_COLORS[c % len(CAM_COLORS)]


def viewer_url(host: str) -> str:
    return f"http://{host}:{WEB_PORT}/?url=ws://{host}:{WS_PORT}"


def run_recording_id(app_id: str) -> str:
    """回ごとの記録の名前（日時入り）。同じ回の中で表示のプロセスを起こし直したときは、これを使い回す"""
    return f"{app_id} {time.strftime('%m-%d %H:%M:%S')}"


def connect(rr, app_id: str, blueprint=None, clear: tuple[str, ...] = (), recording_id: str | None = None) -> None:
    """表示のサーバーにつなぐ。

    recording_id を渡せば回ごとに新しい記録になる（SLAM・保存した結果は run_recording_id で回ごとに分ける）。
    前は記録の名前を app_id に固定し、始めに clear の枝を Clear で消していたが、static で送った点群は Clear では
    消えず、新しい回を起こしても前の回の地図が残って見えた（11-17）。渡さなければ app_id に固定する（較正など）。
    ブラウザが前の記録を出したままなら、左の窓の Recordings で新しい日時の記録を選ぶ。
    """
    rr.init(app_id, recording_id=recording_id or app_id, spawn=False)
    rr.connect_tcp(SDK_ADDR)
    if blueprint is not None:
        rr.send_blueprint(blueprint)
    for path in clear:
        rr.log(path, rr.Clear(recursive=True), static=True)
