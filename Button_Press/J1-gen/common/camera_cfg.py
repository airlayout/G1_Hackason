"""カメラの接続先の設定（configs/camera.yaml など）の読み込み。すべてのスクリプトでこれを使う。

接続先（PC2 の IP）は、設定ファイルの先頭の pc2_host の 1 か所で決める。--host を付けると、その場で上書きできる。
    有線（ラボ PC）: configs/camera.yaml       pc2_host: 192.168.123.164
    無線（ノート PC）: configs/camera_wifi.yaml  pc2_host: 192.168.0.82（DHCP なので変わることがある）
    模擬ロボット:    configs/camera_sim.yaml   pc2_host: 127.0.0.1
"""

from __future__ import annotations

import argparse
from typing import Any

from .config import load_config


def add_camera_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--camera-config", default="camera.yaml",
                   help="カメラの接続先の設定（無線は camera_wifi.yaml、模擬ロボットは camera_sim.yaml）")
    p.add_argument("--host", help="PC2 の IP（設定ファイルの pc2_host より優先）")


def load_camera_config(args: argparse.Namespace, quiet: bool = False) -> dict[str, Any]:
    """rgbd / legacy_rgb の server_address を、--host → 各項目の server_address → pc2_host の順で決める。"""
    cfg = load_config(getattr(args, "camera_config", None) or "camera.yaml")
    host = getattr(args, "host", None)
    for key in ("rgbd", "legacy_rgb"):
        sec = cfg.get(key)
        if sec is None:
            continue
        sec["server_address"] = host or sec.get("server_address") or cfg.get("pc2_host")
        if not sec["server_address"]:
            raise ValueError(f"カメラの接続先が無い（{args.camera_config} に pc2_host を書くか、--host を付ける）")
    if not quiet:
        src = "--host" if host else (getattr(args, "camera_config", None) or "camera.yaml")
        print(f"[camera] 接続先: {cfg['rgbd']['server_address']}（{src}）")
    return cfg
