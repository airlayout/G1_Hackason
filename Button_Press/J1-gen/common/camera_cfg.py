"""カメラの接続先の設定（configs/camera.yaml など）の読み込み。すべてのスクリプトでこれを使う。

接続先（PC2 の IP）は、設定ファイルの先頭の pc2_host の 1 か所で決める。--host を付けると、その場で上書きできる。
    有線（ラボ PC）: configs/camera.yaml       pc2_host: 192.168.123.164
    無線（ノート PC）: configs/camera_wifi.yaml  pc2_host: 192.168.0.82（DHCP なので変わることがある）
    PC2 の中（onboard）: configs/camera_onboard.yaml  pc2_host: 127.0.0.1
    模擬ロボット:    configs/camera_sim.yaml   pc2_host: 127.0.0.1
"""

from __future__ import annotations

import argparse
from typing import Any

import numpy as np

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


def apply_head_camera_mount(robot_cfg: dict[str, Any], cam_cfg: dict[str, Any], quiet: bool = False) -> None:
    """カメラの設定に head_camera_mount があれば、robot_cfg の head_camera の取り付け（xyz / rpy）をその値にする。

    実機の頭カメラの向きは首の傾きで URDF（robot.yaml）と違うことがある（2026-10-21 は 71°、camera.yaml）。robot.yaml は MuJoCo の
    カメラの置き場所にも使うので書き換えず、実機用のカメラ設定でだけ上書きする（camera_sim.yaml には書かない）。
    """
    mount = cam_cfg.get("head_camera_mount")
    if not mount:
        return
    cam = robot_cfg["head_camera"]
    if mount.get("xyz") is not None:
        cam["xyz"] = [float(v) for v in mount["xyz"]]
    if mount.get("rpy_deg") is not None:
        cam["rpy"] = [float(np.radians(v)) for v in mount["rpy_deg"]]
    if not quiet:
        rpy = ", ".join(f"{np.degrees(v):.1f}" for v in cam["rpy"])
        print(f"[camera] 頭カメラの取り付け: rpy [{rpy}]°、xyz {cam['xyz']} m（カメラ設定の head_camera_mount）")


def apply_depth_anchor(loc_cfg: dict[str, Any], cam_cfg: dict[str, Any], quiet: bool = False) -> None:
    """カメラの設定に depth_anchor があれば、localize.yaml の depth.anchor（深度を取る枠の中の点）をその値にする。

    基準点の良い位置は見下ろす角度で変わる。首が傾いて MuJoCo より 24° 下を向いたとき、ボトルの上半分が
    画像の外に出るため、localize.yaml の値（MuJoCo の視点に合わせた値）を実機用のカメラ設定でだけ上書きする。
    """
    anchor = cam_cfg.get("depth_anchor")
    if anchor is None:
        return
    loc_cfg["depth"]["anchor"] = [float(v) for v in anchor]
    if not quiet:
        print(f"[camera] 深度を取る基準点: 枠の {loc_cfg['depth']['anchor']}（カメラ設定の depth_anchor）")
