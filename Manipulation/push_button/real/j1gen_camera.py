"""J1-genのRGB-D受信・頭カメラFKを、既存の床基準の押下処理につなぐ。"""

from __future__ import annotations

import importlib
import importlib.util
import json
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
CACHE = REPO_ROOT / "_local/button_press_camera"


class J1CameraRuntime:
    """commonという名前を共有せず、準備済みのJ1-genをそのまま読み込む。"""

    def __init__(self, root=None):
        if root is None:
            root = REPO_ROOT / "Button_Press/J1-gen"
            if not (root / "common/camera_rgbd.py").is_file():
                current = CACHE / "current.json"
                if not current.is_file():
                    raise FileNotFoundError("J1-genのカメラ部品がありません。real/prepare_camera.pyを実行してください")
                commit = json.loads(current.read_text())["commit"]
                if len(commit) != 40 or any(c not in "0123456789abcdef" for c in commit):
                    raise ValueError("カメラ部品のコミットIDが不正です")
                root = CACHE / commit / "Button_Press/J1-gen"
        self.root = Path(root).expanduser().resolve()
        common = self.root / "common"
        if not (common / "camera_rgbd.py").is_file():
            raise FileNotFoundError(f"J1-genのカメラ部品がありません: {self.root}")
        # Yada用のj1gen_commonやPerception/commonと衝突させない。
        import hashlib

        self.package = "push_button_camera_j1gen_" + hashlib.sha256(str(common).encode()).hexdigest()[:12]
        if self.package not in sys.modules:
            spec = importlib.util.spec_from_file_location(
                self.package, common / "__init__.py", submodule_search_locations=[str(common)])
            if spec is None or spec.loader is None:
                raise ImportError(f"J1-gen/commonを読み込めません: {common}")
            module = importlib.util.module_from_spec(spec)
            sys.modules[self.package] = module
            try:
                spec.loader.exec_module(module)
            except BaseException:
                sys.modules.pop(self.package, None)
                raise

    def module(self, name):
        return importlib.import_module(f"{self.package}.{name}")


def add_camera_arguments(parser):
    parser.add_argument("--j1gen-root", help="J1-genの配置場所（省略時は通常配置か準備済みのカメラ部品）")
    parser.add_argument("--camera-config", default="camera.yaml", help="J1-genのカメラ設定。無線はcamera_wifi.yaml")
    parser.add_argument("--host", help="PC2のIP。camera-configのpc2_hostより優先")


class J1RgbdCamera:
    """RgbdZmqSourceの画像時刻・IDを検査し、撮影時刻をローカルの単調時計へ変換する。"""

    def __init__(self, runtime, camera_config="camera.yaml", host=None, max_age_s=0.5):
        from types import SimpleNamespace

        cfg = runtime.module("camera_cfg").load_camera_config(
            SimpleNamespace(camera_config=camera_config, host=host))
        rgbd = cfg["rgbd"]
        port, timeout_ms = int(rgbd.get("port", 5556)), int(rgbd.get("timeout_ms", 5000))
        if not 1 <= port <= 65535 or timeout_ms <= 0 or not np.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("RGB-Dのポート・待機時間・画像の有効時間が不正です")
        # 計測全体の期限はread_afterで管理する。単発のrecvで数秒間止まらないようにする。
        self.source = runtime.module("camera_rgbd").RgbdZmqSource(
            rgbd["server_address"], port, min(timeout_ms, 100))
        self.max_age_s = max_age_s
        self.last_id = -1
        self.last_timestamp = -np.inf
        self.last = None
        try:
            self.source.open()
        except BaseException:
            self.source.close()
            raise

    def close(self):
        self.source.close()

    def poll(self):
        """制御ループから待機せず、新しい撮影時刻のフレームだけを取得する。"""
        frame = self.source.read_rgbd(timeout_ms=0)
        if frame is None:
            return None
        self.validate_frame(frame)
        age = time.time()-frame.timestamp
        if age < -.05:
            raise RuntimeError("RGB-Dの撮影時刻が未来です")
        if (age > self.max_age_s or frame.frame_id <= self.last_id or frame.timestamp <= self.last_timestamp):
            return None
        self.last_id,self.last_timestamp,self.last = frame.frame_id,frame.timestamp,frame
        return frame,time.monotonic()-max(0.,age)

    @staticmethod
    def validate_frame(frame):
        intr = frame.intrinsics
        if (frame.camera != "head_camera" or frame.depth.dtype != np.uint16
                or frame.depth.shape != (intr.height, intr.width)
                or frame.color_bgr.shape != (intr.height, intr.width, 3)
                or frame.color_bgr.dtype != np.uint8):
            raise ValueError("頭カメラのカラー・位置合わせ済み深度・内部パラメータの大きさが不正です")
        if (not np.all(np.isfinite([frame.timestamp, frame.depth_scale, intr.fx, intr.fy, intr.cx, intr.cy]))
                or frame.depth_scale <= 0 or intr.fx <= 0 or intr.fy <= 0 or frame.frame_id < 0):
            raise ValueError("RGB-Dの時刻・深度単位・内部パラメータが不正です")

    def read_after(self, capture_after, deadline):
        while time.monotonic() < deadline:
            frame = self.source.read_rgbd()
            if frame is None:
                continue
            self.validate_frame(frame)
            now, epoch = time.monotonic(), time.time()
            age = epoch - frame.timestamp
            if age < -0.05:
                raise RuntimeError("RGB-Dの撮影時刻が未来です。PC2と受信PCの時計同期を確認してください")
            if (now >= deadline or age > self.max_age_s or frame.timestamp <= capture_after
                    or frame.frame_id <= self.last_id or frame.timestamp <= self.last_timestamp):
                continue
            self.last_id, self.last_timestamp = frame.frame_id, frame.timestamp
            self.last = frame
            return frame, now - max(0.0, age)
        raise RuntimeError("新しいRGB-Dを受信できません。5556の配信・接続先・PC間の時計同期を確認してください")


class CalibratedHeadCamera:
    """公式モデルのpelvis←optical変換を、既存の床基準のFKと同じ座標へ変換する。"""

    def __init__(self, runtime, kin, calibration):
        self.kin = kin
        cfg = runtime.module("config").load_config("robot.yaml")
        if calibration["camera_body"] != cfg["head_camera"]["parent_body"]:
            raise ValueError("J1-gen方式はtorso_linkに固定した頭カメラ用です")
        self.head = runtime.module("camera_geometry").HeadCameraTransform(cfg)
        if calibration["schema"] == "g1-button-calibration-v1":
            # 従来の実測行列を、公式モデルの取り付け位置への固定補正として引き継ぐ。
            reference_q = kin.data.qpos.copy()
            reference_q[12:15] = calibration["reference_waist_q_rad"]
            nominal = self.nominal_transform(reference_q, calibration["reference_imu_rpy_rad"])
            self.correction = np.linalg.inv(nominal) @ np.asarray(calibration["T_base_optical"], dtype=float)
        else:
            self.correction = np.eye(4)
            self.correction[:3, :3] = runtime.module("camera_geometry").rpy_to_matrix(
                *calibration["camera_rpy_offset_rad"])
            self.correction[:3, 3] = calibration["camera_offset_m"]

    def nominal_transform(self, q, imu_rpy):
        self.kin.update_fixed_base(np.asarray(q), imu_rpy)
        pelvis_from_optical = np.eye(4)
        pelvis_from_optical[:3, :3], pelvis_from_optical[:3, 3] = self.head.pelvis_from_optical(np.asarray(q)[12:15])
        return self.kin.body_transform("pelvis") @ pelvis_from_optical

    def transform(self, q, imu_rpy):
        return self.nominal_transform(q, imu_rpy) @ self.correction
