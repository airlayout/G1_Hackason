# G1 実機側カメラサーバー調査メモ（参考程度）

> ⚠️ **参考程度の情報。** G1（PC2, `ssh g1`）上のプログラムは整理されておらず、将来変更・移動される可能性が高い。
> 2026-09-29 に SSH で調べた時点のスナップショット。使う前に現物を確認すること。
> ロボット未接続の状態で、取得済みの情報だけから書いている（起動・動作確認はしていない）。

## G1 の状況（2026-09-29 時点）
- ホスト `ubuntu`。eth0 `192.168.123.164` / wlan0 `192.168.0.82` / Tailscale `100.78.135.14`
- カメラサーバーは**どれも起動していなかった**（8080 / 5555 / 5556 は空き。80 は Unitree 標準 Tornado）
- **D435i は USB3.0 で PC2 に接続済み**。`lsusb` は `8086:0b3a "Intel Corp. 4-Port USB 3.0 Hub"` と表示するが、
  これは usb.ids が古いための誤表示で実体は D435i（pyrealsense2 でも認識: serial 250122075509, USB 3.2）
- `/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435i_...-video-index0〜3`（video4 / 1 / 2 / 3）
- ⚠️ Unitree 純正の `/unitree/module/video_hub_pc4/videohub_pc4 /dev/video4` が D435i のカラー側を常時開いている。
  RealSense は 1 プロセスしか開けないので、`rgbd_server.py` と衝突する可能性あり（**未検証**）

## D435i を開発PCへ出す候補
| # | G1 上のファイル | 対象 | 方式 | ポート | 更新日時 | 起動 |
|---|---|---|---|---|---|---|
| 1 | `~/button_press/real/depth_server/rgbd_server.py`（+ `start_rgbd_server.sh`） | **D435i** | ZMQ PUB（RGB JPEG + 深度 16bit zlib + 内部パラメータ） | 5556 深度付き / 5555 RGB 互換（`ZmqFrameSource` でそのまま受けられる） | 09-29 19:09 | `bash ~/button_press/real/depth_server/start_rgbd_server.sh`（`/usr/bin/python3` 3.8 で動かす） |
| 2 | `~/g1_ui_camera/camera_server.py` | 外付け Webcam | ZMQ PUB（JSON + base64 JPEG、lerobot 互換） | 5555 | 09-23 10:48 | `python3 camera_server.py --list` → `--device /dev/v4l/by-id/...` |
| 3 | `~/g1-bottle-reaction-wander/tools/g1_usb_send.py` | 外付け USB Webcam（**RealSense は自動選択から除外**） | GStreamer RTP/JPEG over UDP | 56000（Wi-Fi 版 56001） | 09-17 20:53 | 開発PC側 `g1_dual_camera.py --start-usb-sender ...` から SSH 経由 |
| 4 | `~/mapping_tools/dual_webcam_server.py` | 内蔵 video2 + Webcam video6 | HTTP MJPEG | 8080 | 09-11 20:52 | `python3 dual_webcam_server.py --port 8080` |

D435i を公開できるのは **#1 のみ**。#3 は D435i ではない。

## rgbd_server.py の設定（`~/button_press/configs/depth_server.yaml`）
- 640x480、カメラ 30fps で読み、**配信は 10fps に間引き**（`max_fps: 10`）
- `bind_address: "*"`、`camera_name: head_camera`、`serial` 空 = 最初に見つかった 1 台
- 受信側の接続先は `configs/camera.yaml`（開発PC側）
- `pyrealsense2` は `/usr/bin/python3`（3.8）に導入済み。conda の lerobot（3.12）では glibc 不足で動かない

## 未確認・次にやること（実機接続後）
1. `start_rgbd_server.sh --list-devices` で D435i が見えるか
2. `videohub_pc4` が掴んでいる状態で `rgbd_server.py` がカラー/深度を取れるか（衝突の有無）
3. 開発PC側へのポート公開（SSH ポートフォワード等）の経路確認
4. 受信側は `Perception/common/camera/zmq_camera.py`（この worktree では sparse 対象外）（`ZmqFrameSource`）を利用
