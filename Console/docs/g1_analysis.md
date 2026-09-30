# G1 (Jetson Orin NX) 取得データの分析

対象: 2026-09-30 23:46〜23:56 JST に取得した読み取り専用スナップショット([g1_raw/MANIFEST.md](g1_raw/MANIFEST.md))。
以後ロボットは電池切れで到達不能。**手元のファイルだけで言えること**を書く。

表記: **[観測]** = ファイルにそのまま出ている / **[推測]** = 観測から導いた仮説(確度つき) / **[不明]** = データが無い。

---

## 0. 要点(先に結論)

1. **RGB(video4/5)が OpenCV で開けない最有力原因は、Unitree 標準サービス `videohub_pc4` が `/dev/video4` を専有していること。**[観測: ps に `root ... /unitree/module/video_hub_pc4/videohub_pc4 /dev/video4`、CPU 19%。過去の社内 docs に「video_hub_pc4 は /dev/video4 固定指定」] 確度: 高。
2. **by-id の `...435i...-video-index0` は video4 を指す**(interface 1.0 の index0 = video0 と 1.3 の index0 = video4 が同名で衝突し、後者が勝つ)。`camera_ctl.sh` の `*RealSense*435i*index0` は **videohub が握っている RGB ノード**を選んでしまう。[観測: cameras.txt の by-id/by-path 対応] 確度: 高。
3. **pyrealsense2 は「未導入」ではない。** システム Python 3.8 に `pyrealsense2 2.55.1.6486` がある。[観測: pip.txt] ただし import/実機動作は未確認。
4. **`bad_alloc` は手元の全ファイルに 1 件も無い。** 「ros2 CLI daemon が約 7GB を確保して bad_alloc」は、このスナップショットからは**裏付けが取れない**(取得時の空きメモリは 12.7GB/15.4GB で健全)。[観測] 別の記録(会話・別ログ)由来の可能性。要再確認。
5. ROS 2 は **foxy と noetic が /opt/ros にあるのみ**(humble は conda/pixi 内に別建て)。DDS は Unitree 純正(CycloneDDS 0.10.2、domain 0、eth0)。[観測]
6. **crontab は無し。** ただし systemd は `docker`, `nginx`, `tailscaled`, `unitree-upgrade`, `key_server`, `openvpn`, `nxserver` などが enabled。カメラ配信の自動起動は存在しない。[観測]
7. LiDAR は Livox MID360、PointCloud2 約 2 万点/フレーム、約 10 Hz。1 フレームをバイナリで完全にデコードできた。[観測]
8. 取得の直前まで **バッテリー SoC は 7% → 0%**(ライブログ約 450 秒で 6 → 0)。取得が途切れたのは電池切れ。[観測]

---

## 1. ハードウェア / OS

| 項目 | 値 | 種別 |
|---|---|---|
| ボード | NVIDIA Jetson Orin NX(ALSA 名 `NVIDIA Jetson Orin NX HDA/APE`、`OrinNXDeveloperKit`) | 観測 |
| L4T / JetPack | L4T **R35.3.1**(`R35 (release), REVISION: 3.1`、2023-03-19 ビルド)。JetPack **5.1.1 相当** | L4T は観測、JetPack 対応は推測(高) |
| OS | Ubuntu 20.04.6 LTS (focal), aarch64 | 観測 |
| カーネル | `5.10.104-tegra #1 SMP PREEMPT` | 観測 |
| Python | システム 3.8.10 / lerobot conda 環境 3.12(`miniforge3/envs/lerobot`)/ pixi 環境 3.11(ROS Humble) | 観測 |
| メモリ | 15,388 MB、swap 7,694 MB。取得時 used 2.3〜2.4 GB / available 12.7 GB | 観測 |
| ストレージ | NVMe 1.9 TB、使用 71 GB(4%) | 観測 |
| USB | tegra-xusb。Bus 2 に Intel 4-port USB3 Hub(8086:0b3a)=D435i(その下に uvcvideo × 5 IF + HID)。Bus 1 に Realtek Hub、`rtk_btusb`(Bluetooth)、`rtl8852bu`(Wi-Fi) | 観測 |
| ネットワーク IF | `eth0` 192.168.123.164/24(ロボット内 LAN)、`wlan0` 192.168.0.x、`tailscale0`(100.x)、`docker0` 172.17.0.1(DOWN)、`dummy0` | 観測 |
| その他 | i2c-0〜9、CAN インタフェース無し(ip link に can が無い) | 観測 |

## 2. インストール済みパッケージ

システム Python 3.8(`/usr/bin/python3`)[観測: pip.txt]:

| パッケージ | 版 | 備考 |
|---|---|---|
| `pyrealsense2` | 2.55.1.6486 | **導入済み**。pip 由来なら RSUSB(libuvc)バックエンド。動作は未確認 |
| `opencv-python` | 5.0.0.93 | pip 版。V4L2 が有効なのは cap_v4l のログで観測。GStreamer は pip 版の通例で**無効の可能性大**(推測・中) |
| `pyzmq` | 18.1.1(3.8 側)/ 27.2.0(lerobot 側) | あり |
| `cyclonedds` | 0.10.2(両環境) | Unitree 純正世代 |
| `unitree_sdk2py` | 1.0.1(editable: `~/unitree_sdk2_python`) | 両環境 |
| `numpy` 1.24.4 / `tensorrt` 8.5.2.2 / `Jetson.GPIO` 2.1.1 / `aiortc` 1.9.0 / `av` 12.3.0 / `pupil_labs_uvc` 1.0.4 / `teleimager` 1.6.0 | | |
| colcon / catkin / rosdep 一式 | | ROS 開発ツール |

lerobot 環境(3.12)[観測]: `lerobot 0.6.2`(editable)、`torch 2.11.0`、`torchvision 0.26.0`、CUDA 13 系 wheel 群、`onnxruntime 1.29.0`、`opencv-python 5.0.0.93` と `opencv-python-headless 4.13.0.92` の**両方**(競合しうる。推測・中)。

**[不明]** librealsense2(apt)の有無、`v4l2-ctl` の有無、GStreamer プラグイン、`cv2.getBuildInformation()`。dpkg 一覧を取っていない。

## 3. ROS 2 / DDS 構成

| 項目 | 内容 | 種別 |
|---|---|---|
| /opt/ros | `foxy`, `noetic`(`ros.txt`) | 観測 |
| Humble | `~/g1_humble/pixi.toml`(robostack-humble, `ros-humble-rmw-cyclonedds-cpp`, `foxglove-bridge 0.8.*`, `nav2-msgs`)。**foxy 同梱 cyclonedds 0.7.0 が Unitree(0.10.2)の discovery で SIGSEGV** するため 0.10 世代を持ち込む設計(pixi.toml コメント、2026-09-03 gdb 確認) | 観測 |
| RMW | pixi 側は CycloneDDS を明示。**システム側の `RMW_IMPLEMENTATION` は未取得**(env.txt が空) | 観測 / 不明 |
| ROS_DOMAIN_ID | 取得できず。Unitree SDK は domain 0 で動作(ヘルパーは `ChannelFactoryInitialize(0, "eth0")`) | 不明 / 観測 |
| CycloneDDS 設定 | `cyclonedds_ws/cyclonedds.xml`: `NetworkInterface name="eth0"`、`AllowMulticast=spdp` | 観測 |
| ポート | UDP 7400/7401(DDS discovery、python3 が保持)、5353(mDNS)、TCP 22, 80(nginx), 4000, 111, 1026(eth0), 1028, 7001, 12001, 22751/22752(localhost) | 観測 |
| 47600 | `cmd_vel_bridge`→`loco_driver`(127.0.0.1:47600、ログにのみ登場。取得時は不在) | 観測(ログ) |

### 実行中プロセス(取得時)

- `videohub_pc4 /dev/video4`(root, CPU 19%, 8:29 分累積)— Unitree 標準カメラ配信。**RGB 専有者**。
- `master_service`(root)— Unitree 純正のサービス管理。videohub を**再起動する側**(履歴に `sudo kill 13068` のあと同名 PID が再出現。[推測・中])。
- `tailscaled`、`dockerd/containerd`、NoMachine(`nxserver`)、`gnome-shell`(gdm)、`/upgradePythonServer/server.py`(root)。
- Console 由来: 常駐ヘルパー(`remote_helper.py` 相当の python3 -u、PID 14059)と DDS サンプル取得(15795、一時)。
- **メモリを食うプロセスは無かった**: RSS 最大は gnome-shell の 75MB。取得時点では ros2 CLI も未起動。[観測]

### 「ros2 CLI daemon が ~7GB 確保 / bad_alloc(fastrtps)」について

- 全ファイル grep で `bad_alloc` 0 件。[観測]
- 推測: `ros2 topic list/info/hz` は履歴に多数(`ros2 topic list | grep`、`ros2 topic hz ...`)。ros2 CLI daemon は **RMW 未指定=既定の fastrtps** で起動し、Unitree の大量のトピック(104)と大型型(PointCloud2/GridMap)を discovery で受けると膨張しうる、という説明は筋が通る。ただし**根拠ログが手元に無い**ので確度は低〜中。次回ロボット接続時は読み取り専用で `free -m` を先に取り、`ros2` CLI は使わない(後述)。

## 4. カメラ

### 4.1 デバイス構成 [観測: cameras.txt, jetson_hw.txt]

| ノード | by-path | 役割(標準的な RealSense の割付に基づく **[推測]**) |
|---|---|---|
| video0 | xhci-usb-0:2.2:**1.0** index0 | Depth (Z16) |
| video1 | 1.0 index1 | Depth の UVC メタデータ |
| video2 | 1.0 index2 | 赤外(IR。Y8 等) |
| video3 | 1.0 index3 | IR のメタデータ |
| video4 | xhci-usb-0:2.2:**1.3** index0 | **RGB(YUYV/MJPG)** |
| video5 | 1.3 index1 | RGB のメタデータ |

- 全ノード権限 `crw-rw-rw-+ root:plugdev`(誰でも open 可)。[観測]
- 全ノードの sysfs 名は `Intel(R) RealSense(TM) Depth Ca...`(切詰)で、名前からは RGB/Depth を区別できない。[観測]
- **by-id は 4 本だけ**(`video-index0→video4`, `index1→video1`, `index2→video2`, `index3→video3`)。`index0` が 1.0 と 1.3 で衝突し video4 が勝つ。video0 と video5 は by-id 名を持たない。[観測]
- USB tree: D435i は USB 3.0(5000M)で uvcvideo × IF0〜4 + usbhid(IF5)。[観測]
- 標準ウェブカメラ(SunplusIT Full HD, USB ID 1bcf:2283)は **今回の lsusb / v4l に存在しない**。過去ログ `camera.log` は `VIDIOC_REQBUFS errno=19 (No such device)` と `can't open camera by index /dev/video0`。[観測] → 取得時点で未接続/未列挙(物理要因が有力。推測・中〜高)。

### 4.2 「グレースケールで開ける」ノード [推測]

OpenCV で開けて 1ch/低彩度に見えたのは、Y8/Y16 系の **IR ノード(video2 が最有力、次に video0=Depth)**。**どのノードかを直接示すデータは無い**(`--list` の実行結果が未保存)。過去の `dual_camera_*.log` は `/dev/video2 + /dev/video6` を開いており、その時は video2 が使えていた[観測]。この事実は video2=IR 説と整合する。

### 4.3 RGB(video4/5)が OpenCV で開けない原因 — 有力順

| 順位 | 仮説 | 根拠 | 確度 |
|---|---|---|---|
| 1 | **`videohub_pc4` が /dev/video4 を専有**(V4L2 は同時 open で `EBUSY`、または取得に失敗) | ps に `videohub_pc4 /dev/video4`(観測)。社内 docs: 「`/unitree/etc/master_service/service/video_hub_pc4` は /dev/video4 を固定指定」「videohub 稼働中は競合」「標準経路は `VideoClient.GetImageSample()`」(`g1-bottle-reaction-wander/docs/G1_PERSON_YOLO.md`, `G1_INTEGRATION.md`, `G1_CAMERA_WIRED.md`) | **高** |
| 2 | **video5(メタデータ)を映像として開いている / 割付誤り** | by-id が index0=video4 に潰れているため `camera_ctl.sh` の選択が曖昧。video5 は UVC メタデータで、`VideoCapture` は成功しても `read()` が失敗する | 中 |
| 3 | **RealSense 用カーネルパッチ無し(L4T 標準 uvcvideo)でのフォーマット/メタデータ非対応** | Jetson 5.10.104-tegra + 標準 uvcvideo。D435i は本来 `realsense-dkms` か librealsense の RSUSB(libuvc)経由を推奨。`video1/3/5` の存在自体は標準 uvcvideo が作ったメタデータノード | 中〜低 |
| 4 | **OpenCV 側の要因**(pip 版 5.0.0.93 の V4L2 が YUYV の特定モード/`CAP_PROP_FOURCC` 指定に弱い、`select() timeout`) | 過去ログに `cap_v4l.cpp: select() timeout` あり(観測)。ただし webcam で発生した可能性 | 低 |
| 5 | 電力/USB 帯域 | 5000M 接続で Depth+IR+RGB 同時は許容範囲 | 低 |

**決め手になるデータは未取得**: `v4l2-ctl -d /dev/video4 --list-formats-ext`、`fuser -v /dev/video4`、`cv2` の errno。→ [DEV_GUIDE_OFFLINE.md](DEV_GUIDE_OFFLINE.md) のチェックリストに手順を書いた。

### 4.4 カメラ経路の実績(過去ログから)

- webcam(video0)を OpenCV で使った撮影は実績あり: 1280x720 MJPG、実効 **8.3 fps**、JPEG 品質 95、732 / 1448 フレーム(`g1_mapping_rgb_20260923T1633*/1636*.launch.log`)。
- 直接 D435i(color/depth)は `/tmp/realsense_depth_probe.cpp` を noetic の librealsense2 でビルドして試した履歴あり(bash_history)。結果は未取得。
- USB を増設すると RealSense は video2〜7 へ移動、videohub は video4 固定のため**不在になる**ことが社内 docs に記録(`G1_PERSON_YOLO.md`)。番号ではなく by-id で扱う根拠。

## 5. DDS トピック(`dds_samples.json`、domain 0、eth0)

- **発見トピック 104**(`rt/api/*` の RPC request/response が約半数)。
- 型: `unitree_hg::LowCmd_/LowState_/BmsState_/IMUState_/MainBoardState_/HandState_`、`unitree_go::SportModeState_/WirelessController_/Error_/SymState_/Go2FrontVideoData_`、`std_msgs::String_`(JSON 文字列)、`sensor_msgs::PointCloud2_/Imu_`、`nav_msgs::Odometry_/OccupancyGrid_`、`GridMap_`、`unitree_api::Request_/Response_`。
- **12 秒間でデータが取れた 16 トピック**: `rt/lowcmd, rt/lowstate, rt/lf/lowstate, rt/odommodestate, rt/lf/odommodestate, rt/secondary_imu, rt/lf/secondary_imu, rt/lf/bmsstate, rt/lf/mainboardstate, rt/arm/action/state, rt/slam_info, rt/rtc/state, rt/public_network_status, rt/audio_msg, rt/audio_msg/filter, rt/gpt_state`。
- **無データ(12 秒)25 トピック**: `rt/arm_sdk`, `rt/wirelesscontroller`(リモコン非操作時)、`rt/lf/emergency_stop`, `rt/lf/battery_alarm`, `rt/servicestate`, `rt/servicestateactivate`, `rt/multiplestate`, `rt/selftest`, `rt/dex3/*`(ハンド無し)、`rt/slam_key_info`, `rt/unitree_slam/waypoints`, `rt/videohub/inner`, `rt/webrtcreq/res` 系、`rt/gpt_cmd` 等。`rt/servicestate` はライブログ中 1 回のみ受信。
- **レート**: ライブログは TimeBasedFilter で間引き(period 500 ms)しているため**実レートではない**(ログ 5.6 件/秒/トピック)。実レートの手掛かりは過去ログ: LiDAR `cloud_livox_mid360` 約 9.7〜10.0 Hz、`imu_livox_mid360` 約 200 Hz、`slam_mapping/points,odom` 約 9.97 Hz、`/dog_odom` 約 10 Hz。lowstate の実レートは**未計測**(SDK 仕様は 500 Hz 級。ヘルパーのコメントに「毎秒数百通」)。
- `rt/lowstate`: `mode_machine = 5`(ライブログ 2523 サンプル全て)、`motor_state` は 35 枠(先頭 29 が関節)。`rt/lowcmd` に **kp=300, kd=3 の指令が常時流れている**(=本体側の制御ループが指令を出している。取得は購読のみ)。
- バッテリー(`rt/lf/bmsstate`): 取得時 SoC 7% → ライブ終盤 0%、電流 −2.1〜−2.2 A(放電)、バッテリー電圧 41.1 V、セル 3.16〜3.18 V。
- `rt/rtc/state = not_connected`、`rt/public_network_status = ON_WIFI_CONNECTED`、`rt/gpt_state = {state:true, llm_name:"clound"}`。
- `service_reads.txt`: 音量 85、腕アクション 22 種+ダンス 4 種、`CheckMode` は `name=''`(**デバッグモード=モーションサービス解除状態**)。`loco.GetFsmId` は 3102(RPC 応答なし)。

## 6. LiDAR フレーム(`lidar_frame0.bin` / `lidar_frames_meta.json`)

| 項目 | 値 | 種別 |
|---|---|---|
| フレーム | `frame_id=livox_frame`、PointCloud2、height=1、`dense=true`、little endian | 観測 |
| 点数 | 20,110 / 20,064 / 20,064(3 フレーム) | 観測 |
| point_step / 総バイト | 22 B / 442,420 B(= 20,110 × 22、bin と一致) | 観測 |
| fields | x f32@0, y f32@4, z f32@8, intensity f32@12, **ring u16@16**, **time f32@18** | 観測 |
| 受信間隔 | 0.093 s, 0.106 s(約 10 Hz) | 観測 |
| デコード結果(frame0) | x −2.42〜27.36 m、y −21.6〜17.6 m、z −1.36〜8.60 m、intensity 0〜151、**ring 0〜3(4 本)**、time 0.255 ms〜100.8 ms(1 フレーム = 0.1 s の点ごとの相対時刻、単位ns 相当)、NaN 0、最大距離 29.5 m、平均 2.07 m | 観測(使い捨て解析) |

- ring が 0〜3 の 4 値のみ = Livox MID360 のスキャンラインを 4 本にまとめた形式と読める [推測・中]。
- 平均距離 2 m は室内の近距離点が多いことを示す。座標系は `livox_frame`(センサー座標)。ロボット座標への変換は `g1_cfg/` の TF スクリプト(`dog_odom_to_tf.py`, `cloud_to_scan.py`, `filter_self_returns.py`)を参照。

## 7. 自動起動しているもの

- **crontab**: 無し(`no crontab for unitree`)。[観測] `cron.service` と `anacron`, 各種 apt/motd timer は enabled(OS 標準)。
- **systemd enabled(自作/ロボット固有に見えるもの)**: `unitree-upgrade.service`, `key_server.service`(`/unitree/sbin/key_server`)、`nginx`(TCP 80)、`docker`, `tailscaled`, `openvpn`, `nxserver`, `nvargus-daemon`, `nvfancontrol` ほか。[観測]
- `/unitree/module/master_service` + `video_hub_pc4`: systemd ではなく Unitree 独自の `master_service` が管理して起動(systemd の一覧に videohub は無い)。[観測 + 推測(中)]
- **`camera_stream` / `camera_server` を起動するユニット・cron・rc.local は無い**(取得範囲内)。[観測]
- ホームの `*.pid` ファイル群(`record_*.pid` 等)は過去の手動録画の残骸。[観測]

## 8. ログ末尾から読める問題(`g1_info/log_tails.txt`)

| ログ | 内容 |
|---|---|
| `foxglove_bridge.log` | `unitree_api`, `unitree_hg`, `unitree_go`, `octomap_msgs` パッケージが pixi 環境に無く、多数のチャンネルを追加できない(WARN)。→ 独自メッセージ型の colcon ビルドが未導入 |
| `odom_to_tf.log`, `restamp_points.log`, `bridge_dry.log` | Ctrl-C 後に `RCLError: failed to shutdown: rcl_shutdown already called`(rclpy 二重 shutdown。無害な終了時例外) |
| `g1_mapping_rgb_20260923T162556.launch.log` | `FileNotFoundError: 'ros2'`(PATH に ros2 が無い状態で起動) |
| `g1_mapping_rgb_20260923T162624.launch.log` | `VIDEOIO(V4L2:/dev/video0): select() timeout` → `camera opened but no RGB frame was produced` |
| `camera.log`(g1_ui_camera) | `errno=19 No such device`(SunplusIT が抜けた) |
| `camera_web.log`, `dual_camera_*.log` | KeyboardInterrupt で手動停止(=常駐させていない運用) |
| `driver.log`, `cmd_vel_bridge.log` | 素振り(dry-run)モード。上限 vx=0.30 vy=0.20 vyaw=0.50、0.5 s 指令が来なければ停止 |
| `mapping_*.log`, `record_*.log`, `UiS_room_*` | 建図・録画。rosbag は 30 秒 150MB〜15 分 4.7GB、`/utlidar/imu_livox_mid360` 約 200 Hz |
| `bad_alloc` | **無し** |

## 9. 未確定事項(不明)

1. `videohub_pc4` を止めずに RGB を得る経路(a) `VideoClient.GetImageSample()`(社内 docs は「動作済み」)、(b) pyrealsense2(videohub と競合するか未確認)。
2. `ros2` CLI daemon の 7GB / bad_alloc の再現条件(ログ無し)。
3. `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` の値。
4. D435i の各 video ノードのフォーマット一覧、opencv のビルド情報。
5. Jetson 側の時刻/タイムゾーン。
6. 標準ウェブカメラが列挙されなかった理由(物理接続 or 電力 or 別ポート)。
7. `key_server`, `unitree-upgrade`, `nginx`(TCP 80)が何を提供しているか(中身未取得)。

関連: [g1-safety/](g1-safety/), [unitree-g1-developer/40_lidar_Instructions.md](unitree-g1-developer/40_lidar_Instructions.md), [unitree-g1-developer/41_depth_camera_instruction.md](unitree-g1-developer/41_depth_camera_instruction.md)
