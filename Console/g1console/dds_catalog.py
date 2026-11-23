"""G1 上で観測した DDS トピック / RPC サービスの一覧と、このコンソールでの扱い。

トピックは 2026-09 に Jetson から発見（BuiltinTopicDcpsPublication）した 104 件のうち、
サービスの request/response を除いた分。型は発見時の型名。
状態:
  displayed … remote_helper.py が購読し、画面に表示している
  todo      … 未購読（取得できそうだが未実装）
  excluded  … 対象外（大容量の点群・映像・地図、または指令用で送信しないもの）
"""

DISPLAYED, TODO, EXCLUDED = "displayed", "todo", "excluded"

# (トピック, 型, 状態, 説明)
TOPICS = (
    # --- 表示済み ---
    ("rt/lf/bmsstate", "BmsState_", DISPLAYED, "バッテリー（SOC・電圧・電流・温度）"),
    ("rt/lowstate", "LowState_", DISPLAYED, "IMU と 29 関節の状態（角度・速度・トルク・温度）"),
    ("rt/odommodestate", "SportModeState_", DISPLAYED, "オドメトリ（位置・速度・胴体高さ・mode）"),
    ("rt/lowcmd", "LowCmd_", DISPLAYED, "関節への指令（デバッグモードでも約 200 Hz で流れている。発行元は未特定）"),
    ("rt/secondary_imu", "IMUState_", DISPLAYED, "2 つ目の IMU"),
    ("rt/lf/mainboardstate", "MainBoardState_", DISPLAYED, "メインボード状態（value 配列の意味は不明）"),
    ("rt/lf/emergency_stop", "Error_", DISPLAYED, "非常停止イベント（イベント型。平常時は受信なし）"),
    ("rt/wirelesscontroller", "WirelessController_", DISPLAYED, "無線リモコン入力（操作時のみ）"),
    ("rt/rtc/state", "String_", DISPLAYED, "RTC 状態（JSON 文字列）"),
    ("rt/public_network_status", "String_", DISPLAYED, "通信状態（JSON 文字列）"),
    ("rt/arm/action/state", "String_", DISPLAYED, "腕アクションの状態"),
    ("rt/gpt_state", "String_", DISPLAYED, "音声対話（GPT）状態"),
    ("rt/servicestate", "String_", DISPLAYED, "サービス状態（測定時は受信なし）"),
    ("rt/servicestateactivate", "String_", DISPLAYED, "サービスの有効状態"),
    ("rt/lf/battery_alarm", "String_", DISPLAYED, "バッテリー警報（イベント型。測定時は受信なし）"),
    ("rt/rtc_status", "String_", DISPLAYED, "RTC ステータス（測定時は受信なし）"),
    ("rt/multiplestate", "String_", DISPLAYED, "複合状態（測定時は受信なし）"),
    ("rt/selftest", "String_", DISPLAYED, "セルフテスト（測定時は受信なし）"),
    # --- 未購読（取得できそう。未実装） ---
    ("rt/audio_msg", "String_", TODO, "音声認識（ASR）結果か音声イベントのテキストと推測。マイク入力の手がかり。中身は未確認"),
    ("rt/audio_msg/filter", "String_", TODO, "上記のフィルタ後と推測。未確認"),
    ("rt/dex3/left/state", "HandState_", TODO, "Dex3 ハンド（左）の状態。ハンド未接続なら受信なし"),
    ("rt/dex3/right/state", "HandState_", TODO, "Dex3 ハンド（右）の状態"),
    ("rt/lf/dex3/left/state", "HandState_", TODO, "同（lf 系。低頻度版と推測）"),
    ("rt/lf/dex3/right/state", "HandState_", TODO, "同（lf 系）"),
    ("rt/lf/lowstate", "LowState_", TODO, "lowstate の lf 系（低頻度版と推測）。表示は rt/lowstate で足りている"),
    ("rt/lf/odommodestate", "SportModeState_", TODO, "odommodestate の lf 系"),
    ("rt/lf/secondary_imu", "IMUState_", TODO, "secondary_imu の lf 系"),
    ("rt/config_change_status", "ConfigChangeStatus_", TODO, "設定変更の通知"),
    ("rt/slam_info", "String_", TODO, "SLAM の状態"),
    ("rt/slam_key_info", "String_", TODO, "SLAM のキー情報"),
    ("rt/unitree_slam/waypoints", "String_", TODO, "SLAM のウェイポイント"),
    ("rt/event/action_store", "String_", TODO, "アクション保存のイベント"),
    ("rt/gpt_cmd", "String_", TODO, "音声対話への指令"),
    ("rt/gptflowfeedback", "String_", TODO, "音声対話のフィードバック"),
    ("rt/log_system_inbound", "String_", TODO, "ログ系（入力側）"),
    ("rt/log_system_outbound", "String_", TODO, "ログ系（出力側）"),
    ("rt/videohub/inner", "String_", TODO, "映像ハブの内部通知（映像本体ではない）"),
    ("rt/webrtcreq", "String_", TODO, "WebRTC 要求（アプリ連携）"),
    ("rt/webrtcres", "String_", TODO, "WebRTC 応答"),
    ("rt/xfk_webrtcreq", "String_", TODO, "WebRTC 要求（xfk）"),
    ("rt/xfk_webrtcres", "String_", TODO, "WebRTC 応答（xfk）"),
    ("rt/SymState", "SymState_", TODO, "用途不明"),
    ("rt/dog_imu_raw", "Imu_", TODO, "IMU 生データ（Go 系由来の名前）。用途不明"),
    ("rt/dog_odom", "Odometry_", TODO, "オドメトリ（nav_msgs）。用途不明"),
    ("rt/utlidar/imu_livox_mid360", "Imu_", TODO, "lidar 内蔵 IMU（点群ではない。高頻度と推測）"),
    ("rt/unitree/slam_mapping/odom", "Odometry_", TODO, "SLAM マッピング中のオドメトリ"),
    ("rt/unitree/slam_relocation/odom", "Odometry_", TODO, "SLAM 再測位中のオドメトリ"),
    # --- 対象外 ---
    ("rt/arm_sdk", "LowCmd_", EXCLUDED, "腕への指令用。送信しない（購読して見ることはできる）"),
    ("rt/frontvideostream", "Go2FrontVideoData_", EXCLUDED, "カメラ映像（不要と指示）"),
    ("rt/utlidar/cloud_livox_mid360", "PointCloud2_", EXCLUDED, "lidar 点群（不要と指示）"),
    ("rt/collision_clouds", "PointCloud2_", EXCLUDED, "点群（衝突回避）"),
    ("rt/ele_clouds", "PointCloud2_", EXCLUDED, "点群"),
    ("rt/grid_clouds", "PointCloud2_", EXCLUDED, "点群"),
    ("rt/no_warning_clouds", "PointCloud2_", EXCLUDED, "点群"),
    ("rt/pre_collision_clouds", "PointCloud2_", EXCLUDED, "点群"),
    ("rt/pre_safe_clouds", "PointCloud2_", EXCLUDED, "点群"),
    ("rt/safe_clouds", "PointCloud2_", EXCLUDED, "点群"),
    ("rt/warning_clouds", "PointCloud2_", EXCLUDED, "点群"),
    ("rt/global_map", "OccupancyGrid_", EXCLUDED, "地図（大容量）"),
    ("rt/gridmap", "GridMap_", EXCLUDED, "地図（大容量）"),
    ("rt/planner_map", "GridMap_", EXCLUDED, "地図（大容量）"),
    ("rt/unitree/slam_mapping/points", "PointCloud2_", EXCLUDED, "SLAM 点群"),
    ("rt/unitree/slam_relocation/points", "PointCloud2_", EXCLUDED, "SLAM 点群"),
    ("rt/unitree/slam_relocation/global_map", "PointCloud2_", EXCLUDED, "SLAM 地図"),
    ("rt/unitree/slam_relocation/web_points", "PointCloud2_", EXCLUDED, "SLAM 点群"),
)

# RPC サービス（rt/api/<名前>/request|response）。使用中 / SDK にあるが未実装 / SDK にクライアントなし
SERVICES = (
    ("voice", "使用中", "音量・LED・TTS（AudioClient）。PlayStream / PlayStop は未実装"),
    ("motion_switcher", "使用中", "CheckMode（表示）。SelectMode / ReleaseMode / SetSilent は未実装"),
    ("sport", "使用中", "G1 の LocoClient（GetFsmId / SetFsmId）。移動・動作系は未実装"),
    ("arm", "SDK あり・未実装", "ExecuteAction / GetActionList（腕の動作）"),
    ("audiohub", "SDK なし", "音声ハブ（マイク・再生の管理と推測）"),
    ("gpt", "SDK なし", "音声対話"),
    ("bashrunner", "SDK なし", "リモートコマンド実行と推測。扱いは慎重に"),
    ("config", "SDK なし", "設定"),
    ("robot_state", "SDK なし", "ロボット状態"),
    ("robot_type_service", "SDK なし", "機種情報"),
    ("slam_operate", "SDK なし", "SLAM 操作"),
    ("videohub", "SDK なし", "映像ハブ"),
    ("action_store", "SDK なし", "アクション保存"),
    ("dex3_msg_controller", "SDK なし", "Dex3 ハンド制御"),
    ("gesture", "SDK なし", "ジェスチャー"),
    ("basic_demarcate", "SDK なし", "キャリブレーション系。触らない"),
    ("basic_demarcate_lease", "SDK なし", "キャリブレーション系（lease）。触らない"),
    ("basic_softlimit", "SDK なし", "関節ソフトリミット設定と推測。触らない"),
    ("basic_softlimit_lease", "SDK なし", "同（lease）。触らない"),
    ("basic_taumax", "SDK なし", "最大トルク設定と推測。触らない"),
    ("basic_taumax_lease", "SDK なし", "同（lease）。触らない"),
    ("basic_clearoip", "SDK なし", "用途不明（設定系）。触らない"),
    ("basic_clearoip_lease", "SDK なし", "同（lease）。触らない"),
    ("rm_con", "SDK なし", "用途不明"),
    ("vui", "SDK なし", "音声 UI"),
)
