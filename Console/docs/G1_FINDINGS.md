# G1 実機調査の記録（2026-09-30 取得）

実機（Jetson 経由・デバッグモード・電池切れで停止するまで）から取ったものと、そこから分かったこと。
**確認済み** = 実機の値・応答を見た。**未確認** = 推測・二次情報。
役割: この文書は**事実の記録**（値・応答）。解釈と仮説は `g1_analysis.md`。実機での確認手順は `REAL_ROBOT_CHECKLIST.md`。

## 1. 取得データの置き場

| 種類 | 場所 | git |
|---|---|---|
| 生ログ 452 秒（JSONL, 75 MB） | `docs/g1_logs/live_20260930_234841.jsonl` | 対象外 |
| スナップショット一式（SDK tgz, DDS サンプル, サービス応答, 環境, lidar 1 フレーム 442 KB 他） | `docs/g1_snapshot/` | 対象外（bash_history・ネットワーク情報を含む） |
| 間引きリプレイ用（20 秒に 1 件, 約 0.5 MB） | `tests/fixtures/g1_live_sample.jsonl` | 入れる |
| 再取得用プローブ | `tools/`（discover / audio_probe2 / lidar_probe / arm_precheck / arm_one_joint / check_log / snapshot.sh） | 入れる |
| ロガー | `tools/live_logger.py` | 入れる |
| トピック/サービス台帳 | `g1console/dds_catalog.py`（65 トピック + rt/api 39 + 25 サービス） | 入れる |

ログの行形式: `{"t":epoch,"topic":..,"v":{..}}`、先頭に `meta`、5 秒毎に `beat`。
間引き版は `t` を開始からの秒に直し `sample_info` を除いてある。

## 2. 状態（確認済み）

- モード: デバッグ（`CheckMode` の name が空）、`GetFsmId` は `(3102, None)`（3102 は FSM ID ではなく RPC の戻りコード「送信」、ID は取得できていない。§8）、`mode_machine`=5。**通常モード(ai)へユーザーは戻せない。**
- 自由度: **29 DoF**（手首ピッチ/ヨー・腰ロール/ピッチあり）。`motor_state` は 35 スロットで先頭 29 が有効。Dex3 のデータなし。
- `rt/lowcmd` は内部プロセスが約 200 Hz で出し続ける（脚・腰 kp300、肩/肘 kp80、手首 kp40、q_cmd≒q_state）。**外から lowcmd を送ると競合する。**
- 腕関節 15–28 の q_cmd と q_state の平均誤差は 0.001〜0.008 rad（保持しているだけ）。
- IMU: メイン rpy≈(0.31, 0.90, 0.09) rad、IMU 温度 79–80 ℃。secondary_imu は別姿勢（rpy≈(0.07,-0.39,-0.02)）で、取付位置が別。
- 音量 85。

## 3. バッテリー（452 秒の実測）

- SOC 6% → 0%、その後停止。電圧 `bmsvoltage[0]` 40949 → 40124（mV と仮定, 約 -1.8 mV/s）。電流 -2063…-2280（mA と仮定, 放電がマイナス）。
- `cell_vol`: 13 セル（3148–3166 mV）、残り 0 埋め。`temperature` は先頭 4 つ有効（33–39 ℃）。`cycle`=27、`bmsstate`=[8,8,262144,7,0] は不変で意味不明。
- 電圧は SOC 段が変わっても連続的に下がる。**SOC 5% 未満は数分で切れる**（約 6 分で 0% → 停止）。UI の警告閾値は 10%/5% が妥当。
- 単位（mV/mA/℃）は**未確認の仮定**。

## 4. mainboardstate

`value`=[41.0→40.1, 40.7→39.8, 1.6→1.0, 0,0,0] は電圧に連動して下がる → 先頭 2 つは電圧 [V]（約 40 V）と推定、3 つ目は 1.0–1.7（電流 A か別の電源系, **不明**）。`temperature[0]`=50 固定、`fan_state` 全 0、`state`=[32,0,…] 不変。**意味は未確認**。

## 5. モータ

- 温度: 脚 32–41 ℃、腰 33–39 ℃、腕 38–52 ℃（左肩ピッチ(15)が最高 51–52、左手首ヨー(21)が上昇 40→50 ℃）。腕が熱い。
- `tau_est`: 全関節 ±1 N·m 未満（静止保持）。
- `rt/lf/*` は同じ内容の低頻度コピーと**仮定**（値は一致したが未確認。§13）。

## 6. 音声（確認済み）

- 生音声トピックなし。`rt/audio_msg`（JSON）に認識結果、`rt/audio_msg/filter` と `rt/audio_msg` に再生イベント。
- 認識: `{"index","timestamp"(ms),"type","text","angle","speaker_id","emotion","confidence","language","is_final"}`。452 秒で 7 件、`confidence` は常に 0.5、`angle`=0、`is_final`=false、言語は zh/ja/en が混在（無音〜環境音を誤認識している）。**精度は当てにならない。**
- 再生: `{"play_state":1}` → `{"play_state":0}`（36 件 = 18 回）。**TTS の開始・終了は観測できる**（以前の「終了信号なし」は訂正）。
- Jetson の `arecord -l` は Tegra APE のみ（実マイクなし）。音声 API id 1002(ASR) は SDK にクライアントなし。

## 7. その他の String トピック（452 秒、値は不変）

- `rt/rtc/state` = `{"connection_state":"not_connected"}`（1 Hz）
- `rt/public_network_status` = `NetworkStatus.ON_WIFI_CONNECTED`
- `rt/gpt_state` = `{"state":true,"llm_name":"clound","chat_state":true}`（"clound" は原文ママ）
- `rt/arm/action/state` = `{"holding":false,"id":0,"name":""}`（約 10 Hz）
- `rt/slam_info` = 全ポーズ 0・`is_arrived`=false（SLAM 未起動）
- 無音: `multiplestate` `selftest` `battery_alarm` `rtc_status` `servicestateactivate`（電池 0% でも `battery_alarm` は出なかった）
- `rt/servicestate`（1 回）: 29 サービス。稼働(status=1): `ai_sport`(8.7.3.4), `auto_test_arm`, `auto_test_low`, `ota_box`。他は停止。protect=1: basic_service, robot_state, webrtc_bridge, webrtc_signal_server。

## 8. SDK / API 面

- LocoClient: GetFsmId, SetFsmId, Damp, Start, Sit, ZeroTorque, Squat2StandUp, Lie2StandUp, StandUp2Squat, HighStand, LowStand, Move/SetVelocity, StopMove, SetStandHeight, SetBalanceMode, SetSpeedMode, SetTaskId, WaveHand, ShakeHand, SwitchToUser/InternalCtrl（GetFsmMode なし）。
- G1ArmActionClient: ExecuteAction, GetActionList。MotionSwitcherClient: CheckMode, SelectMode, ReleaseMode（GetSilent なし）。AudioClient: TtsMaker, GetVolume, SetVolume, LedControl, PlayStream, PlayStop。
- 腕アクション 27 種: release_arm(99) turn_back_wave(1) blow_kiss(11–13) both_hands_up(15) clamp(17) high_five(18) hug(19) make_heart(20,21) refuse(22) right_hand_up(23) ultraman_ray(24) wave_under_head(25) wave_above_head(26) shake_hand(27) box wins(28–30) right_hand_on_heart(33) both_hands_up_deviate_right(34) forward_push(36)、時間指定: Throw_money 8.1s / Spin_discs 6.9s / Scratch_head 8.1s / Waist_Drum_Dance 9.5s。一部は mode_machine 5/6 が前提。**実行は未検証。**
- RPC 返り値: 3102 送信 / 3103 未登録 / 3104 タイムアウト / 3105 不一致 / 3106 データ / 3107 lease 無効。

## 9. LED

モード表示のインジケータ（二次情報: Weston Robot 診断ガイドの検索要約。Unitree 公式で未確認）:
青=通常, オレンジ=ダンピング, 緑=着座, 黄=デバッグ, 紫=ゼロトルク, 濃青=待機, 赤=エラー。
`LedControl` に読み戻しなし。再起動後に戻るかは**未確認**。実機では LED を変更した状態のまま（元は青と記憶）。

**2026-10-01 追記（目視）**: LED は**2 か所ある**（別々に光る）。このときの色は、外周が青、目の位置が緑。上の色の表がどちらの LED を指すかは不明で、`LedControl` がどちらを変えるかも**未確認**（C2 で確かめる）。LED からモードは判定しない。

## 10. arm_sdk（重要な否定的結果）

デバッグモードで `rt/arm_sdk`（公式 arm7 例と同じ経路: motor_cmd[29].q=weight, CRC）に左手首ヨー(21)へ ±0.05 rad を送っても、観測変化は **0.0000 rad**。デバッグモードでは効かない（通常モード `ai` 用と推定）。`tools/arm_one_joint.py` は通常モードで再試験するためのもの（既定ドライラン）。ログ中の `rt/arm_sdk` 46 件はこの試験の送信。

## 11. lidar

`rt/utlidar/cloud_livox_mid360`（PointCloud2）は frame_id `livox_frame`、約 20.1k 点、`point_step`=22（x,y,z,intensity float32 @0/4/8/12、ring uint16 @16、time float32 @18）、442 KB/フレーム、約 10 Hz（≈4.4 MB/s）。受信に独自 IDL が必要（`tools/lidar_probe.py` に定義）。Mac へ流すと帯域を食うので**コンソールでは購読しない**（台帳で EXCLUDED）。IMU は `rt/utlidar/imu_livox_mid360`。

## 12. 性能上の知見

- コールバック購読で約 2200 msg/s を受けると RPC が飢える → DataReader + KeepLast(1) + TimeBasedFilter、`take(N=1)` にする。ロガーの実測は各トピック約 2 Hz（period 0.5 s）。
- 通信は 操作用 PC → ssh(tailscale) → Jetson → DDS。鮮度閾値 FRESH 2.5 s / STALE 8 s / GIVEUP 15 s。

## 13. 未確認リスト（`REAL_ROBOT_CHECKLIST.md` の項番に対応）

| 未確認の内容 | 項番 |
|---|---|
| 通常モードでの `SetFsmId`・矢印移動 | B1〜B5、E1〜E3 |
| `arm_sdk` の実行（通常モード） | D2、G4 |
| 腕アクションの実行 | D3、D4 |
| LED の永続性と既定色 | C2、C3 |
| 関節名の割当（`tools/arm_precheck.py` の並びは SDK 例に従った仮定） | A6 |
| バッテリーとメインボードの単位・意味 | A2、A7 |
| `lowcmd` の出所 | A8 |
| `rt/lf/*` が低頻度コピーである点 | A9 |
| `audio_msg/filter` の役割 | C4 |
| D435i の RGB（OpenCV で開けない） | H1〜H4（§15 の方針で取る） |

## 14. 使い方（開発）

- 実機なしで UI を動かす: `server.py` の MockHelper（シナリオ ok/g1_off/jetson_off/debug）。実データに近い値は `tests/fixtures/g1_live_sample.jsonl`（`t` 秒・topic・v）から読む。
- 再取得: `python3 Console/tools/live_logger.py --host g1-ts`、要約は `tools/check_log.py`。

## 15. 他班の実機調査から分かったこと（PR #34、2026-10-01 時点でレビュー中）

PR #34（`Dev/Entame` 向け、マージされない前提で番号のみ参照）に、他班が実機で調べた記録がある。ここでは**このコンソールに関わるものだけ**を、出典の文書名つきで記録する。
いずれも**他班の記録**であり、このコンソールの実機では未確認。パスは PR #34 の `Entame/real/integrated_demo/g1-bottle-reaction/docs/` 以下。

### カメラ

- **内蔵 RGB は公式 `VideoClient.GetImageSample()` で取れる。** 有線で 1920x1080・300 秒・9,762 フレーム・平均 32.5 FPS・クラッシュなし（`G1_CAMERA_WIRED.md`）。`videohub_pc4` → `VideoClient` の経路で、OpenCV で `/dev/video4` を開く必要はない。→ 「RGB が OpenCV で開けない」問題は、この経路で回避できる。
- `GetImageSample()` に撮影時刻・連番はない。FPS を撮影→表示の遅延と読まない。取得失敗は一時的に 2 回出て、次の取得で復帰した。
- **`videohub_pc4` は止めない・kill しない**（`AGENTS.md`）。停止しても watchdog で再起動したという利用者報告がある（xr_teleoperate issue #299、`G1_GAME_VISION.md`）。
- 公式 `VideoClient` は **Depth を返さない**（RGB の画像バイナリのみ）。Depth が要るなら PC2 で D435i を直接開く経路だが、videohub と競合する可能性があり、PR #34 でも未実行（`G1_CAMERA_WIRED.md`）。
- 追加 USB カメラ（SunplusIT Full HD webcam, USB ID `1bcf:2283`）は G1 に挿して使う。**追加 USB を挿したまま再起動すると video 番号が変わり、内蔵配信が起動失敗した。外して起動し、映像が戻ってから挿す**（`G1_USB_DUAL.md`）。G1 側に ffmpeg / v4l2-ctl は無く、GStreamer 1.16.3（v4l2src）は使える。
- 公式 `ChannelFactoryInitialize` の設定トレースが有効だと SIGABRT になる事例がある（`snprintf` のバッファオーバーフロー）。SDK の XML から Tracing を外して回避している（`G1_CAMERA_WIRED.md`）。

### ROS 2 / DDS

- **`ros2 topic list` で死活確認をしない。** foxy では FastDDS 2.0 も `bad_alloc` で落ち、cyclonedds 0.7.0 では SIGSEGV（相手の自己紹介を受けた瞬間）。0.10 世代の cyclonedds（`unitree_sdk2py`）なら同じ機体で 9.98 Hz 受信できた。死活確認は DDS を直接読む `probe_dds_topics.py`（`Mapping/real/quickstart/README.md`）。
- 読み取り専用の DDS 設定例: `config/g1-readonly-dds.xml`（`CYCLONEDDS_URI` で指定）。

### 腕

- 腕の制御は `rt/arm_sdk`（`LowCmd_`）に公式 arm7 例と同じ流れで送り、**完了時に weight を 0 に戻す**。実装の既定は小さな相対オフセット（肩 pitch -0.04、肩 roll -0.03、肘 +0.05 rad）（`CUSTOM_G1_MOTION.md`）。
- プリセット動作（`G1ArmActionClient.ExecuteAction`）と `arm_sdk` は排他にする（同一プロセスのロック）。**プリセットの終了を `rt/arm/action/state` で確認する機能は PR #34 でも未実装。** 終了が分からないときは `arm_sdk` を保守的に使わない。
- `ExecuteAction` が **3104（RPC タイムアウト）** を返しても、動作が始まっている可能性がある。失敗と断定せず、自動で再試行しない（二重動作の防止）。
- 使っているアクション ID: 23（right hand up）、26（high wave）、解除は 99。起動時に `GetActionList()` で ID の存在を確認してから実行する（`G1_INTEGRATION.md`）。

### モード・FSM

- `sportmodestate`（`SportModeState_`）は `fsm_id` / `fsm_mode` / `task_id` / `task_time` の 4 項目（`G1_NAVIGATION_READONLY_FOLLOWUP_2026-09-12.md`）。FSM ID の API は 7001（GetFsmId）・7002（GetFsmMode）・7101（SetFsmId）。**PR #34 でも `GetFsmId` / `SetFsmId` は実行していない**（制御として未実行）。
- デバッグモードへの入り方、`L2 + A`、`L2 + B` / `L1 + A` の版判定、通常モードへの戻り方は、PR #34 にも記載がない。→ 実機が必要（`REAL_ROBOT_CHECKLIST.md` の A11、G1〜G2、G7）。

### 運用

- 「2 人以上」: PR #34 の運用ルール（`G1_STATIC_LOCALIZATION_2026-09-12.md`: 少なくとも 2 人で低速の手動移動）。Unitree 公式の記載ではない。

