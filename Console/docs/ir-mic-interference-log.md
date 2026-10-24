# G1 IR ワイヤレスマイク干渉 調査ログ

目的: G1 起動中に IR 式ワイヤレスマイクの接続が不安定になる原因を切り分ける。
方針: 安全（読み取りのみ）な作業から順に進める。状態を変える操作は実行前に確認する。

## 要約（2026-10-24 時点）
- 結果: LiDAR 本体を IDLE にするとワイヤレスマイクが正常に使えた（n=1）。
- 手段: Console の LiDAR タブのボタン（本体の停止・起動）。電源を入れ直すと SAMPLING に戻る。
- 未確認: A/B 3 回の再現性 / レーザーの物理停止 / 停止中の G1 の動き / D435i の影響。
- 次の一手: A/B 3 回。IDLE ×3 すべて正常 かつ SAMPLING ×3 すべて不安定 → 原因確定。1 回でも逆なら仮説を棄却し D435i を調べる。

## 前提（2026-10-24）
- 事象: G1 電源 ON 中、IR ワイヤレスマイクの接続が実際に不安定になった（ユーザー確認）。
- 仮説（未検証）: MID360(905nm) または D435i(約850nm) の IR が受信機に干渉。
  - 出典: MID360 公式スペック（905nm）。D435 の 850nm はコミュニティ回答のみ。
- Jetson(PC2) へは SSH 接続可。
- 指定アドレス: http://192.168.0.82 （役割は未確認）

## 作業記録
- [x] 1. 192.168.0.82 の疎通・HTTP 応答を確認（読み取りのみ）
  - ping: 3/3 応答（5〜13ms）。
  - HTTP 200、Tornado サーバー、ページタイトル `unitree-upgrade`（Unitree のアップグレード用 Web UI と思われる。ここから LiDAR は制御できない見込み）。
  - TCP 22 open → SSH 可。この IP が Jetson(PC2) かどうかは未確認（SSH で hostname を見て確定する）。
- [x] 2. SSH でログインし、hostname / 実行中ノード / Livox ドライバの有無を確認（読み取りのみ）
  - ユーザー `unitree`、パスワード認証（Unitree 既定値とユーザー申告。ここには記録しない）で接続成功。
  - hostname `ubuntu`、aarch64、uptime 5分（起動直後）。
  - `ps` に livox / lidar / realsense を含むプロセスなし → **LiDAR ドライバはこの時点で未起動**。
  - `ros2 node list` は 5 秒でタイムアウト（ros2 未導入か DDS 応答なし。未切り分け）。
  - 注意: LiDAR ドライバが未起動でもレーザーが照射されているかは別問題（未確認）。
- [x] 3. OS / ネットワーク / docker / LiDAR(192.168.123.120) 疎通を確認（読み取りのみ）
  - Ubuntu 20.04.6 / JetPack R35.3.1 / ROS: foxy, noetic が /opt/ros にある。
  - `192.168.0.82` は **Jetson の wlan0**。eth0 は 192.168.123.164（ロボット内部網）。tailscale0 もあり。
  - **LiDAR 192.168.123.120 が ping 応答（0.9〜4ms）** → LiDAR は通電・動作中。ドライバ未起動でもレーザー照射の可能性は残る（未確認）。
  - docker は権限不足（unitree ユーザーが docker グループ外）。変更はしない。
  - 動作中サービス: docker, unitree-upgrade のみ。ホームに過去の LiDAR 関連作業（g1_console_lidar, driver.log 等）あり。
- [x] 4. LiDAR のデータが流れているか（eth0 受信量）、過去ドライバ構成を確認（読み取りのみ）
  - eth0 受信 約 4.08 MB/s（3秒平均）。LiDAR 単独の量とは断定できないが、ロボット網にデータが流れている。
  - `~/g1_console_lidar/lidar_stream.py` は DDS トピック `rt/utlidar/cloud_livox_mid360` を**購読するだけ**。LiDAR 本体は制御しない。`lidar_stream.pid` なし・`ps` にも無く、現在は停止中（`lidar_ctl.sh status` 未実行、ps で確認）。
  - → **点群を出しているのは G1 本体側（Unitree 標準サービス）。Jetson のユーザープロセスを止めても LiDAR のレーザーは止まらない**見込み。
  - Jetson に USB 接続の Intel デバイス `8086:0b3a`（RealSense 系と思われる。型番は未確認）あり。
  - 停止手段は未確定。候補（未検証・状態を変える操作なので実行前に要確認）:
    (a) Unitree の DDS で LiDAR スイッチ用トピックに OFF を送る（G1 で有効かは未確認）
    (b) Unitree アプリ/サポートに公式の停止方法を確認
    (c) 受信機の移動・遮光 → 不採用（根本解決にならないため）
- [x] 5. 停止手段 (a) の事前調査（読み取りのみ）
  - 候補トピック: `rt/utlidar/switch`、型 `std_msgs::msg::dds_::String_`、`data="ON"/"OFF"`。
    根拠は公式 SDK の **Go2 用** 例 `~/unitree_sdk2_python/example/go2/high_level/go2_utlidar_switch.py`。G1 での対応は未確認。
  - Go2(L1) の外部フォーラム報告: OFF で点群とスピンは止まったが消費電力は変わらず、待機になったかは不明（回答なし）。
  - **既存の自班メモ `~/g1-bottle-reaction-wander/docs/G1_NAVIGATION_READONLY_FOLLOWUP_2026-09-12.md`**: 2026-09-12 の live discovery で `rt/utlidar/switch` は endpoint/reader/writer とも 0 件。
    → G1 側にこのトピックを購読するものがなく、**OFF を送っても効かない可能性が高い**。当時 ON/OFF とも送っていない。
  - 同メモの注意: 例スクリプトの main は OFF を送るので、そのまま実行しない。
  - 副作用の懸念: LiDAR が止まると点群依存のマッピング・ナビが止まる（歩行への影響は未確認）。ON で戻せるかも未確認。
  - 参考: `~/mapping_runs` に 2026-10-21 の `check_1021_ir_*` 実験記録あり（IR 関連の先行調査の可能性。内容は未読）。
- [x] 6'. **OFF 試験の結果（完了）**: 基準 4,072,847 B/s → OFF 送信後 4,065,373 B/s（比 1.00）。**変化なし＝`rt/utlidar/switch` は G1 では効かない。** ON は送っていない（LiDAR 状態は変更されていない見込み）。
  - 先の SSH タイムアウトは試験の待機時間中の一時的なものと思われる（原因は未確認）。
  - 結論: 手段 (a) は不採用。次は (c) 受信機側の切り分け、または (b) Unitree への問い合わせ。
- [x] 6''. LiDAR 停止確認（ユーザー依頼）: OFF 試験後も eth0 受信 約 4.07 MB/s ×3回、LiDAR ping 応答。**止まっていない**。ユーザー報告: ワイヤレスマイクは引き続き使えない。
- [x] 6'''. 別経路の調査（読み取りのみ）: Jetson に **Livox SDK2** あり（`/usr/local/include/livox_lidar_api.h`, `liblivox_lidar_sdk_shared.so`、g++/cmake あり）。
  - `SetLivoxLidarWorkMode(handle, mode, cb, data)` と `kLivoxLidarSleep = 0x03` がある → LiDAR 本体（192.168.123.120）に直接スリープ指令を送れる可能性。
  - 懸念（未検証）: (1) G1 本体側(PC1)のドライバが再度 Normal に戻す、(2) SDK2 は host_ip/設定 JSON が必要で、G1 本体の通信とポート衝突しうる、(3) Sleep が実際にレーザーを止めるか、(4) 点群依存のナビ・マッピングが止まる、(5) WakeUp で戻るか。
- [x] 7'. **Livox 制御プロトコル直送試験（ユーザー了承「go」）→ 中止**: CRC 方式 36 通りの「読み取りクエリ」(work_mode 取得) に LiDAR から応答なし。
  **スリープ指令は送っていない（LiDAR には何も設定していない）。** G1/LiDAR の状態は試験前と同じ。
  - 理由は未特定: パケット形式の記憶違い、LiDAR が登録済みホスト以外の送信元を無視する、等（どれも未検証）。
  - SDK2 を使わず直送にした理由: SDK2 が点群の送信先を Jetson に書き換える恐れがあり、G1 本体側の動作を壊しかねないため（未検証の懸念）。
- [x] 8. SDK2 の安全性調査（読み取りのみ）:
  - 点群・IMU・状態の送信先を変える API は `SetLivoxLidarPointDataHostIPCfg` / `...ImuDataHostIPCfg` / `...StateInfoHostIPCfg` と**別個に存在**。work mode 変更（`SetLivoxLidarWorkMode`）だけを呼ぶ限り、これらは呼ばれないはず（ライブラリ内部の自動設定の有無は未確認）。
  - 読み取り専用 API あり: `QueryLivoxLidarInternalInfo` / `QueryLivoxLidarFirmwareVer` など。
  - Unitree の MID360 設定例（`/upgradePythonServer/temp/unitree/module/graph_pid_ws/src/lio_sam_ros2/mid360_config.json`）: lidar ポート 56100/56200/56300/56400/56500、host_ip `192.168.123.18`、multicast `224.1.1.5`、host ポート 56101/56201/56301/56401/56501。
    → G1 の LiDAR は host `.18`（または multicast）宛に送っている可能性。**送信元が違う Jetson(.164) からの直送クエリに無応答だった理由の手がかり**（未検証）。
  - 並行して、ユーザーが受信機側の切り分け（位置変更・遮光・距離）を実施中。結果待ち。
- [x] 9. **ゴール明確化（ユーザー厳命）**: 「G1 を動かしている横でワイヤレスマイクを使えること」が絶対条件。受信機の位置変更・遮光・距離取りは**やらない**（根本解決にならないため）。
- [x] 10. SDK2 読み取りクエリ試験（設定変更 API は不使用、Jetson の一時ディレクトリでビルド→実行→削除）:
  - SDK 初期化は成功（host_ip=192.168.123.164）。12秒待っても **LiDAR を検出できず**（検出コールバック未発火）→ クエリ未送信。
  - 直送プロトコル試験と合わせ、**Jetson(.164) からは LiDAR の制御チャネルに到達できていない**。
  - 推定（未検証）: MID360 は登録済みホスト(Unitree 例では 192.168.123.18 / multicast 224.1.1.5)にしか情報を push せず、SDK2 の検出がそれに依存する。
  - 副次の観測: 実行前の eth0 受信が **8.6 MB/s**（以前は 4.07 MB/s）に増えていた。試験前に測定したので本試験の影響ではない。原因未確認（G1 側の動作変化か別の購読者か）。
  - G1/LiDAR の設定は変更していない。
- [x] 11. 別名 IP 試験（ユーザー了承「go」）: `192.168.123.18` は ping 無応答（arping なしのため ping のみで未使用と判断）→ eth0 に一時追加 → SDK2 読み取りクエリ → 削除。
  - **やはり LiDAR を検出できず**（15秒）。別名 IP は削除済み（残存 0 件）。eth0 受信は前後とも約 8.6 MB/s で変化なし。
  - G1/LiDAR の設定は変更していない。sudo は使えることを確認（Jetson の `unitree`）。
  - 次: tcpdump で LiDAR 発のパケットの宛先（unicast/multicast）と、LiDAR 宛の制御通信の有無を読み取りだけで観察する。
- [x] 12. **CRC 確定と LiDAR との通信成立**: LiDAR が約1秒ごとに 255.255.255.255:56000 へ流す 90 バイトの UDP を Jetson で採取（読み取りのみ。tcpdump なしのため python の AF_PACKET を sudo で実行、一時ファイルは削除）。
  - 実パケット2個で照合: **CRC16 = CCITT-FALSE（init 0xFFFF, poly 0x1021, MSB first, xorout 0）、CRC32 = 標準 zlib 方式**。ヘッダ 24 バイトの配置は想定どおり。
  - 点群は LiDAR から Jetson へは来ていない（Jetson 宛の LiDAR 発パケットは 56000 の告知のみ）。→ LiDAR は G1 本体(PC1)へ直接送り、PC1 が DDS に流していると見られる。
  - 正しい CRC で読み取りクエリ成功（work_mode=1）。
- [x] 13. **ret_code=3 の原因**: Livox 公式プロトコル資料の戻りコード表で `0x03 = 設定値が範囲外`。送った値 `3`（SDK2 列挙の Sleep）は MID360 では設定不可。
  - 設定可能な値（キー 0x001A）: `0x01` SAMPLING / `0x02` IDLE / `0x09` READY。
  - 出典: https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md
  - 副次: 復帰処理で送った `2`（意図は WakeUp）は IDLE で **ret_code=0 受理**。直後に `1` を送り SAMPLING に復帰済み。→ **Jetson から LiDAR を IDLE にする指令は通る**。
  - 復帰後の eth0 受信が 8.6 → 4.06 MB/s に変化（以前の 4.07 MB/s に戻った）。因果は未確認（増減の理由は不明）。
- [x] 14. IDLE(0x02) を約90秒保持 → SAMPLING(0x01) で復帰（完了）。
  - IDLE 指令 ret_code=0（受理）。読み戻しで **work_mode=2 を 90 秒間維持**（12/27/42/57/72/87 秒時点すべて 2）。PC1 による巻き戻しは起きなかった。
  - 復帰指令 ret_code=0、復帰後 work_mode=1。
  - eth0 受信: 開始時 8.62 MB/s → IDLE 中 4.07 MB/s → 復帰後 4.07 MB/s。**IDLE 前後で受信量が変わらない時期もあり（セッション冒頭は通常動作で 4.07 MB/s）、eth0 受信量は LiDAR 停止の指標にならない**。物理的にレーザーが止まったかは未確認（回転音・スマホカメラでの確認が必要）。
  - **マイクの結果（ユーザー報告）: IDLE 中（LiDAR 停止中）にワイヤレスマイクが使えた。** 1回の観測のみ。IDLE/SAMPLING の交互繰り返し（A/B）は未実施 → 再現性は未確認。
- [x] 15. **コンソール(http://192.168.0.82:18790/#sensors)の LiDAR タブに「LiDAR 本体を停止／起動」ボタンを追加**（2026-10-24）。
  - 注意: 既存の「起動／停止」は点群配信プロセス(lidar_stream.py)で、LiDAR 本体ではない。新ボタンは別行「LiDAR 本体: …」。
  - 実装(Jetson の `~/G1_Hackason/Console`): `g1console/lidar_sensor.py` 新規、`server.py`(GET /api/lidar/sensor、POST /api/lidar/sensor/idle|sampling)、`api_spec.py`、`modes.py`(機能 lidar_sensor・未確認)、`web/index.html`・`ui-core.js`・`mobile.js`、テスト `tests/test_lidar_sensor.py`。許可リストは idle(2)/sampling(1) のみ（Sleep=3 は範囲外で通らない）。
  - 検証: 新規テスト 8 件 OK。既存テストは元から 33 件失敗（Jetson 作業ツリーが途中状態）で、変更前後で同一（新たな失敗 0）。
  - 反映: Jetson へファイル配置済み・バックアップ `Console/.bak-lidar-sensor-20261024`。ハブ再起動済み（ユーザー了承）。`GET /api/lidar/sensor` が実機で `mode=1` を返すことを確認。
  - 追加: LiDAR 本体の状態バッジ（SAMPLING=緑 / IDLE=橙 / READY=灰 / 確認できません）、確認時刻、LiDAR タブ表示中は 5 秒ごとの自動読み直し。
  - **ユーザー確認: 画面のボタンで止めてもマイクが使えた（手順 14 と同じ結果）。** LiDAR 停止で改善（n=1）。原因の確定は A/B 3 回の結果待ち。レーザー停止の物理確認は未実施。
  - 反映先: PR（base Dev/UI）。Dev/UI の画面は index.html にスクリプトを内蔵した版のため、そちらへ移植した。
  - 方法: Jetson(wlan0 経由 SSH)から LiDAR(192.168.123.120:56100/UDP) へ Livox 制御プロトコル cmd 0x0100 で key 0x001A(work_tgt_mode)=2(IDLE) を送信。復帰は =1(SAMPLING)。
- [~] 6. （廃止: 手順 9 以降の IDLE 方式で置換）（旧記載）OFF 試験を実行（ユーザー了承済み「実行」）。自前の最小スクリプトで `rt/utlidar/switch` に `"OFF"` を1回送り、eth0 受信量で判定、低下したら `"ON"` で復帰を試す設計。
  - 実行結果: 90秒を超えてもバックグラウンドで継続中（出力は grep のバッファで未表示）。**OFF が送られたか・LiDAR が止まったかは未確認。**
  - 同時刻に別 SSH が `Operation timed out`。ping は通る（5〜12ms）。Jetson の負荷か Wi-Fi 不調の可能性（未切り分け）。
  - 注意: 結果が出るまで追加の操作はしない。
- [~] 7. （廃止: 手順 9 以降の IDLE 方式で置換）（元の計画）OFF を 1 回送る試験。実行前にユーザーの了承を得る。成否は `rt/utlidar/cloud_livox_mid360` の受信周波数と eth0 受信量で判定。
