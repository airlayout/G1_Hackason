# g1_ws — ROS 2 colcon workspace(D-04のROS側プロセス)

`g1_sdk_bridge_cpp/`(D-08、ROS非依存の独立CMakeプロジェクト)を薄くラップして、
実際のrclcppノードにしたもの。

## パッケージ

| パッケージ | 役割 | 対応する仕様書/Planning.md項目 |
|---|---|---|
| `g1_cmd_router` | Nav2からの速度指令を安全処理し、SDK側プロセスへIPC送信 | 仕様書7章・8章、D-04/D-10/D-14/D-27 |
| `g1_state_bridge` | SDK側プロセスのstate(IPC)を`/odom`+診断情報に変換 | 仕様書5.1 |

いずれも `../g1_sdk_bridge_cpp/src/{protocol,ipc_transport,safety_manager}.cpp` を
相対パスで直接コンパイルして使う（`g1_sdk_bridge_cpp`自体はament化しない。D-08の
「ROS 2非依存の独立CMakeプロジェクト」という前提を壊さないため）。

## ビルド方法

```bash
source /opt/ros/jazzy/setup.bash
cd g1_ws
colcon build --symlink-install
source install/setup.bash
```

### このホスト固有の注意点(条件によっては不要)

このAWS環境ではminicondaが`PATH`に`/opt/ros/jazzy`より優先して入っており、
`ament_cmake`が内部で呼ぶ`python3`がconda環境のものに解決されて
`ModuleNotFoundError: No module named 'catkin_pkg'`で**ビルドに失敗する**現象が起きた。
G1接続PCの環境がminiconda等でPATHが汚れていなければ発生しないはずだが、
同様の症状が出た場合は以下のようにconda環境変数を外してビルドする:

```bash
env -u CONDA_DEFAULT_ENV -u CONDA_PREFIX -u CONDA_PYTHON_EXE -u CONDA_SHLVL \
    PATH="/usr/bin:/bin:$PATH" colcon build --symlink-install
```

`ros2 run` 等の実行時にも同様の対処が必要な場合がある。

## 動作確認(2026-09-09、実機なし・モックSDK backend)

`g1_sdk_bridge_cpp`の`g1_sdk_bridge_mock_server`(MockMoveBackend使用の開発用スタンドアロン
実行ファイル)を使い、エンドツーエンドの疎通を確認した。

```bash
# 1. SDK側プロセス(モック)を起動
cd ../g1_sdk_bridge_cpp/build
./g1_sdk_bridge_mock_server

# 2. 別ターミナルでROS側ノードを起動
source install/setup.bash
ros2 run g1_cmd_router g1_cmd_router_node &
ros2 run g1_state_bridge g1_state_bridge_node &

# 3. Twistを流し始めてからNavigationを有効化(Nav2の実運用に近い順序)
ros2 topic pub --rate 20 /cmd_vel_smoothed geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.1, y: 0.0, z: 0.0}}}" &
ros2 service call /g1/enable_navigation std_srvs/srv/SetBool "{data: true}"

# 4. 確認
ros2 topic echo /odom --once   # x が増えていく、twist.linear.x=0.1 が反映される
```

**確認できたこと**:
- `g1_cmd_router` がSDK側プロセスへIPC接続し、`/g1/enable_navigation`でREADY→NAVIGATINGに遷移する
- `TwistStamped`がクランプ・加速度制限を経てIPC経由でSDK側に届き、モックバックエンドが受け取る
- SDK側プロセスの状態(`NAVIGATING`)と積分されたodometryが`g1_state_bridge`経由で`/odom`に正しく反映される
- **D-10のwatchdogが設計通り動作すること**を偶然実証した: `enable_navigation`成功後すぐにTwistを
  送らないと、`cmd_timeout`(既定0.30秒)超過で自動的に`FAULT`へ遷移する。これはテスト手順の
  ミスから発覚したものだが、safety watchdogが意図通り機能している証拠でもある

## Nav2統合dry-run(2026-09-09、`g1_navigation`パッケージ)— Goal到達をエンドツーエンドで確認

`g1_navigation/`(Nav2設定+疑似データでの動作確認)を実際にNav2フルスタックで動かし、
2件の不具合を発見・解消したうえで、`NavigateToPose`によるGoal到達を実際に確認できた
（Nav2自身のログで`Reached the goal!` / `Goal succeeded`）。詳細は
[../Planning.md](../Planning.md) と [../g1_navigation/README.md](../g1_navigation/README.md) を参照。

1. **`EnableNavigation(true)`直後のcmd_timeout誤爆(修正済み)**: Nav2はGoal計画に
   数百ms〜数秒かかるが、有効化と同時にcmd_timeoutのカウントを始めていたため、
   最初の指令が届く前にFAULTへ誤って遷移していた。「最初の指令を受け取るまではタイムアウト
   判定を待機する」設計に修正し(C++・Python両方)、回帰テストを追加した
2. **D-14デッドバンドによる恒久的な停止(解消済み)**: Nav2のRegulatedPurePursuit
   Controllerが起動直後の"rotate to heading"フェーズで極小さい角速度(実測0.02 rad/s)を
   要求し続けるが、`min_wz`(既定0.03)のデッドバンドがこれを常にゼロへ切り捨てていた。
   ロボットが一切動かないためNav2側も実際の姿勢変化のフィードバックを得られず、
   永久に同じ極小指令を出し続ける「にらみ合い」状態に陥っていた。
   **ユーザー判断によりQUESTIONS.md Q8の(d)を採用: Phase 1のU-12実測が判明するまで
   `min_vx`/`min_wz`の既定値を0(無効)にした**（C++・Python両方）
3. **`cmd_timeout`とNav2再計画サイクルの衝突(未修正・運用上の注意点)**: 解消後の
   最終確認中、Goal到達直前で"Failed to make progress"→再計画のサイクルが
   `cmd_timeout`(0.30秒)より長い間隔で発生し、`g1_cmd_router`が再度FAULTへ落ちた。
   D-13により自動復帰しないため、Nav2は気づかず空振りのリトライを永久に続けていた。
   **手動で`/g1/clear_fault`→`/g1/enable_navigation`を呼び直すことで復帰し、
   その後Goalに到達した。** 本番ではオペレータへの通知の仕組みか、cmd_timeoutと
   Nav2の再計画間隔の整合調整が必要（Phase 1のU-08/U-12実測と合わせて見直す）

## 最終確認結果

```bash
ros2 service call /g1/enable_navigation std_srvs/srv/SetBool "{data: true}"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 4.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

途中で発見3のFAULTに一度遭遇し、`/g1/clear_fault`→`/g1/enable_navigation`で手動復帰した後、
`controller_server`のログに `Reached the goal!`、`bt_navigator`のログに `Goal succeeded` が
出力されることを確認した。

## 既知の制約・残作業

- **`STANDBY→READY`の遷移が簡易実装。** 本来は「歩行可能・センサー正常」を確認してから
  遷移すべき(仕様書7章)だが、TF/センサー鮮度チェックをまだ配線していないため、
  現状は「SDK側プロセスに接続できたら即READY」という簡略化をしている
  (`cmd_router_node.cpp`にTODOコメントで明記済み)。Phase 2c(Nav2接続)着手時に
  正しい判定に置き換えること
- `/g1/stop`はNav2 Goalのキャンセル自体は行わない(ゼロ速度送信とNAVIGATING解除のみ)。
  Goalキャンセルは上位のmission/bringup層(未実装)の責務
- E_STOPからの復帰サービス(`/g1/estop`はtrueでのE_STOP遷移のみ実装、手動解除サービス未実装)
- `g1_interfaces`（独自msg/srv）、`g1_bringup`、`g1_description`はまだ作っていない
- 実機の`unitree_sdk2`を使う`RealMoveBackend`は未実装(実機到着まで検証不可)
