# g1_navigation — Nav2設定 + 疑似データでの動作確認(実機なし)

Planning.md「Nav2設定ファイル下書き+疑似データでの動作確認」に対応。costmap/controller/
plannerのパラメータをD-14/D-15/D-21/D-22/D-23の決定に沿って作成し、実機・実センサーなしで
Nav2のライフサイクル・経路計画・制御ループを検証した。

## 構成

- `config/nav2_params.yaml`: controller_server(Regulated Pure Pursuit, D-22)、
  local_costmap(3D点群ベース, D-21)、global_costmap(既知地図+2D scan, D-21)、
  planner_server(NavFn)、velocity_smoother(D-23)
- `maps/synthetic_room.yaml`: Nav2の配線検証用の合成地図(12m×12m、中央に短い仕切り壁)。
  連結した自由空間を保証するために合成した(下記「既知の課題」参照)
- `maps/test_room.yaml`: A-7ツールで生成した実点群由来の地図(参考用。経路計画のデモには使えない)
- `scripts/fake_sensor_publisher.py`: 空の`/scan`・`/g1/points_local`を配信(costmapが詰まらないように)
- `launch/navigation.launch.py`: 上記一式 + `g1_state_bridge`/`g1_cmd_router` + `map→odom`の
  静的TF(dry-run専用のスタンドイン)をまとめて起動する

SDK側プロセス(`g1_sdk_bridge_mock_server`)は別途起動しておくこと。

## 動作確認結果(2026-09-09)

| 項目 | 結果 |
|---|---|
| Nav2ライフサイクルノード(map_server, controller_server, planner_server, behavior_server, bt_navigator, velocity_smoother)の起動・activate | ✅ 全て成功 |
| `NavigateToPose`アクションでのGoal受理・グローバルパス計画(NavFn) | ✅ 成功 |
| `/cmd_vel_nav` → velocity_smoother → `/cmd_vel_smoothed`の配信(20Hz) | ✅ 安定して確認 |
| `g1_cmd_router`のIPC接続・状態遷移 | ✅ 確認(3件の不具合を発見、うち2件修正・1件は運用上の注意点として記録) |
| Goal到達(ロボットが実際に動いてゴールに着く) | ✅ **達成**。`Reached the goal!` / `Goal succeeded`を確認 |

**発見した不具合**(詳細は[../README.md](../README.md)「Nav2統合dry-run」参照):

1. `EnableNavigation(true)`直後のcmd_timeout誤爆 → **修正済み**(g1_sdk_bridge_cpp / g1_sdk_bridge両方)
2. D-14デッドバンドがNav2のrotate-to-heading時の微小角速度(0.02 rad/s)を常時ゼロに切り捨て、
   ロボットが永久に動き出せない → **解消済み**（QUESTIONS.md Q8(d)、`min_vx`/`min_wz`既定値を0に変更）
3. `cmd_timeout`がNav2の再計画リトライ間隔と衝突し、一度FAULTになると自動復帰せず
   永久に空振りリトライが続く → **未修正、運用上の注意点**。手動`clear_fault`で復帰可能

## 既知の課題: A-7ツールの占有格子は連結した自由空間をほぼ持たない

vendoringしたサンプル地図(`test_room.yaml`、A-7ツールで生成)を使うと、NavFnプランナーが
「経路が見つからない」で失敗した。原因を調査したところ、A-7ツールは**点群の点があるセルだけを
free/occupiedと判定しており、センサーから点までの光線経路(レイトレーシング)を行っていない**ため、
自由空間が数万個の孤立した小片に分断されていた(最大連結成分でも3m²未満)。

これはA-7ツール自体の実装ギャップとして記録する(Planning.md参照)。今回のNav2動作確認では、
連結した自由空間を保証した合成地図(`synthetic_room.yaml`)に切り替えて検証を続行した。

## 使い方

```bash
# 1. SDK側プロセス(モック)
cd ../../g1_sdk_bridge_cpp/build && ./g1_sdk_bridge_mock_server &

# 2. Nav2一式
source install/setup.bash
ros2 launch g1_navigation navigation.launch.py

# 3. Navigation有効化 + Goal送信
ros2 service call /g1/enable_navigation std_srvs/srv/SetBool "{data: true}"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 4.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

`min_vx`/`min_wz`の既定値は既に0(無効)になっているため、追加のパラメータ指定は不要。
上記の手順でGoal到達まで確認できる（途中で発見3のFAULTに遭遇した場合は
`/g1/clear_fault`→`/g1/enable_navigation`を呼び直すこと）。
