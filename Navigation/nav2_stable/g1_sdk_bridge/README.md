# g1_sdk_bridge プロトタイプ

Planning.md の Phase A-3（IPCプロトコル設計・実装）に対応するプロトタイプ実装。
実機・ROS 2 なしで、SDK側プロセスと ROS側 Safety Manager のロジックを検証する。

## 実行方法

```bash
cd g1_sdk_bridge
./tests/run_tests.sh
```

Python標準ライブラリのみで動く(`pytest` 等の追加インストール不要)。37件のテストが通る。

## ファイル構成

| ファイル | 内容 |
|---|---|
| `protocol.py` | IPCペイロードの定義（`CmdPacket` / `StatePacket`）。D-05 |
| `ipc_transport.py` | Unix domain socket (`SOCK_SEQPACKET`) のラッパー。D-05 |
| `sdk_process_mock.py` | SDK側プロセスのロジック（`SdkBridgeProcess`）。D-09, D-11, D-13, D-27 |
| `safety_manager.py` | ROS側 Safety Manager のロジック（`SafetyManager`）。D-10, D-13, D-14, D-15 |
| `tests/` | 上記4つの単体・統合テスト(37件) |

## このプロトタイプが検証していること

- IPCプロトコルのエンコード/デコードの正しさ、異常系（サイズ不正・magic不一致）の拒否
- Unix domain socket 上での実際の送受信、backpressure時の破棄（latest-value semantics）、
  相手プロセスの切断検知（`PeerClosed`）
- Safety Manager の状態機械（仕様書7章）と優先順位（仕様書8章: E-stop > 異常 > Nav2指令）
- クランプ・加速度制限・デッドバンド（D-14）の計算そのものの正しさ
- SDK側プロセスの watchdog（cmd_timeout超過でゼロ速度）、起動直後のゼロ速度送信（D-11）、
  連続SDKエラーでのFAULT遷移と手動復帰（D-13）
- `SetVelocity()` 相当の呼び出しに常に有限の `duration` が渡ること（D-27）

## このプロトタイプが検証して**いない**こと（実機必須）

`unitree_mujoco` が高レベルAPI非対応と判明した（[findings/unitree_mujoco.md](../findings/unitree_mujoco.md)）ため、
以下は原理的にこのプロトタイプでは検証できない。

- 実際のG1が速度指令通りに歩くか
- `SetVelocity()` の `duration` 満了後にG1が実際に停止するか、直前速度を保持し続けるか（U-07）
- 各速度での実際の停止距離（U-08）
- odometryの符号・単位（U-10）、その場旋回時のドリフト（U-11）
- 歩容が成立する下限速度＝デッドバンド閾値の実際の値（U-12。`min_vx`/`min_wz` の初期値は仮値）

これらは Planning.md の Phase 0 / Phase 1 で実機を使って確定する。

## 実機投入時の扱い（D-28: 本番実装言語はC++に確定、2026-09-09）

このPythonコードは**プロトタイプのまま本番投入はしない**。ロジック設計を検証する目的のみで、
本番実装はC++に移植する。移植時の対応関係は以下の通り（インターフェースの形はそのまま踏襲する）。

- `sdk_process_mock.py` の `MoveBackend` プロトコル → `unitree_sdk2`（C++版）の
  `LocoClient::SetVelocity()` を呼ぶ実装に置き換える。呼び出し規約（常に有限`duration`を渡す、
  D-27）はそのままC++側でも守る
- `safety_manager.py` の状態機械・クランプ・加速度制限・デッドバンドのロジックは、
  ROS 2ノードに依存しない純粋ロジックとして書いてあるため、C++移植時もそのままrclcppノードから
  呼べる形（純粋な関数・クラスとして分離した設計）を踏襲する
- `protocol.py` / `ipc_transport.py` のワイヤフォーマット（`SOCK_SEQPACKET`、固定長構造体）は
  言語非依存の設計にしてあるため、C++側でも同じバイナリレイアウトで送受信できる想定
- Python版の単体テスト38件は、C++移植後も**同じ入出力関係を検証するテストとして移植する**
  （ロジックの正しさを再確認する目的。テストコード自体を1対1で書き直す必要がある）

## 既知の制約・今後の課題

- colcon workspace化していない（Planning.md QUESTIONS.md Q3、確定）。本番実装（C++）時に
  ROSパッケージへ昇格させる
- このPython実装はロジック検証用のプロトタイプであり、本番のC++実装のベースにはなるが、
  そのまま流用するコードではない
- `SdkBridgeConfig` の `__post_init__` で `send_period < duration <= cmd_timeout` を強制している。
  この関係式自体は [findings/sdk2_api.md](../findings/sdk2_api.md) の調査結果に基づく設計判断（D-27）であり、
  実機での妥当性検証はまだ
