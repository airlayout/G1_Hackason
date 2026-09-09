# g1_sdk_bridge_cpp — 本番実装(D-28)

Python版プロトタイプ（[../g1_sdk_bridge/](../g1_sdk_bridge/)）のロジックを
C++に1対1移植したもの。D-28（2026-09-09、ユーザー確認済み）により、
SDK側プロセスの本番実装言語はC++に確定している。

## ビルド・テスト方法

ROS 2 環境を source していない状態でビルドできる(D-06/D-08の要件)。

```bash
cd g1_sdk_bridge_cpp
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"
./g1_sdk_bridge_tests
```

必要なもの: CMake 3.16+、C++17対応コンパイラ、`libgtest-dev`（`find_package(GTest)`で解決）。
ROS 2は不要。

## Python版との対応関係

| Python (プロトタイプ) | C++ (本番) | 対応内容 |
|---|---|---|
| `protocol.py` | `include/g1_sdk_bridge/protocol.hpp` + `src/protocol.cpp` | ワイヤフォーマットは**バイト単位で同一**(`#pragma pack`でPythonの`struct "<...">`と同じレイアウトに固定。`static_assert`でサイズを検証) |
| `ipc_transport.py` | `ipc_transport.hpp/cpp` | `SOCK_SEQPACKET`ラッパー。非ブロッキング送受信、`PeerClosed`例外 |
| `safety_manager.py` | `safety_manager.hpp/cpp` | 状態機械・クランプ・加速度制限・デッドバンド。rclcpp非依存の純粋ロジック |
| `sdk_process_mock.py` | `sdk_bridge_process.hpp/cpp` | `MoveBackend`インターフェース、`SdkBridgeProcess`(3スレッド構成) |
| テスト38件(unittest) | テスト38件(gtest) | **1対1で移植し、全て通過を確認済み(2026-09-09)** |

テスト内訳: `CmdPacket`(5) + `StatePacket`(2) + `SeqPacketRoundtripTest`(5) + `PureFilters`(5) +
`SafetyManagerStateMachineTest`(13) + `SdkBridgeTest`(8) = 38件。

## Python版との違い(C++化にあたり見直した点)

Python版はGIL(Global Interpreter Lock)により、複数スレッドからの単純な属性アクセスが
偶然安全になっていた箇所があった。C++にはGILが無いため、同じ設計をそのまま持ち込むと
データ競合になる。移植にあたり以下を**明示的にmutexで保護**した(Python版には無かった対応):

- `SdkBridgeProcess`: `cmd_endpoint_` / `state_endpoint_` / `last_cmd_` / `status_` /
  `sdk_error_count_` / `pose_` / `state_seq_` を単一の`mutex_`で保護。`AcceptLoop` /
  `CmdRecvLoop` / `PeriodicLoop`の3スレッドが同時に触るため必須
- `MockMoveBackend`: `SetVelocity()`(周期スレッドから)と`LastCall()`/`AllCalls()`/
  `CallCount()`(テストスレッドから)が同時に呼ばれるため、内部に`mutex_`を追加した

これはD-28で「C++の方が周期送信の安定性で有利」と判断した理由の裏返しでもある——GILが無い分、
並行処理の正しさを自分で保証する責任も増える。

## 実機投入時に差し替える箇所

- `MoveBackend`の実装を`MockMoveBackend`から、`unitree_sdk2`(C++)の
  `LocoClient::SetVelocity()`を呼ぶ`RealMoveBackend`に差し替える
- `SdkBridgeConfig`の`sdk_command_duration_s`は現在0.20秒(仮値)。Phase 0で
  `duration`満了後の実機挙動を確認してから最終値を決定する([../Planning.md](../Planning.md) §3.3参照)

## 未検証・既知の制約

- **ThreadSanitizerでの検証は本開発環境(AWSサンドボックス)では実行不可**だった。
  トリビアルな2スレッドプログラムですら`FATAL: ThreadSanitizer: unexpected memory mapping`で
  落ちることを確認済み(`/proc/sys/vm/mmap_rnd_bits`の変更も権限が無くできない)。
  これは環境のサンドボックス制約であり、コードの欠陥ではない。**G1接続PCなど別環境で
  改めてTSan検証することを推奨する**（`cmake -DCMAKE_CXX_FLAGS="-fsanitize=thread -g -O1"
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=thread"`でビルドし直すだけでよい）
- `unitree_mujoco`が高レベルAPI非対応（[../findings/unitree_mujoco.md](../findings/unitree_mujoco.md)）
  のため、`RealMoveBackend`の実機接続テストは実機到着まで検証できない
- 通常のテスト(TSan無し)は複数回実行して安定してPASSすることを確認しているが、
  スレッドタイミングに依存するテスト(`SdkBridgeTest`系)は理論上まれにタイミング依存の
  flakinessが残る可能性がある(`WaitUntil`のポーリングで大部分は吸収される設計)
