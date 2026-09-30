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

## 本番バックエンド `RealMoveBackend`(2026-09-09 実装・実機で確認済み)

`unitree_sdk2`(C++)の `LocoClient::SetVelocity()` を呼ぶ実装。
[include/g1_sdk_bridge/real_move_backend.hpp](include/g1_sdk_bridge/real_move_backend.hpp) /
[src/real_move_backend.cpp](src/real_move_backend.cpp)、実行ファイルは
[src/real_server_main.cpp](src/real_server_main.cpp)。

### 発進ゲート(`--arm`)

**`--arm` を付けない限り SDK を一切呼ばない。** ROS 側の状態機械(`SafetyManager`)とは
独立した防御層で、配線ミスや誤起動でプロセスを立ち上げただけでは機体が動かないことを
構造的に保証する(D-10 の「防御を二重化する」方針と同じ。`Navigation/real/loco_driver.py` の
`--arm` と同じ考え方)。

```bash
# ① 素振り。SDK を呼ばずに IPC と状態機械だけ確認する
./g1_sdk_bridge_real_server --network-interface eth0

# ② 実機を動かす。**人が支え、純正リモコンで停止できる状態で**
./g1_sdk_bridge_real_server --network-interface eth0 --arm
```

**実機での確認結果(PC2、`--arm` 無し、15秒)**:

```
[real_backend] LocoClient を初期化した (iface=eth0 domain=0 timeout=10.0s) / 発進ゲート=閉(SDKを呼ばない)
[real_server] status -> DISCONNECTED
[real_server] SDK送信 0 件 / ゲートで停止 292 件      ← 20Hz周期(D-09)が回り、全てゲートで遮断
```

`ldd` チェックも通過（リンクは SDK 同梱の `libddscxx`/`libddsc` のみで、
**`rmw`/`rclcpp`/`ament` は無し**。CycloneDDS が出るのは正常で、D-08 が禁じているのは
ROS 側のライブラリの混入）。

### D-27 の遵守

`Move()` も `SwitchMoveMode(true)` も使わず、`SetVelocity(vx,vy,omega,duration)` に
常に有限の duration を明示して渡す。バックエンド側でも `duration` が
`(0, max_duration_s]` の範囲かを検証して弾く(上位の検証と二重化)。

### ビルド(実機接続機)

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release \
    -DUNITREE_SDK2_ROOT=/home/unitree/work/unitree_sdk2
```

SDK が見つからない環境（このリポジトリを読むだけの開発機・CI 等）では
本番実行ファイルだけをスキップし、モックとテストは従来どおりビルドされる。

**⚠️ ビルドで踏んだ落とし穴(PC2 実測、2026-09-09)**

1. **`/usr/local` に install 済みの unitree ヘッダには G1 の loco ヘッダが無い**
   (`g1_loco_client.hpp` が存在しない)。**ソースツリーを `UNITREE_SDK2_ROOT` に指定すること。**
   ヘッダとライブラリが別の場所から拾われると危ないので、CMake は
   「1つの prefix の中で全部揃っているか」を確かめる方式にしてある
2. **C++ 版の `<dds/dds.hpp>` は `thirdparty/include/ddscxx/` 配下**にある。
   `thirdparty/include/dds/` は C 版(`dds.h`)なので取り違えると
   `fatal error: dds/dds.hpp: No such file or directory` になる
3. **`LocoClient` は `ChannelFactory::Init()` の後に構築しなければならない。**
   pimpl のメンバ実体として持つと Init より前にコンストラクタが走り、**segfault する**。
   `unique_ptr` で遅延構築している(公式サンプルも Init → client 構築の順)

### 残作業

- `SdkBridgeConfig`の`sdk_command_duration_s`は現在0.20秒(仮値)。Phase 0で
  `duration`満了後の実機挙動(U-07)を確認してから最終値を決定する([../Planning.md](../Planning.md) §3.3参照)
- **`--arm` を付けた実際の歩行検証は未実施**(Phase 0/1 の安全手順に従って実施する)
- systemd サービス化(D-07。`Restart=always`、ROS 環境を継承させない)

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
