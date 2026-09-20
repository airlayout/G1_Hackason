# unitree_mujoco 調査結果

調査日: 2026-09-08
対象リポジトリ: https://github.com/unitreerobotics/unitree_mujoco
調査方法: `git clone --depth 1` してソースコード・README(英/中)・G1関連ドキュメントを直接確認
確認時点の最新コミット: `1eb6642` (2026-09-07, Merge PR #131) — 継続的にメンテナンスされている

## 結論（先出し）

**G1 はサポートされているが、あくまで低レベル（`LowCmd`/`LowState` 直接制御）のみ。
高レベル移動 API（`LocoClient.Move(vx, vy, omega)` 相当）は一切実装されていない。**
README にも「Current version only supports low-level development」と明記されている。
実機の `LocoClient`/Sport/Loco サービスに相当する歩行コントローラプロセスは同梱されていない。

---

## 1. G1 サポートの有無

サポートあり。`unitree_robots/g1/` に MJCF モデルが存在する。

```
unitree_robots/g1/g1_23dof.xml
unitree_robots/g1/g1_29dof.xml
unitree_robots/g1/g1_joint_index_dds.md
unitree_robots/g1/scene.xml
unitree_robots/g1/scene_23dof.xml
unitree_robots/g1/scene_29dof.xml
```

- 23DOF版・29DOF版の両方のG1モデルに対応（腕・手首DOF構成の違い）。
- `unitree_robots/g1/g1_joint_index_dds.md` に、`unitree_hg::msg::dds_::LowCmd_.motor_cmd` /
  `LowState_.motor_state` 配列のインデックスと実機の関節名の対応表が記載されている
  （例: index0=L_LEG_HIP_PITCH, ... index12=WAIST_YAW など、29DOF版はウエスト3DOFを含む）。
  これは実機の関節番号と完全に一致させる設計になっている。
- C++シミュレータの設定 `simulate/config.yaml` の `robot:` 選択肢に `"g1"` が明記されている
  （`"go2", "b2", "b2w", "h1", "go2w", "g1", "h2", "as2"`）。
- `simulate/src/unitree_sdk2_bridge.h` に `G1Bridge` クラスが専用実装されており、
  G1固有の `BmsState`（バッテリー残量ダミー値100%固定）・`rt/secondary_imu`（胴体IMU、G1のみ）・
  `mode_machine`（23dof/29dofの自動判別）まで再現している。

## 2. 高レベルAPI（Move相当）のサポート有無と根拠

**サポートなし。** 根拠は以下の通り。

### README の明記（`readme.md` 冒頭、Supported Unitree sdk2 Messages節）

> **Current version only supports low-level development, mainly used for sim to real verification of controller**
> - `LowCmd`: Motor control commands
> - `LowState`: Motor state information
> - `SportModeState`: Robot position and velocity data
> - `IMUState`: Torso IMU state at `rt/secondary_imu` topic (G1 only)

つまり公式に「low-level development のみ対応」と宣言している。中国語版 README (`readme_zh.md`) にも同内容が記載されている。

### `SportModeState` はあるが、これは「高レベル制御」ではなく「状態フィードバックのみ」

`simulate/src/unitree_sdk2_bridge.h` の `RobotBridge::run()` を見ると、`highstate`（`SportModeState`型）は
MuJoCoのフレーム位置・速度センサ値をそのままpublishしているだけで、**歩行制御ロジックは一切含まれていない**。
README の注記にも:

> In the actual robot hardware, the `SportModeState` message is not readable after the built-in motion
> control service is turned off. However, the simulator retains this message to allow users to utilize
> the position and velocity information for analyzing the developed control programs.

とあり、あくまで「開発者が自作した制御プログラムを検証するための姿勢・速度情報の提供」が目的であり、
実機の `LocoClient`/`sportmode`サービスのような**歩行コマンドを受けて自律的に歩容生成する高レベルコントローラは実装されていない**。

### コード内に "loco" / "sport mode controller" / `Move()` の実装が存在しない

リポジトリ全体を `loco|sportmode|high.level|Move(` 等でgrepした結果、ヒットしたのは:
- README・ドキュメントでの `SportModeState`（=位置/速度フィードバック用メッセージ型の名前）への言及のみ
- `example/ros2/include/motor_crc.h` の `HIGHLEVEL = 0xee`（CRC計算用の定数。これは実機の`unitree_ros2`のCRCコードをそのまま流用したものでモード制御とは無関係）

`LocoClient`、`loco_client`、歩行パターン生成、`sport_client` に相当するクラス・バイナリはコード中に存在しない。

### サンプルプログラムも全て低レベル直接関節制御

`example/cpp/stand_go2.cpp`、`example/python/stand_go2.py`、`example/ros2/src/stand_go2.cpp` は
いずれも Go2 の12関節に対し `stand_up_joint_pos` / `stand_down_joint_pos` というハードコードされた
関節角配列へPD制御（kp/kd + tau）で直接指令を送るだけの実装であり、`vx, vy, omega` のような速度指令を
受け取って内部で歩容生成するようなAPIは提供されていない。**G1向けのサンプルは同梱されていない**
（README内に「G1をテストする場合はunitree_goメッセージをunitree_hgメッセージに変更する必要がある」という
注記があるのみで、専用サンプルはユーザー側で改造する前提）。

### 別プロセスとして歩行コントローラを模擬するものが同梱されているか

**同梱されていない。** `simulate`（C++版）と `simulate_python`（Python版）のいずれも、
「MuJoCo物理演算 + DDSブリッジ（LowCmd購読→トルク変換、LowState/SportModeState/IMU/WirelessController publish）」
のみで完結しており、歩行制御・バランス制御・軌道生成を行う別プロセス／バイナリは存在しない。
歩行制御ロジックはすべて「ユーザーが自分で書いて`LowCmd`を送り込む」ことが前提の設計。

## 3. DDSトピック名は実機と同一か

**同一。** これがこのシミュレータの最大の価値。

| トピック | 用途 | 実機と同一か |
|---|---|---|
| `rt/lowcmd` | 関節指令（購読） | 同一 |
| `rt/lowstate` | 関節状態（配信） | 同一 |
| `rt/sportmodestate` | 位置・速度状態（配信、Go2系IDL） | 同一（ただしG1実機では運動制御サービスOFF後は読めない点に注意、README記載） |
| `rt/wirelesscontroller` | 無線コントローラ（ジョイスティックエミュレート） | 同一 |
| `rt/secondary_imu` | 胴体IMU（G1のみ） | 同一 |
| `rt/lf/bmsstate` | バッテリー状態（G1、ダミー値100%固定） | 同一 |

メッセージIDLも実機と同じ `unitree_go`（Go2/B2/H1/B2w/Go2w系）と `unitree_hg`（G1/H1-2/AS2系）を使用しており、
`unitree_sdk2`/`unitree_sdk2_python`/`unitree_ros2` をそのままリンクして、
`ChannelFactory::Instance()->Init(1, "lo")`（シミュレーション用domain_id）と
`ChannelFactory::Instance()->Init(0, <実NIC名>)`（実機用）を切り替えるだけで
**同一の低レベル制御コードを実機とシミュレータの両方に向けられる**ことがREADMEのSim to Real節に明記されている。

ただし、この「実機と同一」が保証されるのは**低レベルLowCmd/LowState層のみ**。
実機G1の高レベルサービス（`LocoClient`が内部でやり取りするDDSトピック・サービス、例えば歩行パラメータや
sport modeのRPCサービス）はこのシミュレータには一切実装されておらず、そこは互換性の対象外。

## 4. Go2/H1など他ロボットでの高レベル対応状況

Go2・H1・B2・B2w・Go2w・H2・AS2・A2・R1 いずれについても、コードベース内に高レベル歩行コントローラ相当の
実装は見つからなかった。全ロボット共通で `RobotBridge` テンプレートクラス1つがLowCmd/LowState/SportModeState
（位置・速度フィードバックのみ）/WirelessControllerを処理しているだけで、ロボット種別による違いは
IDL型（`unitree_go` or `unitree_hg`）とG1固有の追加публish（BMS, secondary IMU）程度。
すなわち、**Go2を含め、いずれのロボットについても高レベルAPIのシミュレーション対応は無い**。

将来対応が見込めるかについての手がかり:
- リポジトリは活発にメンテナンスされている（2026-09-07時点で直近マージあり、AS2など新ロボット追加が続いている）が、
  README冒頭の「Current version only supports low-level development」という文言がバージョンを重ねても
  変わっていない（設計方針として意図的に低レベルのみに絞っている可能性が高い）。
- Unitreeの実機高レベルコントローラ（歩行生成・バランス制御を含む`sport`/`loco`サービス）は
  クローズドソースのファームウェア内で動作しており、これをオープンソースのMuJoCoシミュレータ側で
  再実装するのは技術的難易度が高い（歩行のバランス制御自体を自作することになるため）。
  Unitreeが公式にこれを提供する強いインセンティブも見当たらず、短中期での高レベル対応は期待薄。

## 5. Phase 1（SDK Bridge）開発への示唆

### このシミュレータで前倒しできること

1. **DDS通信層の実装・検証** — `rt/lowcmd`購読、`rt/lowstate`/`rt/sportmodestate`/`rt/wirelesscontroller`配信という
   実機と同一トピック名・同一IDL（`unitree_hg`）でのDDS通信コードは、そのままシミュレータ上で開発・デバッグでき、
   実機接続時は`ChannelFactory`の初期化パラメータ（domain_id・ネットワークIF）を変えるだけで良い。
2. **関節インデックス・座標系のマッピング検証** — `g1_joint_index_dds.md`の対応表を使い、
   G1の23DOF/29DOF構成に応じた関節配列⇔ROS 2 joint_states等のマッピングロジックを正しく実装できているか検証可能。
3. **IMU・オドメトリ（位置・速度）データパイプラインの検証** — `SportModeState`（position/velocity）と
   IMU（`imu_state`内、`rt/secondary_imu`）をNav2の`odom`やlocalizationパイプラインに流し込む変換コードの
   単体検証が可能（値そのものはMuJoCo物理演算由来で、実機のセンサ特性・ノイズとは異なる点に注意）。
4. **低レベル直接関節制御によるスタンドアップ等の基礎動作検証** — Go2の`stand_go2`例のように、
   PD制御で決め打ち関節角に追従させる程度の検証は可能。G1向けに同様のコードを自作すれば、
   静止姿勢制御やIMUフィードバックのテストができる。

### このシミュレータで前倒しできないこと（＝Phase 1のうち残るリスク）

- **`LocoClient.Move(vx, vy, omega)` 相当のAPIコールを送った時にロボットが実際に歩く**という、
  Nav2から見た「速度指令→実際の移動」の統合テストはできない。これは実機側のクローズドソース高レベル
  コントローラに依存する部分であり、シミュレータでは代替不可。
- 歩行時のバランス制御・実際の脚の踏み出しパターン・転倒挙動などの動力学的な検証はできない
  （高レベルコントローラ自体が存在しないため、`LowCmd`を送らなければロボットはただ脱力して立っているだけ）。
- SDK Bridgeが実機の高レベルサービスとやり取りする部分（RPC呼び出し、サービスの応答形式など）は
  このシミュレータでは全く模擬されていないため、その部分のインターフェース検証は別手段が必要
  （例: 実機ドキュメント・SDKのヘッダ定義から仕様を読み解いてスタブを自作する等）。

### 結論

`unitree_mujoco` は「Phase 1（SDK Bridge の実機成立前段階）」のうち、
**DDS通信層・メッセージ変換・関節/センサマッピングといった「配線」部分は実機なしでかなり前倒しできる**。
一方、**Nav2の速度指令を実際の歩行に変換する「高レベル移動API」部分は、このシミュレータでは検証不可能**であり、
実機（またはUnitree公式の高レベルSDK/ファームウェアが手に入る何らかの環境）が来るまではスタブ実装・
インターフェース定義レベルの作業に留めるほかない。
プロジェクトの優先順位としては、`unitree_mujoco`を使ったDDS Bridge層の先行実装は価値が高いが、
「歩行が実際に動くかどうか」の検証は実機入手まで持ち越しとなる点をリスクとして明記すべき。
