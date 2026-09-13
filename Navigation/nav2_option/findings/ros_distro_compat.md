# ROS 2 ディストリ互換性の実測調査（2026-09-13、実機不要）

Phase 1 の残作業として挙げていた「**`g1_cmd_router` を実機で動かすには ROS のバージョン
問題を解く必要がある**」を、実機に触らずに決着させるための調査。

- PC2（Orin NX）にネイティブで入っているのは **ROS 2 Foxy**（+ ROS 1 Noetic）
- `g1_ws` は **Jazzy 向け**として書かれている（D-01）
- Mapping 用の Docker コンテナは **Humble**

D-05 により ROS 側ノードは SDK 側プロセスと**同一ホスト**（PC2）で動く必要がある
（IPC が Unix domain socket のため）。そのため「PC2 でどの ROS を使うか」は
先送りできない設計判断になっていた。

---

## 1. 結論（先に要点）

| # | 判明したこと |
|---|---|
| ① | **自作 3 パッケージは Foxy / Humble の両方で無修正ビルドできる**（警告ゼロ） |
| ② | **Nav2 本体は Foxy では成立しない。** `nav2_velocity_smoother` と `nav2_behaviors` が Foxy に存在しない |
| ③ | **Humble の `velocity_smoother` は `/cmd_vel_smoothed` を `Twist` で出す。**`TwistStamped` だけを購読していた `g1_cmd_router` とは**エラーも警告も出ないまま繋がらなかった** |

→ **配置の答え: Nav2 と ROS 側ノードは Humble コンテナに入れ、SDK 側プロセスは
PC2 ホストにネイティブ常駐させる**（コンテナは `/tmp/g1_bridge` を bind mount する）。
これは D-05 / D-06 / D-07 / D-08 をいずれも壊さない。

③ は実装を修正して解消済み（後述 §4）。

---

## 2. ①自作パッケージのビルド互換性

`g1_cmd_router` / `g1_state_bridge` / `g1_navigation` を各コンテナでビルドした。

| 環境 | gcc | 結果 |
|---|---|---|
| `ros:foxy-ros-base`（Ubuntu 20.04） | 9.4.0 | ✅ 3 パッケージ成功・警告ゼロ |
| `g1-mapping-visualization:local`（Humble / Ubuntu 22.04） | 11 | ✅ 3 パッケージ成功・警告ゼロ |
| ホスト（Ubuntu 24.04、ROS 無し）※SDK側のみ | 13.3.0 | ✅ 成功。`ldd` に rmw/rclcpp/ament の混入なし（D-08 受入基準） |

Jazzy 固有の API は使っていなかった。依存も `rclcpp` / `geometry_msgs` / `std_msgs` /
`std_srvs` / `diagnostic_msgs` / `nav_msgs` / `tf2_ros` というコアのみで、
インクルードも Foxy 時代の命名（`tf2_ros/transform_broadcaster.h`）だったため通った。

### Foxy でのエンドツーエンド動作確認

ビルドが通るだけでなく、**Foxy で実際に全経路が動くこと**を確認した
（SDK 側はモック `g1_sdk_bridge_mock_server`）。

| 試験 | 結果 |
|---|---|
| 起動 → IPC 接続 → `READY` | ✅ |
| `/g1/enable_navigation` → `NAVIGATING` | ✅ |
| `/cmd_vel_smoothed`(TwistStamped, vx=0.3) → SDK 側 | ✅ `vx=0.300` が到達 |
| 加速度制限（`max_ax=0.20`）のランプ | ✅ `0.000 → 0.015 → 0.115 → 0.215 → 0.300` |
| `/odom` / `/tf` の publish（`g1_state_bridge`） | ✅ |
| 指令断 → **ROS 側 watchdog**（D-10） | ✅ `cmd_timeout` で `FAULT` |
| `/g1/clear_fault` → `READY` 復帰 | ✅ |

📌 デッドバンド（U-12 実測の `min_vx=0.25`）も効いていることを確認した。
`vx=0.20` を送ると SDK 側には 1 件も届かない。**これは仕様どおりの挙動**で、
最初これをバグと誤認しかけた。

---

## 3. ②Nav2 本体の Foxy 非対応

`ros:foxy-ros-base` の apt で `g1_navigation` の `exec_depend` を全て引いた。

| パッケージ | Foxy |
|---|---|
| `navigation2` / `nav2_bringup` | ✅ 0.4.7 |
| `nav2_map_server` / `nav2_lifecycle_manager` | ✅ |
| `nav2_controller` / `nav2_planner` / `nav2_costmap_2d` | ✅ |
| `nav2_bt_navigator` | ✅ |
| `nav2_regulated_pure_pursuit_controller` | ✅（D-24 の Controller は Foxy にもある） |
| **`nav2_behaviors`** | ❌ **無い**（Foxy では `nav2_recoveries` という旧名） |
| **`nav2_velocity_smoother`** | ❌ **無い**（Humble で新規追加） |
| `nav2_smoother` | ❌ 無い |

`velocity_smoother` は速度・加速度のスムージングという本設計の要（D-23 の
`/cmd_vel_smoothed` はこのノードの出力）なので、**Foxy では設計どおりに組めない。**

→ Nav2 は Humble コンテナで動かす。`g1_cmd_router` も同じコンテナに入れてよい
（②で Humble ビルドが通ることを確認済み）。SDK 側プロセスはホスト常駐のまま、
コンテナに `/tmp/g1_bridge` を bind mount すれば Unix domain socket は跨げる。

---

## 4. ③`Twist` と `TwistStamped` の食い違い（実際に踏んだ）

### 症状

Humble の `nav2_velocity_smoother` を起動して `g1_cmd_router` と繋ぐと、
`/cmd_vel` に 20Hz で `vx=0.3` を流しているのに **SDK 側には `vx=0.000` しか届かない。**

さらに悪いことに、

- **エラーも警告も一切出ない**
- `cmd_router` は `NAVIGATING` のまま（`FAULT` にすら落ちない）
- ロボットは黙って動かないだけ

`FAULT` に落ちないのは、SafetyManager が「**最初の指令を受け取るまで
`cmd_timeout` の計測を始めない**」設計になっているため（Nav2 の計画時間を待つ、
2026-09-09 に意図して入れた猶予）。配線ミスのときはこの猶予が**無期限の沈黙**になる。

### 原因

`ros2 topic info /cmd_vel_smoothed --verbose` が決定的な証拠を出した:

```
Type: ['geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped']
```

**同じトピック名に 2 つの型が同居している。** ROS 2 は型が違う publisher と
subscriber を単にマッチさせないだけで、エラーにはしない。

`velocity_smoother` の出力型はディストリで異なる:

| ディストリ | `/cmd_vel_smoothed` の型 | `enable_stamped_cmd_vel` |
|---|---|---|
| **Humble** | `Twist` 固定 | **パラメータ自体が存在しない** |
| Jazzy | 既定 `Twist`、`true` で `TwistStamped` | あり（既定 false） |
| Kilted 以降 | 既定 `TwistStamped` | あり（既定 true） |

`nav2_params.yaml` に書いてあった `enable_stamped_cmd_vel: true`（D-23）は、
**Humble では宣言されていないパラメータなので黙って無視される**（起動は失敗しない）。
これも実測で確認した。

### 対処

`g1_cmd_router` が **`Twist` と `TwistStamped` の両方を購読する**ようにした。
同じトピック名に 2 つの subscription を張り、どちらで来ても同じ経路に流す。

加えて、今回のような配線ミスが黙って通り過ぎないように 2 つの可視化を入れた:

1. 最初の 1 件を受けた時点で**どちらの型で受けているか**を INFO で出す
2. `NAVIGATING` なのに指令が `no_cmd_warn_s`（既定 3.0 秒）届かないとき WARN を出す
   —— **状態遷移はさせない**。SafetyManager の猶予設計は意図的なものなので変えず、
   「黙って動かない」状況を見えるようにするだけにとどめた

### 修正後の検証

**Humble の本物の `nav2_velocity_smoother` と繋いで**確認した:

```
--- SDK側に到達した vx の一覧(加速度制限 max_ax=0.20 のランプ)
vx=0.000
vx=0.050
vx=0.150
vx=0.250
vx=0.300
```

`NAVIGATING` を維持し、指令を止めると ROS 側 watchdog で `FAULT`。
Foxy 側も退行なし（TwistStamped / Twist の両経路とも受信を確認）。

---

## 5. 残る未確認事項

- **PC2 実機での確認は未実施。** 上記は全てローカル PC の Docker 上での結果であり、
  PC2（Orin NX / arm64）で同じイメージが動くかは別途確認が必要。
  特に Humble コンテナの arm64 対応と、`/tmp/g1_bridge` の bind mount。
- **Nav2 を Humble で回し続けるか、Jazzy へ上げるか**は未決（QUESTIONS.md 参照）。
  D-01 は Jazzy 継続としているが、実際に動かしてきたのは Humble コンテナ。
- 閉ループ制御の検証は依然として実機必須（A-10b と同じ原理的制約）。

## 6. 再現方法

```bash
# Foxy
docker pull ros:foxy-ros-base
docker run --rm -v <repo>/nav2_option:/nav2_option -w /nav2_option/g1_ws \
  ros:foxy-ros-base bash -c 'source /opt/ros/foxy/setup.bash && colcon build'

# Humble（Nav2 入りイメージ）
docker run --rm -v <repo>/nav2_option:/nav2_option -w /nav2_option/g1_ws \
  g1-mapping-visualization:local bash -c 'source /opt/ros/humble/setup.bash && colcon build'
```

⚠️ `g1_cmd_router/CMakeLists.txt` は `g1_sdk_bridge_cpp` を**相対パス**
（`../../../g1_sdk_bridge_cpp`）で参照するので、`g1_ws` だけをコンテナに渡すと
ビルドできない。**`nav2_option/` ごと渡すこと。**

⚠️ コンテナは root で `build/` `install/` `log/` を作るため、ホスト側から
`rm -rf` できなくなる。消すときはコンテナ経由で消す。
