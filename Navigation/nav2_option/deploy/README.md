# PC2(Orin NX) への配置手順

Planning.md **D-07**（SDK 側プロセスは systemd 管理とし、ROS launch から起動しない）
に対応する配置一式。

## なぜ systemd なのか

1. **ROS 環境を継承させないため。** ROS launch の `ExecuteProcess` から起動すると
   `LD_LIBRARY_PATH` / `AMENT_PREFIX_PATH` を継承し、ROS 側の CycloneDDS に
   誤リンクする（D-06 / D-07）
2. **安全の最終防衛線を ROS launch のライフサイクルに従属させないため。**
   ROS 側が落ちても SDK 側 watchdog は動き続ける必要がある（D-09 / D-10）

## 配置

```
/opt/g1/bin/g1_sdk_bridge_real_server        ← ビルド成果物
/etc/default/g1-sdk-bridge                   ← g1-sdk-bridge.default をコピー
/etc/systemd/system/g1-sdk-bridge.service    ← g1-sdk-bridge.service をコピー
/run/g1_bridge/{cmd,state}.sock              ← RuntimeDirectory= が自動で作る
```

### 手順

```bash
# ① ビルド(PC2 上で。ROS 環境を source していない端末で行うこと)
cmake -S nav2_option/g1_sdk_bridge_cpp -B /tmp/g1build \
      -DUNITREE_SDK2_ROOT=/home/unitree/work/unitree_sdk2
cmake --build /tmp/g1build -j4

# ② D-08 の受入チェック（ROS シンボルが混入していないこと）
ldd /tmp/g1build/g1_sdk_bridge_real_server | grep -E 'rmw|rclcpp|ament'
#   → 何も出なければ合格。ddsc/ddscxx は出てよい（SDK2 自身が使うため）

# ③ 配置
sudo install -D -m755 /tmp/g1build/g1_sdk_bridge_real_server /opt/g1/bin/
sudo install -D -m644 g1-sdk-bridge.default /etc/default/g1-sdk-bridge
sudo install -D -m644 g1-sdk-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload

# ④ ⚠️ まず発進ゲートを閉じたまま起動する（機体は動かない）
sudo systemctl start g1-sdk-bridge
systemctl status g1-sdk-bridge
journalctl -u g1-sdk-bridge -f
#   → "発進ゲートが閉じています(--arm 無し)" が出ていれば正しい
```

### 発進ゲートを開く（**機体が実際に動くようになる**）

```bash
sudo sed -i 's/^G1_ARM=.*/G1_ARM=--arm/' /etc/default/g1-sdk-bridge
sudo systemctl restart g1-sdk-bridge
```

⚠️ 開ける前に必ず確認すること（Planning.md §7）:

- 人が支えられる位置にいる
- 純正リモコンで停止できる
- **前方に 2m 以上の空間がある。** 指令を止めても約2秒動き続ける（§3.3.1 の実測）

試験が終わったら `G1_ARM=` に戻して `restart` すること。

## ROS 側との接続

ソケットは `/run/g1_bridge/` に作られる（実行ファイルの既定は `/tmp/g1_bridge`
なのでユニット側で明示指定している）。

**Nav2 と ROS 側ノードは Humble コンテナで動かす**（A-10c で確定。PC2 ネイティブの
Foxy には `nav2_velocity_smoother` が存在しないため）。

```bash
docker run --rm -it \
  --network host \
  -v /run/g1_bridge:/tmp/g1_bridge \
  <humble イメージ>
```

bind mount 先をコンテナ内で `/tmp/g1_bridge` にすれば、`g1_cmd_router` /
`g1_state_bridge` は既定パスのまま繋がる（パラメータ変更が不要）。

ROS 側をホストでネイティブに動かす場合は、`cmd_sock_path` / `state_sock_path` を
`/run/g1_bridge/...` に合わせること。

## 停止時の挙動

`systemctl stop` は SIGTERM を送り、シグナルハンドラ経由で `SdkBridgeProcess::Stop()`
が **明示的にゼロ速度を送る**。加えて、最後に送った `SetVelocity` の `duration`
（既定 0.20 秒）の満了でも停止する（D-27）。二重になっている。

`Restart=on-failure` で異常終了時は上げ直すが、起動時に必ずゼロ速度を送る（D-11）ため
再起動で前回の速度が残ることはない。60 秒に 5 回を超えて落ちる場合は上げ続けずに
止まる（異常を隠さないため）。

## 未確認

⚠️ **PC2 実機での動作確認は未実施**（2026-09-13 時点）。ユニットファイルの構文は
`systemd-analyze verify` で検証済みだが、実際の起動・ソケット生成・権限は未検証。
特に `User=unitree` で G1 の内蔵スイッチ側インターフェースに届くかは要確認。
