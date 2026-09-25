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
cmake -S nav2_stable/g1_sdk_bridge_cpp -B /tmp/g1build \
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

## 2026-09-15 に実機で分かったこと

- **ビルドは通る**（PC2 = aarch64 / cmake 3.16.3 / gcc 9.4）。`ldd` 混入チェック合格、
  単体テスト 55 件全通過
- ⚠️ **`sudo` はパスワードが必要**（`unitree` は sudo グループだが NOPASSWD ではない）。
  配置の 3 行は人が対話で実行すること
- ⚠️ **`ssh` ログイン時に `ros:foxy(1) noetic(2) ?` と聞かれる**（`~/.bashrc` の fishros
  ブロック）。**そのまま Enter**。`1` を選ぶと ROS 環境が入り D-06/D-07 に反する
- **発進ゲート閉のまま手起動しても正しく動く**ことを確認した
  （`SDK送信 0 件 / ゲートで停止 N 件`、ソケット生成 OK、`User=unitree` で `eth0` に届く）

## systemd ユニットも実機で通った（2026-09-15）

- `systemctl start/restart` で起動し、`RuntimeDirectory=` が `/run/g1_bridge/` に
  ソケットを作ることを確認した。`User=unitree` で `eth0` に届く
- **`restart` でソケットが作り直されても、ROS 側は自動で再接続する**
  （`SDK側プロセスに接続した: /run/g1_bridge/cmd.sock`）
- ⚠️ **ROS 側は既定で `/tmp/g1_bridge` を見る。** ユニットは `/run/g1_bridge` を使うので、
  ROS をホストでネイティブに動かす本構成では launch 引数 `bridge_sock_dir` で
  合わせること（既定を `/run/g1_bridge` にした）。合わせないと `DISCONNECTED` のまま
- ⚠️ **systemd 版と手起動版が二重に立つ事故を踏んだ。** arm する前に
  `pgrep -af g1_sdk_bridge_real_server` で**1本だけ**であることを確認する
- ⚠️ **サービスは `enable` していない。** バッテリ交換等で PC2 が再起動すると
  起動しない（`/etc/default` の `G1_ARM=--arm` は残るが自動起動はしないので安全側）

## 立ち上げの自動化（`g1up.sh`）

§2〜§7 を1コマンドで通す。**PC2 で叩く。**

```bash
ssh g1                    # ros:foxy(1) noetic(2)? には **Enter だけ**
~/g1_nav2/g1up.sh         # → §2 ブリッジ → §3 SLAM → §4 記録 → §5 Nav2 → §7 自己位置合わせ
```

⚠️ **発進ゲートの開放と Goal 送信はしない。** 人が判断して叩く部分として意図的に
残してある（D-07 の「再起動したら勝手に動けるようになっていた、を構造的に防ぐ」）。
最後に、次に何を叩けばよいかを画面に出す。

### 配置

```bash
rsync -a deploy/pc2_humble/ g1:/home/unitree/g1_nav2/pc2_humble/
rsync -a deploy/pc2_humble/{g1up.sh,start_nav.sh,start_record.sh,start_localizer.sh,cyclonedds_eth0.xml} \
         g1:/home/unitree/g1_nav2/
rsync -a tools/ g1:/home/unitree/g1_nav2/tools/      # record_waypoints.py / patrol_ctl.sh を含む
rsync -a --exclude clouds/ maps/ g1:/home/unitree/g1_nav2/maps/   # ⚠️ 2026-09-24 に必要になった
rsync -a --exclude '__pycache__' ../nav2_stable/g1_ws/src/ g1:/home/unitree/g1_nav2/g1_ws/src/
```

⚠️⚠️ **`maps/` を送り忘れると PC2 のビルドが落ちる**（2026-09-24 に地図を
`nav2_stable/maps/` へ集約したため）。`g1_navigation` の `CMakeLists.txt` が
`../../../maps/grids` を参照しており、無いと `maps/grids が見つからない` で
**FATAL_ERROR** になる。`clouds/`（116MB の点群）は地図を作り直すときしか要らないので
PC2 へは送らない。

⚠️ **`g1up.sh` は `~/g1_nav2/` 直下に置く**（`pc2_humble/` `g1_ws/` `tools/` と並ぶ位置）。
自分の居場所を基準に部品を探すため。

### 一度だけ必要な設定（NOPASSWD）

```bash
sudo tee /etc/sudoers.d/g1-bridge >/dev/null <<'EOS'
unitree ALL=(root) NOPASSWD: /bin/systemctl start g1-sdk-bridge
EOS
sudo chmod 440 /etc/sudoers.d/g1-bridge
```

⚠️ **許可するのは「ゲートを閉じたままの起動」だけ。** `restart` も
`/etc/default` の書き換えも含めない。**発進ゲートを開ける操作は人のパスワードを要求する**
状態に保つ。

### 主なオプション

| | |
|---|---|
| `--localizer` | §7 を連続 localization（`map_localizer.py`）にする。既定は静的 |
| `--patrol <yaml>` | **巡回路を読ませる。** ⚠️ 渡しても走り出さない（`patrol_ctl.sh start` を叩くまで `IDLE`）。巡回路は現地で `tools/record_waypoints.py` で作る |
| `--lidar-yaw 180` | RViz で赤軸が逆を向いていたとき |
| `--map <yaml>` | 地図を差し替える |
| `--operator-timeout 1.0` | 有線運用なら 1.0（既定 2.0 は無線向け。Q12） |
| `--enable` | 走行許可だけを出す。**発進ゲートは開けない** |
| `--dry-run` | 何もせず、やることだけ表示する |
| `--force-posture` | 姿勢の検査を無視する（座位でわざと試すとき） |

### ⚠️ 実行前に機体を立たせておくこと

**「歩かせるときと同じ通常の立位」にして、出発位置に置く。** §5 の自動校正と
§7 の地図照合は**いまの姿勢と位置で決まる**ので、あとで動かすと無効になる。
**人は機体から 2m 以上離れる**（近いと costmap と地図照合の両方が劣化する）。

スクリプトは姿勢を2段で検査する:

| 段 | 内容 | 判定 |
|---|---|---|
| §1 | 起動前に IMU で傾きを測る | 10° 超で停止、6° 超で警告 |
| §5 | 校正に**実際に焼き込まれた**傾きを検査 | 10° 超で停止 |

実測の目安: 通常の立位 **1.6〜4.4°**（基準 3.81°）/ 座位 **15.7°** / 傾いた立位 **19.0°**。

## 未確認

⚠️ `UnsetEnvironment=` の効き目は実機で未確認。
