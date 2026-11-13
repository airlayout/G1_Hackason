# 無線で Nav2 を動かす（当日手順・最短版）

更新: 2026-10-14 / ✅ **実機で巡回6点を無線で1周した**（room_a、`Physical_AI_5G`。結果は末尾）

**無線で立ち上げ、RViz の確認のときだけ有線ケーブルを挿す。** 歩かせるのは無線だけ。

## 方針: DDS は無線に通さない

9/15 にテザリングで踏んだ失敗は、すべて DDS を無線に通そうとして起きた
（[Planning.md](Planning.md) ⑩⑪：「トピックは見えるのにデータが来ない」「RViz が真っ暗」）。
無線で要るのは次の 2 つだけで、どちらも DDS を使わない。

| 無線に乗るもの | 方式 | 根拠 |
|---|---|---|
| heartbeat（操作PC → PC2、5Hz） | UDP。PC2 は全インターフェースで受信 | `g1_sdk_bridge_cpp/src/heartbeat.cpp` の `INADDR_ANY` |
| 起動・走行許可・巡回の開始停止・状態確認・停止 | ssh で PC2 上のコマンドを叩く | `g1up.sh` / `tools/patrol_ctl.sh` / `Navigation/real/nav_stop.sh` |

PC2 の CycloneDDS は eth0 固定のまま触らない。

**代償**: 無線のあいだは RViz が見られない。代わりに **`wifi_ops.sh mapview start` でブラウザに地図と現在地を出す**
（[tools/map_web.py](tools/map_web.py)。PC2 の中で ROS を購読し、無線には HTTP だけを流す）。数値なら `wifi_ops.sh pose`。
どちらも機体が推定した位置なので、自己位置がずれても正常に見える。目視も併用する。RViz の「2D Goal Pose」も
使えないので、**Goal は巡回路（yaml）で与える。** 通信断で止まるまでの時間は 2.0 秒（`g1up.sh` の既定）。

## 前提

**ルータ**: TP-Link **TL-WR1502X**（AX1500 トラベルルータ）の **`Physical_AI_5G`（5GHz）**。
構成・パスワードの在処は [Common/network/WIFI_ROUTER.md](../../Common/network/WIFI_ROUTER.md)。
PC2 は **`192.168.0.82`**（9/29・10/14 とも同じ。DHCP なので `wifi_ops.sh ip` か `ping` で確かめる）。

- [ ] 操作PC を `Physical_AI_5G` に繋ぐ（**2.4GHz の `Physical_AI` は使わない**。9/29 の実測で遅延が最大 1.4 秒）
- [ ] PC2 が `Physical_AI_5G` に繋がっている（10/14 時点で設定済み。WiFi 省電力も off 済み）
- [ ] 操作PC に `g1_heartbeat_sender` がある（`command -v g1_heartbeat_sender`）
- [ ] 巡回路は `config/patrol_room_a.yaml`（6点。9/25・10/14 に1周した版）
- [ ] PC2 に `tools/map_web.py` を配置済み（10/15 追加。未配置なら有線で `rsync -a tools/ g1:/home/unitree/g1_nav2/tools/`）
- [ ] **発進ゲートが閉じているか先に見る**: `ssh unitree@192.168.0.82 'grep ^G1_ARM /etc/default/g1-sdk-bridge'`
      → `G1_ARM=`（空）であること。⚠️ **10/14 は前回から開いたまま（`--arm`）になっていた**

## 当日の流れ

道具はすべて [tools/wifi_ops.sh](tools/wifi_ops.sh)（操作PC で叩く）。

```bash
cd <repo>/Navigation/nav2_stable
export G1_WIFI_IP=192.168.0.82
tools/wifi_ops.sh check           # ping / 無線 ssh / 省電力 / DDS が eth0 固定か
```

### 1. 立ち上げる（無線のまま。機体は動かない）

機体を出発位置に**通常の立位で**立たせ、人は 2m 以上離れる（吊ったままでは校正できない）。

```bash
ssh unitree@$G1_WIFI_IP 'cd ~/g1_nav2 && ./g1up.sh --tf-timeout 1.0 --skip-record \
  --patrol ~/g1_nav2/g1_ws/install/g1_navigation/share/g1_navigation/config/patrol_room_a.yaml'
```

- `--tf-timeout 1.0`: 内蔵SLAM を 1801 で上げ直すと odom が約4Hz に落ち、既定 0.5 秒では enable 直後に
  `tf_stale` FAULT になる（9/25）
- ⚠️ **内蔵SLAM が古いまま（吊っていたときなど）だと使い回される。** 16〜26分で止まるので、
  時間が経っていたら先に `ssh unitree@$G1_WIFI_IP 'cd ~/g1_nav2/tools && python3 send_slam_api.py 1901'`
  で止めてから g1up.sh を叩く（g1up.sh が 1801 で上げ直す）
- 照合の一致率が低くても（10/14 は `50cm以内` 44%）、RViz で位置が合っていれば歩けた

### 2. RViz で目視する（ここだけ有線）

LAN ケーブルで G1 と操作PC を繋ぎ:

```bash
nmcli connection up g1-link
G1_RMW=rmw_cyclonedds_cpp G1_RVIZ_IFACE=enp3s0 tools/rviz_operate.sh
```

⚠️ **`G1_RVIZ_IFACE` を必ず付ける**（NIC 名は `ip -br link` で確かめる）。有線と WiFi の両方に繋がった
操作PC では、付けないと CycloneDDS が WiFi 側を選んで何も映らないことがある。

見るもの: ①位置が実際の立ち位置と合っているか ②**赤軸（前方）が実機の正面と同じ向きか**
③機体の周りがマゼンタで囲まれていないか。確認したら `tools/rviz_operate.sh stop`、ケーブルを抜く。

### 3. 無線に切り替えて歩かせる

```bash
tools/wifi_ops.sh clearloop start # costmap の消し残りを 20 秒ごとに消す（9/25・10/14 の1周はこれ込み）
tools/wifi_ops.sh handover        # 無線だけで heartbeat が alive になるのを確かめる
tools/wifi_ops.sh mapview start   # ブラウザで http://192.168.0.82:8090/ を開く（地図・現在地・巡回路）
```

地図表示の見かた: 黄の矢印=機体（尖った方が前）、水色=軌跡、緑=計画経路、橙=巡回路（★が次の目標）。
上の帯に状態（NAVIGATING / FAULT）・巡回の進み・heartbeat。**位置が 2 秒以上更新されないと矢印が赤くなり警告が出る。**
ドラッグで移動、ホイールで拡大、ダブルクリックで機体の追従に戻る。見るだけで操作はできない。

安全確認（停止係が純正リモコンを持つ／前方 2m 空いている／全員 2m 以上離れる）のあと、
**発進ゲートを人が開ける**（sudo のパスワードを聞かれる。意図的に自動化していない）:

```bash
ssh -t unitree@$G1_WIFI_IP "sudo sed -i 's/^G1_ARM=.*/G1_ARM=--arm/' /etc/default/g1-sdk-bridge \
  && sudo systemctl restart g1-sdk-bridge && sleep 2 && pgrep -af g1_sdk_bridge_real_server"
# → 末尾に --arm があること
```

```bash
tools/wifi_ops.sh enable          # → NAVIGATING
tools/wifi_ops.sh patrol start
tools/wifi_ops.sh status          # 巡回の状態（current / loop_count / last_result）
tools/wifi_ops.sh pose 60         # 地図上の位置と向きを 60 秒流す
```

⚠️ 巡回は**1周しても止まらず2周目に入る**。止めたいところで `patrol pause`。

⚠️ `clearloop` は暫定処置（[findings/collision_ahead_cause_20260925.md](findings/collision_ahead_cause_20260925.md) §4）。
**消す瞬間は本物の障害物も消え、死角(〜1.6m)の中のものは立ち直らない。人が見ている場でのみ使う。**

### 止める・撤収する

| 場面 | すること |
|---|---|
| まず止めたい | **純正リモコン**（停止係が常に持つ） |
| 巡回だけ止める | `tools/wifi_ops.sh patrol pause`（次の `start` は同じ点から） |
| FAULT から戻す | `tools/wifi_ops.sh clear` → 原因を確かめてから `enable` |
| 撤収 | `tools/wifi_ops.sh clearloop stop` → `tools/wifi_ops.sh mapview stop` → `G1_SSH=unitree@$G1_WIFI_IP bash ../real/nav_stop.sh --no-ros` → `tools/wifi_ops.sh hb-stop` |
| **最後に必ず** | 発進ゲートを閉じる: `ssh -t unitree@$G1_WIFI_IP "sudo sed -i 's/^G1_ARM=.*/G1_ARM=/' /etc/default/g1-sdk-bridge && sudo systemctl restart g1-sdk-bridge"` |

`nav_stop.sh` は `clearloop` を止めない。発進ゲートも sudo が要るので閉じられない（コマンドを表示する）。

## 実機の結果（2026-10-14、room_a）

| 項目 | 結果 |
|---|---|
| 巡回 | **6点を無線で1周**（20:27〜20:32、約5分）。失敗・再試行・FAULT・通信断は 0 回 |
| heartbeat の遅れ（`hbtest 30`、580 サンプル） | 中央 0.10 秒 / **最大 0.24 秒** / 0.5 秒超え 0 回（テザリングは最大 0.63 秒） |
| 通信断の試験（吊った状態・ゲート閉で実施） | NAVIGATING → `hb-stop` → `operator_lost` で FAULT → `clear` で READY に戻る |
| 無線経由の `nav_stop.sh` | Nav2・巡回が全部止まる |
| 出発地点の照合 | `50cm以内` 44%（基準の 80% を下回る）。RViz で位置は正しく、実際に1周できた |

**Q12（通信断で止まるまでの時間）への材料**: 最大 0.24 秒なので、`--operator-timeout 1.0` に戻せば
受入目標 0.35m を満たせる見込み。ただし測ったのは 30 秒間だけ。次回、巡回中に `hbtest 300` で確かめてから判断する。

**実機で直した不具合**: `wifi_ops.sh` の `clearloop`（自分自身に当たる `pgrep` で「すでに動いている」と誤判定）と
`pose`（引用符の衝突）。

## 分かっていないこと

- 巡回中（歩いている最中）の heartbeat の遅れ。測ったのは立っているときだけ
- 機体から離れたとき・壁の陰での電波の強さ（room_a の 1 周では問題が出なかった）
