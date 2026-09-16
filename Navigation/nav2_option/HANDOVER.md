# 引き継ぎ — 次にこれを触る人へ

最終更新: **2026-09-15**（実機セッション）

このファイルは「**次に何をすればいいか**」だけを書く。何があったかの記録は
[Planning.md](Planning.md) の A-10n、当日の操作手順は
[safety/runbook_nav2_session.md](safety/runbook_nav2_session.md) を見ること。

---

## 1. 到達点（2026-09-15 時点）

**実機が Nav2 の指令で歩いた。** §3〜§8 の経路がすべて実機で成立している。

| | |
|---|---|
| 自律歩行 | 3 回（**0.34m / 3.17m / 4.96m**）。Goal は **1 件 SUCCEEDED** |
| **U-16（歩行中の odometry 精度）** | ✅ **答えが出た。約5m 歩いてドリフト 7cm（測定分解能）、yaw ゼロ。** Phase 2a の FAST-LIO 構築（A-6）は省略できる見込み |
| D-31（heartbeat） | 実機で初通電。通信断で正しく停止することを確認（意図せず 1 回発動した） |
| 残る壁 | **その場旋回が始まらない**（下記 2-①）と、**地図が現状と合っていない**（2-②） |

---

## 2. 次にやること（この順で）

### ① 旋回デッドロックの対処を実機で試す ← **最優先。10分で終わる**

**今日の失敗のほぼ全部がこれで説明できる。** 対処はコミット済みだが**実機で未検証**。

RPP の rotate-to-heading は**実測角速度を基準に**加速度制限をかけるため、

```
指令 = 実測 + max_angular_accel × dt = 0 + 0.40 × 0.05 = 0.02 rad/s
```

となり、G1 は 0.02 rad/s では歩容が成立しないので回らない → 実測 0 のまま →
また 0.02、という**デッドロック**に入る。実測で wz=-0.02 が 2,363 周期続き、
機体は 4mm も動かなかった。

**入れた対処**（[config/nav2_params.yaml](g1_ws/src/g1_navigation/config/nav2_params.yaml)）:
- `max_angular_accel` 0.40 → **6.0**（1 周期の増分が歩容の閾値を超えるように）
- progress checker を **`PoseProgressChecker`**（角度も進捗と数える）+ 20 秒に

✅ **2026-09-16、モックで再現と修正確認を済ませた**
（[findings/mock_deadlock_repro.md](findings/mock_deadlock_repro.md)。
`./tools/mock_deadlock_test.sh old` / `new` で誰でも再現できる）:

| | 旧設定 | 新設定 |
|---|---|---|
| wz | **0.020 で頭打ち** | **0.300**（立ち上がる） |
| 60秒後 | **1mm も動かない** | **1.88m 移動** |
| 結末 | abort → spin → 繰り返し | **`Reached the goal!`** |

**実機での判定基準**:

| 見るもの | 期待 |
|---|---|
| `/cmd_vel_smoothed` の wz | **0.02 で止まらず 0.3 前後まで上がる** |
| 機体 | その場旋回が**始まる** |
| `follow_path` の abort | **10 秒ちょうどで切れなくなる** |

⚠️ **モックに物理は無い。閾値を階段関数で模しただけ。**
実機の旋回の作動閾値は未知（`0.02` で不動・`0.3` で回る、しか分かっていない）。
**真の閾値が 0.3 より上なら 6.0 でも足りない**（6.0 × 0.05 = 0.30 が1周期ぶんのため）。
そのときは `rotate_to_heading_angular_vel`（未指定なので既定 1.8）を明示するか、
`max_angular_accel` をさらに上げる（10.0 なら1周期で 0.5）。

### ② 地図を測り直す

room_a の地図は **9/07 取得**で、現状と合っていない。スキャンの一致率が
**20cm以内 40〜45% / 50cm以内 47〜99%（場所による）**まで落ちている。

⚠️ **一致率が低い ＝ 自己位置が間違い、ではない。** 今日、利用者が RViz で目視確認し、
かつ odometry と 6cm 以内で一致していた。**判定に使うなら「50cm以内」を見ること。**

**軌跡（trajectory）が要る。** 点群だけでは自由空間が連結せず、経路計画に使えない
（A-7 の `test_room.yaml` と同じ失敗）。

- `Navigation/map_wireless_dualcam_01_cleaned_no_ceiling.pcd`（880万点、58m×40m、数日前）が
  手元にあるが、**対応する軌跡が見つかっていない**。これ単体では今日の地図を上回れない
- Mapping のパイプラインは実行ごとに `trajectory/trajectory.tum` を自動生成する
  （`g1_mapping_tools/trajectory_writer.py`）。**その run ディレクトリを探すこと**
- bag が残っていれば `/unitree/slam_mapping/odom` から再生成できる（XY だけでよい）

⚠️ **軌跡ファイルの形式には落とし穴がある。**
`pointcloud_to_occupancy_grid.py` は **「2列目が x、3列目が y」しか見ない**。
TUM（`timestamp tx ty tz ...`）はそのまま通るが、
`nav_msgs/Odometry` を CSV に落としたもの（`sec,nsec,frame,child,x,y,z,...`）は
**x が5列目**なので**全行スキップされ、警告も出ないまま「軌跡なし」と同じ地図ができる**。

```bash
# Odometry CSV → 読める形式へ
awk -F, '{print $1"."$2, $5, $6, $7}' <odom.csv> > trajectory.txt
python3 tools/pointcloud_to_occupancy_grid/pointcloud_to_occupancy_grid.py <点群.pcd> \
    --trajectory trajectory.txt --resolution 0.05 --out newmap
```

### ③ 内蔵SLAM の16分停止 — **方針は (a) に決めた。タダで取れるデータがある**

巡回は16分では終わらない。落ちるたびに odom 原点がリセットされ、`map→odom` を
求め直す必要がある。**2026-09-16 に原因を調査したが特定できなかったので、
「約16分ごとに `1801` を送り直し §7 をやり直す」運用を採る。**

公式ドキュメントで分かったこと:

- **16分の制限は文書化されていない**
- 関連記述は「**連続使用は30分まで**を推奨。長時間・高温だと dock の CPU が
  周波数低下し**測位が異常になる**」。実測は CPU 51〜59% / 59〜66℃ で、
  16分の説明としては弱い
- 未適用の注意事項: 「**建図中は機体内蔵の障害物回避を切る**（頭部ライトが青になる）」
  「建図中は機体を速く動かしすぎない」

📌📌 **次のセッションで、ロボットの時間を追加で使わずに判別できる。**
`1801` を**最初に1回だけ送り、以後は再送せずに** `watch_slam_alive.sh` を回しっぱなしにする。
①②③で機体は歩くので、**16分を超えて生き延びれば「無動作で切れる」**と分かる。

> **⚠️ 観測に偏りがある。** 2026-09-15 の3回とも**機体が静止している間**に落ちた。
> 歩行中のセッションは一度も16分を超えていないので、「時間で切れる」のか
> 「無動作で切れる」のかを区別できていない。

**もし「時間で切れる」と確定したら**、選択肢は2つに絞られる。

| | 内容 | 評価 |
|---|---|---|
| (a) 16分ごとに再定位する | いま採っている方針 | 実装は軽いが巡回が分断される |
| (c) **自前の LIO（FAST-LIO）に置き換える** | 落ちない | **U-16 で「精度の観点では不要」と結論したが、可用性の観点で復活しうる。** 下記 25m×25m の制約も併せて効くなら、こちらが本筋になる |

### ④ Q12 の採否を決める（**人の判断が要る**）

`operator_timeout_s` を何秒にするか。実測は [QUESTIONS.md](QUESTIONS.md) に記録した。

| 経路 | heartbeat age | 1.0 秒で足りるか |
|---|---|---|
| 有線 | 中央 0.1 秒以下 | ✅ |
| テザリング（WiFi 省電力 on） | 1.0 秒超 | ❌ **機体が一歩も動かないまま FAULT** |
| テザリング（省電力 off） | 中央 0.101 / **最大 0.628** 秒 | △ 余裕 0.37 秒 |

暫定で **2.0 秒**にしてある。代償として **D-31 の受入目標 0.35m は満たせない**。
(a) 有線で 1.0 秒維持 / (b) 無線で 2.0 秒 / (c) 無線の質を上げて 1.0 秒維持、のどれか。

---

## 3. ゼロから立ち上げる手順

### まずこれを試す（2026-09-16 追加。**実機未検証**）

⚠️ **先に機体を「歩かせるときと同じ通常の立位」にして、出発位置に置くこと。**
§5 の校正と §7 の照合は**いまの姿勢と位置で決まる**ので、あとで動かすと無効になる。
**人は機体から 2m 以上離れる。**

```bash
ssh g1              # ros:foxy(1) noetic(2)? には **Enter だけ**
~/g1_nav2/g1up.sh   # §1〜§7 を一気に通し、各段の✅判定を出す
```

スクリプトは**姿勢を2段で検査する**（2026-09-15 に最も時間を溶かした落とし穴のため）:

- 起動前に IMU で傾きを測る。**10° を超えたら止める**（座位 15.7° / 傾いた立位 19.0° /
  通常の立位 1.6〜4.4°）
- §5 で実際に焼き込まれた値も検査する

意図的に座位などで進めたいときは `--force-posture`。

⚠️ **発進ゲートの開放と Goal 送信はしない**（人の判断を残すため）。最後に次の手順を表示する。
⚠️ **まだ実機で動かしていない。** 失敗したら下の手動手順に落ちること。
配置と NOPASSWD の設定は [deploy/README.md](deploy/README.md) を参照。

### 手動手順（`g1up.sh` が転んだとき / 中身を知りたいとき）

⚠️ **手順書 §1〜§8 の実体はこれ。** 手順書には書ききれていない前提が入っている。

```bash
# --- 操作PC ---
nmcli connection up g1-link                 # 有線が落ちていることが何度もあった
ssh g1                                      # ros:foxy(1) noetic(2)? → **Enter だけ**

# --- PC2（ここから）---
# ① SDK側プロセス（ゲートは閉じたまま）
grep '^G1_ARM=' /etc/default/g1-sdk-bridge  # → 空であること
sudo systemctl start g1-sdk-bridge
pgrep -af g1_sdk_bridge_real_server         # → **1本だけ**、--arm が無いこと

# ② 内蔵SLAM（約16分で勝手に止まる。巡回の直前に送ること）
cd ~/g1_nav2/tools && python3 send_slam_api.py 1801
setsid nohup bash ~/g1_nav2/tools/watch_slam_alive.sh >/dev/null 2>&1 &

# ③ 記録（Nav2 より先に）
bash ~/g1_nav2/start_record.sh

# ④ Nav2（機体を静止させたまま）
bash ~/g1_nav2/start_nav.sh "0 0 0" 0 2.0   # map_to_odom / lidar_yaw / operator_timeout_s

# ⑤ 地図照合 → 適用
python3 ~/g1_nav2/tools/find_map_offset.py --yaw-range 180 --yaw-step 1
bash ~/g1_nav2/start_nav.sh "<dx> <dy> <yaw_rad>" 0 2.0

# --- 操作PC ---
g1_heartbeat_sender --host 192.168.123.164 &     # ~/.local/bin にある
cd Navigation/nav2_option/tools && G1_RMW=rmw_cyclonedds_cpp ./rviz_operate.sh
```

**ここで RViz を見て、位置と向きの両方が実機と合っていることを目視確認する。**
合っていなければ先に進まない。

```bash
# --- 武装（全員が2m以上離れてから）---
sudo sed -i 's/^G1_ARM=.*/G1_ARM=--arm/' /etc/default/g1-sdk-bridge
sudo systemctl restart g1-sdk-bridge
pgrep -af g1_sdk_bridge_real_server         # → --arm が付いていること
# ROS 側は自動で再接続する
ros2 service call /g1/enable_navigation std_srvs/srv/SetBool "{data: true}"
# → RViz の「2D Goal Pose」で 1〜2m 先を指す
```

---

## 4. 知らないと必ず詰まること

| # | 落とし穴 | 対処 |
|---|---|---|
| 1 | **内蔵SLAM が約16分で勝手に止まる**（3回とも 941〜1022 秒）。LiDAR 生点群は流れ続けるので気づきにくい | `1801` 再送。⚠️ **odom 原点がリセットされるので §7 をやり直す** |
| 2 | **人が機体を追って歩くと必ず失敗する。** 追従者が常に障害物として立ち、マークも一緒に動く | **2m 以上離れる。後ろも含む** |
| 3 | **`lidar_yaw` は姿勢で 0/180 が入れ替わっていた**（水平化を直したので今は不要のはず） | RViz で**赤軸の向き**を毎回目視。一致率では判別できない |
| 4 | **機体自身の脚が障害物として立つ** | `obstacle_min_range: 0.9` + `footprint_clearing_enabled: true`（設定済み） |
| 5 | **PC2 の FastDDS は点群を受け取れない**（`net.core.rmem_max` が 212992） | ROS 側は **CycloneDDS**（設定済み）。`sudo sysctl -w net.core.rmem_max=16777216` でも直るはず |
| 6 | **既定ルートが wlan0 になると DDS が壊れる。** 「トピックは見えるのにデータが来ない」 | WiFi を `ipv4.never-default yes`。CycloneDDS は eth0 固定（設定済み） |
| 7 | **テザリング越しに DDS は通らない**（AP がマルチキャストを遮断）。RViz が真っ暗になる | 有線を使う。どうしても無線なら [tools/make_fastdds_peers.sh](tools/make_fastdds_peers.sh)（⚠️ マルチキャストアドレスも peer に入れる） |
| 8 | **`backup` を `behavior_plugins` から外すと bt_navigator が起動しない** | 外さない。塞ぐなら BT XML を差し替える（未実施） |
| 9 | **`sudo` は PC2 の `unitree` のパスワード。** 操作PC で叩いても通らない | `ssh g1` してから |
| 10 | **`/etc/default` を書き換えただけではゲートは開かない** | `systemctl restart` が必須 |
| 11 | **PC2 の時計は CST、操作PC は JST で 1 時間ずれている** | ログの突き合わせに注意 |
| 12 | **バッテリ交換で PC2 も機体も再起動する。** `/tmp` が消えて全部落ちる | 交換後は §3 から組み直す |

---

## 5. いまの状態（散らかっているもの）

### リポジトリ

- ブランチ **`Dev/Navigation02`**、2026-09-15 のコミット **5件。push はしていない**
- 未追跡のまま残してあるファイル2つ（扱い未定）:
  - `Navigation/nav2_option/map_20260907.pcd`（6.5MB、いまの地図の元データ）
  - `Navigation/map_wireless_dualcam_01_cleaned_no_ceiling.pcd`（101MB、軌跡が無く未使用）

### PC2（`~/g1_nav2/`）— 次回そのまま使える

| | |
|---|---|
| `pc2_humble/` | pixi の Humble + navigation2 1.1.20（aarch64）。**docker も sudo も不要** |
| `g1_ws/` | ビルド済み（3パッケージ） |
| `tools/` | リポジトリと同期済み |
| `start_nav.sh` / `start_record.sh` / `start_localizer.sh` | 起動スクリプト |
| `runs/` | **記録が 4 本残っている**（回収して `explain_run.py` に掛けること） |
| 動いたままかもしれないもの | Nav2 一式、`g1-sdk-bridge`。⚠️ **`/etc/default` は `G1_ARM=--arm` のまま**。ただしサービスは `enable` されていないので再起動後は自動起動しない |

⚠️ **次回接続したら真っ先に `grep '^G1_ARM=' /etc/default/g1-sdk-bridge` を見ること。**
`--arm` のままなので、`systemctl start` した瞬間に武装する。

### 操作PC

- `~/.local/bin/g1_heartbeat_sender`（ビルド済み。PATH が通っていなければフルパスで）
- heartbeat と RViz は停止済み

---

## 6. 積み残し（機体が無くてもできる）

- **`find_map_offset.py` の選択基準がおかしい。** 「重なりセル数」で最良を選ぶため、
  距離指標で見ると**恒等より悪い解を返すことがある**（実測: 最良 45% に対し恒等 69%）。
  距離指標で選ぶか、両方を報告して警告を出すべき
- **`backup` を含まない BT XML** を作って差し替える（落とし穴8の恒久対策）
- **`obstacle_min_range: 0.9` の副作用**の確認。0.9m より近い実在の障害物は
  新たにはマークされない。死角(0.91〜1.12m)と整合しているが、詰めきれていない
- PC2 の `runs/` 4本の回収と解析
- ⚠️ **会場が内蔵SLAM の推奨範囲を超えている件の扱い**（2026-09-16 に判明）。
  公式ドキュメントは適用範囲を「**25m×25m 未満。推奨範囲を超えないこと**」と明記している。
  **room_a は 28.5m×33.0m（自由空間 457m²）で既に超過。** 新しい点群
  `map_wireless_dualcam_01` は 58m×40m でさらに外れる。
  **巡回範囲を 25m×25m に絞るのか、自前 LIO（上記 ③ の (c)）へ移るのかの判断が要る。**
  16分停止とは別の話だが、**同じ「内蔵SLAM にどこまで依存するか」という問い**に繋がる
