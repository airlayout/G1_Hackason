# live/ — 無線のまま RViz2 で G1 を歩かせる

2026-09-16 に実機で通した手順を、そのまま走らせられる形にしたもの。

## 次回の再開（これだけ）

```bash
cd ~/git_research/physical_ai/G1_Hackason/Mapping/real

bash quickstart/live/preflight.sh          # ① 点検（何も起こさない）
bash quickstart/live/up.sh                 # ② 測位〜RViz2 まで
#    → ブラウザで http://192.168.123.201/

bash quickstart/live/legs.sh --arm         # ③ 足を繋ぐ（yes の確認が入る）
#    → RViz2 の「2D Goal Pose」でクリック

bash quickstart/live/down.sh               # ④ 止めて記録を Mac へ回収
```

初期姿勢は `up.sh` が**床の AprilTag から自動で取る**（大域探索も種も要らない）。
手で渡すなら `up.sh --init "X Y YAW_DEG"`。

---

## 構成（なぜこの形か）

```
機体 --> foxglove_bridge --(WebSocket/TCP・AP 越え)--> 中継(in)  --> コンテナの DDS --> RViz2
RViz2 --> コンテナの DDS --> 中継(back) --(WS)--> foxglove_bridge --> 機体の /goal_pose --> Nav2
Nav2 --> /cmd_vel --> cmd_vel_bridge --(UDP)--> loco_driver --> 足
```

### ⚠️ DDS は AP を越えられない（2026-09-16 に全部実測）

| PC2 の DDS | 同一ホスト内 | PC1 の LiDAR | コンテナから見える |
|---|---|---|---|
| **eth0 単独** | OK | **OK 130 枚** | ⛔ |
| wlan0 単独 | OK | **⛔ 0 枚** | OK |
| 2 NIC（peer=コンテナのみ）＝出荷時の `dds_dual.env` | **⛔ 2 件** | ⛔ | — |
| 2 NIC（+ `Peer localhost`）| OK 25 件 | **⛔ 0 枚** | — |

2 NIC にすると CycloneDDS が `selected interface "wlan0"` を選び、**eth0 側に居る PC1 の
LiDAR が見えなくなる**。`priority` 指定・`ExternalNetworkAddress`・PC1 を peer に追加、
いずれも効かなかった（CycloneDDS は 0.10.5＝多ホーム対応版）。

⇒ **DDS は eth0 に固定し、外へは WebSocket（foxglove_bridge）で出す。**
RViz2 は 2.38 コア、foxglove_bridge は **0.06 コア**（約 40 倍差）。

---

## 置いてあるもの

| ファイル | 何をするか |
|---|---|
| `preflight.sh` | 切れる順に上から点検。**何も起動しない**。NG なら止まる |
| `up.sh` | 測位 → `/scan` → Nav2 → 橋 → 姿勢ロガー → 中継 → 地図 → RViz2 |
| `legs.sh` | 足を繋ぐ／外す／状態を見る。**ここだけが機体を動かす** |
| `down.sh` | 危ない順に止め、記録を Mac へ回収する |
| `_common.sh` | 宛先・DDS 設定・自分を殺さない kill など |

中継の本体は 1 つ上に置いてある:

| ファイル | 向き |
|---|---|
| `../foxglove_to_ros.py` | 機体 → こちら（`/tf` `/scan` コストマップ `/plan` …） |
| `../ros_to_foxglove.py` | こちら → 機体（**`/goal_pose` だけ**） |

⚠️ `/cmd_vel` は**絶対に WebSocket で流さない**。速度指令の経路には
ウォッチドッグと発進ゲートが仕込んであり、AP 越しにすると切れたときに止まらない。

---

## 起動のたびに引っかかるもの（全部 2026-09-16 に踏んだ）

### 1. LiDAR の IMU が出ないことがある

**症状**: 測位が `Waiting for Odometry_loc...` から進まない。
**原因**: PC1 が `/utlidar/imu_livox_mid360` の writer を持ったままサンプルを 1 件も
書かない。ROS でも純正 SDK の生 DDS でも 0 件（＝購読側の問題ではない）。
点群だけは正常に 10 Hz で流れるので気づきにくい。

**手当て**: PC1 は SSH 全ポート閉で入れない。**機体の電源を入れ直す**しかない。
入れ直したら 200.0 Hz で復活した。09-10 の記録にも IMU が無く、**起動ごとに出たり
出なかったりする**。`preflight.sh` の 6. で捕まえる。

### 2. `cmd_vel_bridge.py` が wlan0 を掴む

**症状**: Nav2 は経路も速度も計算しているのに**足が一切動かない**。
**原因**: `CYCLONEDDS_URI` 未設定 → 自動選択 → wlan0。`/cmd_vel` は eth0 側なので
購読者 0 件のまま。`legs.sh` は必ず eth0 を渡す。
**確認**: `legs.sh --status` の **Subscription count が 1 以上**であること。

### 3. Mac の `bridge101` から `en8` が抜ける

ケーブルを抜き差しすると L2 の後段だけが死ぬ。`col0` は UP で IP も付いたままなので
「VM だけ届かない」としか出ない。`bash ../repair_bridge.sh`（sudo）。
⚠️ その合否判定は `192.168.123.164` を見るので、AP 経由の構成では
「戻らなかった」と出る。**`addm` 自体は効いている**ので、実際の疎通で判断する。

### 4. 記録を `/tmp` に置くと消える

2026-09-16 に電源が落ちて歩行 2 回分を失った。`up.sh` は `~/g1_runs/` に書く。

### 5. AP と PC2 の起動順

AP（OMEN の `g1-teleop-ap`）より先に PC2 が起動すると、PC2 の wlan0 が社内 Wi-Fi に
付いてしまう。⚠️ 無線 1 枚で AP と STA は同居できないので、OMEN 側も
`g1-teleop-ap` を上げると社内 Wi-Fi が切れる。

### 6. 自分のコマンド行を `pkill` で殺す

`pkill -f foo` はもちろん、**`pkill -f fo[o]` でも、呼び出し側のコマンド行に `foo` が
含まれていれば自分に一致する**。ssh が黙って落ちて「止まったのか分からない」状態に
なる。1 セッションで 4 回踏んだ。`_common.sh` の `pc2_kill` は文字列を分割して
組み立て、PID を取ってから kill する。

### 7. シェルごとの落とし穴

| どこ | 罠 |
|---|---|
| macOS の zsh | **`/dev/tcp` が無い**。疎通判定が常に失敗する。`#!/usr/bin/env bash` を守る |
| macOS | **`setsid` も `timeout` も無い**（`gtimeout` は Homebrew） |
| コンテナ | `/bin/sh` が **dash**。`/dev/tcp` は bash でだけ使える |
| PC2 | `jros2` は**シェル関数**。`timeout jros2 ...` は動かない |

### 8. 中継を書くときの QoS

- **`/tf_static` は `depth=1` にしない。** 静的 TF ごとに 1 通来るので、最後の 1 通しか
  残らず、後から開いた RViz2 で**鎖が切れる**（`tf2_echo map base_link` が無言になる）。
  `depth=100` にしてある
- **`/map` は中継できない。** latched なので橋が再送しない。コンテナ側で `map_server` を
  立てる（`up.sh` がやる）。同じ格子を AP 越しに何度も運ばずに済む

---

## 測る

`up.sh` が姿勢ロガーを常駐させ、`~/g1_runs/pose_<時刻>.txt`（TUM・10 Hz）に書く。
`down.sh` が Mac の `runs/_live/<時刻>/` へ回収する。

⚠️ **静止中も震えで積算が伸びる**（0.035 m/s 程度＝ 5 秒で 15〜20 cm）。
移動の判定閾値を 0.03 m/s にすると**立っているだけの時間が「歩いた」に数えられる**
（2026-09-16 に誤った数字を出した）。**0.06 m/s 以上**を使うか、5 秒ごとの表で
実際に動いた窓を目で切ること。

---

## 関連

- 取付の補正: `base_link` は**床基準**で定義し直してある（2026-09-16）。
  旧値 `rpy (177.93, 3.32, 0)` は内蔵 odom 基準で、床から **5.90 度**傾いていた。
  `preflight.sh` は見ないが、`up.sh` が起動直後に roll/pitch を出す。**±3 度**を
  超えたら取付を疑う（`preflight.sh` と同じ基準が `../preflight.sh` にも在る）
- タグ測位: `../apriltag/README.md`
- PC2 の環境（jammy を別ローダで動かす変則構成）: `../pc2/README.md`
