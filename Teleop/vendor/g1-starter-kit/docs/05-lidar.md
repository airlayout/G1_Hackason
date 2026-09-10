# LiDAR（ロボットが見ている世界）

G1 の頭部 LiDAR（Livox）の点群と IMU をライブ表示し、記録します。

---

## ⚠️ 最初に：機種を必ず自動検出してください

G1 に載る Livox には **Mid-360** と **Mid-360S** があり、**見た目では区別できません**。そして**ドライバの設定ファイルの形式が両者で違います**。

| 機種 | dev_type | 設定の形 |
|---|---|---|
| Mid-360 | 9 | `"MID360"` キー / `host_net_info` は**オブジェクト**（チャネル別に IP） |
| Mid-360S | 35 | `"Mid360s"` キー / `host_net_info` は**配列**（`host_ip` 1 つ） |

間違った方を使うと、ドライバは

```
[INFO] Init lds lidar success!
GetFreeIndex key:livox_lidar_2021370048.
```

まで出して**そこで止まります**。トピックも作られず、エラーも出ません。原因が非常に分かりにくいので、まず検出してください。

> 補足: ログの `GetFreeIndex key:livox_lidar_<数字>` は設定ファイルの IP を整数に直しただけの値で、**機器からの応答ではありません**。「接続できた証拠」と読み違えやすいので注意してください。

```bash
python3 tools/lidar_probe.py                 # 何が繋がっているか見る
python3 tools/lidar_probe.py --write-config  # 検出結果から設定を生成する
```

出力例:

```
  1 台の LiDAR を検出しました

    機種       : Mid-360S  (dev_type=35)
    IP         : 192.168.123.120
    シリアル   : ARMCPxxxxxxxxxx
    設定形式   : MID360s

  この機種の設定形式は MID360s です。
  LiDAR のデータ送信先（このPC）: 192.168.123.222

  --write-config を付けると config/livox/active.json を生成します
```

LiDAR の IP と機種は `config/livox/active.json` に直接書かれるので、`config/g1.env`
に転記する必要はありません。複数台つながっている場合は `--lidar-ip` か `--serial`
で 1 台を指定してください（応答順で勝手に選ばないようにしてあります）。

### 仕組み

Livox SDK2 と同じ手順です。ホストが UDP **56000** 番へ探索要求（`cmd_id=0x0000`）を
1 秒間隔でブロードキャストし、LiDAR がそれに応答します。**LiDAR は自発的には名乗りません**。

応答は CRC-16（先頭18バイト）と CRC-32（データ部）まで検証してから採用するので、
無関係なパケットを機器情報と誤認することはありません。

問い合わせるだけなので、LiDAR にもロボットにも一切書き込みません。

### 検出できないとき

| 原因 | 対処 |
|---|---|
| **ドライバが起動中で 56000 を占有している** | 先に `lidar_view.sh` を止める |
| **LiDAR が既に他ホストに接続済み**（探索要求に応答しない） | ロボットを再起動し、起動直後に実行する |
| 有線が繋がっていない / ロボットの起動が未完了 | `./scripts/preflight.sh` で確認 |
| このPC が同じサブネットにいない | [02-network.md](02-network.md) の有線設定を確認 |

---

## 表示する

```bash
./scripts/lidar_view.sh              # 点群 + IMU 波形
./scripts/lidar_view.sh --decay      # 点を 5 秒残す
./scripts/lidar_view.sh --no-imu     # PlotJuggler を出さない
./scripts/lidar_view.sh --fake       # 実機なしで画面を確認（動作確認用）
```

`Ctrl+C` で RViz も PlotJuggler もまとめて止まります。

| トピック | 内容 |
|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2`。既定 10 Hz・約 20000 点/フレーム |
| `/livox/imu` | `sensor_msgs/Imu`。200〜350 Hz |

### `--decay` の使いどころ

点を 5 秒残すと、動かしたときに点が「積もって」いきます。ただし**ロボットが動くとズレます**（同じ場所を別の姿勢から見た点が重なるため）。

これは不具合ではなく、**SLAM が必要な理由そのもの**です。「点を貯めるだけではズレる。だから自己位置を推定しながら重ねる仕組み（SLAM）が要る」という説明に使えます。

### IMU の波形を出す

PlotJuggler は起動するだけではトピックを購読しません。3 手順で選びます。

1. 左上の **`Streaming`** → **`ROS2 Topic Subscriber`** → `Start`
2. `/livox/imu` にチェックして OK
3. 左のツリーから `angular_velocity` / `linear_acceleration` の各軸をグラフ領域へドラッグ

うまくいかない場合の代替:

```bash
source scripts/lib.sh && load_config && use_ros
ros2 run rqt_plot rqt_plot /livox/imu/angular_velocity/x
```

---

## 搭載向き（点群が天地逆に見えるとき）

G1 の LiDAR は**上下逆さまに搭載されている**ことがあります。そのままだと RViz で天井と床が入れ替わって見えます。

このキットは表示用の親フレーム `viz_base` を立て、`livox_frame` を 180 度回して見せています（`config/g1.env` の `G1_LIDAR_FLIP`）。

実データで判定できます。

```bash
./scripts/lidar_view.sh --no-rviz &          # ドライバだけ起動
python3 tools/lidar_probe.py --orientation   # ROS 環境を source した端末で
```

```
  最も点が集中する z = +0.82 m（床とみなす）
  → 床がセンサより上にある = **上下逆さま搭載**
     config/g1.env は  G1_LIDAR_FLIP=true  が正しい
```

> **点群だけを回転させる設定（ドライバ側の extrinsic）は使わないでください。** 点群と IMU は同じセンサ座標系で整合が取れているので、点群だけ回すと SLAM 用途で不整合になります。表示用の TF で回すのが正解です。

---

## 記録と再生

```bash
# 別ターミナルで（先に lidar_view.sh を起動しておく）
./scripts/record_bag.sh room1
```

`Ctrl+C` で停止。保存先は `bags/room1_<日時>/`。

⚠️ **点群は 1 分あたり 300MB 程度になります**（実測）。1 回 2〜3 分を目安にしてください。残量チェックは入っていますが、長く録ると一気にディスクを食います。

```bash
du -sh bags/*

source scripts/lib.sh && load_config && use_ros
ros2 bag info bags/room1_20260817_190831
ros2 bag play bags/room1_20260817_190831      # RViz は lidar_view.sh のものを使う
```

再生時は `lidar_view.sh` を起動しておけば、同じ画面がそのまま再現されます。**bag さえ録ってあれば、ロボットが無い場所でも後から解析できます。**

---

## SLAM に使う bag を録るとき

FAST-LIO などの SLAM 実装は、点群を `PointCloud2` ではなく **Livox の独自形式（CustomMsg）**で要求することがあります。その場合はこちらで録ってください。

```bash
./scripts/lidar_view.sh --custommsg
```

RViz では表示されなくなります（`PointCloud2` ではないため）。SLAM 用の記録専用と考えてください。

---

## 実機が無いとき

```bash
./scripts/lidar_view.sh --fake
```

部屋の形をしたダミー点群（10 Hz）と IMU（200 Hz）を配信します。RViz の設定、bag の記録・再生、PlotJuggler の操作をロボット無しで練習できます。

実機ドライバと同じ QoS（Reliable）で配信しているので、購読側の挙動も実機と同じになります。**デモで見せるときは「これは実機データではない」と必ず断ってください。**
