# Mapping（G1による3Dマップ計測・作成）

G1 に載せた 3D LiDAR で周囲を計測し、**自己位置推定と 3D 地図構築（SLAM）**を行う。

## 構成

```
Mapping/
├── common/          sim / real 共通のロジック（numpy / scipy のみに依存）
│   ├── lidar_spec.py    LiDAR のビーム配置の定義
│   ├── pointcloud.py    点群のダウンサンプル・蓄積・PLY 出力
│   ├── icp.py           ICP によるスキャンの位置合わせ
│   └── slam.py          scan-to-map SLAM 本体
├── sim/             MuJoCo 上での計測（詳細は sim/README.md）
│   ├── scene.py         検証用の部屋シーンの生成
│   ├── mujoco_lidar.py  レイキャストによる LiDAR の模擬
│   └── record_scans.py  [エントリポイント] 歩かせながらスキャンを収録
├── real/            実機での計測（動作未検証。詳細は real/README.md）
│   ├── discover_topics.py     [エントリポイント] DDS トピックの探索
│   ├── record_scans_real.py   [エントリポイント] LiDAR 点群の収録
│   └── pointcloud2.py         PointCloud2 -> numpy 点群
└── run_slam.py      [エントリポイント] スキャン列 -> 3D 地図（sim / real 共通）
```

**実機では G1 を手動で歩かせ、操作 PC は Ethernet で点群を受け取るだけ**
（ロボットには一切コマンドを送らない）。そのため実機側は収録スクリプトだけで済み、
地図化は sim と同じ `run_slam.py` を使う。

**計測と地図化を分けてある。** `record_scans.py` が `scans.npz` を吐き、
`run_slam.py` がそれを読んで地図にする。SLAM のパラメータを変えるたびに
シミュレーションを回し直さずに済むうえ、実機でも「歩いて計測する」と
「持ち帰って地図にする」は別作業になるため、同じ切り方にしている。
実機側は `scans.npz` を出しさえすれば、地図化は同じ `run_slam.py` が処理する。

## 実行方法

`G1_HuggingFace/venv/`（操作PC側）をそのまま使う。追加インストールは不要。

シミュレーション:

```bash
# 1. 部屋の中で G1 を歩かせ、LiDAR スキャンを収録する（約 90 秒 + 起動時間）
./G1_HuggingFace/venv/bin/python Mapping/sim/record_scans.py --scan-hz 8

# 2. 収録したスキャンから自己位置推定 + 地図構築を行う
./G1_HuggingFace/venv/bin/python Mapping/run_slam.py
```

実機（手順の詳細と注意点は [real/README.md](real/README.md)）:

```bash
bash Common/network/setup_ethernet_for_g1.sh          # 1. ケーブル接続
./G1_HuggingFace/venv/bin/python Mapping/real/discover_topics.py     # 2. トピック探索
./G1_HuggingFace/venv/bin/python Mapping/real/record_scans_real.py \
    --topic <見つけたトピック> --duration 90          # 3. 手動歩行させながら収録
./G1_HuggingFace/venv/bin/python Mapping/run_slam.py --scans Mapping/data/scans_real.npz
```

出力は `Mapping/data/` 配下（`.gitignore` 済み。再収録できるので追跡しない）:

| ファイル | 内容 |
|---|---|
| `scans.npz` | 収録したスキャン列 |
| `map.ply` | 3D 地図（高さで色分け済み。MeshLab / CloudCompare / open3d で開ける） |
| `map_trajectory.ply` | 推定した軌跡 |

## 検証結果（2026-08-25、シミュレーション）

10m × 8m の部屋（壁 + 柱 2 本 + 箱 4 個）を G1 に歩かせて計測した実測値:

| 項目 | 値 |
|---|---|
| 収録 | 689 フレーム / 748 万点（約 90 秒の巡回、8Hz） |
| LiDAR | 32ch × 360 方位 = 11,520 レイ/フレーム、有効点 平均 10,860 |
| 地図 | 154,490 点（5cm ボクセル） |
| 地図化の処理時間 | 48.5 秒（70 ms/frame） |
| ICP が失敗して等速度モデルに落ちたフレーム | 0 / 689 |
| **軌跡の誤差 (ATE RMSE)** | **0.020 m**（最大 0.064 m、移動距離 16.7m） |

地図の寸法を部屋の実寸と突き合わせた結果（**真値は SLAM に渡していない**）:

| | 真値 | 地図から計測 | 誤差 |
|---|---|---|---|
| 部屋の内寸 x | 9.90 m | 9.94 m | +0.04 m |
| 部屋の内寸 y | 7.90 m | 7.94 m | +0.04 m |

壁 4 面・柱 2 本・箱 4 個がいずれも正しい位置に再構成され、推定軌跡は真値と
目視でほぼ重なる。10m スケールで 0.4% の誤差。

なお歩行ポリシーの旋回が遅く（`remote.rx=-0.5` で 6.6 deg/s）、90 秒の巡回でも
実際に歩けた距離は 16.7m にとどまる。それでも部屋全体が撮れているのは、
LiDAR が水平 360 度・測距 40m で、部屋の中央からでも全ての壁が見えているため。

## 方式

**scan-to-map ICP。** 1 フレーム前のスキャンとだけ合わせる scan-to-scan は
毎フレームの誤差がそのまま積算されるため、蓄積済みの地図全体に対して合わせている。
初期値は等速度モデル（直前フレーム間の相対移動をそのまま外挿）で与える。

ICP は point-to-point 版を自前で実装した（`common/icp.py`）。point-to-plane の
ほうが収束は速いが地図側の法線を毎フレーム再計算する必要があり、scipy だけで
書くと法線推定のほうが重くなる。壁・柱・箱で幾何拘束が十分な屋内が対象なので
point-to-point で足りると判断した。

**open3d は使っていない。** `G1_HuggingFace/venv/` は `SimpleWalk/` の
動作確認済みフローと共有しており、点群処理のためだけに重い依存を足して
その venv を壊すリスクを避けた。出力は標準の PLY なので、可視化したければ
別環境で開けばよい。

**ループクロージャは入っていない**（一周して戻ったときに軌跡全体を補正する処理）。
そのため長距離・長時間になるほどドリフトは残る。まずは地図が形になるかを
確かめる段階のため。

## 未検証・今後

- **実機での計測**。コードは書いたが**実機で動かしていない**。
  LiDAR の DDS トピック名・点群の座標系・LiDAR が最初から ON かどうかが
  未確定で、これらは実機に繋がないと分からない（`real/README.md` に一覧）。
- **ループクロージャ**。長時間の計測でドリフトが問題になったら着手する。
- **占有格子 / octomap への変換**。ナビゲーションに使うなら必要になる。
  `SLAM/` との役割分担はその段階で整理する。

## 関連

- 環境構築: [SETUP.md](../SETUP.md) / [G1_HuggingFace/README.md](../G1_HuggingFace/README.md)
- 通信・疎通確認: [Common/network/README.md](../Common/network/README.md)
- 実機歩行の例: [SimpleWalk/](../SimpleWalk/README.md)
- 失敗の記録: [FAILURES.md](FAILURES.md)
