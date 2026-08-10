# G1 3D LiDAR ナビゲーション環境（SimEnv3D）

Unitree G1 に Livox Mid-360 相当の 3D LiDAR を載せ、3D voxel マップを作って
自律ナビゲーションさせる環境。`SimEnvTest/`（2D LiDAR 版）の後継。

`SimEnvTest/` とは**完全に独立**している（`g1_twin/` などをコピーして持っている）。
動作実績のある 2D 環境を壊さずに 3D 化を進めるため。

## 現在の状態

| ステップ | 内容 | 状態 |
|---|---|---|
| 1 | レイキャストのコスト実測 | **完了（重大な発見あり・下記）** |
| 2 | `lidar3d.py`（Mid-360 相当） | 実装済み・単体テスト通過 |
| 3 | `/points` (PointCloud2) 配信 | 実装済み・エンコード検証済み |
| 4 | `"base"` vs `"yaw"` の歩行中比較 | **未実施（要 Isaac Sim 実走行）** |
| 5 | `build_map_3d.py` | 実装済み・座標変換を検証済み |
| 6 | `octomap_server` 統合 | **パイプライン検証済み**（合成点群で壁を再現） |
| 7 | 足元の障害物で固着しないか実走行確認 | 未着手 |

検証の状況:

| テスト | 内容 | 結果 |
|---|---|---|
| `test_lidar3d_math.py` | 前傾クォータニオンと座標変換（7 件） | 通過 |
| `test_pointcloud_encoding.py` | PointCloud2 の往復（公式デコーダで照合） | 通過 |
| `test_build_map_3d.py` | yaw・平行移動・高さフィルタ・保存 | 通過 |
| `test_octomap_pipeline.py` | octomap が既知の壁を 3.05 m に再現 | 通過 |
| `check_lidar3d.py` | Isaac Sim 上での点群検証 | **未実行** |

`check_lidar3d.py` はコスト実測が GPU を使い切っていたため未実行。
実測が終わったら次に走らせること（下記「次にやること」参照）。

## 仕様（決定事項）

| 項目 | 決定 |
|---|---|
| センサ | **Livox Mid-360 相当**（垂直 -7〜+52 度、水平 360 度） |
| 搭載姿勢 | **前傾させる**（既定 20 度）。下記参照 |
| `ray_alignment` | **`"base"`**（3D は 2D と逆。下記参照） |
| ROS 出力 | `/points` (PointCloud2) |
| 3D マップ | **`octomap_server`**（`nvblox` はこの環境に無い） |
| 地図作成 | **`build_map_3d.py`（新規・別名）**。`build_map.py` は触らない |
| Nav2 連携 | octomap の `/projected_map` を costmap に食わせる |

### 前傾させる理由
Mid-360 の下向きは **-7 度しかない**。地上 1.1 m に水平搭載すると足元は
水平から 7 度 = **8.9 m 先の床**しか見えず、「パレットに引っかかる」という
既存の課題が解決しない。20 度前傾させると下向き -27 度を確保でき、
2.2 m 先の床から見える。実機の G1 も Mid-360 を前傾させて搭載している。

**走査パターンは近似である。** Mid-360 は非リピート型ロゼッタ走査だが、
`LidarPatternCfg` は等間隔グリッドしか作れない。FOV と点数を合わせた
等間隔グリッドで代用している。マッピング用途では実用上問題にならない。

### `ray_alignment` は 2D と逆にする
2D では `"yaw"` が必須だった（歩行の pitch/roll がレイに乗ると距離が暴れ、
SLAM が壊れた）。**3D では `"base"` が正しい。** 3D 点群は各点が 3 次元座標を
持つため傾いても点は正しい位置に落ちる（2D は高さ情報を捨てるから壊れた）。
実機の Mid-360 も胴体に固定されるので、これが実挙動に近い。

ただし歩行の揺れが octomap にノイズとして入る可能性はあるので、
ステップ 4 で実測して検証する。

## 最大の発見：mesh_prim_paths の指定方法で 146 倍変わる

**`MultiMeshRayCaster` に親パスを渡してはいけない。配下の Mesh を個別に列挙する。**

同じ Warehouse（Mesh 3473 個）・同じ 11520 ビームでの実測:

| `mesh_prim_paths` の指定 | 1 スキャン |
|---|---|
| `["/World/Warehouse"]`（親パス 1 つ） | **467.0 ms** |
| Mesh を個別に 3473 個列挙 | **3.2 ms** |

親パスを渡すと毎スキャンで配下の prim を走査し直しているものと思われる。
`lidar.py` の `expand_mesh_paths()` がこの列挙を行う。

### これは 2D LiDAR（SimEnvTest）にも当てはまる
`SimEnvTest/src/g1_twin/lidar.py` は `["/World/Warehouse"]` を渡しているため、
**1 スキャン 74 ms の原因はこれだった。**

`SimEnvTest/README.md` の以下の記述は**誤りなので訂正が必要**:

> レイキャストが重い。1 回 74 ms かかり 50Hz（20 ms）に収まらない。
> ビーム数を 1/4 に減らしても 45 ms までしか下がらず、コストはビーム数ではなく
> **レイキャスト演算自体に支配される**。

ビーム数を減らしても効かなかったのは事実だが、原因はレイキャスト演算ではなく
prim の走査だった。ビーム数を 360 → 11520 と 32 倍にしても時間が変わらないという
実測がその証拠（演算が支配的ならビーム数に比例して増えるはず）。

**この修正により、2D 側も 10Hz への間引きが不要になり実時間比が大きく改善する
見込み。** 長距離ナビゲーションの成功率が低い原因の一つ（実時間比 0.23x）が
解消する可能性がある。

### コスト実測の結果（ステップ 1）

Mesh を個別列挙した状態で、層数 × 水平ビーム数を振った実測値（Warehouse、
中央値 [ms]）。全条件が **50Hz（20 ms）に収まる**。

| 層 \ 水平 | 90 | 180 | 360 | 720 |
|---|---|---|---|---|
| **1** | 2.69 | 2.67 | 2.66 | 3.46 |
| **4** | 2.65 | 2.74 | 2.87 | 2.83 |
| **8** | 2.70 | 2.80 | 2.93 | 3.17 |
| **16** | 2.81 | 2.94 | 3.13 | 3.52 |
| **32** | 2.83 | 3.07 | 3.42 | — |

**ビーム数を 128 倍（90 → 11520）にしても 2.7 ms → 3.4 ms** しか増えない。
レイキャストは完全に固定費に支配されており、**3D 化のコストは実質ゼロ**。

当たり率は 1 層で 96.1%、4 層以上で 99.0〜99.4%。多層化すると床や
棚の上段にも当たるため当たり率が上がる。

**採用する構成: 32 層 × 360 ビーム（11520 点、3.42 ms）。**
理由は、
- 50Hz（20 ms）に対して 6 倍の余裕がある
- 垂直 59 度を 32 層 = 約 1.9 度刻み。Mid-360 の実解像度に近い
- 水平 360 は 2D 版と同じ 1 度刻みで、既存の地図と比較しやすい

`lidar3d.py` の `CHANNELS` / `HORIZONTAL_BEAMS` が既にこの値になっている。
より高密度が必要になれば 720 水平（3.5 ms）へ上げる余地もある。

### 誤った計測でハマった点（記録）
最初の実装では 24 条件ぶんのセンサを 1 プロセス内に全部構築し、順に測っていた。
**`MultiMeshRayCaster` はシーンに登録された全センサがまとめて更新される**ため、
1 条件を測っているつもりで毎回 24 センサ分のコストを払っていた。
結果「層数を 64 倍にしても時間が変わらない」という物理的にありえない数字が出た。

センサは **1 プロセスに 1 つだけ**構築すること。条件を変えるにはプロセスを
作り直す（`run_cost_map.sh` がそうしている）。

## 実行方法

環境設定は `SimEnvTest` と同じ（`env.sh` をコピーしてある）。

```bash
cd /home/spacedata/isaac_dev/G1/SimEnv3D
source env.sh
```

### コスト実測

```bash
bash run_cost_map.sh              # 全グリッド（24 条件、30〜50 分）
bash run_cost_map.sh --quick      # 実用候補のみ（8 条件）

# 単一条件だけ測る
"$ISAAC_SIM/python.sh" src/probe_lidar3d_cost.py --viz none --channels 16 --beams 360
```

結果は `logs/cost_map.tsv` に TSV で溜まる。

### 検証スクリプト

```bash
# パターン生成が channels を反映しているか（Isaac Sim 不要）
python3 src/probe_pattern_check.py

# メッシュ指定方法とコストの関係
"$ISAAC_SIM/python.sh" src/probe_mesh_cost.py --viz none --num-meshes 0      # 親パス
"$ISAAC_SIM/python.sh" src/probe_mesh_cost.py --viz none --num-meshes 99999  # 個別
```

## はまり点：前傾を二重に適用しない／忘れない

`/points` は **真のセンサ座標系**（前傾も歩行の pitch/roll も含む）で配信される。
`lidar3d.read_point_cloud()` がセンサのワールド姿勢 `quat_w` の**逆回転**で
変換しているためで、yaw だけでなく前傾も既に除かれている。

したがって点群を使う側は**必ず前傾を適用して**ワールドへ戻す:

| 消費者 | 前傾の適用 |
|---|---|
| `octomap_server` | `base_link -> lidar3d` の静的 TF が適用（正しい） |
| `build_map_3d.py` | コード内で明示的に適用（`FORWARD_TILT_DEG`） |

**実装中に実際にこのバグを入れた。** `build_map_3d.py` で yaw だけを適用し、
前傾を忘れていた。10 m 先の点で**高さが 3.42 m**（水平は 0.60 m）ずれる。
2 つの地図が食い違う形になるため、octomap と自前地図を比べれば検出できる。

2D 版の「yaw を足してはいけない」（`ray_alignment="yaw"` なので二重適用に
なる）とは**逆の注意点**なので混同しないこと。

`FORWARD_TILT_DEG` は `lidar3d.py` と `build_map_3d.py` の両方にあり、
**変えるときは両方揃えること**（片方だけ変えると地図が食い違う）。

## octomap のはまり点：2D 投影が空のまま

**`incremental_2D_projection` は `false` にすること。** `true` だと 3D 地図は
正常に作られるのに `/projected_map` が全セル -1（未知）のまま埋まらない。

実測（合成した壁 3200 点を 30 回流した結果）:

| 設定 | voxel | `/projected_map` |
|---|---|---|
| `true` | 800 個 | -1 のみ（**占有 0 セル**） |
| `false` | 800 個 | 占有 40 セル・空き 647 セル（正しい） |

Nav2 の costmap は `/projected_map` を使うため、これに気付かないと
「3D 地図は動いているのに Nav2 が障害物を認識しない」という切り分けの
難しい状態になる。3D 側の数値は正常なのでそこだけ見ていると気付けない。

### パラメータ名は実物で確認すること
ROS はパラメータ名を間違えても**黙って無視する**。実際に間違えていた:

| 書いていた名前 | 正しい名前 |
|---|---|
| `filter_ground` | **`filter_ground_plane`** |
| `publish_2d_projected_map` | **存在しない**（`/projected_map` は常に出る） |

`ros2 param list /octomap_server` で実物を確認した（2026-08-10）。
入力トピックは **`/cloud_in` 固定**なので `run_octomap.sh` で remap している。

## 構成

```
SimEnv3D/
├── README.md
├── env.sh                      # Isaac Sim + IsaacLab + ROS 2（SimEnvTest からコピー）
├── run_cost_map.sh             # コストマップ実測（条件ごとにプロセスを作り直す）
├── run_octomap.sh              # octomap_server で 3D 地図を作る
├── config/
│   └── octomap.yaml            # octomap の設定（パラメータ名は実物で確認済み）
├── src/
│   ├── build_map_3d.py         # 真値 odom + 3D 点群から voxel 地図を作る（新規）
│   ├── publish_map_odom_tf.py  # map -> odom を恒等変換で流す（コピー）
│   ├── run_g1_twin.py          # エントリポイント（--lidar3d を追加）
│   │
│   │   # --- 検証（Isaac Sim 必要）---
│   ├── check_lidar3d.py        # 3D 点群の検証（足元・上方が見えているか）
│   ├── probe_lidar3d_cost.py   # 3D LiDAR のコスト実測（単一条件）
│   ├── probe_mesh_cost.py      # メッシュ指定方法とコストの関係
│   ├── inspect_warehouse.py    # Warehouse の Mesh 構造
│   │
│   │   # --- 単体テスト（Isaac Sim 不要）---
│   ├── test_lidar3d_math.py       # 前傾と座標変換（7 件）
│   ├── test_pointcloud_encoding.py # PointCloud2 の往復
│   ├── test_build_map_3d.py       # 地図生成の座標変換
│   ├── test_octomap_pipeline.py   # octomap の設定・remap・TF
│   ├── probe_pattern_check.py     # パターン生成が channels を反映するか
│   │
│   └── g1_twin/
│       ├── lidar.py            # 2D。expand_mesh_paths() を追加（146 倍高速化）
│       ├── lidar3d.py          # 3D LiDAR（Mid-360 相当・前傾）【新規】
│       ├── ros_bridge.py       # /points 配信と lidar3d の TF を追加
│       └── runner.py           # enable_lidar3d を追加
├── checkpoints/                # 歩行ポリシー（コピー済み・再取得不要）
├── maps/
└── logs/
    └── cost_map.tsv            # コスト実測の結果
```

## 使い方（3D 地図を作る）

```bash
# 端末 1: 3D LiDAR 付きで自動巡回させる
source env.sh
"$ISAAC_SIM/python.sh" src/run_g1_twin.py --viz none \
    --lidar3d --command-source patrol --max-steps 90000

# 端末 2A: octomap で 3D voxel マップを作る
bash run_octomap.sh
# 保存: ros2 run octomap_server octomap_saver_node -f maps/warehouse_3d.bt

# 端末 2B: あるいは自前で voxel 地図を作る（スライス画像が出る）
source env.sh
python3 src/build_map_3d.py --duration 1500 --output maps/warehouse_3d
```

`build_map_3d.py` は高さ帯ごとのスライス画像
（`maps/warehouse_3d_slices/`）を出す。足元・胴体・頭上の 3 帯に分けてあり、
**2D LiDAR では見えなかった足元と頭上が写っているか**を目視で確認できる。

## 次にやること

1. **`check_lidar3d.py` を実行する**（コスト実測で GPU が埋まっていて未実行）
   ```bash
   source env.sh
   "$ISAAC_SIM/python.sh" src/check_lidar3d.py --viz none
   ```
   足元（地上 0.3 m 未満）の点が取れているか＝前傾が効いているかを確認する。

2. **ステップ 4: 歩行中の `"base"` vs `"yaw"` を比較する。**
   3D では `"base"` が正しいと判断したが、歩行の揺れが octomap に
   ノイズとして入らないかは実測していない。

   **既知の近似がここに絡む。** `base_link -> lidar3d` の静的 TF は前傾しか
   持たないため、歩行中の pitch/roll は TF に反映されない。一方 `/points` は
   その pitch/roll が除かれた座標で来るので、octomap 側では歩行の揺れが
   そのまま誤差になる。対策の候補:
   - 動的 TF にして毎フレームの姿勢を流す（正確だが TF の負荷が増える）
   - `ray_alignment="yaw"` にして pitch/roll をレイに乗せない（2D と同じ発想。
     ただし実機の挙動から離れる）
   - 誤差が小さければ放置する

   まず実測してどれが必要か決めること。

3. **実走行でパレットへの固着が消えるか確認する**（ステップ 7）。
   これが 3D 化の目的そのもの。

4. **`SimEnvTest` 側にも `expand_mesh_paths` を適用する**か決める。
   2D 環境の実時間比が大きく改善する見込みだが、動作実績のある環境なので
   適用は別途判断すること。

## 引き継いでいる注意点

`SimEnvTest` で踏んだ落とし穴はそのまま当てはまる。特に:

- **エントリポイントを import しない**（AppLauncher が二重起動して無言で落ちる）
- **`finally` で `simulation_app.close()` を呼ばない**（例外が隠れる）
- **地図は必ず画像で目視する**（数値だけでは壊れた地図に気付けない）
- **クォータニオンは ROS が `(x,y,z,w)`、IsaacLab の `OffsetCfg.rot` は `(w,x,y,z)`**
- **起動に 2〜5 分かかる**（タイムアウトは長めに取る）
- `data.*[0]` は warp 配列なので `wp.to_torch()` が必要
