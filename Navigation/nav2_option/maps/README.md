# maps — 地図まわりの置き場（2026-09-24 にここへ集約した）

占有格子・点群・軌跡が4か所に散っていたのをまとめた。**地図に関するものはここを見る。**

```
maps/
├── grids/       占有格子(.pgm + .yaml)。**Nav2 が読むのはこれ**  ← git 管理下
├── clouds/      地図の素になった点群(.pcd)                        ← ⚠️ git 管理外(大きい)
└── trajectory/  地図を作るのに使った軌跡(.tum)                     ← git 管理下
```

## grids/ — Nav2 に渡す地図

| ファイル | 会場 | 由来 | 備考 |
|---|---|---|---|
| **`room_a_map_20260911`** | room_a | 9/11 の rosbag の `/unitree/slam_mapping/odom` + `map_wireless_dualcam_01_cleaned_no_ceiling.pcd` | **いまの既定。** 未知 27.4% / 最大連結 438.7m² |
| `room_a_map` | room_a | 9/07 の `map_20260907.pcd` | 旧。未知 40.5% / 最大連結 397.2m² |
| `room_b_map_Sorasta_20260923` | **Sorasta** | 9/23 の計測（⚠️ **内蔵SLAM の軌跡が無く `/dog_odom` で代用**） | 壁が点線状。[../findings/map_from_dog_odom_20260923.md](../findings/map_from_dog_odom_20260923.md) |
| `synthetic_room` | （合成） | 手書き | モックの配線確認用。自由空間が連結していることを保証 |
| `test_room` | （合成） | A-7 の初期出力 | ⚠️ レイトレーシング前なので**経路が引けない**。デモに使わない |

⚠️ **会場と地図が違うと §7 の照合は必ず失敗する。** room_a と Sorasta を取り違えないこと。

### 使い方

既定は `deploy/pc2_humble/start_nav.sh` と `g1up.sh` に書いてある。差し替えは:

```bash
~/g1_nav2/g1up.sh --map .../share/g1_navigation/maps/room_a_map.yaml
bash ~/g1_nav2/start_nav.sh "0 0 0" 0 2.0 "" .../maps/room_b_map_Sorasta_20260923.yaml
```

📌 **ROS からは `share/g1_navigation/maps/` に見える。** `g1_navigation` の
`CMakeLists.txt` が `maps/grids/` を相対参照して install している（実体を1つにするため。
`tools/g1_slam_odom_tf.py` と同じ方針）。
⚠️⚠️ **PC2 へ配置するときは `maps/` も送ること。** `g1_ws/src/` だけ送ると
`CMakeLists.txt` が `maps/grids が見つからない` で **ビルドに失敗する**。

## clouds/ — 地図の素（⚠️ git 管理外）

| ファイル | 大きさ | 何の素か |
|---|---|---|
| `map_wireless_dualcam_01_cleaned_no_ceiling.pcd` | 101MB / 880万点 | `room_a_map_20260911` |
| `map_20260907.pcd` | 6.3MB / 54万点 | `room_a_map` |
| `room_b_sorasta_20260923.pcd` | 8.4MB / 73万点 | `room_b_map_Sorasta_20260923` |

**消さないこと。** 地図を作り直すときに要る（grids と trajectory はリポジトリに入っているので、
作り直さないなら無くても動く）。

## trajectory/ — 軌跡（.tum）

点群だけでは「**何も無い場所**」が分からないので、レイトレーシングで自由空間を彫るのに使う。
理由は [../findings/map_rebuild_20260911.md](../findings/map_rebuild_20260911.md) §1。

⚠️ `pointcloud_to_occupancy_grid.py` は **2列目が x、3列目が y** しか見ない。
Odometry を素直に CSV に落とすと x が5列目になり、**全行スキップされて警告も出ない**。

## ここに無いもの

| | 場所 | なぜ |
|---|---|---|
| 生の記録（rosbag） | `../0911_robag/` `../0923_rosbag/` | 4.7GB。移すと rsync とディスクの都合が変わる |
| Mapping トラックの地図 | `Mapping/real/runs/*/map/` | **担当が別**。勝手に動かさない |
| nav3（別実装）の地図 | `Navigation/nav3/` | 同上 |
| シミュレータ用 | `IsaacSim_Env/maps/` | 用途が別 |
