# Sorasta の地図を 2026-09-23 の記録から作った（軌跡が `/dog_odom` しか無い）

会場が room_a から **Sorasta** に変わったので、9/23 に取った計測一式から地図を作った。

📌 **結論から: 作れたが、room_a の地図ほどの品質は無い。**
**内蔵SLAM の軌跡が記録に入っていなかった**ため、脚オドメトリ（`/dog_odom`）で
代用した。**壁が点線状に途切れる。** 今日の走行で問題が出たら、内蔵SLAM を上げて
**取り直すのがいちばん早い**（§5）。

---

## 1. 記録の中身

`/home/hironori/ダウンロード/g1_mapping_rgb_20260923T163637`（1.2GB）
→ `0923_rosbag/` にコピー（⚠️ git 管理外）

| トピック | 件数 | |
|---|---|---|
| `/dog_odom` | 175,247 | 1007 Hz。**今回の軌跡はこれ** |
| `/utlidar/cloud_livox_mid360` | 1,735 | 生 LiDAR（livox_frame） |
| `/utlidar/imu_livox_mid360` | 34,788 | |
| **`/unitree/slam_mapping/odom`** | **0** | ⚠️ **内蔵SLAM が動いていない** |
| **`/unitree/slam_mapping/points`** | **0** | 〃 |

- 記録長 174 秒、歩いた距離 35.4 m、**歩いた範囲は x 4.1m × y 4.3m だけ**
  （部屋の真ん中で小さく回っている。部屋自体は 23 × 25 m 見えている）
- RGB 1,448 枚（8.3 fps）。⚠️ **Navigation では使っていない**

---

## 2. 使った素材（どちらも機体不要で作れる）

| | |
|---|---|
| 点群 | `quicklook_cloud_odom.npz`（**odom で融合済み** 727,459 点）を `.pcd` に変換 |
| 軌跡 | `/dog_odom` を TUM 形式に変換 → `maps/trajectory/room_b_sorasta_20260923.tum` |

⚠️ **点群を bag から融合し直してはいない。** 収録スクリプトが作る quicklook の
融合結果をそのまま使った（`tools/quicklook_npz_to_pcd.py`）。
**SLAM 補正は入っていない**（収録側の note のとおり）。

そのための道具を2つ足した:

- [`tools/bag_odom_to_tum.py`](../tools/bag_odom_to_tum.py) — `.db3` を直接読んで
  Odometry を TUM にする。**ROS 不要**（CDR を自前で解く）。`--list` でトピックと件数も出る
- [`tools/quicklook_npz_to_pcd.py`](../tools/quicklook_npz_to_pcd.py) — npz の点群を `.pcd` に

---

## 3. 生成コマンド

```bash
python3 tools/bag_odom_to_tum.py 0923_rosbag/rosbag2 --topic /dog_odom --stride 20 \
    --out maps/trajectory/room_b_sorasta_20260923.tum
python3 tools/quicklook_npz_to_pcd.py 0923_rosbag/quicklook_cloud_odom.npz \
    --out maps/clouds/room_b_sorasta_20260923.pcd
python3 tools/pointcloud_to_occupancy_grid/pointcloud_to_occupancy_grid.py \
    maps/clouds/room_b_sorasta_20260923.pcd --trajectory maps/trajectory/room_b_sorasta_20260923.tum \
    --floor-z 0.33 --min-height 0.8 --max-height 1.8 --occupied-min-points 8 \
    --resolution 0.05 --out maps/grids/room_b_map_Sorasta_20260923
```

### ⚠️ 既定値のままでは「通れない地図」になる。2つ効かせる必要があった

**① `--floor-z` を手で与える。** 自動検出は **+0.911m** と答えるが、床候補（0.25m
セルごとの最低点）に平面を当てると **z = +0.020x +0.010y +0.330**（傾き 1.28°、
残差 std 0.125m）で、**本当の床は +0.33m**。融合された床が ±12cm 厚で散っているため、
自動検出が高いほうの塊を拾ってしまう。

**② `--occupied-min-points` を上げる。** ここがいちばん効いた。

| 設定（床 0.33m） | occupied | **最大連結の自由空間** |
|---|---|---|
| 既定（高さ 0.3〜1.8 / 1 点） | 31.1% | **8.9 m²** ← ほぼ通れない |
| 高さ 0.8〜1.8 / 1 点 | 22.5% | 32.4 m² |
| 高さ 0.8〜1.8 / **3 点** | 12.1% | 122.4 m² |
| **高さ 0.8〜1.8 / 8 点（採用）** | **4.2%** | **219.3 m²** |
| 高さ 0.8〜1.8 / 15 点 | **0.4%** | 639.7 m² ⚠️ **壁まで消えている。採らない** |

⚠️ **「最大連結の自由空間」だけで選んではいけない。** 点数の下限を上げるほど
この指標は良くなるが、**壁が消えれば自由空間は当然つながる**。15 点は occupied が
0.4%（＝2.9 m²）しか残らず、**壁の無い地図**になる。必ず絵を見て決めること。

---

## 4. できた地図（`room_b_map_Sorasta_20260923`）

| | **room_a（9/11・SLAM 軌跡）** | **Sorasta（9/23・dog_odom）** |
|---|---|---|
| 大きさ | 23.8 × 37.5 m | 25.4 × 27.5 m |
| occupied | 18.1% | **4.2%** |
| free | 54.5% | **36.0%** |
| unknown | 27.4% | **59.7%** |
| 最大連結の自由空間 | 438.7 m² | **219.3 m²** |
| 見た目 | 壁が連続。什器の島がはっきり出る | **壁が点線状。内部に粒状のノイズが散る** |

**unknown が 6 割**なのは、**歩いた範囲が 4m 四方しか無い**ため。レイトレーシングは
「実際に立った場所から見えた範囲」しか彫れないので、これは点群の問題ではなく
**歩き方の問題**。

⚠️ **壁が点線状だと、Nav2 の大域プランナが壁の隙間を通る経路を引くことがある。**
局所 costmap（実 LiDAR）が止めるはずだが、**そこに頼る設計にはしていない。**

---

## 5. 取り直すなら（こちらが本筋）

**内蔵SLAM を上げてから同じことをすれば、room_a と同じ品質になる。**

```bash
cd ~/g1_nav2/tools && python3 send_slam_api.py 1801   # ⚠️ これを忘れたのが今回の原因
# 収録スクリプトを回して 3〜5 分歩く。⚠️ **部屋の端まで歩くこと**（unknown が減る）
python3 tools/bag_odom_to_tum.py <bag> --topic /unitree/slam_mapping/odom --out ...
```

📌 **歩き方のほうが効く。** 今回 unknown が 6 割なのは 4m 四方しか歩いていないため。
**外周を一周するだけで連結した自由空間は大きく増える**（9/11 は 119m 歩いて 438m²）。

⚠️ 収録前に `--list` で **`/unitree/slam_mapping/odom` が増えているか**を確かめること。
0 件のまま歩いても、今回と同じところに戻ってくる。
