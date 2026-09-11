# `mola_relocalization` の試作（段 7）

「追跡が外れたら戻す」＝ Global Localization を、MOLA 純正の SE(2) 尤度探索で組む。
2026-09-11 にここまで実測した。

## なぜ自作が要るのか

- 稼働中の MOLA が出している `/relocalize_near_pose` は **探索をしない**。
  実装（`LidarOdometry_Relocalization.cpp`）は渡された姿勢を `FixedPose` で置き、
  共分散を ICP の対応づけ閾値に換算するだけ。
  「`/initialpose` は引き込まない。投げた場所に居座る」の正体はこれで、バグではなく仕様
- 探索は `mola_relocalization` が担うが、**C++ ライブラリしか無い**
  （ノードもサービスも Python バインディングも無い。apt / `dpkg -L` で確認）

## 建て方（コンテナの中。Mac に MRPT は無い）

```bash
docker exec -u ubuntu rviz bash -c 'source /opt/ros/humble/setup.bash && \
  cmake -S /work/G1_Hackason/Mapping/real/quickstart/reloc -B /tmp/reloc_build \
        -DCMAKE_BUILD_TYPE=Release && cmake --build /tmp/reloc_build -j4'
```

`ros-humble-mola-relocalization` は `start_rviz_mac.sh` が入れる。

### 建てるときに踏んだ穴

| 症状 | 原因 | 直し方 |
|---|---|---|
| `cannot find -lmp2p_icp_map` | cmake の的が名前空間つき | `mola::mp2p_icp_map` と書く |
| `load_from_file` が `class 'mola::HashedVoxelPointCloud' is not registered` | `map.mm` の層がこの型で、登録は `mola_metric_maps` が持つ | `find_package(mola_metric_maps)` してリンクする |
| 層に `CPointsMap` が無い | `map.mm` の層は `HashedVoxelPointCloud`（`CPointsMap` を継承していない） | 尤度のつまみは型ごとに別の場所にある。両方見る |

## 使い方

```bash
# 1. 記録から 1 スキャンを base_link 系で出す（Mac 側の venv）
Navigation/.venv/bin/python quickstart/export_scan.py \
    runs/stage_20260910T084053/bag --out runs/reloc_eval_20260911/still_mid

# 2. 探索する（コンテナの中）
/tmp/reloc_build/bin/reloc_probe --map <...>/mola_floor0/map.mm \
    --scan <...>/still_mid.xyz --center X Y --roi 3.0 --sigma 0.05 \
    --truth X_true Y_true YAW_DEG --top 3
```

## 実測（2026-09-11。静止の対照の 45 枚目、12,070 点）

**ROI の中心を真値から 1.5 m ずらしても、上位 1 位は真値ちょうどに戻る。**

| 刻み | 格子点 | 所要 | 上位 1 位の xy 誤差 | yaw 誤差 |
|---|---|---|---|---|
| **0.50 m / 30°** | 2,028 | **1.1 s** | **0.000 m** | 5.92°（格子の限界） |
| 0.50 m / 10° | 6,084 | 3.1 s | 0.000 m | −4.08° |
| 0.25 m / 10° | 22,500 | 11.7 s | **0.250 m（悪化）** | −4.08° |
| 0.25 m / 5° | 45,000 | 23.0 s | 0.000 m | 0.92° |

⚠️ **刻みを細かくしても単調には良くならない。** 0.25 m / 10° では隣のセルを拾って
0.25 m 外した。尤度の面がほぼ平ら（下記）なので、argmax が隣接セルの間で揺れる。
**格子探索の役目は「大づかみに捕まえる」ところまで**で、詰めは ICP に渡す
（`/relocalize_near_pose` に格子の刻み相当の共分散を添えれば MOLA 側の ICP が詰める）。

### 尤度のつまみ

`map.mm` に載っている既定は `sigma_dist 1.0 / max_corr_distance 2.0 / decimation 10`。
これは MOLA が ICP 向けに持っている値で、**再定位の識別には緩すぎる**:

| `sigma_dist` | log 尤度の幅 | 99 パーセンタイルに残る姿勢 |
|---|---|---|
| 1.00（既定） | −0.23 .. 0 | 5 件以上（0.5 m ずれても 0.08% しか減らない） |
| 0.25 | −0.92 .. 0 | 2 件 |
| 0.10 | −2.30 .. 0 | 2 件 |
| **0.05** | **−4.61 .. 0** | **1 件（真値）** |

→ **推奨: `--sigma 0.05`、ROI ±3 m、刻み 0.5 m / 30°。** 1.1 s で 1 件に絞れる。

## ここまでで確かめていないこと

- 素材が**静止の対照の 1 枚だけ**。机が動いた場合・向きの取り違え（180°）・
  歩行のすべった終端からの復帰は**段 8（6 パターン検証）でやる**
- `RelocalizationICP_SE2`（格子の各点で ICP を実走する重い方）は未評価
- 壁の帯（z > 1.3 m）に絞ったときの挙動は未評価（段 6）
