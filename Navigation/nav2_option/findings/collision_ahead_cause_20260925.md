# `collision ahead` の正体 — local costmap の消し残り（2026-09-25 実機で確定）

**結論: 実在の障害物でも、地図のノイズでも、自己位置のずれでもなかった。**
`local_costmap` に残った**古い観測の消し残り**が、機体の足元を障害物にしていた。

これは [HANDOVER.md](../HANDOVER.md) の最優先課題 ⓪ の答えである。

## 1. 何を測ったか

room_a、巡回中に `RegulatedPurePursuitController detected collision ahead!` で
止まった直後、機体をその場に立たせたまま2つの道具を同時に回した。

| 見たもの | 結果 |
|---|---|
| `tools/count_costmap.py`（local costmap） | **footprint(0.32m)内に lethal 33 セル**。最短 **0.17 m** |
| `tools/why_costmap.py`（**生の LiDAR**） | 障害物点の**最短が 2.67 m**。1.57 m 以内に点が**1つも無い** |

**LiDAR は何も見ていないのに、costmap には足元に障害物がある。**

## 2. 決定的な実験

`/local_costmap/clear_entirely_local_costmap` を呼んだだけで消えた。

| | footprint 内の lethal |
|---|---|
| 消す前 | **33 セル** |
| 消した 4 秒後 | **0 セル** |
| 消した **14 秒後**（静止したまま） | **0 セル** ← 戻ってこない |
| その後 **約100秒 歩いた**あと | **37 セル** ← また溜まる |

2回とも独立に同じ結果（33→0、37→0）。**静止では戻らず、歩くと溜まる。**

## 3. なぜ消えないのか

`config/nav2_params.yaml` の `local_costmap` は
**`plugins: ["voxel_layer", "inflation_layer"]`** で、**静的地図の層が無い**。
つまりこの lethal は保存地図ではなく **LiDAR 観測そのもの**。

`voxel_layer` は `clearing: true` / `raytrace_max_range: 5.5` なので、
本来はレイが通った格子を消す。だが **MID-360 には機体まわりに死角がある**:

- 立位の実測（既存の記録）: 半径 **0.91〜1.12 m**
- **この地点での実測: 半径 1.57 m**

**死角の中はレイが1本も通らないので、一度立った格子は永久に残る。**
歩くほど溜まり、やがて自分の足元が「障害物」になり、RPP が
`collision ahead` → `Controller patience exceeded` → abort → 復帰動作
（`spin` も `Collision Ahead - Exiting Spin` で失敗）→ `wait` →
指令が途切れて `cmd_timeout` → **FAULT → 巡回 HOLD** まで一気に落ちる。

⚠️ **`cmd_timeout` を伸ばしても直らない**（HANDOVER に既出）。本当の原因はここ。

## 4. 暫定処置（今日これで一周できた）

**20秒ごとに local costmap を消すループを併走**させたら、
p5→p6→p1 と回り切り **1周完走**し、2周目に入った
（止まったのは内蔵SLAM の寿命切れによる `tf_stale`）。

```bash
while true; do
  ros2 service call /local_costmap/clear_entirely_local_costmap \
    nav2_msgs/srv/ClearEntireCostmap "{}"
  sleep 20
done
```

⚠️⚠️ **これは暫定処置であって直し方ではない。** 消す瞬間、**本物の障害物も一瞬消える**。
見えている範囲は次のフレーム（約0.25秒）で立ち直るが、
**死角(〜1.6m)の中にある本物の障害物は立ち直らない**。
人が見ている場でしか使わないこと。

## 5. 本筋の直し方（未実施・次にやること）

| 案 | 中身 | 懸念 |
|---|---|---|
| (a) **死角の中だけ毎周期消す** | 半径 1.6m 以内を `voxel_layer` の更新後に無条件で free にする層を足す | 死角内の本物の障害物を見落とす。ただし**そもそも観測できていない**ので実質の損は小さい |
| (b) 復帰動作に costmap 消去を入れる | BT の recovery の先頭を `ClearEntireCostmap` にする | 詰まってから消すので、詰まる回数は減らない |
| (c) `min_obstacle_height` を上げる | 既定 0.05m。機体自身の脚を拾っている可能性 | 低い障害物を見落とす。**まだ「脚が原因」とは確かめていない** |
| (d) G1 内蔵の障害物回避を使う | `/collision_clouds` `/pre_collision_clouds` `/safe_clouds` が出ている | 中身と有効/無効を調べていない |

📌 **(a) が本命**。死角は幾何で決まるので、その中を信用しないのは筋が通っている。
ただし **(c) の「脚を拾っているのか」を先に確かめる**べき。今日の記録
`runs/20260925_182814_nav2`（106MB / 232,650 メッセージ）に点群が入っている。

## 6. ついでに分かったこと

- **G1 の LiDAR は「満杯のフレーム」と「1点だけのフレーム」を交互に出す**
  （実測: `1, 41951, 1, 44255, 1, ...`。発行元は**1つ**）。
  `find_map_offset.py` は最初に変換できた1通だけを使うので、1点のほうを掴むと
  「スキャン 1点で大域探索する」→「見つからず」になる。**下限を入れて直した。**
- **内蔵SLAM の odom レートは一定ではない。** 起動直後 10.077 Hz →
  1801 を送り直したあと **約4 Hz に半減**した。TF の間隔が最大 0.425 秒＋遅延で
  既定の `tf_timeout_s: 0.5` を超え、**enable した直後に必ず `tf_stale` FAULT**
  になった（`Lookup would require extrapolation into the future`／最新TFが0.513秒前）。
  → `tf_timeout_s` を launch と `start_nav.sh` の引数にし、**1.0 秒**で運用した。
- **内蔵SLAM の寿命は 19分39秒 と 26分**（同日 n=2）。従来の「16〜18分」より幅がある。
