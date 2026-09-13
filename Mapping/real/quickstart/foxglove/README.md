# Foxglove のレイアウト（段 5 の下ごしらえ）

**⚠️ このレイアウトは 2026-09-13 時点で Foxglove に読み込ませたことが無い。**
機体も PC2 も電源が入っておらず、**データ源を繋いだ状態で確かめられなかった**。
`g1_nav.json` は `rviz/g1_nav_nocloud_720.rviz` の表示項目を写した**下書き**である。
読み込んで直したら、**直した結果をここに上書きすること**。

## なぜ Foxglove なのか

RViz2 の点群表示は**約 1.7 コアを食い**（ON 238% / OFF・HD 窓 67%）、
そのぶんが測位から奪われると推定姿勢が壊れる。
2026-09-13 の実測で、**このコストはラスタライズではなく点ごとの CPU 仕事**
（受信・色変換・頂点配列の構築）だと分かった。**GPU に載せても減らない。**

Foxglove なら**描画の仕事ごとブラウザ（WebGL）へ移る**ので、
PC2 にも Mac にも描画の負荷が乗らない。これが段 5 で ③ を選んだ理由である
（① PC2 で noVNC は「同じ取り合いをより遅いコアで再現する」だけ）。

## 使い方

```bash
# PC2 側（pixi の Humble 環境）
ssh g1 'bash ~/mapping_tools/start_foxglove_bridge.sh'

# Mac 側
ssh -N -L 8765:localhost:8765 g1
# ブラウザ → https://app.foxglove.dev → Open connection
#          → Foxglove WebSocket → ws://localhost:8765
#          → レイアウトは g1_nav.json を import
```

⚠️ **トンネルを挟む理由**: `app.foxglove.dev` は HTTPS なので、素の `ws://` で
リモートに繋ぐと混在コンテンツとして遮断される。**`localhost` 宛だけは例外**。

## 何が入っているか

| パネル | 中身 |
|---|---|
| **3D** | 事前地図 `/map` / Global・Local コストマップ / `/plan` / Footprint / TF |
| Log | `rosout` を `guard` `planner_server` `controller_server` `bt_navigator` で絞る |
| Plot | `/dog_odom` の vx と `/cmd_vel` の vx・vyaw |
| Raw | `/navigate_to_pose/_action/feedback`（残り距離・経過時間・`recoveries`）|

**点群（`/utlidar/cloud_livox_mid360`）と事前地図 3D（`/cloud_pcd`）は既定で切ってある。**
RViz2 と同じ運用にする: 立たせた状態で ON にして重畳を目視 → **投げる直前に OFF** → 歩かせる。

## ⚠️ ゴールを投げるところ

3D パネルの publish 設定を **`pose` / `/goal_pose`** にしてある。
これで **RViz2 の「2D Goal Pose」の役目を引き継げる**＝ `stray_guard.py` の見張りも効く
（あれは `/goal_pose` を購読しているので、投げ方には依らない）。

⚠️ **「進みたい向きへドラッグする」のは Foxglove でも同じ。**
向きが機体と無関係な値になると復帰動作の `Spin` を呼ぶ（README-nav2 §7）。

## まだ分かっていないこと

- **`foxglove_bridge` の Jetson での CPU コスト**（未測定。段 5-c）
- **PC2 に docker があるか**（無ければ pixi 上に置く。`start_foxglove_bridge.sh` は既に pixi 前提）
- ブラウザから IP を直に叩く形にするか、SSH トンネルのままでよいか
