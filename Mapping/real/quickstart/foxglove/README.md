# Foxglove のレイアウト（段 5 の下ごしらえ）

**⚠️ ブラウザで開いたことはまだ無い。**ただし 2026-09-13 に**機体を動かさずに**、
実機稼働中の `foxglove_bridge` に対して**参照トピックの実在を全件突き合わせた**
（`check_foxglove_layout.py`）。パネルが空になる原因のほとんどはトピック名なので、
**残る未確認は「描画・操作の見え方」だけ**である。開いて直したらここに上書きすること。

突合で**実欠陥が 1 件**見つかり、直した ⇒ 下の「アクションのトピックには 2 段の関門がある」。

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

## ⚠️ アクションのトピックには 2 段の関門がある（2026-09-13 に両方踏んだ）

`recoveries`（合否 3-3）は `/navigate_to_pose/_action/feedback` にしか出ない。
**既定ではこれが広告されない。**理由が 2 つ重なっている:

1. **hidden 扱い** — 名前に `_action` を含むトピックは ROS 2 では隠しトピックで、
   `foxglove_bridge` の既定 (`include_hidden:=false`) では広告されない。
   ⇒ `start_foxglove_bridge.sh` に **`-p include_hidden:=true`** を入れた
2. **schema が作れない** — それでもまだ出なかった。ログに
   `Failed to add channel ... (nav2_msgs/action/NavigateToPose_FeedbackMessage):
   package 'nav2_msgs' not found` と出る。**PC2 の pixi 環境に nav2_msgs が無い**ため。
   ⇒ `pixi add ros-humble-nav2-msgs`（**解決結果は追加 1 件のみ・更新も削除もゼロ**）

⚠️ **1 だけでは直らない。**`/navigate_to_pose/_action/status` は標準型 (`action_msgs`) なので
1 だけで出てしまい、「開いた」と誤認しやすい。**feedback が出るかで確かめること。**

## 届かないトピック 4 件は、すべて意図して止めてあるもの

`check_foxglove_layout.py` が NG と出すが、**欠陥ではない**:

| トピック | なぜ来ないか |
|---|---|
| `/cloud_pcd` | 事前地図 3D。`g1_mapping_visualization/view.launch.py` の `pcd_to_pointcloud` が出す。**`nav_stack.sh` は起こさない**（見たいときだけ別に上げる） |
| `/projected_map` | OctoMap。**2026-09-11 から既定 off**。来ないのが正常 |
| `/occupied_cells_vis_array` | 同上 |
| `/unitree/slam_mapping/points_stamped` | PC2 の `restamp_points.py` が出す。純正 SLAM の点群で、MOLA 構成では使わない |

`/clicked_point` は**出力専用**（購読者が居なくても publish できる）なので数に入れない。

## foxglove_bridge の Jetson での CPU コスト（段 5-c・2026-09-13 実測）

**実機を動かさず**、電源の入った G1 の生 LiDAR が流れている状態で測った。
窓の中の平均（`/proc/<pid>/stat` の utime+stime 差分）である。

| 条件 | bridge の CPU | コア換算 | 受信 |
|---|---|---|---|
| 購読者なし | **3.1%** | 0.03 | — |
| 生 LiDAR のみ | **5.7%** | 0.06 | 9.9 Hz / 4.39 MB/s |
| **RViz2 相当の全部**（点群・両コストマップ・`/map`・`/plan`・TF・Footprint） | **6.1%** | **0.06** | 4.67 MB/s |

**比較: Mac の RViz2 は点群 ON で 238%（2.38 コア）。**
⇒ **③ の前提が実測で裏づけられた。**描画の仕事はブラウザへ移り、
PC2 に残るのは **0.06 コア**だけである。Orin NX の 8 コアに対して**無視できる**。

⚠️ **測り方の罠。**`ros2 run foxglove_bridge foxglove_bridge --ros-args ...` という
**起動役の python も同じ語を全部含む**。そちらを測ると「0.0%」＝「タダ」という
誤った結論が出る。実体は `lib/foxglove_bridge/foxglove_bridge` の方。

## まだ分かっていないこと

- **ブラウザで開いたときの見え方**（パネル配置・カメラ・色）。トピックの実在は確認済み
- ブラウザから IP を直に叩く形にするか、SSH トンネルのままでよいか
  （**CLI は混在コンテンツの制限を受けない**ので `check_foxglove_stream.py --host` は直結できる。
  ブラウザだけがトンネルを要する）

## PC2 に docker はあるか（**答えは出た**）

**ある（24.0.7）。だが使えない。** `unitree` は docker グループに入っておらず、
`sudo` にパスワードが要る（`/var/run/docker.sock` は `root:docker 660`）。
そして **PC2 は pixi だけで運用する方針**なので、ここは塞がっていて構わない。
`start_foxglove_bridge.sh` は元から pixi 前提である。

⚠️ **ただしこの方針は段 5-b と衝突する** ⇒ `docs/plan/2026-09-13-rviz-click-walk-live.md` §5。
