# 引き継ぎ — 次にこれを触る人へ

最終更新: **2026-09-25 夜**（⭐ **巡回を1周完走した。** そして `collision ahead` の
原因が確定した → [findings/collision_ahead_cause_20260925.md](findings/collision_ahead_cause_20260925.md)）

⚠️ 会場は **room_a**（9/24 に一時 Sorasta を既定にしたが 9/25 に戻した）。

このファイルは「**次に何をすればいいか**」だけを書く。何があったかの記録は
[Planning.md](Planning.md) の A-10n、当日の操作手順は
[safety/runbook_nav2_session.md](safety/runbook_nav2_session.md) を見ること。

---

## 1. 到達点（2026-09-25 時点）

⭐ **実機で巡回路 6 点を1周完走した**（2周目の途中で内蔵SLAM の寿命切れ）。
§3〜§8 の経路がすべて実機で成立している。

| | |
|---|---|
| 会場 | **room_a**。既定地図は `maps/grids/room_a_map_20260911_edited.yaml`（9/11 版を手で28箇所開けたもの）。⚠️ 9/24 に一時 Sorasta にしたが 9/25 に戻した |
| **9/25 の到達点** | ⭐ **巡回 1 周完走**（p1→p2→p3→p4→p5→p6→p1、6区間すべて）。`collision ahead` の**原因が確定**（下記 2-⓪） |
| 9/24 の到達点 | 自律歩行 **2 回（0.78m / 0.62m）**。**旋回デッドロックは再現せず**（wz が 0.30 出た） |
| 自律歩行（9/15・room_a） | 3 回（**0.34m / 3.17m / 4.96m**）。Goal は **1 件 SUCCEEDED** |
| **U-16（歩行中の odometry 精度）** | ✅ **答えが出た。約5m 歩いてドリフト 7cm（測定分解能）、yaw ゼロ。** Phase 2a の FAST-LIO 構築（A-6）は省略できる見込み |
| D-31（heartbeat） | 実機で初通電。通信断で正しく停止することを確認（意図せず 1 回発動した） |
| 残る壁 | **`collision ahead` の恒久対処**（原因は判明。直し方は未実装 → 2-⓪）と、**内蔵SLAM が 20〜26 分で落ちる**（2-③） |

---

## 2. 次にやること（この順で）

### ⓪' 次の実機セッションの目標: **巡回を通す**（2026-09-24 に決めた）

モックでは **6 点の巡回路を 4 周、FAULT・abort・`collision ahead` ゼロ**で回った
（1 周 約75秒 / room_a の実地図の上）。**実機で同じことをやる。**

**✅ 2026-09-25 に PC2 へ配布・ビルド済み**（`g1_navigation` / `g1_cmd_router` とも成功）。
`maps/` `tools/` `g1_ws/src/` `g1_sdk_bridge_cpp/` 起動スクリプト一式が入っている。
次に PC2 へ配るときはこれ:

```bash
nmcli connection up g1-link
rsync -a --exclude clouds/ maps/                    g1:/home/unitree/g1_nav2/maps/        # ⚠️ 忘れるとビルドが落ちる
rsync -a --exclude '__pycache__' g1_ws/src/         g1:/home/unitree/g1_nav2/g1_ws/src/
rsync -a --exclude build --exclude '__pycache__' \
        g1_sdk_bridge_cpp/                          g1:/home/unitree/g1_nav2/g1_sdk_bridge_cpp/   # ⚠️⚠️ 忘れやすい
rsync -a --exclude '__pycache__' tools/             g1:/home/unitree/g1_nav2/tools/
rsync -a deploy/pc2_humble/{g1up.sh,start_nav.sh,start_record.sh,start_localizer.sh} g1:/home/unitree/g1_nav2/
ssh g1 'cd ~/g1_nav2/pc2_humble && ~/.pixi/bin/pixi run bash -lc "cd ~/g1_nav2/g1_ws && colcon build --symlink-install"'
```

⚠️⚠️ **`g1_sdk_bridge_cpp/` は `g1_ws/src/` の外**にある（独立 CMake プロジェクト・D-08）。
2026-09-25 に送り忘れて `SafetyManager has no member named 'SuspendCmdTimeout'` でビルドが落ちた。

⚠️ **発進ゲートは `/etc/default/g1-sdk-bridge` で `G1_ARM=--arm` のまま**。
ただし **サービスは `inactive` かつ `disabled`**（PC2 再起動で落ちた）ので、いまは武装していない。
`systemctl start` した瞬間に武装するので、動かす気が無いなら先に閉じること:
`sudo sed -i 's/^G1_ARM=.*/G1_ARM=/' /etc/default/g1-sdk-bridge`

**当日の流れ（③で初めて歩く。①②では動かない）:**

| 段 | やること | 機体 |
|---|---|---|
| 準備 | 機体を**通常の立位**で出発位置へ。人は2m以上離れる | 立つだけ |
| §2〜§7 | `~/g1_nav2/g1up.sh`（巡回路は既定で `config/patrol_room_a.yaml` を読む） | 動かない |
| 目視 | **RViz で位置と赤軸の向き**。§7 の一致率も見る（50cm以内が80%を切ったら地図を疑う） | 動かない |
| ① | ゲートを開く（`G1_ARM=--arm` + `systemctl restart`） | **まだ動かない** |
| ② | `g1up.sh --enable`（`NAVIGATING` になる） | **まだ動かない** |
| ③ | `~/g1_nav2/tools/patrol_ctl.sh start` | **ここで歩き出す。まず p1 へ** |

**⚠️ 巡回路の 6 点は「地図の上でクリックした座標」**（`tools/mark_map.py`）。
各点の `yaw_deg` は**次の点を向く**ように入れてある（`patrol_dryrun.py --face-next`）。
現地で測ったものではない。**最初の1周は人がついて見ること。** ずれるようなら
`tools/record_waypoints.py` で取り直す（機体を実際にその場所へ連れて行く方が確か）。

**⚠️ 今日の対処で `spin` が実際に動くようになった**（`behavior_server` の cmd_vel を
`/cmd_vel_nav` へ付け替えた）。これまで復帰動作の指令はどこにも届いていなかった。
**その場旋回が起きることを想定しておくこと。**

📌 **16〜18分で内蔵SLAM が落ちる。** 1周75秒なら **13周前後**で止まる。落ちたら
TF 途絶 → FAULT → 巡回は HOLD。`1801` 再送 → **§7 をやり直し** → `clear_fault` →
`patrol_ctl.sh start` で再開（自動では戻らない）。

### ⓪'' 自己位置の補正を「その場で切り替えられる」ようにした（2026-09-24）

**`map→odom` を Nav2 の外に出せるようにした。** 従来は §7 の値を入れるのに
Nav2 ごと上げ直していた（20〜25秒・巡回も IDLE に戻る）が、
`start_nav.sh` の第1引数に **`none`** を渡せば `g1_slam_odom_tf.py` は
map→odom を出さなくなるので、外から差し替えられる。

```bash
bash ~/g1_nav2/start_nav.sh none 0 2.0                    # map→odom を外部供給にする
~/g1_nav2/tools/map2odom_ctl.sh start   <dx> <dy> <yaw>   # 供給開始（hold＝補正しない。従来と同じ挙動）
~/g1_nav2/tools/map2odom_ctl.sh observe <dx> <dy> <yaw>   # ⭐ 測るだけ（TF を出さない・併走可）
~/g1_nav2/tools/map2odom_ctl.sh correct                   # 補正を入れる（**走行中に切り替えてよい**）
~/g1_nav2/tools/map2odom_ctl.sh hold                      # 補正を止める
~/g1_nav2/tools/map2odom_ctl.sh status
```

⚠️⚠️ **静的 TF と動的 TF の「入れ替え」は成立しない**（2026-09-24 にモックで実測）。
`map_localizer` は起動後 **距離場の作成に約3秒** TF を出せず、その間に静的を止めると
`tf_stale` → FAULT → 巡回が HOLD。順序を逆にしても、**静的 TF は tf2 のバッファに
残り続ける**ので入れ替え自体が成立しない。だから**供給者は常に `map_localizer` 1本**にし、
`hold`/`correct` をサービスで切り替える形にした。モックでは**走行中に `correct` へ
切り替えても `NAVIGATING` のまま巡回が続く**ことを確認済み。

📌 **`observe` は static と併走してよい**（TF を出さないため）。走行に影響せず、
`runs/drift_*.csv` に**累積補正量＝実際のずれ**が残る。**次回これを回すこと。**

⚠️ **この経路は実機で未検証。** 従来どおり `g1up.sh`（§7b で Nav2 を上げ直す）も残してある。

### ⓪⓪ ⚠️⚠️ **PC2 へ未配布の変更がある**（2026-09-25 夜）

`config/nav2_params.yaml` の `goal_checker` を緩めたが、**リンクを切ったあとに
直したので PC2 に入っていない**。次に繋いだら最初にこれを送ること:

```bash
rsync -a g1_ws/src/g1_navigation/config/nav2_params.yaml \
        g1:/home/unitree/g1_nav2/g1_ws/src/g1_navigation/config/
# ⚠️ install 側が symlink でなければ colcon build も要る
```

### ⓪''' ゴールに着いても止まれず回り続ける ← **最優先（原因は判明、対処は未検証）**

⭐ **2026-09-25 に実機で発生・原因特定** → [findings/goal_orbit_and_drift_20260925.md](findings/goal_orbit_and_drift_20260925.md)

**p2 に 0.009m まで着いたのに、180 秒間その場で回り続けた**（位置と向きが
**一度も同時に**許容内にならない。775 サンプルで該当 0 件）。
**その旋回で内蔵SLAM がドリフトし、最終的に自己位置が 10.8m ずれた。**

対処として `goal_checker` を **`xy 0.20→0.30` / `yaw 0.20→1.00 rad(57°)`** にした
（記録から計算すると **到達せず → 20.3 秒**）。⚠️ **実機で未検証。**
駄目なら yaw を 1.57 → 向き指定をやめる、の順で落とす（findings の §5）。

### ⓪'''' 次の実機セッションで必ずやること

1. **§7 のあと RViz で位置と向きを目視確認する。**
   ⚠️ 2026-09-25 の2回目はこれを飛ばして走らせ、見当違いの場所へ歩かせた。
   **§7 の値が何回やっても同じでも、正しさの証拠にはならない。**
2. **`tools/map2odom_ctl.sh observe` を併走させる。**（TF を出さないので走行に影響しない）
   ドリフトが実際どれだけ育つかを測る。**用意済みで未使用。**
3. **記録に点群を含める。** 9/25 は `diag` プロファイル（19 トピック）で録ったため、
   **オフラインで自己位置を再計算できず、ドリフト説を確証できなかった。**
4. **巡回を start する前に costmap 消去が動いているか確認する。**（下の ⓪ 参照）

### ⓪ `collision ahead` を**恒久的に**直す ← **最優先**

⭐ **2026-09-25 に原因が確定した** → [findings/collision_ahead_cause_20260925.md](findings/collision_ahead_cause_20260925.md)

**実在の障害物でも、地図のノイズでも、自己位置のずれでもない。**
`local_costmap` に残った **LiDAR 観測の消し残り**が、機体の足元を障害物にしていた。

```
local costmap : footprint(0.32m)内に lethal 33 セル / 最短 0.17 m  →「衝突」
生の LiDAR    : 障害物として立つ点の最短が 2.67 m 先（センサー基準）
costmap を消す→ 0 セル。静止14秒では戻らない。約100秒歩くと 37 セルまで再蓄積（2回とも同じ）
```

`local_costmap` は `plugins: ["voxel_layer","inflation_layer"]` で**静的地図の層が無い**。
**MID-360 の死角（立位の実測で半径 0.91〜1.12m。D-21）の中はレイが通らず、
一度立った格子を消せない。** 歩くほど溜まる。
⚠️ 当初「この地点では 1.57m」と書いたが**誤りなので取り消した**
（`why_costmap.py` が odom 原点基準で測っていた。同日センサー基準に修正）。

⚠️ **`cmd_timeout` を伸ばしても直らない。** 本当の原因はここだった。

**暫定処置（今日これで1周できた）**: 20秒ごとに
`/local_costmap/clear_entirely_local_costmap` を呼ぶループを併走させる。
⚠️⚠️ **本物の障害物も一瞬消える。死角(〜1.6m)の中のものは立ち直らない。人が見ている場でのみ。**

**次にやること（この順で）:**

1. **記録 `runs/20260925_182814_nav2`（106MB）を再生して、消し残りの正体を見る。**
   機体自身の脚（`min_obstacle_height: 0.05`）なのか、通り過ぎた人なのか。
2. **死角の中だけ毎周期 free にする層を足す**（本命）。死角は幾何で決まるので、
   その中を信用しないのは筋が通っている。
3. G1 内蔵の `/collision_clouds` `/pre_collision_clouds` `/safe_clouds` を調べる（未着手）。

### ③ 内蔵SLAM の寿命（9/25 に n=2 で計測）

**19分39秒** と **26分**。従来の「16〜18分」より幅がある。落ちると TF 断 →
`tf_stale` → FAULT → 巡回 HOLD。**自動では戻らない。**

復帰: `cd ~/g1_nav2/tools && /usr/bin/python3 send_slam_api.py 1801`
（⚠️ **pixi ではなく `/usr/bin/python3`**。pixi には `cyclonedds` が入っていない）
→ **§7 をやり直し**（odom 原点がリセットされるため）→ `clear_fault` → `patrol_ctl.sh start`

⚠️ **1801 を送り直すと odom レートが変わることがある。** 9/25 は
**10.077 Hz → 約4 Hz に半減**し、`tf_timeout_s` の既定 0.5 秒では
**enable した直後に必ず `tf_stale` FAULT** になった。
→ `start_nav.sh` の**第6引数**（または launch の `tf_timeout_s:=`）で **1.0** にして回避した。

### ① ~~旋回デッドロックの対処を実機で試す~~ ✅ **2026-09-24、再現しなかった**

`/cmd_vel_smoothed` の wz が **0.30** まで上がり、機体は歩いた（9/15 は 0.02 で
2,363 周期張り付き）。⚠️ **「その場旋回だけ」を分離しては見ていない**ので、
`rotate_to_heading` 単体の合否は未確定。以下は当時の記録。

**今日の失敗のほぼ全部がこれで説明できる。** 対処はコミット済みだが**実機で未検証**。
📌 **2026-09-23 に PC2 へ配置・再ビルド済み**（`install/` の `nav2_params.yaml` が`max_angular_accel: 6.0` / `PoseProgressChecker` になっていることを確認）。**残っているのは実機で見ることだけ。**

RPP の rotate-to-heading は**実測角速度を基準に**加速度制限をかけるため、

```
指令 = 実測 + max_angular_accel × dt = 0 + 0.40 × 0.05 = 0.02 rad/s
```

となり、G1 は 0.02 rad/s では歩容が成立しないので回らない → 実測 0 のまま →
また 0.02、という**デッドロック**に入る。実測で wz=-0.02 が 2,363 周期続き、
機体は 4mm も動かなかった。

**入れた対処**（[config/nav2_params.yaml](g1_ws/src/g1_navigation/config/nav2_params.yaml)）:
- `max_angular_accel` 0.40 → **6.0**（1 周期の増分が歩容の閾値を超えるように）
- progress checker を **`PoseProgressChecker`**（角度も進捗と数える）+ 20 秒に

✅ **2026-09-16、モックで再現と修正確認を済ませた**
（[findings/mock_deadlock_repro.md](findings/mock_deadlock_repro.md)。
`./tools/mock_deadlock_test.sh old` / `new` で誰でも再現できる）:

| | 旧設定 | 新設定 |
|---|---|---|
| wz | **0.020 で頭打ち** | **0.300**（立ち上がる） |
| 60秒後 | **1mm も動かない** | **1.88m 移動** |
| 結末 | abort → spin → 繰り返し | **`Reached the goal!`** |

**実機での判定基準**:

| 見るもの | 期待 |
|---|---|
| `/cmd_vel_smoothed` の wz | **0.02 で止まらず 0.3 前後まで上がる** |
| 機体 | その場旋回が**始まる** |
| `follow_path` の abort | **10 秒ちょうどで切れなくなる** |

⚠️ **モックに物理は無い。閾値を階段関数で模しただけ。**
実機の旋回の作動閾値は未知（`0.02` で不動・`0.3` で回る、しか分かっていない）。
**真の閾値が 0.3 より上なら 6.0 でも足りない**（6.0 × 0.05 = 0.30 が1周期ぶんのため）。
そのときは `rotate_to_heading_angular_vel`（未指定なので既定 1.8）を明示するか、
`max_angular_accel` をさらに上げる（10.0 なら1周期で 0.5）。

### ①' 巡回モードを実機で通す ← ①と同じ日にできる

**2026-09-16 に実装し、モックで通した**（[findings/patrol_mode.md](findings/patrol_mode.md)）。
**単純ゴール指定モードと併存する。再起動なしで切り替わる。**

⚠️ **実機で足りていないのは「巡回路そのもの」。** `config/patrol_room_a.yaml` は
**空のひな形**で、現地で記録するまで `start` は断られる。地図が 9/07 取得で現状と
合っていないので、**座標を手で書かず、機体を実際にその場所へ持って行って拾う**:

```bash
# --- (1) RViz で引く（速い。機体は動かない）---
~/g1_nav2/tools/patrol_ctl.sh teach   # 以後「Publish Point」のクリックが点になる
#   ⚠️⚠️「2D Goal Pose」ではない。あれはクリックした瞬間に機体が歩き出す
~/g1_nav2/tools/patrol_ctl.sh save    # yaml に書き、そのまま読み込む（再起動不要）

# --- (2) 機体を連れて行く（確実）---
cd ~/g1_nav2/tools && python3 record_waypoints.py -o ~/g1_nav2/patrol_room_a.yaml
~/g1_nav2/g1up.sh --patrol ~/g1_nav2/patrol_room_a.yaml   # 上げ直しが要る

# --- 走らせる（8.3 の enable_navigation の後で）---
~/g1_nav2/tools/patrol_ctl.sh start
```

⚠️ **(1) でクリックするのは「地図の上の座標」**なので、地図が古いぶんだけずれる。
(2) は**機体が実際にそこに立てた**という事実が裏付けになる。
**併用が実務的** — (1) でざっと引いて1周流し、着けなかった点だけ (2) で取り直す。

⚠️ **単純ゴール指定（手順書 8.4）が通ってから**にすること。巡回は同じ
`NavigateToPose` を連続で投げるだけなので、1回の Goal が通らないなら巡回も通らない。

### ② 地図 — ✅ **作り直して切り替えた（2026-09-16）。新旧比較はやらないと決めた**

9/11 の rosbag に**軌跡が入っていた**（`/unitree/slam_mapping/odom` 7002件/9.97Hz）。
これが見つからず止まっていたので、これで解決した。
詳細: [findings/map_rebuild_20260911.md](findings/map_rebuild_20260911.md)

| | **9/07（旧）** | **9/11（新・既定）** |
|---|---|---|
| 大きさ | 28.5 × 33.0 m | **23.8 × 37.5 m** |
| **unknown** | **40.5%** | **27.4%** |
| **最大連結の自由空間** | 397.2 m² | **438.7 m²（+10%）** |
| 自由空間の断片 | 7,221 個 | **4,743 個** |

**既定を `room_a_map_20260911.yaml` に切り替えた。** 旧地図も残してある。

📌 **新旧を実機で比べるのはやらないと決めた（2026-09-20、利用者の判断）。**
「同じスキャンを両方の地図に当てて一致率を並べる」案は提案したが、**採らない。**
新しい地図でそのまま進める。

- **どちらが良いかは構造の比較で十分ついている**（未知 40.5%→27.4%、
  連結した自由空間 +10%、断片 3割減）。旧地図に戻す積極的な理由が無い
- 現地の時間は**①旋回の確認・巡回路の作成・③16分停止の切り分け**に使う
- ⚠️ **ただし「現状に近いか」は測らないまま進むということ。** §7 の一致率が
  出発地点で 80% を切ったら、**そこで初めて**旧地図と比べればよい

⚠️ **合わなければ旧地図に戻せる**（比較をやめても退路は残っている）:
`g1up.sh --map .../room_a_map.yaml`
⚠️ **2枚は座標系が約10°違う。** 旧地図の座標で書いたものは全部無効
（`map→odom` はどのみち毎回取り直すので実害は無い）。
⚠️ **occupied が 1.6 倍になった**（102.8→161.2m²）。本物の什器なのか、
ノイズや**動いている人が焼き付いた**のかは数字では区別できない。**現地で目視確認すること。**

📌 **25m×25m 制約が緩んだ。** x は **23.8m で 25m を切った**。
実際に歩いた範囲は **15.8 × 29.4 m**。**y を 25m 以内に区切れば推奨範囲に収まる**
（会場全体を1周するのは諦める、という判断にはなる）。

### ③ 内蔵SLAM の16分停止 — **方針は (a) に決めた。タダで取れるデータがある**

巡回は16分では終わらない。落ちるたびに odom 原点がリセットされ、`map→odom` を
求め直す必要がある。**2026-09-16 に原因を調査したが特定できなかったので、
「約16分ごとに `1801` を送り直し §7 をやり直す」運用を採る。**

公式ドキュメントで分かったこと:

- **16分の制限は文書化されていない**
- 関連記述は「**連続使用は30分まで**を推奨。長時間・高温だと dock の CPU が
  周波数低下し**測位が異常になる**」。実測は CPU 51〜59% / 59〜66℃ で、
  16分の説明としては弱い
- 未適用の注意事項: 「**建図中は機体内蔵の障害物回避を切る**（頭部ライトが青になる）」
  「建図中は機体を速く動かしすぎない」

### ✅ 2026-09-24 に 1 回ぶんの答えが出た: **18.5 分。歩いていても切れた**

`1801` を 1 回だけ送り、07:22:45 → **07:41:22 に停止（18.5 分）**。
**この間に他の人が機体を歩かせている**（odom が 0.6m 以上動いた）ので、
**「無動作で切れる」説は弱まり、「時間で切れる」が濃厚**になった。
⚠️ **n=1**。過去 3 回（15.7〜17.0 分）より長い理由も説明できていない。

📌📌 **次のセッションで、ロボットの時間を追加で使わずに判別できる。**
`1801` を**最初に1回だけ送り、以後は再送せずに** `watch_slam_alive.sh` を回しっぱなしにする。
①②③で機体は歩くので、**16分を超えて生き延びれば「無動作で切れる」**と分かる。

> **⚠️ 観測に偏りがある。** 2026-09-15 の3回とも**機体が静止している間**に落ちた。
> 歩行中のセッションは一度も16分を超えていないので、「時間で切れる」のか
> 「無動作で切れる」のかを区別できていない。

📌 **弱い証拠が1つ増えた（2026-09-16）。** 9/11 の bag は **11.7 分間、
歩きっぱなしで一度も落ちていない**（odom 9.97Hz、最大の欠測 0.16 秒）。
「無動作で切れる」説をわずかに支持するが、**11.7 分は 16 分に届いていないので
決着しない。** 上の実験はやはり必要。

**もし「時間で切れる」と確定したら**、選択肢は2つに絞られる。

| | 内容 | 評価 |
|---|---|---|
| (a) 16分ごとに再定位する | いま採っている方針 | 実装は軽いが巡回が分断される |
| (c) **自前の LIO（FAST-LIO）に置き換える** | 落ちない | **U-16 で「精度の観点では不要」と結論したが、可用性の観点で復活しうる。** 下記 25m×25m の制約も併せて効くなら、こちらが本筋になる |

### ④ Q12 の採否を決める（**人の判断が要る**）

`operator_timeout_s` を何秒にするか。実測は [QUESTIONS.md](QUESTIONS.md) に記録した。

| 経路 | heartbeat age | 1.0 秒で足りるか |
|---|---|---|
| 有線 | 中央 0.1 秒以下 | ✅ |
| テザリング（WiFi 省電力 on） | 1.0 秒超 | ❌ **機体が一歩も動かないまま FAULT** |
| テザリング（省電力 off） | 中央 0.101 / **最大 0.628** 秒 | △ 余裕 0.37 秒 |

暫定で **2.0 秒**にしてある。代償として **D-31 の受入目標 0.35m は満たせない**。
(a) 有線で 1.0 秒維持 / (b) 無線で 2.0 秒 / (c) 無線の質を上げて 1.0 秒維持、のどれか。

---

## 3. ゼロから立ち上げる手順

### まずこれを試す（2026-09-16 追加。**実機未検証**）

⚠️ **先に機体を「歩かせるときと同じ通常の立位」にして、出発位置に置くこと。**
§5 の校正と §7 の照合は**いまの姿勢と位置で決まる**ので、あとで動かすと無効になる。
**人は機体から 2m 以上離れる。**

```bash
ssh g1              # ros:foxy(1) noetic(2)? には **Enter だけ**
~/g1_nav2/g1up.sh   # §1〜§7 を一気に通し、各段の✅判定を出す

# 巡回もするなら（巡回路を現地で記録したあと）
~/g1_nav2/g1up.sh --patrol ~/g1_nav2/patrol_room_a.yaml
```

スクリプトは**姿勢を2段で検査する**（2026-09-15 に最も時間を溶かした落とし穴のため）:

- 起動前に IMU で傾きを測る。**10° を超えたら止める**（座位 15.7° / 傾いた立位 19.0° /
  通常の立位 1.6〜4.4°）
- §5 で実際に焼き込まれた値も検査する

意図的に座位などで進めたいときは `--force-posture`。

⚠️ **発進ゲートの開放と Goal 送信はしない**（人の判断を残すため）。最後に次の手順を表示する。
⚠️ **まだ実機で最後まで動かしていない。** 2026-09-23 に `--dry-run` が §0〜§2 まで通ることだけ確認した（§2 で `G1_ARM=--arm` を検出して正しく止まった）。失敗したら下の手動手順に落ちること。
⚠️ **NOPASSWD が未設定なので、§2 の `sudo systemctl start` でパスワードを聞かれる。**
配置と NOPASSWD の設定は [deploy/README.md](deploy/README.md) を参照。

### 手動手順（`g1up.sh` が転んだとき / 中身を知りたいとき）

⚠️ **手順書 §1〜§8 の実体はこれ。** 手順書には書ききれていない前提が入っている。

```bash
# --- 操作PC ---
nmcli connection up g1-link                 # 有線が落ちていることが何度もあった
ssh g1                                      # ros:foxy(1) noetic(2)? → **Enter だけ**

# --- PC2（ここから）---
# ① SDK側プロセス（ゲートは閉じたまま）
grep '^G1_ARM=' /etc/default/g1-sdk-bridge  # → 空であること
sudo systemctl start g1-sdk-bridge
pgrep -af g1_sdk_bridge_real_server         # → **1本だけ**、--arm が無いこと

# ② 内蔵SLAM（約16分で勝手に止まる。巡回の直前に送ること）
cd ~/g1_nav2/tools && python3 send_slam_api.py 1801
setsid nohup bash ~/g1_nav2/tools/watch_slam_alive.sh >/dev/null 2>&1 &

# ③ 記録（Nav2 より先に）
bash ~/g1_nav2/start_record.sh

# ④ Nav2（機体を静止させたまま）
bash ~/g1_nav2/start_nav.sh "0 0 0" 0 2.0   # map_to_odom / lidar_yaw / operator_timeout_s

# ⑤ 地図照合 → 適用
python3 ~/g1_nav2/tools/find_map_offset.py --yaw-range 180 --yaw-step 1
bash ~/g1_nav2/start_nav.sh "<dx> <dy> <yaw_rad>" 0 2.0

# --- 操作PC ---
g1_heartbeat_sender --host 192.168.123.164 &     # ~/.local/bin にある
cd Navigation/nav2_stable/tools && G1_RMW=rmw_cyclonedds_cpp ./rviz_operate.sh
```

**ここで RViz を見て、位置と向きの両方が実機と合っていることを目視確認する。**
合っていなければ先に進まない。

```bash
# --- 武装（全員が2m以上離れてから）---
sudo sed -i 's/^G1_ARM=.*/G1_ARM=--arm/' /etc/default/g1-sdk-bridge
sudo systemctl restart g1-sdk-bridge
pgrep -af g1_sdk_bridge_real_server         # → --arm が付いていること
# ROS 側は自動で再接続する
ros2 service call /g1/enable_navigation std_srvs/srv/SetBool "{data: true}"
# → bridge_status が NAVIGATING になること（READY のままなら許可できていない）

# --- ここから2つのモードを再起動なしで選べる ---
# 単純ゴール指定: RViz の「2D Goal Pose」で 1〜2m 先を指す
# 巡回          : ~/g1_nav2/tools/patrol_ctl.sh start   （watch / pause / stop / skip）
# ⚠️ 巡回中に RViz から Goal を送ると**巡回のほうが退く**。戻すときは再度 start
```

---

## 4. 知らないと必ず詰まること

| # | 落とし穴 | 対処 |
|---|---|---|
| 1 | **内蔵SLAM が約16分で勝手に止まる**（3回とも 941〜1022 秒）。LiDAR 生点群は流れ続けるので気づきにくい | `1801` 再送。⚠️ **odom 原点がリセットされるので §7 をやり直す** |
| 2 | **人が機体を追って歩くと必ず失敗する。** 追従者が常に障害物として立ち、マークも一緒に動く | **2m 以上離れる。後ろも含む** |
| 3 | **`lidar_yaw` は姿勢で 0/180 が入れ替わっていた**（水平化を直したので今は不要のはず） | RViz で**赤軸の向き**を毎回目視。一致率では判別できない |
| 4 | **機体自身の脚が障害物として立つ** | `obstacle_min_range: 0.9` + `footprint_clearing_enabled: true`（設定済み） |
| 5 | **PC2 の FastDDS は点群を受け取れない**（`net.core.rmem_max` が 212992） | ROS 側は **CycloneDDS**（設定済み）。`sudo sysctl -w net.core.rmem_max=16777216` でも直るはず |
| 6 | **既定ルートが wlan0 になると DDS が壊れる。** 「トピックは見えるのにデータが来ない」 | WiFi を `ipv4.never-default yes`。CycloneDDS は eth0 固定（設定済み） |
| 7 | **テザリング越しに DDS は通らない**（AP がマルチキャストを遮断）。RViz が真っ暗になる | 有線を使う。どうしても無線なら [tools/make_fastdds_peers.sh](tools/make_fastdds_peers.sh)（⚠️ マルチキャストアドレスも peer に入れる） |
| 8 | **`backup` を `behavior_plugins` から外すと bt_navigator が起動しない** | 外さない。塞ぐなら BT XML を差し替える（未実施） |
| 9 | **`sudo` は PC2 の `unitree` のパスワード。** 操作PC で叩いても通らない | `ssh g1` してから |
| 10 | **`/etc/default` を書き換えただけではゲートは開かない** | `systemctl restart` が必須 |
| 11 | **PC2 の時計は CST、操作PC は JST で 1 時間ずれている** | ログの突き合わせに注意 |
| 12 | **バッテリ交換で PC2 も機体も再起動する。** `/tmp` が消えて全部落ちる | 交換後は §3 から組み直す |
| 13 | **`bridge_status: READY` は「走行許可」ではない。** TF とセンサーが健全になっただけで、この状態では速度指令が SDK へ1件も通らない（`OnNavTwist` が `SendZero`）。**機体は1mmも動かないまま Goal が abort し続ける** | `enable_navigation` を呼んで **`NAVIGATING`** にする。巡回ノードは `NAVIGATING` 以外では `start` を断る |
| 14 | **Goal 到達の約 1.3 秒後に `cmd_timeout` で FAULT に落ちる**（`velocity_smoother` の velocity_timeout 1.0 + `cmd_timeout` 0.30）。巡回で各点に止まろうとすると1点目で止まる | 巡回の `dwell_s` は **0**（既定）。長く止まりたいなら [findings/patrol_mode.md](findings/patrol_mode.md) §6 の(a)(b)(c)から選ぶ判断が要る |
| 15 | **`transient_local`（latched）にしても、あとから張った購読に届かないことがある。** CycloneDDS で実測。RViz は Nav2 より後に立ち上げるので、これに頼ると**見えるはずのものが見えない** | 周期配信で出し直す。確認は `ros2 topic echo --once --qos-durability transient_local` |

---

## 5. いまの状態（散らかっているもの）

### リポジトリ

- ブランチ **`Dev/Navigation02`**
- ⚠️ **git 管理外のまま残してある大きいファイル**（**消さないこと**）:
  | | |
  |---|---|
  | `Navigation/nav2_stable/0911_robag/` | **3.5GB。9/11 の rosbag。いまの地図の素**。軌跡はここから取り出した |
  | `Navigation/nav2_stable/maps/clouds/map_wireless_dualcam_01_cleaned_no_ceiling.pcd` | 101MB、880万点。**いまの地図の素** |
  | `maps/clouds/map_20260907.pcd` | 6.5MB、**旧**地図の素 |

  📌 軌跡（`maps/trajectory/map_wireless_dualcam_01.tum`、550KB）と生成した地図は
  **リポジトリに入れてある**ので、上の巨大ファイルが無くても地図は使える。
  作り直したいときだけ要る。

### PC2（`~/g1_nav2/`）— 次回そのまま使える

| | |
|---|---|
| `pc2_humble/` | pixi の Humble + navigation2 1.1.20（aarch64）。**docker も sudo も不要** |
| `g1_ws/` | ✅ **2026-09-23 に再ビルド済み**（3パッケージ）。`max_angular_accel: 6.0` / `PoseProgressChecker` / `patrol_node.py` / 新地図 `room_a_map_20260911` が `install/` に入っていることを確認した |
| `tools/` | ✅ リポジトリと同期済み（`patrol_ctl.sh` / `record_waypoints.py` を含む） |
| `start_nav.sh` / `start_record.sh` / `start_localizer.sh` / `g1up.sh` | ✅ **2026-09-23 に配置済み**（`--dry-run` が §0〜§2 まで通ることを確認） |
| `runs/` | ✅ 9/15 の 6 本・**9/24 の 5 本（519MB）**とも `_local/nav2_runs/` に回収済み。⚠️ **9/24 の 5 本はまだ `explain_run.py` に掛けていない** |
| 動いたままかもしれないもの | Nav2 一式、`g1-sdk-bridge`。⚠️ **`/etc/default` は `G1_ARM=--arm` のまま**。ただしサービスは `enable` されていないので再起動後は自動起動しない |

⚠️⚠️ **2026-09-24 の撤収時、ゲートを開けたまま終わった**（`G1_ARM=--arm` / サービス active）。
Nav2・内蔵SLAM・記録はすべて止めてあるので指令は流れないが、**次に触る人は真っ先に閉じること**:
`sudo sed -i 's/^G1_ARM=.*/G1_ARM=/' /etc/default/g1-sdk-bridge && sudo systemctl restart g1-sdk-bridge`

⚠️ **次回接続したら真っ先に `grep '^G1_ARM=' /etc/default/g1-sdk-bridge` を見ること。**
`--arm` のままなので、`systemctl start` した瞬間に武装する。
**2026-09-23 時点でも `--arm` のまま**（サービスは `inactive`）。`sudo` はパスワードが要るので人が閉じること。`g1up.sh --dry-run` は §2 でこれを検出して止まる。

⚠️ **`/etc/sudoers.d/` に `g1-bridge`（NOPASSWD）が入っていない**（2026-09-23 確認。中身は配布時の `README` だけ）。`g1up.sh` は §2 の `sudo systemctl start g1-sdk-bridge` でパスワードを聞く。入れるなら [deploy/README.md](deploy/README.md) の「一度だけ必要な設定」を実行すること（許可するのは**ゲートを閉じたままの起動だけ**）。

### 操作PC

- `~/.local/bin/g1_heartbeat_sender`（ビルド済み。PATH が通っていなければフルパスで）
- heartbeat と RViz は停止済み

---

## 6. 積み残し（機体が無くてもできる）

- **`find_map_offset.py` の選択基準がおかしい。** 「重なりセル数」で最良を選ぶため、
  距離指標で見ると**恒等より悪い解を返すことがある**（実測: 最良 45% に対し恒等 69%）。
  距離指標で選ぶか、両方を報告して警告を出すべき
- **`backup` を含まない BT XML** を作って差し替える（落とし穴8の恒久対策）
- **`obstacle_min_range: 0.9` の副作用**の確認。0.9m より近い実在の障害物は
  新たにはマークされない。死角(0.91〜1.12m)と整合しているが、詰めきれていない
- ~~PC2 の `runs/` の回収と解析~~ ✅ **2026-09-23 に完了**（6 本。`_local/nav2_runs/` に回収し、各ディレクトリに `explain.txt` を置いた）
  - ⚠️ **5 本は `metadata.yaml` が無く、そのままでは開けなかった**（記録を正常終了させずに落としたため）。`ros2 bag reindex -s sqlite3 <dir>` で復旧できる
  - 停止理由は **`cmd_timeout` 6 回 / `operator_lost` 2 回**。Goal は **SUCCEEDED 2 件**、残りは ABORTED / CANCELED
  - **旋回デッドロックが記録にそのまま残っている**: `/cmd_vel_smoothed` の wz が**`-0.020` に張り付いたまま**動かない。①の実機確認では同じ区間で **0.3 前後まで上がるか**を見ればよい（比較の基準値がこれで手に入った）
  - 4 本は同じ時間帯を重複して記録している（`explain.txt` の時刻が食い違うのはこのため）
- **自己位置の異常検知**（2026-09-16 に整理。**結論: いまは作らない**）

  走行を止める条件は TF鮮度 / センサー鮮度 / heartbeat / cmd_timeout / E-stop /
  SDKエラーの6つで、**自己位置の「品質」を見る口は1つも無い**。ただし
  **いちばん起きるとわかっている壊れ方は、すでに止まる**:

  | 壊れ方 | いまどうなるか |
  |---|---|
  | **内蔵SLAM が落ちる（16分問題）** | ✅ **TF 途絶 → FAULT → 停止 ＋ Goal キャンセル**。`g1_slam_odom_tf.py` は odom を受けたときだけ TF を出す（タイマーではない）ので配信が止まり、`cmd_router` は `now − 0.5秒` の変換を要求する（`time 0` ではない）ので鮮度が効く |
  | odometry がじわじわドリフト | 実測 5m で 7cm（U-16）。実害の水準にない |
  | **SLAM は生きているが値が間違っている** | ❌ 検知できない（TF は新鮮なまま中身だけ嘘）。**ただし未観測。** 公式の「高温で dock の CPU が周波数低下し測位が異常になる」が該当しうる |

  ⚠️ **残る穴に対して「照合スコアの閾値」は今は使えない。** 9/15 の実測は
  20cm以内 44% / 50cm以内 49% で、**それでも自己位置は正しかった**。
  `map_localizer.py` の `--max-score 0.35m` ならほぼ全周期が棄却されるので、
  「rejected 連続で FAULT」は**正常なのに走り出してすぐ止まる**機体になる。

  📌 **地図を作り直す（②）のが先。** 新しい地図なら絶対スコアがそのまま使え、
  閾値も素直に引ける。古い地図の上に検知器を載せると、**検知器の調整と地図の
  古さが混ざってどちらが悪いか分からなくなる。**
  それでも先に作るなら、絶対スコアではなく**補正量が急に飛ぶか**を見ること
  （地図の古さは一定のバイアスとして乗るだけなので原理的には分離できる。
  ただし閾値を決める実測データがまだ無い）
- **巡回の実機検証**（モックまで済み）。⚠️ **Sorasta の巡回路はまだ 1 点も記録していない**
- **9/24 の記録 5 本の解析**（`_local/nav2_runs/20260924_*`。`explain_run.py`）。
  特に **`collision ahead` の時間帯の costmap** を見れば ⓪ の切り分けがつく
- **Sorasta の地図の取り直し**（SLAM 軌跡つき。⓪(2) が本命ならこれが本筋）
- **内蔵SLAM の時計ドリフトの扱い**（2026-09-24 に `--restamp-now` で回避した）。
  **回避であって解決ではない。** dock 側の時計を合わせる手段があるなら、そちらが筋
- ⚠️ **「各点で止まって見回す」ができない件の判断**（人が決めること）。
  Goal 到達の約1.3秒後に `cmd_timeout` で FAULT に落ちるため（落とし穴14）。
  [findings/patrol_mode.md](findings/patrol_mode.md) §6 に (a)何もしない /
  (b)`velocity_timeout` を伸ばす（⚠️ D-10 の watchdog が死ぬ）/
  (c)dwell 中だけ `enable_navigation(false)`（⚠️ 巡回が走行許可を出し直せてしまう）
  のトレードオフを並べた。**将来の VLA 動作（巡回中のモード切替）はこれが前提になる**
- ⚠️ **会場が内蔵SLAM の推奨範囲を超えている件の扱い**（2026-09-16 に判明）。
  公式ドキュメントは適用範囲を「**25m×25m 未満。推奨範囲を超えないこと**」と明記している。
  **room_a は 28.5m×33.0m（自由空間 457m²）で既に超過。** 新しい点群
  `map_wireless_dualcam_01` は 58m×40m でさらに外れる。
  **巡回範囲を 25m×25m に絞るのか、自前 LIO（上記 ③ の (c)）へ移るのかの判断が要る。**
  16分停止とは別の話だが、**同じ「内蔵SLAM にどこまで依存するか」という問い**に繋がる
