# 巡回モードを実装し、モックで通した（2026-09-16、実機不要）

単純ゴール指定モード（RViz の 2D Goal Pose）と**併存**する巡回モードを入れた。
Planning.md の Phase 4-7「複数 Goal / waypoint follower への拡張」の前倒し。

再現手順: `./tools/mock_patrol_test.sh`（4件、約4分） / `... dwell`（§6、約3分）

| 入れたもの | |
|---|---|
| `g1_ws/src/g1_navigation/scripts/patrol_node.py` | 巡回ノード本体 |
| `g1_ws/src/g1_navigation/config/patrol_synthetic.yaml` | モック用の巡回路（4点） |
| `g1_ws/src/g1_navigation/config/patrol_room_a.yaml` | 実会場用。**空のひな形**（§7） |
| `tools/record_waypoints.py` | 巡回路を機体の実位置から記録する |
| 教示モード（`patrol_node.py` 内） | **RViz の「Publish Point」で巡回路を引く**（§8） |
| `tools/nav2_operate.rviz` | `PublishPoint` ツールと `PatrolRoute` 表示を追加 |
| `tools/patrol_ctl.sh` | `start` / `pause` / `stop` / `skip` / `status` / `watch` |
| `tools/mock_patrol_test.sh` | 回帰テスト |
| launch 引数 | `patrol` / `patrol_waypoints` / `patrol_loop` / `patrol_dwell_s` / `patrol_on_failure` / `patrol_autostart` |
| `g1up.sh --patrol <yaml>` / `start_nav.sh` の第4引数 | PC2 の立ち上げから渡せるように |

---

## 1. 2つのモードの関係

| | 単純ゴール指定 | 巡回 |
|---|---|---|
| 入口 | `/goal_pose`（RViz の 2D Goal Pose） | `/g1/patrol/start` |
| 中身 | `bt_navigator` の `NavigateToPose` | **同じ**。`patrol_node.py` が1点ずつ投げる |
| 切替 | — | **再起動は要らない** |

📌 **巡回ノードは既定で常駐するが `IDLE` で何もしない。** `start` を呼ぶまで Goal を
1件も出さないので、置いてあるだけなら従来と完全に同じ挙動になる。

📌 **手動 Goal が常に勝つ。** 巡回中に `/goal_pose` が来たら巡回が退いて `HOLD` する。
`bt_navigator` は Goal を1件しか持てないので、**退かないと「人が送った Goal を
巡回が奪い返す」**という最悪の挙動になる。人の指示が優先されるべきなので、こちらが退く。

---

## 2. なぜ `nav2_waypoint_follower` を使わなかったか

| # | 理由 |
|---|---|
| ① | **周回できない。** リストを1回なめて終わる。警備の巡回は回り続けるもの |
| ② | ⚠️ **停止が停止にならない。** `FollowWaypoints` は内部で `navigate_to_pose` を呼ぶ。`g1_cmd_router` は FAULT 時に `navigate_to_pose` をキャンセルする（A-10f）が、**waypoint follower はそれを「1点失敗」と解釈して次の点へ進んでしまう** |
| ③ | **中断と再開が必ず要る。** 内蔵SLAM が約16分で落ちる（U-17）以上、再開地点を持てる仕組みがどのみち必要だった |

---

## 3. 安全側の設計（迂回を作らないこと最優先）

- `patrol_node.py` は **`NavigateToPose` に Goal を送るだけ**。速度指令は従来どおり
  `velocity_smoother → g1_cmd_router → SDK` を通る。
  **発進ゲート・heartbeat・デッドバンド・E-stop を一切迂回しない**
- `/g1/bridge_status` が **`NAVIGATING`** 以外なら `start` を断る。走行中にそうなったら `HOLD`
- ⚠️ **`HOLD` から自動復帰しない。** `clear_fault` した瞬間に巡回が再開すると、人は
  「復帰させた」だけのつもりなので驚きが大きい（A-10f と同じ思想）

---

## 4. 🐛 モックで見つかった欠陥: **`READY` を「走ってよい」と読んでいた**

最初の実装は `BRIDGE_OK_STATES = ("READY", "NAVIGATING")` にしていた。**間違い。**

```
DISCONNECTED → STANDBY → READY            ← TF とセンサーが健全になっただけ
                           ↓ enable_navigation(true)
                       NAVIGATING          ← ここで初めて速度指令が SDK へ通る
```

`READY` の段階では `SafetyManager::OnNavTwist` が `SendZero()` を返す。つまり
**巡回を始めても機体は1mmも動かないまま Goal が abort され、全点を空振りで消化する。**

モック確認の①で `bridge: READY` のまま `start` が通ってしまい発覚した。
`BRIDGE_OK_STATES = ("NAVIGATING",)` に直した。

📌 **これは巡回に限った話ではない。** 手順書で `READY` を「準備OK」と読んでいたが、
**`READY` は走行許可ではない**。HANDOVER の落とし穴13に追加した。

---

## 5. 結果（モック、`--like-g1`）

| # | 確かめたこと | 結果 |
|---|---|---|
| ① | `enable_navigation` 前に `start` を断るか | ✅ `success=False` / `bridge=READY` / state は `IDLE` のまま |
| ② | `start` で2点を順に回るか | ✅ `a`↔`b`（1.3m）を**約10秒/点**で往復。`dwell_s=1.0` で 5 周し、FAULT に落ちなかった |
| ③ | 手動 Goal を送ると退くか | ✅ 4秒以内に `HOLD`（`hold_reason: 手動Goalに譲った`） |
| ④ | `HOLD` から自動復帰しないか | ✅ 20秒放置しても `HOLD` のまま（このとき bridge は FAULT。§6 の 1.3 秒問題が出ている） |

`tools/record_waypoints.py` も同じモックに対して実行し、
`map→base_link` を拾って YAML を書き出せることを確認した
（`- {name: wp_test, x: -3.0, y: -4.0, yaw_deg: 29.3}`）。

⚠️ **30秒ごとの位置サンプルは往復周期（約20秒）とエイリアスする。** 「位置が動いて
いないのに `loop_count` が増える」ように見えるが、到達ログの時刻を見れば
1脚 9〜18 秒で実際に往復している。**判定には到達ログを使うこと。**

---

## 6. 🐛 各点で止まっていられるのは **1.3 秒ほど**しかない（実測）

再現: `./tools/mock_patrol_test.sh dwell`（`G1_DWELL` で秒数を変えられる）

`dwell_s=5.0` にしたら、**1点目に着いた直後に機体が FAULT** に落ちた。

```
30s bridge=FAULT
    {"state": "HOLD", ..., "last_result": "SUCCEEDED", "hold_reason": "bridge=FAULT"}
fault_reason: cmd_timeout
```

**機構**（設定を足し合わせるとこうなる）:

| | 値 | 出どころ |
|---|---|---|
| Goal 完了後、`controller_server` が指令を出さなくなる | — | Nav2 |
| `velocity_smoother` が余分に出し続ける | **1.0 秒** | `velocity_timeout` の既定（未設定） |
| そこから `g1_cmd_router` が FAULT にするまで | **0.30 秒** | `cmd_timeout`（D-10） |
| **止まっていられる時間** | **約 1.3 秒** | |

📌 巡回そのものは正しく振る舞っている（FAULT を検知して `HOLD` した）。
壊れていたのは**私が入れた `dwell_s` の既定 2.0** のほう。**0.0 に変えた**
（`dwell_s=1.0` なら 5 周回っても落ちないことは §5 ② で確認済み）。
1.0 秒を超える待ちを設定すると起動時に警告を出すようにした。

📌 **同じ壁は失敗時のバックオフにも効く。** 失敗して 3 秒待つ実装にしていたら、
その間に FAULT に落ちて「再試行のはずが HOLD」になる。`retry_dwell_s` も **1.0** にした。

### 🐛 この計測でいったん**真逆の結論**を出しかけた

最初、`mock_patrol_test.sh dwell` は上の巡回路（点ごとに `dwell_s: 1.0` が書いてある）を
使い回していた。**点ごとの `dwell_s` はノードのパラメータより優先される**ので、
`patrol_dwell_s:=5.0` は**黙って無視**され、

```
30s bridge=NAVIGATING   ← FAULT にならない
60s bridge=NAVIGATING
```

と出た。手で書いた版（点ごとの `dwell_s` なし）では FAULT が出ていたのに、である。
**「再現しない＝問題ない」と読んでいたら、実機で1点目から止まっていた。**

対処: dwell モード専用に**点ごとの `dwell_s` を書かない巡回路**を別に作り、
さらに**実際に効いた秒数をログから裏取りして表示する**ようにした。直した版の出力:

```
dwell_s=5.0 で巡回を開始した
[g1_patrol]: ⚠️ 待ち時間が 5.0 秒ある。**1.3 秒を超えると cmd_timeout で FAULT に…**
30s bridge=FAULT
     {"state": "HOLD", "index": 1, "loop_count": 0, "hold_reason": "bridge=FAULT"}
fault_reason: cmd_timeout
```

`loop_count: 0` ＝ **1点目の直後に止まっている**。

📌 **これは巡回に限った話ではない。** 単純ゴール指定でも、Goal 到達の
約1.3秒後に FAULT に落ちる（§5 ④ で実際にそうなった）。
`g1_navigation/README.md` の既知不具合③がこれ。**巡回で初めて実害になった。**

### ⚠️ 長く止まりたいなら、この制約自体を片付ける必要がある（未決）

| | 内容 | 得るもの | 失うもの |
|---|---|---|---|
| (a) **何もしない**（いま） | `dwell_s=0` で通過するだけ | 安全機構に手を触れない | 「止まって見回す」ができない。将来の VLA 動作（Planning: 巡回中のモード切替）が入らない |
| (b) `velocity_smoother` の `velocity_timeout` を伸ばす | 停止後もゼロを出し続ける | 1行で済む | ⚠️ **D-10 の ROS 側 watchdog が死ぬ。** `controller_server` が落ちても `cmd_timeout` が発火しなくなる |
| (c) dwell 中だけ `enable_navigation(false)` → 終わったら `true` | 状態機械の設計どおり（`READY` では `Tick` が cmd_timeout を見ない）。SDK にはゼロが行く | 安全機構を弱めない | ⚠️ **巡回ノードが「走行許可」を自分で出し直せてしまう。** 人が `enable_navigation false` で止めても巡回が戻してしまう |

📌 **(c) が筋は良いが、「走行許可は人が出す」という約束（手順書 §8.3 / D-07）と
正面からぶつかる。** ここは人の判断が要る。決めるまでは (a)。

---

## 7. RViz で巡回路を引く（教示モード）

    patrol_ctl.sh teach     # TEACH に入る
    # RViz の「Publish Point」で回りたい順に地面をクリック（PatrolRoute に出る）
    patrol_ctl.sh undo      # 直前の1点を取り消す
    patrol_ctl.sh save      # yaml に書き、そのまま読み込む（**再起動不要**）
    patrol_ctl.sh cancel    # 全部捨てて抜ける

### ⚠️⚠️ 「2D Goal Pose」は教示に使えない

`bt_navigator` が `/goal_pose` を**直接**購読している。教示のつもりでクリックすると
**機体が本当にそこへ歩き出す**。「クリックされたら即キャンセルする」で誤魔化すことも
考えたが、**Goal を受理してから取り消すまでの間に機体が動きうる**ので採らなかった。

RViz は `SetGoal` ツールを2つ置いて別々のトピックに出すこともできるが、
**ツール名はプラグイン名で決まるので、ツールバーに「2D Goal Pose」が2つ並ぶ**。
現場で取り違えたら機体が歩き出す。そこで**別ツールの「Publish Point」**
（`/clicked_point`）を使う。TEACH でなければ誰も拾わないので、誤クリックしても
**何も起きない**（モック確認の①）。

### 向きは自動で入れる

`yaw_deg` は**「次の点へ向かう方位」**を計算して入れる。最後の点は、周回なら
1点目へ向かう方位。巡回では普通それが正しい向きだし、クリックのたびに矢印を
引かせるより速い。特定の向きで止まりたい点は yaml を直すか `record_waypoints.py` で取り直す。

### ⚠️ 教示と `record_waypoints.py` の差は「便利さ」ではなく「正しさ」

| | 拾う座標 | |
|---|---|---|
| 教示（RViz） | **地図の上でクリックした点** | 速い。ただし地図が 9/07 取得で現状と合っていない（A-10n）ので、**地図では通れるように見えて実際には通れない点**を作り込める |
| `record_waypoints.py` | **機体が実際に立った位置** | 手間はかかるが、**機体がそこに立てたという事実**が座標の裏付けになる |

📌 **併用が実務的。** 教示でざっと引いて1周流し、着けなかった点だけ取り直す。

### 🐛 `transient_local` だけでは RViz に出なかった

巡回路（`/g1/patrol/route`）は「変わったときだけ」出す latched トピックにしていた。
`Durability: TRANSIENT_LOCAL` を指定してあるので、あとから繋いだ RViz にも
最後の1件が届く——**はずだった。届かなかった。**

| 購読を張る順 | 受信 |
|---|---|
| 配信より**先**に張る | ✅ 届く |
| 配信より**後**に張る（＝ RViz を後から立ち上げる） | ❌ **0 件** |

**RViz は Nav2 より後に立ち上げるのが普通**（手順書もその順）なので、これだと
実運用でいちばん要るときに巡回路が見えない。**1Hz で出し直す**ことにして依存を断った
（4点の Path なので負荷は無視できる）。`transient_local` 自体は残してある
（届く環境では即座に出るので、あって損はない）。

📌 **「latched にしたから遅れて繋いでも見える」を検証せずに信じないこと。**
この確認は `ros2 topic echo --once --qos-durability transient_local` で取れる。

### 結果（モック）

| # | 確かめたこと | 結果 |
|---|---|---|
| ① | TEACH でないクリックを無視するか | ✅ `teach_points: 0` のまま |
| ② | 3点クリックで溜まるか | ✅ `state: TEACH` / `teach_points: 3`。**`/g1/patrol/route` も 3 点**（RViz に出る） |
| ③ | `undo` で戻せるか | ✅ `(-4.3, -4.0) を取り消した（残り 2 点）` |
| ④ | 教示中に `start` を断るか | ✅ `success=False`「teach_save か teach_cancel してから」 |
| ⑤ | `save` で yaml に書き、そのまま読み込むか | ✅ 3点を書き出し、`waypoints: 3` に差し替わった。`route` も 3 点のまま |

方位も検算どおり（`wp1→wp2` が真西で 180.0°、`wp2→wp3` が真南で -90.0°、
`wp3→wp1` が `atan2(1.5, 1.3)` = 49.1°）。

---

## 8. ⚠️ これで言えないこと

- **実機では未検証。** モックに物理は無い
- **巡回路がまだ無い。** `config/patrol_room_a.yaml` は**空のひな形**。
  地図が 9/07 取得で現状と合っていない（A-10n）ので座標を手で書けない。
  現地で `tools/record_waypoints.py` を回すこと
- **1周の所要時間と内蔵SLAM の16分制限の関係**は現地の巡回路が決まるまで分からない。
  目安として**1点あたり 3〜5m 以内**にしておくこと
