# 実機の手順（ゴミ箱のフラップを押して、中を撮る）

ラボ PC（有線）で行う。接続とカメラ配信サーバの起動は Button_Press/J1-gen の手順書
（[REAL_DAY_PROCEDURE.md](../../Button_Press/J1-gen/real/REAL_DAY_PROCEDURE.md)、
[depth_server/README.md](../../Button_Press/J1-gen/real/depth_server/README.md)）と同じなので、そちらに従う。
ここでは、ゴミ箱に固有の段階だけを書く。

コマンドはリポジトリ直下で、1 行ずつ実行する。以下 `P=G1_HuggingFace/venv/bin/python`。

## 安全のルール

- **腰は動かさない**（`lean_waist_pitch_deg: 0`。安全のための判断）。`--lean-only` や `--waist-pitch` は使わない
- シミュレーションは pelvis を固定しているので、手を前に出したときのバランスは確かめていない。J1-gen には「腕を動かすと前に傾いて
  バランスを崩した報告がある」。初回は特に、倒れそうならすぐ支えられるようにする
- 人がリモコンを持つ（緊急時は **L2+B**）。もう 1 人が G1 のそばで、倒れそうなら支えられるようにする
- 送るとき（`--execute`）は確認モード（段階ごとに Enter。`q` で中止）で行う。`--no-confirm` は使わない
- **腕を伸ばしたまま Enter を待たない。** 2026-10-31 に、待っている間に体の傾きが 3° から 7.5° まで増えた。押す点の上まで来たら確かめてすぐ進むか、`q` で戻す
- **中止の基準はボタン押し（J1-gen）に合わせる**:
  - 送る前: `push_bin.py` が事前確認をする（`real/precheck.py`。J1-gen の `auto_press.py` と同じ: モーションコントローラ稼働、骨盤の傾き 5° 以下・腰 3° 以下、頭カメラの画像が動いている、腰・腕のモータ mode 1）。通らなければ何も送らない（終了コード 10）
  - 腕を動かしている間: 腰の角度が開始時から 2.9° ずれたら自動で中止して来た経路を戻る（J1-gen の `arm.yaml`）。危なければ人が L2+B
  - 1 回ごと: 表示される体の動き（`body_motion`）の傾きの変化が 1° を超えたら、次の段階（ゴミ箱を置く・ハーネスを外すなど）へ進まない（J1-gen が 2026-11-01 に使った目安）
- 吊らずに立たせるときは Lock Standing を使わない（公式の表でバランス制御なし）。ボタン押しと同じレギュラーモードで行う
- 途中で止めるときは **Ctrl+C**: 腕を来た経路で戻してから終わる。もう一度 Ctrl+C でその場で weight を下げる
- 経路は arm_sdk（プランA）だけ。lowcmd（プランB）はバランス制御が止まるので使わない
- **`run_g1_server.py` は使わない**（起動時に歩行の制御を解除する）。カメラは J1-gen の `rgbd_server.py`（5556 番）

## 前日まで（実機なし）

- [ ] 模擬ロボットで、当日と同じコマンドを通しておく（ループバックなので実機とは混ざらない）:
  ```bash
  $P Trash_Bin/sim/bin_robot_server.py                      # 端末 A（模擬ロボット）
  $P Trash_Bin/real/push_bin.py --network-interface lo --camera-config camera_sim.yaml --locate-only   # 端末 B
  $P Trash_Bin/real/push_bin.py --network-interface lo --camera-config camera_sim.yaml                 # dry-run
  $P Trash_Bin/real/push_bin.py --network-interface lo --camera-config camera_sim.yaml --air --execute # 空振り
  $P Trash_Bin/real/push_bin.py --network-interface lo --camera-config camera_sim.yaml --execute
  ```
  2026-11-08 に、この 4 つが通ることを確かめた（台 14 cm・腰 0°。模擬ロボットなので pelvis の高さは設定どおりの 0.750 m。
  模擬ロボットにはゴミ箱があるので、空振りでもフラップを押す）。
  近づく（`approach_bin.py`）は、模擬ロボットに歩行（LocoClient）の受け手が無いので dry-run だけ確かめられる:
  ```bash
  $P Trash_Bin/real/approach_bin.py --network-interface lo --camera-config camera_sim.yaml
  ```
- [ ] 台を用意する。条件は [README の「台の条件」](../README.md)。14 cm を基準に、12〜16 cm へ変えられるようにしておく（板を足すなど）

## 当日

順番は「腕を動かさずに測る → ゴミ箱なしで腕だけ → ゴミ箱を置いて押す」。前の段階が目安を満たさなければ、次へ進まない。
各段階の記録は `_local/trash_bin/runs/<日時>_<ラベル>/`（最後に場所を表示する）。

| 段階 | すること | 次へ進む目安 |
|---|---|---|
| 0 | 接続とカメラ | 接続確認がすべて OK |
| 1 | 頭カメラの向き | 45〜51° くらい（URDF 47.6° との差 10° 以内） |
| 2 | 置く前に、床から pelvis の高さを測り、台を決める | 台の高さが決まった |
| 3 | 空振り（ハーネスあり、ゴミ箱なし） | 体の傾きの変化 1° 以下、足が動かない |
| 4 | ゴミ箱を台に載せ、見つけるだけ・深度を撮る | 緑の長方形が上面に重なる |
| 5 | フラップを押す力を手で測る | 1 kgf（約 10 N）以下で開く |
| 6 | 立つ位置まで歩く（1 歩ずつ） | 着いた。歩いた量の表示が指令と大きく違わない |
| 7 | dry-run → 本番（確認モード） | 中の画像が撮れた |

### 0. 接続とカメラ

J1-gen の手順書の「段階0: 接続確認」と「深度付きカメラサーバ（PC2）」に従う。G1 は立った状態（レギュラーモード）。

### 1. 頭カメラの向きを確かめる

```bash
$P Button_Press/J1-gen/real/check_head_tilt.py --live --frames 5
```

通常は水平から約 45〜51° 下（URDF は 47.6°）。大きく違う（2026-10-21 は首が傾いて 71°、11-03 は 61°）ときは、首を直すか、
J1-gen の手順書のとおり `Button_Press/J1-gen/configs/camera.yaml` の `head_camera_mount` で直す。

### 2. 床から pelvis の高さを測り、台を決める（腕は動かさない）

床が広く写る所に立たせて、空振りの dry-run を流す（何も送らない。事前確認と、床から求めた pelvis の高さが出る）:

```bash
$P Trash_Bin/real/push_bin.py --air
```

- 「床から求めた pelvis の高さ」と「台の高さの目安」を書き留める。台の高さ = 14 cm +（pelvis の高さ − 75 cm）
- 2〜3 回測って、ばらつきが 1 cm 以内か見る。立ち直すと変わることがある（骨盤の傾きが変わる）

### 3. 空振り（ゴミ箱なし、ハーネスあり）

台とゴミ箱は置かない（腕が空中を押すだけ）。立つ位置の前は空けておく。

```bash
$P Trash_Bin/real/push_bin.py --air --execute
```

- 確認モード。押す点の上まで来たら、腕を伸ばしたまま待たずに Enter
- 最後に「体の傾きの変化の最大」と、次へ進んでよいかが出る。**1° を超えたら、ここで止めて相談する**
- 目で見ること: 足が動かないか、体がふらつかないか、腕が体や周りに当たらないか。動画を撮っておく
- 2〜3 回繰り返して、毎回同じくらいか見る

### 4. ゴミ箱を台に載せて、見つけるだけ（腕は動かさない）

台の条件は [README の「台の条件」](../README.md)（**後ろの壁から 30 cm 以上離す**、台はゴミ箱からはみ出さない）。
歩かせずに手で立たせる位置: ゴミ箱の正面がつま先の前 9 cm（pelvis から 23.5 cm 前）、中心が体の中心から右へ 15 cm、正面を向く。

```bash
$P Trash_Bin/real/push_bin.py --locate-only
```

- `*_locate_overlay.png` で、緑の長方形がゴミ箱の上面の縁、黄色が開口に重なっているかを見る
- 「シミュレーションで確かめた位置との差」が出たら、その向きに立ち直して、もう一度
- **深度を撮って持ち帰る**（鏡面でどれだけ欠けるかが、次の改良の材料）: 立つ位置と、0.5 m・1 m 離れた所で 1 回ずつ `--locate-only`。
  見つからなくても `*_locate_fail*_depth.png` が残る
- 見つからないときは、ここで押すのはやめる（深度の記録だけ残す）

### 5. フラップを押す力を測る（人の手で）

ばね秤かキッチンスケールで、フラップの右半分（軸から 5 cm、手前の縁から 9 cm の所）を真下に押し、開き始める力と、40° くらい
開いたときの力を測る。シミュレーションは弱いばね（押す力 4 N 以下）で近似している。10 N を大きく超えるなら、押す前に相談する。

### 6. 立つ位置まで歩く（1 歩ずつ）

リモコンで、ゴミ箱の**おおよそ正面・1 m 以内**まで連れて来る。両腕は下ろしたまま。

```bash
$P Trash_Bin/real/approach_bin.py              # dry-run: 1 回測って、どう歩くかを表示するだけ
$P Trash_Bin/real/approach_bin.py --execute    # 歩く（1 歩ごとに Enter。q で中止）
```

- 最初は dry-run で、表示された「ずれ」と「指令」が実際の位置関係と合っているかを見る（例: ゴミ箱が左にあれば vy が +）
- 1 歩ごとに「前回の 1 歩: 指令 … → 測った …」が出る。最後に「測った ÷ 指令」の集計が出る（前後は 2 cm ほどの精度、
  左右と向きは目安）。数 cm の指令で動かない・行き過ぎるなら、`configs/bin_scene.yaml` の `approach` を直す材料になる
- 歩き方がおかしければ q か Ctrl+C（必ず StopMove する）、危なければ L2+B
- 着いたら（ずれが前後左右 2 cm・向き 3° 以内）7. へ

### 7. dry-run → 本番（確認モード、最初はハーネスあり）

```bash
$P Trash_Bin/real/push_bin.py              # dry-run。「結果: 成功」なら計画が通っている
$P Trash_Bin/real/push_bin.py --execute    # 本番
```

段階ごとに Enter: 開始 → 経由の姿勢 → 押す点の上 → 押し込み → 保持（1.5 秒、この間に 3 回撮る）→ 戻る。
中の確認用の画像は `frames/` の下の `*_end_color.png`（押し込みが止まった瞬間）と `*_hold1〜3_color.png`（保持中）。
最後に体の傾きの変化が出る。1° を超えたら、次の回へ進まない。

## 持ち帰るもの

`_local/` は git に入らない。次の記録をまとめて共有する（Slack や報告）:

- 2 の pelvis の高さ（何回か）、決めた台の高さ
- 3 の空振りの記録フォルダ（`body_motion.csv`・`events.jsonl`）と動画
- 4 の深度（`*_locate_*`、立つ位置・0.5 m・1 m）
- 5 のフラップの力
- 6 の歩いた量の集計（画面の表示と `events.jsonl` の `walk_measured`）
- 7 の中の画像（`*_end_color.png`・`*_hold*_color.png`）と `body_motion.csv`
- Jigen さんの 11/3 の生ログ（`lowstate`・`body_log`）。立ったときの脚の角度で、pelvis の高さの推定を確かめる
