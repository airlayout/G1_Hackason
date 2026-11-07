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
  $P Trash_Bin/real/push_bin.py --network-interface lo --camera-config camera_sim.yaml --execute
  ```
  2026-11-07 に、この 3 つが通ることを確かめた（台 14 cm・腰 0°。模擬ロボットなので pelvis の高さは設定どおりの 0.750 m）。
  近づく（`approach_bin.py`）は、模擬ロボットに歩行（LocoClient）の受け手が無いので dry-run だけ確かめられる:
  ```bash
  $P Trash_Bin/real/approach_bin.py --network-interface lo --camera-config camera_sim.yaml
  ```
- [ ] 台を用意する。条件は [README の「台の条件」](../README.md)。14 cm を基準に、12〜16 cm へ変えられるようにしておく（板を足すなど）

## 当日

### 0. 接続とカメラ

J1-gen の手順書の「段階0: 接続確認」と「深度付きカメラサーバ（PC2）」に従う。G1 は立った状態（歩行の制御が入った状態）。

### 1. 頭カメラの向きを確かめる

```bash
$P Button_Press/J1-gen/real/check_head_tilt.py --live --frames 5
```

通常は水平から約 45° 下（URDF は 47.6°）。大きく違う（2026-10-21 は首が傾いて 71°）ときは、J1-gen の手順書のとおり
`Button_Press/J1-gen/configs/camera.yaml` の `head_camera_mount` で直す。

### 2. ゴミ箱の前まで来て、立つ位置まで歩く

リモコンで、ゴミ箱の**おおよそ正面・1 m 以内**まで連れて来る（頭カメラにゴミ箱の上面が写る範囲。体の前 0.3〜1.2 m）。
**両腕は下ろしたまま**（腕が頭カメラの視野に入らないように）。そこから、頭カメラで測りながら立つ位置まで歩く:

```bash
$P Trash_Bin/real/approach_bin.py              # dry-run: 1 回測って、どう歩くかを表示するだけ
$P Trash_Bin/real/approach_bin.py --execute    # 歩く（1 歩ごとに Enter。q で中止）
```

- 最初は dry-run で、表示された「ずれ」と「指令」が実際の位置関係と合っているかを見る（例: ゴミ箱が左にあれば vy が +）
- `--execute` は 1 歩ずつ Enter で進める。歩き方がおかしければ q か Ctrl+C（必ず StopMove する）、危なければ L2+B
- G1 の安全な歩行速度は未確認。速さの上限は `configs/bin_scene.yaml` の `approach`（前 0.15 m/s など。Navigation より遅い）
- 着いたら（ずれが前後左右 2 cm・向き 3° 以内）3. へ

歩かせずに手で立たせる場合の位置: ゴミ箱の正面がつま先の前 9 cm（pelvis から 23.5 cm 前）、ゴミ箱の中心が体の中心から右へ 15 cm、
ゴミ箱に正面から向く（シミュレーションで繰り返し確かめたのは ±2 cm・±5° まで）。

### 3. 見つけるだけ（腕は動かさない）

```bash
$P Trash_Bin/real/push_bin.py --locate-only
```

- 記録（`_local/trash_bin/runs/<日時>_見つけるだけ/frames/`）の `*_locate_overlay.png` で、緑の長方形がゴミ箱の上面の縁、
  黄色が開口に重なっているかを見る
- 「シミュレーションで確かめた位置との差」が出たら、その向きに立ち直して、もう一度。差が出なくなるまで
- **「床から求めた pelvis の高さ」を見る。** 台の高さ = 14 cm +（その値 − 75 cm）。違っていれば台を変えて、ここからやり直す。
  4 cm より違うと止まる（測り間違いの可能性）ので、1. の頭カメラの向きから確かめる
- **見つからないとき**（「上面の高さの点が少ない」など）: ステンレスの鏡面で深度が欠けている可能性が高い。
  `*_locate_fail*_depth.png` を残して、中止する（この記録が次の改良の材料になる）

### 4. dry-run（送らない）

```bash
$P Trash_Bin/real/push_bin.py
```

「結果: 成功」なら計画が通っている。通らなければ、表示された理由（届かない・ぶつかる）を見て 2. に戻る。

### 5. 本番（確認モード）

```bash
$P Trash_Bin/real/push_bin.py --execute
```

段階ごとに Enter: 開始 → 経由の姿勢 → 押す点の上 → 押し込み → 戻る。
押し切った瞬間の頭カメラの画像は `frames/` の下の `*_end_color.png`（中身の確認に使う）。

## 記録

`_local/` は git に入らない。必要な記録（`*_locate_overlay.png`・`*_end_color.png`・`events.jsonl`）は、結果と一緒に
Slack や報告に残す。
