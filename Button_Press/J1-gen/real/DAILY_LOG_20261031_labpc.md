# 2026-10-31 の記録（ラボ PC 側・有線）

[DAILY_LOG_20261021_labpc.md](DAILY_LOG_20261021_labpc.md) の「次回の再開手順」の続き。接続確認・頭カメラの向き・机の箱・
dry-run・ボトル押し（既定 4 回、広げた設定 1 回）まで進めた。**次回は下の「次回の再開手順」から始める。**
人が読む用のまとめ（判断したこと・予想と違ったこと）は [REPORT_20261031.md](../docs/reports/REPORT_20261031.md)。
端末の出力は `_local/button_press/terminal_logs/20261031_labpc_press_bottle_terminal.log`。
一般化した注意点は `docs/knowledge/`（`tech/shell_and_ssh.md`・`patterns/robot_arm_control.md`・`real_robot_notes.md` の 2026-10-31）。

## 結果のまとめ

| 段階 | 結果 |
|---|---|
| 0 接続確認 | OK（lowstate 1042 Hz、`mode_machine = 5`、立ったあとは腰・腕のモータの mode すべて 1、`ai` 稼働中 → arm_sdk）。develop mode のうちは mode 0 |
| RGB・深度 | PC2 の `rgbd_server` を起動して届いた（640×360、fx 454.2、深度 0 の画素 10%）。**G1 のモードを切り替えるたびに PC2 が再起動して配信が消えた**（この日 3 回） |
| 頭カメラの向き | **50.2°**（立った状態。URDF 47.6° とほぼ同じ。首は傾いていない。上書き不要） |
| 机の箱 | 丸い机。立った状態の深度から、左右 9 cm ごとの 3〜5 個の箱。**立ち直すたびに位置が変わるので作り直した**（最後は 3 個、縁 36〜38 cm、天板 z +0.005 m） |
| ボトル押し（既定） | 4 回: 成功（途中で停止 15.7 mm）、成功（押し切り 61.3 mm）、事前確認で停止（傾き 5.6°）、腰のずれで中止（yaw 2.96°） |
| ボトル押し（広げた設定） | 1 回（確認モード、ボトル固定）: 成功（接触で停止 17.6 mm）。体が前へ 84 mm・7.5° 傾いた |
| 置ける範囲 A〜F | **試せなかった**（他の人が G1 を使っていた）。代わりに実際の配置 4 通りの dry-run（REPORT の表） |
| エレベーター | 行けなかった（ラボのホールに入れない） |

## 今日の実機の押し込み（腕に送った 5 回）

| 回 | 時刻 | コマンド | 設定 | 結果 |
|---|---|---|---|---|
| 1 | 20:34 | `press_bottle.py --path arm_sdk --execute`（確認モード） | 既定、`lag_m` 5 mm、ボトル固定なし | 15.7 mm（29/109 点）で停止。遅れ 10.9 mm（押す前から 5.8、増えた分 5.1）。ボトルの約 2 cm 手前。体 2.8 mm・1.04° |
| 2 | 20:43 | `auto_press.py --target bottle --execute` | 既定、`lag_m` 10 mm、傾き上限 5.0°（開始 4.8°）、固定なし | 61.3 mm（109/109 点）。増えた分 3.4 mm。体 3.4 mm・0.77°。目視で約 25 mm 押し込み（滑った） |
| 3 | 20:56 | 同上 | 同上 | 事前確認で停止（骨盤の傾き 5.6°、腰 pitch −2.1°）。送っていない |
| 4 | 20:57 | 同上（ボトルを固定した直後） | 同上 | 中止（終了コード 5）。腰 yaw 2.96°（上限 2.9°）。経由の姿勢へ移動中に pelvis が前へ 65 mm（5.3 秒に 50 mm へ急に）。戻るときも腰のずれが 4.41° に増えて、その場で weight を下げた。ボトルの深度のばらつきが 4.7 / 2.3 / 8.4 mm と大きかった |
| 5 | 21:18 | `press_bottle.py --path arm_sdk --overlay reach_wide.yaml --execute`（確認モード） | 広げた設定、ボトル固定、立ち直した後（傾き 0.7° で開始） | 17.6 mm（42/109 点）で停止。遅れ 24.9 mm（押す前から 14.6、増えた分 10.3）。体は最大 84 mm・7.5°（IMU 7.9°）、66 秒に急に増えた |

どの回も、深度の較正値は 0（未較正）。`record.py` の録画は、回 1〜4 が `…203216_bottle`、回 4 の前後が `…205544_bottle_fixed`、回 5 が `…210736_bottle_fixed2`。

## はまったこと

### PC2 に ssh できなかった（Claude が起動・確認をするため）
- ラボ PC の鍵が PC2 に登録されておらず、`Permission denied (publickey,password)`。`ssh-copy-id` は `!` ではパスワードを入力できず失敗
- 別のターミナル（パスワードで入れる）で、`authorized_keys` に公開鍵を足してもらった。**このとき `…` で省略した例を書き、そのまま実行されて無効な行が入った**（`sed` で消した）
- 登録後も `~/.ssh/config` が `PubkeyAuthentication no` なので、`-o PubkeyAuthentication=yes -o PreferredAuthentications=publickey -i ~/.ssh/id_ed25519_humanoid_expo` を毎回付ける。手順は `docs/knowledge/tech/shell_and_ssh.md`
- `ssh host 'cmd &'` は、出力を掴んで返らない。`( timeout 20 ssh ... 'cd ~/button_press && setsid nohup <コマンド> > log 2>&1 < /dev/null &' ; true )` で起動し、別の ssh で確かめる

### PC2 が再起動して、カメラの配信が消えた（3 回）
- G1 を develop mode からスタンディングにしたとき（19:20 ごろ）、中止のあと（20:00 ごろ）、もう 1 回、PC2 の `uptime` が数分だった
- `check_connection.py --rgbd` が「深度付きストリームが届かない」になる。`record.py` は lowstate だけを保存し続け、画像は止まる
  （`bottle_fixed` は 20:58:16・148 秒で画像が止まった。ログに `ddsi_udp_conn_write ... failed` が出る）
- 起動し直した: `cd ~/button_press && setsid nohup bash real/depth_server/start_rgbd_server.sh --source videohub --max-fps 5 > rgbd_server_<日時>.log 2>&1 < /dev/null &`
  （ログは PC2 の `~/button_press/rgbd_server_20261031{,b,c,d,e}.log`）
- `videohub_pc4 /dev/video4` は Unitree のサービス（root）で、そのままでよい

### 立ち直すと、位置関係が変わる
- 1 回の立ち直しで、ボトルは pelvis から見て 3 cm 遠く・右が 5.7 cm 内側・3.6 cm 低く、机の天板は z +0.037 → +0.004 m。前の箱は合わなくなる
- 立ち直したら、箱（`make_table_boxes.py --live --save`）とボトルの位置を測り直す

### 事前確認の骨盤の傾き（上限 3.0° → 5.0°）
- 立った状態の傾きが 4.8°（のち 5.6°、5.9°）で、事前確認が止まった。**ユーザーが上限を 5.0° に上げた**（`MAX_PELVIS_TILT_DEG`、まだコミットしていない）
- 傾きが 4.8° のままの次の回は、押している途中で腰 yaw が 2.96° ずれて中止した。立ち直すと 0.7° に戻った
- 骨盤の傾きは、誰かが G1 に触れたり、腕を伸ばして待ったりすると増える

### 止め方（`contact_stop.lag_m`）と、滑るボトル
- 5 mm: 触れる前に止まった（ボトルの約 2 cm 手前）。10 mm（**ユーザーが決めた**、`configs/press.yaml`）: 滑るボトルは止まらず押し切った。
  固定すると 10 mm で止まった（回 5）。力のセンサが無く、位置の遅れで見ているので、ボトルが逃げると当たったと分からない
- 広げた設定は、押す前からの遅れが 14.6 mm と大きい（既定は 5.8〜6.8 mm）。止める判定は増え方で見ているので、押す前の値を引いている

### 広げた設定で、体が前へ傾いた
- 手前の姿勢へ向かう途中（24〜27 秒）に pelvis が 36 mm・3.3° 前へ、その後も少しずつ増え、Enter を待つ間の 66.7 秒にさらに 84 mm・7.5° へ。足の左右の位置関係は 1.7 mm しか変わらず、脚の関節は 3〜5.5° 動いた
- 脚のトルクの記録が無く、バランス制御が働いていたかは決められない（ユーザーの感覚は「途中でバランス制御がなくなったか、最初から無かった」）。
  ボトルは pelvis から見て約 57 mm 手前に動き、押し込みが 17.6 mm で止まった（実際のボトルが近かったことと合う）

### 録画が大きい
- `record.py` は 1 秒に約 2.2 MB（5 fps の深度付き＋lowstate）。この日は合計 約 8 GB（下）

## 測った値

- 頭カメラの向き: 50.2°（`_local/button_press/head_tilt/20261031_202250.json`。develop mode のうちは 50.2〜56° とばらついた）
- 机の箱（立ち直した後の最後）: 3 個、手前の縁 x 36.3 / 36.3 / 37.5 cm、天板 z +0.005 m
- 腕の追従の遅れ（重力補償 0.5、押す前から）: 既定 5.8〜6.8 mm、広げた設定 14.6 mm
- 経由の姿勢の追従誤差: 既定 2.2〜3.5°、広げた設定 3.06°・3.78°（許容 2.9° を超えて ⚠️）
- ボトル（pelvis 座標）: [+0.352, −0.188, +0.136]（回 1）、[+0.357, −0.187, +0.138]（回 2）、[+0.338, −0.192, +0.141]（回 4）、[+0.379, −0.152, +0.100]（回 5）

## 変えたもの（未コミット）

| ファイル | 変更 | 決めたのは |
|---|---|---|
| `configs/press.yaml` | `contact_stop.lag_m` 0.005 → 0.010（`retry_lag_m` は 0.008 のまま。ボトルは押し直さないので効かない） | ユーザー |
| `real/auto_press.py` | `MAX_PELVIS_TILT_DEG` 3.0 → 5.0 | ユーザー（傾きが 0.7° に戻ったので、戻すかは次回決める） |
| `configs/obstacles.yaml` | 立ち直すたびに `make_table_boxes.py --live --save` で作り直し（最後は 3 個の箱） | — |
| `docs/knowledge/`・`docs/reports/REPORT_20261031.md` | 知識の追記、報告書の下書き | — |

## 今日のデータ（`_local/`、git に入らない。**バックアップする**）

| 場所 | 大きさ | 中身 |
|---|---|---|
| `recordings/20261031_203216_bottle` | 2.4 GB（5624 枚） | 回 1〜4 の深度付き映像・lowstate |
| `recordings/20261031_205544_bottle_fixed` | 327 MB（画像は 20:58:16 で止まった） | 回 4 の前後 |
| `recordings/20261031_210736_bottle_fixed2` | 5.2 GB（12898 枚、lowstate 243912 行） | 回 5 を含む |
| `runs/20261031_203402`・`204342_auto`・`205720_auto`・`211830` | 各 4〜8 MB | 実際に送った 5 回のうち 4 回（回 3 は事前確認で止まったので無し） |
| `auto/20261031_*_bottle.json` | — | `auto_press.py` の結果（事前確認を含む） |
| `table_boxes/`・`head_tilt/`・`terminal_logs/` | 合わせて 100 KB 前後 | 机の箱の図、頭カメラの向き、端末の出力 |

dry-run の `runs/`（`202737`・`202746`・`203015`・`203053`・`204141_auto`・`211458_auto`・`211620_auto`・`211659_auto`）は、再現できるので保存しなくてよい。

## PC2・ラボ PC の状態（このログの時点）

- `record.py`: **止めた**（最後に `bottle_fixed2` を正常終了。受信のタイムアウト 0 回）
- PC2: `rgbd_server`（`--source videohub --max-fps 5`、ログ `rgbd_server_20261031e.log`）は**動いたままの可能性がある**。レギュラーモードに切り替えると PC2 が再起動して消える
- G1: 立った状態。腕は戻して weight 0 にしてある（回 5 の「安全終了」）。このあとレギュラーモードに切り替える予定

## 次回の再開手順

1. `git status`（未コミットの変更を確認する）。ラボ PC で `export CYCLONEDDS_HOME=$PWD/G1_HuggingFace/cyclonedds/install`
2. G1 をレギュラーモードにする（立ち直したあとは、**PC2 が再起動する**ので、配信が消える）
3. PC2 のカメラ配信を起動して確かめる（Claude が ssh で起動できる。手順は `docs/knowledge/tech/shell_and_ssh.md`）:
   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_connection.py --rgbd --seconds 5
   ```
4. 頭カメラの向き（50.2° が通常）→ 机の箱を作り直す → ボトルをテープで固定する:
   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_head_tilt.py --live --frames 5
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/make_table_boxes.py --live --save
   ```
5. dry-run（事前確認を含む）→ Enter なしで押す。録画は、別のターミナルで `record.py --label bottle_fixed3`（押し終わったら止める）:
   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/auto_press.py --target bottle
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/auto_press.py --target bottle --execute
   ```
6. 置ける範囲 A〜F（REPORT の「2.」）。広げた設定（`-- --overlay reach_wide.yaml`）は、体の傾き（`runs/<日時>/body_motion.csv` の `imu_tilt_deg`）を見ながら
7. エレベーターは、ホールに行けるときに（REAL_DAY_PROCEDURE の段階 7）。YOLO の重み（`best.pt`・`best.onnx`）がラボ PC に無い → ノート PC から持ってくる
8. 実機で撮ったものをバックアップする（上の表）。`press.yaml`・`auto_press.py` の変更は、残すか戻すかを決めてからコミットする
