# 2026-11-01 の記録（ノート PC 側・無線。onboard）

この日は、**ラボ PC を使わず、ノート PC（WSL2）から Wi-Fi の ssh で G1 の PC2 に入り、押す処理を PC2 の上で動かした**（onboard の段階 2）。
ここには細かいこと・技術的なことをほぼすべて書く。人が読む用の要約は [REPORT_20261101.md](../docs/reports/REPORT_20261101.md)。
午後（12:34 に G1 が起動し直したあと）は、**ノート PC を有線で G1 につなぎ、エレベーターの呼びボタン（車いす用 ▼）を 5 回押した**（「エレベーター」の節）。
前回は [DAILY_LOG_20261031_labpc.md](DAILY_LOG_20261031_labpc.md)。時刻は PC2 の時計（中国時間 CST。日本時間より 1 時間遅い）。runs のフォルダ名も同じ。

## 結果のまとめ

| 段階 | 結果 |
|---|---|
| つなぐ | ノート PC の Wi-Fi が iPhone のテザリングだと届かない。`Physical_AI_5G`（192.168.0.84）にすると `ssh unitree@192.168.0.82` で入れた。PC2 の Tailscale（100.78.135.14）は自分の tailnet に無い |
| PC2 の準備 | コード・重み・`Perception/common` を送り、ボトルの ONNX（`~/perception/yolo26n.onnx`）をコードのコピーの `_local/perception/models/` に置いた |
| カメラの配信 | `rgbd_server --source videohub --max-fps 5 --no-legacy-rgb`（5556 だけ）。10:56 に videohub のカラーが code 3104 で取れず終了 |
| 接続確認 | すべて OK（lowstate 1042 Hz、`mode_machine = 5`、mode すべて 1、深度 0 の画素 11%、カラーの変化 4.62）。立つ前は腰 −12.7 / −3.7 / −6.1° |
| 頭カメラの向き | 51.2°（上書き不要）。骨盤の傾き 2.5° |
| 机の箱 | 余裕 1 cm、11 個。縁 34.5 cm（右 y −0.45）〜40.0 cm（左 y +0.45）で約 4° 斜め、天板 z −0.005 m。PC2 の `~/button_press/obstacles_20261101.yaml` |
| dry-run（PC2 の上） | 前の既定・広げた設定とも、事前確認が通り計画が立った（ボトルの検出は ONNX） |
| `stop` の確認 | PC2 の dry-run の途中で SIGINT → 安全に終了（dry-run は戻らない）。模擬ロボットの `--execute` で、戻る動きを確かめた |
| 送った（前の既定） | **成功**。当たって止まった 43.1 mm、開始姿勢まで戻った、体の動き 1.0 mm |
| 送った（広げた設定） | 当たって止まった 40.4 mm の直後に `NoMotionError`（終了コード 3）、来た経路を戻った。体が前へ 10 mm。別の班の VLA（`--execute`）が同時に動いていた |
| エレベーター：つなぐ | ノート PC から有線（WSL2 を mirrored にして `eth1` = 192.168.123.99）。lowstate 1042 Hz、深度付きの画像 OK |
| エレベーター：見つける | **YOLO の最終版で車いす用の ▲▼ を 15 / 15 枚**（classic は 5 / 15 で、選んだ組も床の模様で誤り） |
| エレベーター：送った（5 回） | 位置は 3 回目で合った（ボタン用の補正 x −2〜−3 cm・y +2 cm）。**押す力が足りず、5 回とも点灯しなかった**。押し込みを 30 mm にした 5 回目で体の傾きが 1.13° 変わったので、そこでやめた |

## 時刻順の記録（PC2 の時計）

| 時刻 | できごと |
|---|---|
| 09:44 | PC2 が起動（`uptime` 4 分） |
| 09:48 | 無線の ssh がつながった。`videohub_pc4 /dev/video4` と `videohub_pc4_chest /dev/video10` が動いていた。5555・5556 は空き |
| 09:50 | コードを送り、カメラの配信を起動。`check_connection`（PC2 の上、eth0・`camera_onboard.yaml`）すべて OK。腰が −12.7 / −3.7 / −6.1°（まだ立っていない） |
| 10:09 | 立った（腰 −0.1 / +0.2 / −2.3°）。頭カメラの向き 51.2° |
| 10:09〜10:10 | 机の箱（確認 → `--obstacles-file ~/button_press/obstacles_20261101.yaml` に保存） |
| 10:14 | dry-run（前の既定）回 1。10:15 dry-run（広げた設定）回 2 |
| 10:15 | 別の班の `python3 -m g1_e2e.vla.robot` が起動（CPU 55%） |
| 10:27〜10:31 | `onboard.sh` の dry-run と `stop` の確認（1 回目はパスの誤り、2 回目は出力のバッファで間に合わず、3 回目で `stop` が効いた） |
| 10:38 | ボトルを動かしたあとの dry-run（[0.342, −0.171, 0.087]）。カメラの画像で配置を確かめた |
| 10:41 | **回 3: 前の既定で送った**（Claude が `onboard.sh start --execute -- --overlay reach_legacy.yaml`、ユーザーが「送ってよい」と確認したあと）。成功 |
| 10:45 | ユーザーが広げた設定の dry-run → `--` のあとが無いと空の引数を渡す誤りで終了コード 2（何も送っていない）。直した |
| 10:49 / 10:51 | ユーザーが広げた設定の dry-run を 2 回（どちらも終了コード 0） |
| 10:49:48 | 別の班の `g1_e2e.vla.robot --execute` が起動 |
| 10:52 | **回 4: ユーザーが広げた設定で送った**。当たって止まった直後に `NoMotionError`、戻った。途中で G1 が「no wifi」のような音声を出した |
| 10:56 | `rgbd_server` が終了（videohub のカラーが code 3104） |
| 11:43 | 別の班の `g1_e2e.vla.robot --execute` が（別の pid で）また動いていた。カメラの配信を起動し直し、接続確認・机の箱（`obstacles_20261101b.yaml`）・dry-run 2 つ |
| 11:47 | VLA が止まっていた。`onboard.sh` に、`g1_e2e` が動いている間は送らない確認を足した（`e80b278`） |
| 11:48 | **回 5: 広げた設定で送った**（VLA なし）。成功（42.2 mm、終了コード 0） |
| 11:49 | **回 6: 前の既定で送った**（VLA なし）。当たって止まった（44.4 mm）直後に `NoMotionError`（0.04 倍）→ 戻った。**VLA でも広げた設定でもないと分かった** |
| 11:55 | 原因（「押し込みを少し戻す」の誤判定）を直した（`d6215b1`）。テスト 265 件 OK、PC2 に送った |
| 11:57〜12:01 | **回 7〜10: 直したコードで、前の既定と広げた設定を交互に 2 回ずつ送った。4 回とも成功**（中止なし） |
| 12:05〜12:25 | 置ける範囲（P1〜P6、dry-run だけ）。P2 の広げた設定の dry-run は、12:09 に止まった配信（下の 12:09:20 の行）で事前確認止まり → 空・壊れたカラーで落ちないように直した（`a0d69a7`）、12:11 に起動し直してやり直した |
| 12:06〜12:10 | 胸のカメラを見た（見るだけ。別の会話の Claude）。**胸のカメラは無い**（「胸のカメラ」の節） |
| 12:09:20 | 頭の `videohub_pc4` が止まり、12:09:26 に `rgbd_server` が落ちた（10:56 と同じ `imdecode` の例外）。videohub は 12:09:51 に master_service が起動し直した |
| 12:34ごろ | G1 が起動し直された（12:56 に uptime 22 分）。ホールへ移動 |
| 12:5x | ノート PC を有線でつないだ。WSL2 が NAT のままでは lowstate が届かず、ユーザーが `.wslconfig` を mirrored にして `wsl --shutdown` |
| 12:56〜12:57 | 接続確認（`eth1`）。`rgbd_server` が止まっていたので起動し直した（最初に venv の Python で起動して pyrealsense2 が無く失敗 → システムの Python 3.8 で）。別の班の `g1_e2e.vla.robot`（`--execute` なし）が動いていた |
| 12:59〜13:02 | ホールで写真（`record.py`）→ `check_button_photos.py`。立つ位置を直して撮り直し |
| 13:02〜13:04 | dry-run（柱まで 41 cm で「遠すぎる」→ 前へ動かして 32 cm）。`max_retries` を 0 に |
| 13:05〜13:17 | **エレベーターの回 1〜5 を送った**（ユーザーが自分のターミナルで。各回の前に Claude が dry-run と `g1_e2e` の確認） |
| 13:21 | `rgbd_server` を止めた（7177 フレーム）。`max_retries` を 1 に戻した |

## 回 4 の `NoMotionError` の細かいこと

- `events.jsonl`: 21.4 秒 手前の姿勢 → 22.9 秒 計算し直し → 25.1 秒 押し込みの結果（`stopped: True`、`tip_advance_m 0.0404`、`baseline_lag_m 0.0063`、`extra_lag_m 0.0100`、83/109 点）→ その直後の動きで例外 → 47.4 秒 結果（終了コード 3）
- 例外の文: 「腕全体で指令の 0.08 倍しか動いていない。関節ごと: right_shoulder_pitch_joint=+0.02, right_elbow_joint=+0.14」
- 前の既定の回 3 では、同じ「押し込みを止めた直後」の 3 つの動き（追従誤差 1.64°・1.63°・1.80°）がすべて OK だった
- 戻り: 押す力が抜ける点まで（14 点、0.3 秒）→ 押す向きと逆に手前の姿勢へ（62 点、1.2 秒）→ 経由の姿勢へ（49.5°、5.2 秒）→ 開始姿勢へ（100.2°、10.5 秒）。追従誤差はどれも 2.0° 以下
- 体の動き: pelvis 最大 10.0 mm [+9.8, −2.2, −0.3]（39.9 秒、押し込みのラベル）、回転 0.59°、IMU の傾きの変化 0.70°、左右の足の間 5.2 mm。回 3 は 1.0 mm・0.29°・0.36°
- 送る間隔は記録から未計算（回 3 は中央値 20.0 ms・99% 点 22.3 ms・最大 173 ms）
- 原因の候補: (a) 別の班の VLA（10:49:48 から `--execute`）が同時に指令を送っていた、(b) 広げた設定の姿勢で、固定したボトルに押し当てたあとの小さな動きに腕がついてこない。どちらかは、別の班が止まっているときにもう一度送れば分かる

## `NoMotionError` の原因（11:49 に分かった。回 4・6）

- 回 5（広げた設定、VLA なし）は成功、回 6（前の既定、VLA なし）で同じ中止 → 原因は別の班の VLA でも、広げた設定でもない
- 中止したのは、押し込みを止めた直後の「押し込みを少し戻す」（`common/press.py`。止めた点から、実際の指先 + keep の点まで、押し込みの直線をさかのぼる）。
  回 4・6 とも、次の段階（`end`）の記録が無い
- この動きは**指令を、実際の指先（当たって遅れている所）まで戻すだけ**なので、腕はほとんど動かないのが正しい。ところが `ArmCommander.follow()` は、どの動きのあとにも
  `check_motion` で「指令した分の 0.3 倍以上、腕全体で動いたか」を見ていて、指令の変化が `min_commanded_rad`（0.02 rad ≈ 1.1°）を超える関節があると判定する。
  遅れ方（押す前の遅れ 4.5〜6.3 mm、増えた分 10 mm）しだいで、超えたり超えなかったりして、通る回（3・5）と中止する回（4・6）に分かれた
- 中止したときに戻る処理の最初の「押す力が抜ける点まで下がる」も同じ形の動き（今日は 2 回とも通ったが、同じ誤判定で戻るのをやめることがありうる）
- 直したこと（`d6215b1`）: `follow(expect_motion=False)` で「動いたか」の判定を外す（追従誤差の表示は残す）。「押し込みを少し戻す」と、戻るときの hold の段で使う。
  単体テスト（腕が動かない偽物で、`expect_motion=False` なら止めず、既定なら `NoMotionError`）を足した
- 直したコードで、前の既定・広げた設定を交互に 2 回ずつ（回 7〜10）送り、4 回とも中止せずに最後まで通った（11:57〜12:01）

## カメラの配信が止まった（10:56）

`~/button_press/rgbd_server_20261101.log` の最後:
```
[rgbd_server] 18378 フレーム配信（直近 5.0 fps、カラーと深度の時刻の差 19 ms、位置合わせ 107 ms）
[rgbd_server] ⚠️ videohub からカラーを受け取れない（code=3104、1 回続けて）
[rgbd_server] 終了（18380 フレーム）
...
  File ".../rgbd_server.py", line 263, in complete
    img = cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_COLOR)
cv2.error: ... (-215:Assertion failed) !buf.empty() in function 'imdecode_'
```
- videohub の RPC が時間切れ（3104）になり、空のデータを `imdecode` に渡して落ちた。1 回の失敗で落ちるのは `rgbd_server.py` の弱いところ（直すなら、空なら飛ばして続ける）
- PC2 の wlan0 はつながったまま。`journalctl` に Wi-Fi が切れた記録は無かった（ssh の切断だけ）

### 12:09 にもう一度止まった（videohub は約 72 分ごとに止まる？）

- `~/button_press/rgbd_server_20261101b.log`: 7518 フレームのあと code 3104 → 終了（7539 フレーム）→ 10:56 と同じ `imdecode` の例外。
  その前に 1 回だけ code 0 で中身が無い失敗もあった（落ちなかった）
- `master_service.LOG`（`/unitree/var/log/master_service/`）: `check current service protect count. name:video_hub_pc4` が
  10:56:43〜10:57:09 と 12:09:20〜12:09:51 に出ている（止まった videohub を起動し直している）。videohub の起動と止まった時刻:
  09:44:36 → 10:56:43（72 分 7 秒）、10:57:08 → 12:09:20（72 分 12 秒）。**どちらも約 72 分**
- 12:09 は、胸の窓口に 3 回・頭の videohub に 1 回、画像を頼んだ時刻（12:09 ごろ、頭は code 3104）と重なる。10:56 のときは頼んでいないので、
  周期の癖と考えるが、頼んだこととの関係は否定しきれない。次に止まるのが 13:21 ごろなら周期
- `rgbd_server` は起動し直していない（本線の会話に任せた）。空のカラーを飛ばす修正は `a0d69a7`

## 置ける範囲（ボトルの左右・奥行き。12:05〜12:25、dry-run だけ）

机・G1 は動かさず（手前の縁 約 37 cm、机の箱 `obstacles_20261101b.yaml`、余裕 1 cm）、ボトルだけを置き直して、前の既定（`--overlay reach_legacy.yaml`）と
広げた設定（今の既定）の dry-run を 1 回ずつ流した（PC2 の上で、`onboard.sh start`。何も送っていない）。ボトルの位置は、毎回カメラと深度で測った値
（pelvis 座標、ボトルの手前の面。較正値 0）。写真は `_local/button_press/placements_20261101/P*.jpg`。作業用のまとめのスクリプトは scratchpad の `place.sh`（リポジトリの外）。

| 配置 | ボトル [x, y, z] m | 前の既定 | 広げた設定 |
|---|---|---|---|
| P1 右 17 cm（A 相当） | [0.337, −0.174, 0.082] | ✅ 通る（経由へ 90.8°） | ✅ 通る（100.3°） |
| P2 右 11 cm | [0.345, −0.111, 0.084] | ✅ 通る（「勧める範囲の外。右に 15〜25 cm がよい」の注意だけ） | ✅ 通る |
| P3 右 5 cm（C 相当） | [0.351, −0.053, 0.083] | ❌ 手前の姿勢で右肩（yaw link）が胴体に当たる（−19.6 mm） | ✅ 指を左へ 15° で通る |
| P4 正面 0 cm（D 相当） | [0.354, −0.004, 0.084] | ❌ 手前の姿勢で右肩が胴体に当たる（−8.0 mm） | ✅ 指を左へ 30° で通る |
| P5 左 5 cm | [0.353, +0.047, 0.082] | ❌ 右肩が胴体に当たる（−17.9 mm） | ❌ 指 0〜30° は右肩が胴体に当たる。**指 45° は体に当たらないが、押し込みの直線の 87〜89/109 点目で関節が 1 周期に 0.0116〜0.0119 rad 動く（上限 0.0100）**で断る |
| P6 右 17 cm・奥へ（A から 4.5 cm 奥） | [0.382, −0.176, 0.084] | ✅ 通る | ✅ 通る |

- 地図の予想（前の既定は右 10 cm より外側だけ、広げた設定は右 20 cm〜左 5 cm）と、P1〜P4 は合った。P5（左 5 cm）は地図では広げた設定で通る所だが、
  MuJoCo（10/29）でも失敗した配置で、実機でも断った
- P5 の 45° は、ボトルの表面（約 84 点目）より奥の「表面を越えて押す」部分で上限を少し超えるだけ。押し込みの速さ（`--speed-mm-s`）を下げるか、
  深さ（`--depth-mm`）を浅くすれば通る見込み（試していない）
- P6 は、縁から約 4.5 cm 奥。地図の注意（縁から 6 cm 以上奥だと押し込みの直線で関節が速すぎる）の手前で、両方通った
- 机の縁までの距離を変える B（縁 25 cm）・E（縁 20 cm）・F（縁 40 cm）は、G1 を歩かせる必要があるので、試していない

## 胸のカメラ（12:06〜12:10、見るだけ）

- `videohub_pc4_chest /dev/video10`（pid 2057）は 09:44:36 から動いている（CPU 0%）。起動の設定
  `/unitree/etc/master_service/service/video_hub_pc4_chest` に `--exec .../videohub_pc4_chest /dev/video10` と書いてあり、起動のたびに同じ引数で起動される。
  `master_service.LOG` では 10/21 も含め毎回 `start service success. name:video_hub_pc4_chest`
- **`/dev/video10` は無い。** あるのは video0〜5（頭の D435i）と video8・9（前方の Innomaker U20CAM、`0c45:6366`、Mapping 班が使う）。
  腕の Full HD webcam（`1bcf:2283`、usb 1-2.2.2）は 09:44 に認識され、10:18:28 に USB が抜けた
- プログラムの文字列: `video device not work, wait 30 seconds, and then try again`、`rt/api/videohub_chest/request`、
  `v4l2src device=` と `1920x1080 YUY2 15fps → nvjpegenc`。デバイスが現れるのを待っているだけ
- `rt/api/videohub_chest` に `VideoClient` と同じ形（API 1001）で 3 回頼んだ → 3 回とも code 3102（送れない＝受け取る相手がいない）。画像は無し
- 10/21 に「再起動後は `videohub_pc4_chest` が無かった」としたのは、`ps -C videohub_pc4_chest` の見落とし（コマンド名が 15 文字
  `videohub_pc4_ch` で切れて一致しない）。`real/pc2_survey.sh` の同じ行を pgrep で探す形に直した

## エレベーター（午後。ノート PC から有線。12:56〜13:21）

ラボのエレベーターのホールで、車いす用の呼びボタン ▼（床から約 87 cm）を押した。押す側は手順書の段階 7 と同じ
`run_dds.py` + `contest_agent`（`auto_press.py --target down --button-set wheelchair --detector yolo`）。onboard ではなく、ノート PC から DDS で直接送った。

### つなぎ方（ノート PC・WSL2 から有線で DDS）

- WSL2 の既定（NAT）では、ping は届くが **lowstate が届かない**（`check_connection.py --network-interface eth0` で 10 秒待って何も来ない）
- `C:\Users\tvdue\.wslconfig` に `[wsl2]` / `networkingMode=mirrored`、Hyper-V のファイアウォールで受信を許可（`Set-NetFirewallHyperVVMSetting ... -DefaultInboundAction Allow`）、
  Windows の有線を 192.168.123.99/24 に固定 → `wsl --shutdown`。WSL の中では有線が `eth1` になった（Wi-Fi は `eth2`、Tailscale は `eth0`）
- その後は lowstate 1042 Hz・深度付きの画像も OK。**コマンドには毎回 `--network-interface eth1`**（`arm.yaml` の既定はラボ PC の `enp129s0`）
- PC2 の有線（192.168.123.164）への ssh は初めてで、known_hosts に足した
- `rgbd_server` は PC2 の **システムの Python 3.8** で動かす（`~/button_press/venv` には pyrealsense2 が無く、`ModuleNotFoundError`）。ログ `~/button_press/rgbd_server_20261101d.log`

### 見つける（写真）

| 収録 | 立つ位置 | classic | YOLO（`button_final8.pt`） |
|---|---|---|---|
| `recordings/20261101_135943_button_hall`（9 枚） | ▼ が前 49.2・右 16.6 cm | 1 組を選べた 5 / 9（選んだ組は z −0.56 m の床の模様で誤り） | **9 / 9**（▲ 0.96・▼ 0.97、ばらつき 1 mm 以内） |
| `recordings/20261101_140209_button_hall2`（6 枚） | ▼ が前 40.0・右 1.7 cm | 0 / 6 | **6 / 6** |

- ▲ と ▼ の間隔 6.8 cm（メジャーで約 6.5 cm）。高さは pelvis から +23 / +16 cm（床からに直して約 96 / 89 cm。メジャーで 93.5 / 87 cm）。一般用の組と取り違えていない
- 画像の左半分に大きなぬいぐるみが写っていたが、ボタンは隠れなかった
- 比べた画像：`_local/button_press/button_check/20261101_140013/`・`20261101_140217/`（`J1GEN_BUTTON_SET=wheelchair`）

### 回ごと（時刻はノート PC の日本時間。body_log・auto のフォルダ名と同じ）

指先の位置は、`body_log.csv` の関節角から FK で求めた（pelvis 座標）。「当たった x」は、押す向きに指先が止まった位置。

| 回 | 変えた設定 | 狙う表面 x / y | 結果（ユーザーの目） | 記録から |
|---|---|---|---|---|
| 1（14:05） | `lag_m` 0.005、深さ 15 mm、補正なし | 0.314 / −0.020 | 約 1.5 cm 足りない（触れていない） | 押し込みを始めて 0.7 秒・13 mm で「進まなくなった（遅れ 5.6 mm）」。**空中で加速しているだけで止まった**。体 3.1 mm・0.35° |
| 2（14:08） | `lag_m` 0.010 | 0.315 / −0.019 | 壁に当たった。**ボタンの右 2 cm**。上下は完ぺき | 当たった x ≈ 0.288（狙いより 27 mm 手前）、遅れ 12.4 mm で止まった。y は狙いどおり −0.020 |
| 3（14:11） | 補正 `[-0.020, 0.020, 0]` | 0.295 / +0.001 | **位置はよかった。押し込みが弱い** | 当たった x ≈ 0.284（狙いより 11 mm 手前）、遅れ 9.9 mm で止まった。保持は実際の指先 + 2 mm で弱い |
| 4（14:14） | `lag_m` 0.020、`keep_m` 0.008、補正 x −0.030 | 0.281 / +0.001 | 点灯せず。ユーザーが手で押しても「だいぶ強く押さないとだめ」 | 止めずに最後まで。指先は x 0.285 で止まり、沈まなかった（目標は約 11 mm 先）。体 5.8 mm・0.55° |
| 5（14:17） | 深さ 30 mm、`lag_m` 0.040 | 0.287 / +0.001 | 点灯せず | 指先は x 0.301 まで（触れてから 16 mm、左へ約 6 mm・下へ約 4 mm）。**骨盤が後ろへ 4.9 mm・傾き 1.13°**（送る前に決めた「1° 以上ならやめる」を超えた）。指が曲がったか滑ったかは【要確認：動画】 |

- 5 回とも、点灯の確認は「点灯を確かめられない」（明るい部分 −5〜+8）。人の目でも点灯なし
- 5 回とも腕は来た経路を戻り、終了コード 0。腕が柱や体に当たる・腰が回ることはなかった
- 各回の前に dry-run し、別の班の `g1_e2e.vla.robot` が `--execute` なしであることを確かめた（13:05〜13:17 ずっと pid 4230 で動いていた）

### 分かったこと

1. **ボタンの位置の推定は、ボタン用の一定の補正で合う。** 柱まで約 32 cm で、推定は実際より約 2.7 cm 奥・2 cm 右だった（上下は合っていた）。
   `localize.yaml` の `calibration.offset_pelvis_m` は [0, 0, 0] のまま（一度も較正していない）。ボトルは的が大きく、当たって止まるので目立たなかった
2. **ラボの呼びボタンは硬く、今の腕の PD（kp 60）では押せない。** 目標を本物のボタンの約 26 mm 先に置いても（回 5）沈まず、先に体が反った。
   ざっと、肩から指先まで約 0.4 m なら指先の硬さは 60 / 0.4² ≈ 375 N/m で、16 mm の遅れで約 6 N
3. 止める判定（遅れの増え方）は、実機では 5 mm だと空中で働く。10 mm で当たって止まるようになった（ボトルと同じ）
4. 頭カメラの最終版の YOLO は、ホールの本物のボタンでも確実（15 / 15）。classic は使えない

### `elevator.yaml` に残した設定（家で見直したあと）

| 項目 | 前 | 今 |
|---|---|---|
| `calibration.offset_pelvis_m` | null（`localize.yaml` の [0, 0, 0]） | **[-0.030, 0.020, 0.0]**（柱まで約 32 cm で合わせた値。立つ位置が変わると合わないかもしれない） |
| `press.planner.press_depth_m` | 0.015 | 0.015（回 5 の 0.030 から戻した） |
| `press.contact_stop.lag_m` / `retry_lag_m` / `keep_m` | 0.005 / 0.008 / 0.002 | **0.010 / 0.013** / 0.002（回 5 の 0.040 / 0.023 / 0.008 から戻した） |
| `press.max_retries` | 1 | 1（送った 5 回は 0。終わって戻した） |

深さ 30 mm・`lag_m` 0.040（回 5）は点灯せず体が 1.13° 傾き、指先が塞がれても早めに止まる守り（シミュで、柱の穴の縁を 21.6 N で押した対策）も
働かないので、家で戻した。残したのは、実機で確かめた補正と、当たって止まる 10 mm（ボトルと同じ）。テスト 2 件をこの値に合わせた
（`test_calibration_offset_shifts_buttons`、`test_lagging_press_still_stops_when_blocked` は面の 4 mm 手前で止める）。
WSL を mirrored にしたままだと、`lo` でマルチキャストが使えず、模擬ロボットのテスト 7 件が落ちる（送信回数 0。G1 には何も送っていない）。

### 次に試すこと（案）

- 動画で、回 5 の指が曲がったか・ボタンの上で滑ったかを見る
- 押す力を上げる方法: 腕の kp を上げる（`Yada/configs/contest.yaml` の `gains.arm`。シミュの評価にも関わるので Yada さんと相談）／
  指を伸ばしたまま手の硬いところで押す（指の関節が曲がらない分、力が伝わる）／ 押す向きに体重を乗せる姿勢
- ボタンを押すのに要る力を測る（ばねばかり・キッチンスケールで）。何 N 出せばよいかが分かれば、kp・深さを計算で決められる

## 始める前に決めること（10/31 からの持ち越し）

- [x] **ノート PC のコミットを push してから**、ラボ PC で `git pull` する（`ea0e260`・`417295a`・`5e21d5d`・`6e38cbd` 以降。広げた設定を既定にした・机の箱の余裕 1 cm・報告書）
- [ ] **既定の設定が変わった**: 10/31 の後に、広げた設定（前の `reach_wide.yaml`）を既定にした。10/31 の「既定」で押すときは `--overlay reach_legacy.yaml` を付ける。
  広げた設定は 10/31 に体が前へ 7.5° 傾いたので、今日どちらを主に使うか → **両方を試した**（前の既定で 1 回、広げた設定で 1 回、どちらも PC2 から送った）
- [x] 事前確認の骨盤の傾きの上限（`real/auto_press.py` の `MAX_PELVIS_TILT_DEG`）は **5.0° のまま**にする（11/1 にユーザーが決めた。10/31 は立ち直すと 0.7° に戻った）
- [x] 押し込みを止める遅れ（`configs/press.yaml` の `contact_stop.lag_m`）は 10 mm のまま（10/31 に決めた。ボトルは固定して押す）
- [x] 机の箱の余裕は 1 cm（`make_table_boxes.py` の既定。10/31 は 3 cm）。手が机に近すぎたら `--margin-m 0.03` に戻す
- 優先順位（実際）: onboard（4.）を先にした。ラボ PC は使わず、すべてノート PC から無線で、PC2 の上で動かした。エレベーター（3.）・置ける範囲（2.）はまだ

## 今日やること（予定）

### 準備（毎回）

手順は [DAILY_LOG_20261031_labpc.md](DAILY_LOG_20261031_labpc.md) の「次回の再開手順」と同じ。11/1 はラボ PC を使わず、ノート PC から無線の ssh で PC2 の上で行った（「4.」の結果）。

- [x] `git pull`、`git status`（ノート PC。ラボ PC は使わなかった）。ラボ PC で `export CYCLONEDDS_HOME=$PWD/G1_HuggingFace/cyclonedds/install`
- [x] G1 をレギュラーモードにして立たせる（09:44 に PC2 が起動。10:09 には立っていた）。**モードを切り替えると PC2 が再起動して、カメラの配信が消える**
- [x] PC2 のカメラ配信を起動する（09:50。Mapping 班の配信は動いていなかったが、`--no-legacy-rgb` で 5556 だけ。Claude が ssh で起動できる。手順は [shell_and_ssh.md](../docs/knowledge/tech/shell_and_ssh.md)）。
  PC2 のポート 5555 を Mapping 班の配信（`innomaker_camera_server.py`）が使っているときは、`--no-legacy-rgb` を付けて 5556 だけで起動する
- [x] 接続確認（`check_connection.py --rgbd`）→ カラー画像が止まっていないか → 頭カメラの向き（`check_head_tilt.py`。10/31 は 50.2〜51.5°、上書き不要）→ 11/1 は 51.2°、上書き不要
- [x] 机の箱を立った状態の深度から作る（`make_table_boxes.py --live` → 図を見て `--save`）。**立ち直すたびに作り直す**（10/31 の `configs/obstacles.yaml` は使わない）
- [x] ボトルをテープで固定する（滑ると、当たっても止まらない）

### 1. Enter なしでボトルを押す（10/31 の続き）

10/31 は、Enter なしで最後まで動いたが、ボトルが滑ったり（押し切り）、腰のずれで中止したりして、同じ配置での繰り返しができなかった。

- [x] 同じ配置（机の縁ぎりぎり・右 15〜19 cm）で、`auto_press.py --target bottle` の dry-run（事前確認を含む）→ PC2 の上で（「4.」、回ごとの結果の 1・2）
- [ ] 前の既定（`-- --overlay reach_legacy.yaml`）で、Enter なしで数回繰り返す（`record.py` を回したまま。横から動画も撮る）→ **1 回だけ**（PC2 から。回 3、成功）。`record.py` は使っていない（記録は `runs/` の lowstate・画像）
- [x] 広げた設定（今の既定）は、まず確認モードで経由 → 手前の姿勢まで動かし、体の傾き（`runs/<日時>/body_motion.csv` の `imu_tilt_deg`）を見る。
  傾きが小さければ Enter なしで押す → 確認モードは使わず（`onboard.sh` は Enter なしだけ）、dry-run のあとに Enter なしで送った（回 4）。
  体の傾きは最大 0.70°（10/31 の 7.5° より小さい）が、pelvis が前へ 10 mm 動き、当たって止まった直後に中止した
- 10/31 の結果（参考）: 前の既定は体の動き 2.8〜3.4 mm・0.8° 以下、広げた設定は 84 mm・7.5°。経由の姿勢の追従誤差は、広げた設定で 3.06°・3.78°（許容 2.9° を超えた）

### 2. ボトルを置ける範囲がどれだけ広がったか（10/31 から移した）

前々回（10/21）は、カメラの向き（71°）と腕の計画の両方で範囲が狭く、計画が通るまで 5 回断られた。**同じ配置を前の既定（`--overlay reach_legacy.yaml`）と
今の既定（広げた設定）の両方で dry-run して、通る・断られるを比べる**。dry-run は何も送らないので、配置を変えて何回でも試せる。

| 配置 | 机の縁（pelvis から） | ボトルの左右 | 地図の予想（前の既定 / 広げた設定） | シミュ（MuJoCo。前の既定 / 広げた設定） | 前の既定の dry-run | 広げた設定の dry-run | 送った結果 |
|---|---|---|---|---|---|---|---|
| A | 32 cm | 右 17 cm（10/21） | 通る / 通る | 成功 50 mm / 成功 50 mm | | | |
| B | 25 cm | 右 17 cm | 断る（経由の姿勢へ動く途中で手首が机に当たる）/ 通る | 断る / 成功 50 mm | | | |
| C | 32 cm | 右 5 cm | 断る（手前の姿勢で右肩が胴体に当たる）/ 通る | 断る / 成功 49 mm | | | |
| D | 32 cm | 左 0 cm | 断る（同上）/ 通る | 断る / 成功 46 mm | | | |
| E | 20 cm | 右 10 cm | 断る（手前の姿勢で手首が体に当たる）/ 通るが、ボトルが頭カメラの画像から出る | 断る / 成功 49 mm | | | |
| F | 40 cm | 右 17 cm | 通る / 通る（ボトルが縁から 6 cm 以上奥だと、押し込みの直線で関節が速く動きすぎて断る） | 成功 50 mm / 成功 50 mm | | | |

- どれもボトルは縁ぎりぎり（縁からボトルの手前の面まで 0 cm）。シミュの数値は「指先が実際に進んだ距離」（計画 65 mm）。
  シミュは 10/30 に `sim/reach_verify.py` で確かめた（記録 `_local/button_press/reach_verify/20261030_placements_AF.log`）
- 10/31 に実機で dry-run できた配置（[REPORT_20261031.md](../docs/reports/REPORT_20261031.md) の表）: D に近い「体の正面・縁 22 cm」は、前の既定で断る・広げた設定で通る（予想どおり）。
  B に近い「縁 25〜27 cm・右 15 cm（丸い机）」は、**広げた設定でも断られた**（経由 → 手前の途中で机の箱に当たる。箱の余裕 3 cm のとき）。今日は余裕 1 cm で、もう一度見る
- **勧める立つ位置の注意**（「立つ位置が勧める範囲の外: …」）は、10/31 の後に広げた設定に合わせた（押す点まで 30〜40 cm・右に 0〜25 cm・机の縁 20 cm 以上）。
  `--overlay reach_legacy.yaml` を付けると、前の範囲（右に 15〜25 cm・縁 29 cm 以上）で出る。計画が立たなかったときに、どう動けばよいかが添えられる
- 地図の予想：前の既定は縁 32 cm 以上・右 10〜30 cm だけ（1,456 か所中 249）。広げた設定は縁 18 cm 以上・右 20 cm〜左 5 cm（1,089）
- **計画だけでなく頭カメラの画像にも入る**のは、前の既定 200・広げた設定 770（1,456 か所中）。範囲の上限はカメラで決まることが多い。
  図と数字のまとめは [press_reach_maps.md](../docs/reports/press_reach_maps.md) と [press_reach_maps.html](../docs/reports/press_reach_maps.html)
- 配置を変えたら、机を動かしていなければ机の箱はそのまま。机を動かしたら（ロボットが立ち直したら）`make_table_boxes.py` で作り直す
- 頭カメラにボトルが写るか（YOLO が見つけるか）も一緒に記録する
- **11/1 も A〜F は試していない**（onboard を先にした）
- 広げた設定で送るときは、**まず確認モードで経由 → 手前の姿勢まで**（押し込みの確認に q と答える）。肩が大きく上がるので周りに人・物がないか確かめる。
  体の傾きも見る

### 3. エレベーター（ホールまで行けたら。10/31 から移した）

- [ ] 車いす用ボタン（`J1GEN_BUTTON_SET=wheelchair`）が頭カメラで見えるか（写真を撮る。車いすの印を「up」と取り違えないか）
- [ ] 立つ位置（柱まで 29〜40 cm・左右 6 cm 以内）で dry-run
- [ ] 押す（確認モード）→ うまくいけば `auto_press.py --target down --button-set wheelchair --detector yolo --execute`
- YOLO の重みは、11/1 から最終版（`weights/button_final8.pt`・`.onnx`）をリポジトリに入れた。`git pull` で届き、`elevator.yaml` の既定もこれ（10/31 はラボ PC に無かった）

### 4. 押す処理を PC2 の中で動かす（onboard の段階 2。11/1 に追加）

目標は、押す処理（ボトル・ボタン）を G1 の PC2 で動かし、最終的には PC からは開始・停止・様子の確認だけにすること
（手順と見積もりは [PC2_ONBOARD_CHECK.md](PC2_ONBOARD_CHECK.md)、10/31 の調査は [onboard_pc2_20261031.md](../docs/reports/onboard_pc2_20261031.md)）。
今日は段階 2 の「**PC2 の中で、ボトルを押す**」まで。PC からの開始・停止・心拍（段階 3）とエレベーター（段階 4）は次の回。

準備できていること:
- PC2 の venv（`~/button_press/venv`。DDS は `lerobot` のもの、pin・mujoco を足した）とコードのコピー（`~/button_press/repo/`）。**コードは古いので今日送り直す**
- lowstate の受け取りを軽くした（1 コアの 75% → 10%）。計画は PC2 で 1 回 0.17〜0.32 秒
- **ボトルの検出を ONNX で**（`--detector onnx`。torch・ultralytics なし。11/1 に足した。重みは `localize.yaml` の `onnx_model`）
- **PC2 用のカメラの設定**（`--camera-config camera_onboard.yaml`。配信を localhost の 5556 で受ける。11/1 に足した）
- **指令を送った間隔を調べるスクリプト**（`real/send_timing.py <runs のフォルダ>`。11/1 に足した）
- 10/31 の夕方に、PC2 の上で dry-run を 1 回流してある（計画は通った。送るはずだった間隔は中央値 20.0 ms・99% 点 22.0 ms・最大 297.7 ms、30 ms 超は 4 回 0.17%。
  最大は押し込みの前に計画し直す時間と思われる。腕はその間、前の指令のまま）

やること（ボトルの「1.」が済んで、同じ配置・机の箱のままで）:
- [x] ノート PC から PC2 にコードと ONNX を送った（10:00 ごろ。送り直すときは `onboard.sh sync`）。机の箱は PC2 で作り、`~/button_press/obstacles_20261101.yaml` に保存した
- [x] PC2 で接続確認（`check_connection.py --rgbd`、eth0・localhost）。PC2 の CPU の様子（`top`。Mapping 班の処理が動いているか）を記録する
- [x] PC2 で `auto_press.py --target bottle` の dry-run（事前確認を含む）。ラボ PC で同じ配置の dry-run と、狙いの点・計画（通る / 断る）が同じか → ラボ PC との比べはしていない（ラボ PC を使わなかった）
- [x] `send_timing.py` で、PC2 の dry-run の送る間隔を見る（ラボ PC の記録と並べる）→ 中央値 20.0 ms（ラボ PC の記録とは並べていない）
- [x] PC2 で送る（Enter なし）。**ノート PC から無線で、`real/onboard.sh` で起動する**（11/1 にユーザーが決めた。付録の「4.」）。リモコンを持った人が立ち会う
  - 押す処理は PC2 の上で端末から切り離して動く（`setsid`）。ssh が切れても止まらず、最後まで動いて戻る。lowstate も記録も PC2 の中で完結する
  - 止めるのは `onboard.sh stop`（Ctrl+C と同じ合図 SIGINT。安全に終了して来た経路を戻る）、戻る途中でもう一度なら `stop-now`（その場で weight を下げる）。緊急停止はリモコンの L2+B
  - **送る前に、PC2 の dry-run の途中で `stop` して、止まることを確かめる** → 11/1 10:31 に確かめた: 経由の姿勢へ動いている途中に `stop` すると
    「SIGINT を受信。安全に終了する」→ weight を下げて終了（終了コード 130、何も送っていない）。**dry-run では来た経路を戻る動きはしない**（腕が動いていないので、
    `_retreat_to_start` は dry-run では何もしない）。戻る動きは、ノート PC の模擬ロボット（`sim/sim_robot_server.py`）で、同じく切り離して `--execute` し、
    手前の姿勢へ動いている途中に SIGINT を送って確かめた: その場で止まる → 経由の姿勢へ戻る（1.8 秒）→ 開始姿勢へ戻る（12.6 秒）→ weight を下げる
  - はまったこと: `onboard.sh` は最初、PC2 の上で移るフォルダにノート PC のパスを使っていた（手元の偽物の試験では、そのパスが手元にあったので見逃した）。
    また、子の `press_bottle.py` の出力がバッファされて、ログ（`follow`）に出るのが終わってからだった → `PYTHONUNBUFFERED=1` で直した
- [x] 記録（PC2 の `~/button_press/repo/_local/button_press/runs/`・`auto/`）をラボ PC に持ってくる → ノート PC に（`onboard.sh fetch`、`_local/button_press/{runs_pc2,auto_pc2,onboard_logs_pc2}/`）

注意:
- PC2 は G1 のモードを切り替えると再起動する。venv とファイルは残るが、カメラの配信は起動し直す
- 設定（前の既定 `reach_legacy.yaml` を使うか）は、ラボ PC で押したときと同じにする（`--` のあとに `--overlay reach_legacy.yaml`）
- ⚠️ PC2 の上の ONNX の検出は、.pt と比べて確からしさの差 0.005 以下（10/31 に確認）。枠の位置が少し違うことはある

- **11/1 10:10〜10:16 の結果（ノート PC から無線の ssh で、PC2 の上で。何も送っていない）**:
  - 接続確認・頭カメラの向き（51.2°）・机の箱（PC2 の `~/button_press/obstacles_20261101.yaml`。余裕 1 cm、11 個、縁 34.5〜40 cm で約 4° 斜め）を PC2 の上で作った
  - dry-run は前の既定・今の既定とも、事前確認（モーションコントローラ `ai`・骨盤の傾き 2.4°・カラー）が通り、計画が立った（「回ごとの結果」の 1・2）。ボトルの検出は ONNX
  - 送る間隔は中央値 20.0 ms で安定。30 ms を超えたのは、手前の姿勢で実測の腰の角度で計算し直すとき（193〜245 ms。腕はその間、前の指令のまま）と、
    押し込みの終わり・戻りの切り替え（34〜53 ms）だけで、移動や押し込みの直線の途中には無かった
  - CPU（`top`、dry-run の後）: カメラの配信 `rgbd_server` 75%、`videohub` 25%。**別の班のプロセス `python3 -m g1_e2e.vla.robot`（55%）が 10:15 に起動していた**。送るときは、誰が G1 を使っているかを確かめる
  - 記録はノート PC の `_local/button_press/runs_pc2/`・`auto_pc2/` に持ってきた

### 時間があれば（10/31 から移した）

- [x] 胸のカメラの有無（PC2 で `ls /dev/video*` など。見るだけ）→ **無い**。`videohub_pc4_chest` は `/dev/video10` を待っているだけで、`/dev/video10` は無い（「胸のカメラ」の節）
- [ ] ボトルを倒す動作（[KNOCK_PROCEDURE.md](KNOCK_PROCEDURE.md)）
- [ ] 10/31 の横の動画で、すき間を測る（`measure_gap.py`）

## 成功の判定基準（実行する前に決める）

案を書いておいた。当日の朝に見直して、使うものを残す。

- 1. Enter なしでボトルを押す（繰り返し）
  - `auto_press.py` の事前確認がすべて通り、Enter を押さずに最後まで動く。【要記入：何回中何回】
  - 押し込みが「指先が進まなくなった … 押し込みを止める」で終わる（固定したボトルに当たって止まった）
  - 横からの動画で、指先がボトルに触れている。**ボトルの横に定規を置いて一緒に写し**、およその距離を動画から読む
  - 「指先が実際に進んだ距離」は判定に使わず、記録だけする（50 mm 前後が上限。50 mm との差を、深度の位置のずれの目安として見る）
  - 押し込みの間、体（pelvis）の移動が数 mm 以内、傾きが【要記入：何°】以内（10/31 は前の既定で 0.8° 以下、広げた設定で 7.5°）
- 2. 置ける範囲
  - 広げた設定で、前の既定では断られる配置（B〜E）のうち【要記入：いくつ】以上で計画が通る
  - 計画が通った配置で、経由 → 手前の姿勢まで、手が机・体に当たらずに動く
  - 計画の地図の予想と、実機の dry-run の結果が合っている（合わなければ、どこで食い違ったかを記録する）
- 4. onboard（PC2 の中で押す）
  - PC2 の dry-run で、事前確認が通り、ラボ PC と同じ配置で同じ結果（通る / 断る）になる。狙いの点の差が【要記入：何 mm】以内
  - 送る間隔の 99% 点が【要記入：何 ms】以下（50 Hz なら 20 ms。ラボ PC の記録と比べる）。押し込みの途中に長い間隔が無い
  - PC2 で `--execute` して、ラボ PC で押したときと同じように「指先が進まなくなった … 押し込みを止める」で終わる。体の動き・傾きもラボ PC と同じくらい
- 3. エレベーターのボタン（ホールまで行けたら）
  - 頭カメラの画像で、車いす用の ▲▼ を見つけられる（車いすの印を「up」と取り違えない）
  - 点灯した（自動判定。明るい部分の上がり +30 以上、または明るさ 220 以上で +6 以上）。動画でも目で確かめる
  - 壁・柱を強く押さない（動画で、押したあとに体が押し返されていない）

## 条件

- コミット：PC2 のコードのコピーは 09:50 にノート PC の作業ツリー（`8bcbbd7` の後）から送った。`onboard.sh` はノート PC で動くので、その後の直し（`6ecc110` まで）も効いている
- 時刻：この報告書の 11/1 の時刻は、PC2 の時計（中国時間 CST。日本時間より 1 時間遅い）。runs のフォルダ名も同じ
- 動かした場所：**PC2 の上**（venv `~/button_press/venv`、DDS は eth0、カメラは localhost:5556、ボトルの検出は ONNX）。ノート PC は Wi-Fi `Physical_AI_5G`（192.168.0.84）から ssh
- 姿勢・モード：レギュラーモードで立った状態（腰 yaw/roll/pitch −0.1/+0.3/−2.5°、骨盤の傾き 2.4〜2.9°）。重力補償の倍率 0.5（既定）
- 首の傾き：51.2°（上書き不要）
- カラー画像が動いているか：動いていた（カラーの変化 1.1〜4.6）。10:56 に `rgbd_server` が止まった（回 4 のあと）
- ボトルの位置（pelvis 座標）・机の配置：机は四角い机で、手前の縁が右（y −0.45）34.5 cm → 左（y +0.45）40.0 cm と約 4° 斜め、天板 z −0.005 m。
  ボトルは机の縁ぎりぎり・右 17〜21 cm（[0.342〜0.360, −0.171〜−0.216, 0.087〜0.092]）。ボトルは固定した
- 設定：深度の較正値は 0 のまま（未較正）。`contact_stop.lag_m` 10 mm。事前確認の骨盤の傾きの上限 5.0°。机の箱の余裕 1 cm。
  主に使った設定：前の既定（回 3）と広げた設定（回 4）を 1 回ずつ
- 録画：`record.py` は使っていない（各回の `runs/` に lowstate・画像が残る）
- 同時に動いていたもの：別の班の `python3 -m g1_e2e.vla.robot`（10:15 起動）、10:49:48 からは `--execute` 付き
- エレベーター（行けたら）：立った位置【要記入】（柱まで 29〜40 cm・左右 6 cm 以内を勧める）。ボタンの高さは床から ▲ 93.5 cm・▼ 87 cm（10/22 に測定）

## 回ごとの結果

| 回 | 対象 | ラベル（runs/ のフォルダ） | 結果 | 数値（進んだ距離・誤差・体の傾きなど） | メモ |
|---|---|---|---|---|---|
| 1 | ボトル・onboard dry-run（前の既定、PC2 の上で。ノート PC から無線の ssh） | `runs_pc2/20261101_101411_onboard_legacy_dry` | 事前確認すべて通過・計画が立った（終了コード 0） | ボトル [0.359, −0.215, 0.090]（ばらつき 3.2 mm 以内）。経由へ最大 90.8°・9.5 秒。送る間隔 中央値 20.0 ms・99% 点 22.3 ms・最大 245 ms（30 ms 超 4 回） | 何も送っていない（dry-run） |
| 2 | ボトル・onboard dry-run（今の既定＝広げた設定） | `runs_pc2/20261101_101501_onboard_wide_dry` | 同上（終了コード 0）。指はまっすぐ（傾けずに通った） | ボトル [0.360, −0.216, 0.089]。経由へ最大 100.3°・10.5 秒。送る間隔 中央値 20.0 ms・99% 点 22.5 ms・最大 193 ms（30 ms 超 3 回） | 同上 |
| 3 | **ボトル・onboard で送った**（前の既定、Enter なし。ノート PC から無線で `onboard.sh start --execute`、PC2 の上で動いた） | `runs_pc2/20261101_104145_onboard_exec` | **成功（当たって止まった）**。事前確認すべて通過 → 押す → 開始姿勢まで戻った（終了コード 0） | ボトル [0.342, −0.171, 0.087]。指先が進んだ 43.1 mm（計画 65 mm、90/109 点。遅れ 14.8 mm、押す前から 4.5 mm、増えた分 10.2 mm）。追従誤差 最大 2.77°（許容 2.9°）。体の動き 1.0 mm・0.29°。送る間隔 中央値 20.0 ms・99% 点 22.3 ms・最大 173 ms（計算し直すとき） | リモコンを持った人が立ち会い。ボトルは固定。11/1 10:41 |
| 4 | ボトル・onboard で送った（**今の既定＝広げた設定**、Enter なし。ユーザーがノート PC から `onboard.sh` を打った） | `runs_pc2/20261101_105224_onboard_exec` | **当たって止まったが、その直後に中止（終了コード 3、`NoMotionError`）→ 来た経路を戻って開始姿勢へ** | ボトル [0.348, −0.173, 0.090]。指先が進んだ 40.4 mm（83/109 点、増えた分 10.0 mm）。押し込みを止めた直後の動きで「腕全体で指令の 0.08 倍しか動いていない」。体の動き 最大 10.0 mm・0.59°（押し込み中。前の既定は 1.0 mm）。経由の姿勢の追従誤差 2.36°（OK） | **同じ時間に別の班の `python3 -m g1_e2e.vla.robot --execute` が PC2 で動いていた（10:49:48 起動）**。途中で G1 が「no wifi」のような音声を出した。10:56 に `rgbd_server` が videohub のカラーを受け取れず（code 3104）終了した。原因は未確定。ターミナルの出力 `_local/button_press/terminal_logs/20261101_laptop_onboard_terminal.log` |
| 5 | ボトル・PC2 から送った（**広げた設定**、VLA なし） | `runs_pc2/20261101_114828_onboard_exec` | **成功**（終了コード 0） | ボトル [0.340, −0.172, 0.088]。42.2 mm（85/109 点、押す前から 5.7 mm、増えた分 10.2 mm）。追従誤差 最大 2.45°。体の動き 6.0 mm・0.67°（IMU 0.63°）。送る間隔 中央値 20.0 ms・99% 点 21.8 ms・最大 171 ms | 11:48。Claude が送った（ユーザーが確認） |
| 6 | ボトル・PC2 から送った（前の既定、VLA なし） | `runs_pc2/20261101_114942_onboard_exec` | 当たって止まった直後に `NoMotionError`（0.04 倍、終了コード 3）→ 戻った | ボトル [0.336, −0.173, 0.082]。44.4 mm（90/109 点、押す前から 4.5 mm、増えた分 10.2 mm）。体の動き 5.4 mm。送る間隔 中央値 20.0 ms・最大 168 ms | 11:49。原因は「押し込みを少し戻す」の誤判定（上の節）。`d6215b1` で直した |
| 7 | ボトル・PC2 から送った（前の既定、直したコード） | `runs_pc2/20261101_115756_onboard_exec` | **成功**（終了コード 0） | ボトル [0.338, −0.173, 0.084]。45.6 mm（95/109 点、押す前から 5.1 mm、増えた分 10.4 mm）。体の動き 3.4 mm・IMU 0.39°。送る間隔 中央値 20.0 ms・99% 点 22.3 ms・最大 213 ms | 押し込みの終わりの追従誤差 3.01°（許容 2.9° を少し超えて ⚠️、注意だけ）。Claude が送った |
| 8 | ボトル・PC2 から送った（広げた設定、直したコード） | `runs_pc2/20261101_115849_onboard_exec` | **成功**（終了コード 0） | ボトル [0.340, −0.174, 0.080]。44.9 mm（90/109 点、押す前から 6.1 mm、増えた分 10.4 mm）。体の動き 4.7 mm・IMU 0.95°。送る間隔 中央値 20.0 ms・99% 点 21.9 ms・最大 176 ms | Claude が送った |
| 9 | ボトル・PC2 から送った（前の既定、直したコード） | `runs_pc2/20261101_115950_onboard_exec` | **成功**（終了コード 0） | ボトル [0.338, −0.174, 0.081]。44.2 mm（93/109 点、押す前から 4.9 mm、増えた分 10.6 mm）。体の動き 3.2 mm・IMU 0.59°。送る間隔 中央値 20.0 ms・99% 点 22.3 ms・最大 191 ms | 押し込みの終わりの追従誤差 2.92° ⚠️（注意だけ）。Claude が送った |
| 10 | ボトル・PC2 から送った（広げた設定、直したコード） | `runs_pc2/20261101_120045_onboard_exec` | **成功**（終了コード 0） | ボトル [0.338, −0.174, 0.083]。44.0 mm（91/109 点、押す前から 5.5 mm、増えた分 10.1 mm）。体の動き 4.2 mm・IMU 0.91°。送る間隔 中央値 20.0 ms・99% 点 22.5 ms・最大 184 ms | Claude が送った |

## 予想と違ったこと → 原因 → どう確かめたか → 何を変えたか

（面接で話す材料になる一番大事な部分。当日か直後に書く）

（下書き。Claude が記録から埋めた。自分の言葉で直す）

### 1. 広げた設定では、当たって止まった直後に「指令どおりに動いていない」で中止した

- 予想：前の既定と同じように、当たって止まり、少し下がって保持してから戻る
- 実際：当たって止まった（40.4 mm）直後の動きで、腕が指令の 0.08 倍しか動かず `NoMotionError`（終了コード 3）。そのあと来た経路を戻って開始姿勢へ。
  押し込みの間に pelvis が前へ 10 mm 動いた（前の既定の回は 1.0 mm）
- 原因：未確定。候補は 2 つ。(a) 同じ時間に別の班の VLA（`--execute` 付き）が PC2 で動いていて、腕に指令を送っていた。(b) 広げた設定の姿勢では、固定したボトルに押し当てたあとの小さな動きに腕がついてこない
- 確かめ方：【要記入】（別の班が G1 を使っていないときに、同じ配置で広げた設定をもう一度送る。前の既定と交互に）
- 変えたこと：なし

### 2. `onboard.sh` の誤りが 3 つ、実機で初めて分かった

- 予想：手元の偽物（偽の ssh と偽の押す処理）で確かめたので、PC2 でもそのまま動く
- 実際：(1) PC2 の上で移るフォルダにノート PC のパスを使っていた、(2) 子の出力がバッファされてログに出るのが終わってから、(3) `--` のあとが無いときに空の引数を渡して `press_bottle.py` が断った（終了コード 2）。どれも何も送っていない
- 原因：手元の試験は、パスが手元に実在し、いつも `--` のあとに引数を付けていたので、この 3 つを通らなかった
- 確かめ方：PC2 で dry-run → `stop` を試した。(3) は、引数が無いときと有るときの両方を手元で確かめ直した
- 変えたこと：3 つとも直した（`d55a4ab`・`6ecc110`）

### 3. カメラの配信が止まった・G1 が「no wifi」のような音声を出した

- 実際：回 4 の途中で G1 が「no wifi」のような音声を出した。10:56 に `rgbd_server` が videohub のカラーを受け取れず（code 3104 = 応答の時間切れ）終わった。PC2 の無線（wlan0）はつながったままで、ログにも Wi-Fi が切れた記録は無かった
- 原因：未確定（Unitree 本体の音声か、別の班のプログラムの音声かも分からない）
- 変えたこと：なし。次に使う前に配信を起動し直す

## 自分が判断・対応したこと

- 【要記入】（例: onboard を今日やると決めた。送るのは無線の `onboard.sh` で、ssh が切れても止まらない形にすると決めた。傾きの上限は 5.0° のまま。ボトルを動かして固定した）
- 【要記入】Claude（AI）との分担：自分が決めたこと／Claude に任せたこと
  - Claude に任せたこと（記録から）：PC2 へのコードの送り込み、カメラの配信の起動、接続確認・頭カメラの向き・机の箱、dry-run、`stop` の確認、`onboard.sh`・ONNX の検出・`send_timing.py` のコード、
    前の既定で送った回（回 3。ユーザーが「送ってよい」と確認したあと）、記録の読み取り
  - 自分が実行したこと：広げた設定の dry-run と送る操作（回 4。`onboard.sh` を自分で打った）、リモコンを持って立ち会い
- エレベーター（午後）：【要記入】（例: 有線でノート PC から送ると決めた。WSL を mirrored にした。各回の結果を目で見て、足りない向き・cm を伝えた。5 回目で今日はやめると決めた）
  - Claude に任せたこと：接続確認・カメラの配信の起動、写真と検出の比較、dry-run、記録の解析（FK で指先の位置）、設定の変更案
  - 自分が実行したこと：`.wslconfig` と有線の IP の設定、G1 を歩かせて立たせる、エレベーターの 5 回を送る（自分のターミナル）、点灯・当たった位置を見る

## 10/31 から変えたこと

| 変えたこと | 決めたのは | コミット |
|---|---|---|
| 押し込みを止める遅れ `contact_stop.lag_m` を 5 mm → 10 mm | ユーザー（10/31） | `e5f7446` |
| 事前確認の骨盤の傾きの上限 3.0° → 5.0° | ユーザー（10/31。11/1 に 5.0° のままと決めた） | `e5f7446` |
| **広げた設定（経由の姿勢 `[68.3, -11.1, -40.9, -43.9, 0, -4.1, 0]`・指を左へ傾ける候補）を既定にした**。前の既定は `configs/reach_legacy.yaml` | ユーザー | `ea0e260` |
| 勧める立つ位置を、右に 15〜25 cm → 0〜25 cm、机の縁 29 cm 以上 → 20 cm 以上 | ユーザー（広げた設定に合わせて） | `ea0e260` |
| 深度から作る机の箱の余裕 3 cm → 1 cm | ユーザー | `417295a` |
| onboard の準備: ボトルの検出を ONNX で（`--detector onnx`）、PC2 用のカメラの設定（`camera_onboard.yaml`）、送った間隔を調べる `send_timing.py` | ユーザー（onboard を今日やると決めた） | `ddb298d` |
| エレベーターのボタンの YOLO の最終版（`weights/button_final8.{pt,onnx}`）をリポジトリに入れ、既定にした | ユーザー（メンバー・ラボ PC に渡すため） | `8bcbbd7` |
| ノート PC から無線で PC2 の押す処理を起動・監視・停止する `real/onboard.sh`（11/1 のうちに 3 回直した） | ユーザー（無線で送る形にすると決めた） | `14828ed`・`d55a4ab`・`6ecc110` |
| エレベーター：ボタン用の補正 `[-0.030, 0.020, 0]`、止める遅れ 10 mm（押し込み 30 mm・遅れ 40 mm は試したあと戻した。「エレベーター」の節） | ユーザー（Claude の案を各回で承認。戻すのも） | エレベーターの記録と同じコミット |

## 証拠の場所

- 動画・写真：【要記入：Google Drive などのリンク】
- 実行の記録（ノート PC）：`_local/button_press/runs_pc2/20261101_*`（PC2 の `runs/`）、`auto_pc2/`（`auto_press.py` の結果）、`onboard_logs_pc2/`（PC2 の上の出力）、
  端末の出力 `_local/button_press/terminal_logs/20261101_laptop_onboard_terminal.log`（回 4 の前後。ユーザーが貼ったもの）
- 録画：午前は `record.py` を使っていない。エレベーターの写真は `_local/button_press/recordings/20261101_135943_button_hall`・`20261101_140209_button_hall2`
- エレベーター：`_local/button_press/auto/20261101_14*_down.json`（dry-run 6 回・送った 5 回）、`body_log/20261101_14*_down/`（各回の関節角・IMU と、点灯の確認の画像 `lit_*`）、
  端末の出力 `_local/button_press/terminal_logs/20261101_laptop_elevator_terminal.log`（送った 5 回。ユーザーが貼ったもの）
- 机の箱：PC2 の `~/button_press/obstacles_20261101.yaml`（ノート PC に `_local/button_press/table_boxes/obstacles_20261101.yaml`）、図 `_local/button_press/table_boxes/20261101_100954.png`。
  頭カメラの向き：PC2 の `~/button_press/repo/_local/button_press/head_tilt/20261101_100948.json`
- 写真（頭カメラ）：`_local/button_press/probe/20261101_color.jpg`（立つ前）・`20261101_pre_exec.jpg`（回 3 の前）、`table_boxes/20261101_color_standing.jpg`

## 次回の再開手順

（下書き。11:00 ごろの時点）
1. PC2 に無線でつながるか（`ssh unitree@192.168.0.82 uptime`）。ノート PC は `Physical_AI_5G` に。PC2 の IP が変わったら `PC2=unitree@<IP>` を付ける
2. **別の班（`g1_e2e.vla.robot`）が G1 を使っていないか**を確かめる（`ssh unitree@192.168.0.82 'pgrep -fa g1_e2e'`）。使っている間は送らない
3. カメラの配信を起動し直す（10:56 に止まった）→ 接続確認 → 立ち直したなら机の箱を作り直す（`--obstacles-file ~/button_press/obstacles_<日付>.yaml`）
4. 広げた設定をもう一度送り、回 4 の `NoMotionError` が繰り返すかを見る（`onboard.sh start` → `start --execute`）。前の既定と交互に
5. 置ける範囲 A〜F・エレベーターは、そのあと

## 付録：当日のコマンド（朝に用意したもの。上から順に、1 行ずつ貼る）

どれもリポジトリの一番上（`G1_Hackason/`）で実行する。dry-run は何も送らない。送るコマンドは、リモコンを持った人が立ち会う（緊急停止は L2+B）。
**今の既定は広げた設定。** 10/31 の「既定」は `--overlay reach_legacy.yaml`（`auto_press.py` では `-- --overlay reach_legacy.yaml`）。

### 準備

```bash
git pull
```
```bash
git rev-parse --short HEAD
```
```bash
export CYCLONEDDS_HOME=$PWD/G1_HuggingFace/cyclonedds/install
```
PC2 のカメラ配信（PC2 で。Mapping 班が 5555 を使っているときは最後に `--no-legacy-rgb` を足す）:
```bash
cd ~/button_press && setsid nohup bash real/depth_server/start_rgbd_server.sh --source videohub --max-fps 5 > rgbd_server_20261101.log 2>&1 < /dev/null &
```
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_connection.py --rgbd --seconds 5
```
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_head_tilt.py --live --frames 5
```
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/make_table_boxes.py --live
```
図を見て机と合っていれば:
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/make_table_boxes.py --live --save
```
別のターミナルで記録を回したままにする:
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label bottle_fixed3
```

### 1. Enter なしでボトルを押す

前の既定（10/31 と同じ動き）:
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/auto_press.py --target bottle -- --overlay reach_legacy.yaml
```
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/auto_press.py --target bottle --execute -- --overlay reach_legacy.yaml
```
広げた設定（今の既定）。まず確認モードで、押し込みの確認に q と答える（経由 → 手前の姿勢まで動かして戻る）:
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/press_bottle.py --path arm_sdk --execute
```
体の傾きが小さければ、Enter なしで:
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/auto_press.py --target bottle --execute
```

### 2. 置ける範囲（配置 A〜F）

配置ごとに、前の既定と広げた設定の dry-run を 1 回ずつ:
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/press_bottle.py --path arm_sdk --overlay reach_legacy.yaml
```
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/press_bottle.py --path arm_sdk
```

### 4. onboard（PC2 の中で押す。ノート PC から無線で）

ノート PC で、リポジトリの一番上で。`real/onboard.sh` が ssh で PC2 の上の `auto_press.py` を起動・監視・停止する
（PC2 は `unitree@192.168.0.82`。変わったら `PC2=unitree@<IP>` を前に付ける。机の箱は PC2 の `~/button_press/obstacles_*.yaml` の一番新しいもの）。
コードを送り直す（`configs/obstacles.yaml` は使わないので、上書きされてもよい）:
```bash
bash Button_Press/J1-gen/real/onboard.sh sync
```
dry-run（何も送らない）。前の既定のとき:
```bash
bash Button_Press/J1-gen/real/onboard.sh start -- --overlay reach_legacy.yaml
```
出力を見る（Ctrl+C は見るのをやめるだけ。押す処理は止まらない）:
```bash
bash Button_Press/J1-gen/real/onboard.sh follow
```
止める練習（dry-run の途中で。出力に「SIGINT を受信。安全に終了する」が出るか）:
```bash
bash Button_Press/J1-gen/real/onboard.sh stop
```
送る（Enter なし。リモコンを持った人が立ち会う）:
```bash
bash Button_Press/J1-gen/real/onboard.sh start --execute -- --overlay reach_legacy.yaml
```
```bash
bash Button_Press/J1-gen/real/onboard.sh follow
```
止める（安全に終了して戻る）。戻る途中でもう一度なら `stop-now`（その場で weight を下げる）:
```bash
bash Button_Press/J1-gen/real/onboard.sh stop
```
動いているか・記録を持ってくる:
```bash
bash Button_Press/J1-gen/real/onboard.sh status
```
```bash
bash Button_Press/J1-gen/real/onboard.sh fetch
```
送る間隔（PC2 の記録のフォルダ名は、出力の「記録:」）:
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/send_timing.py _local/button_press/runs_pc2/<フォルダ>
```

### 3. エレベーター（行けたら）

細かい手順・見るところは [REAL_DAY_PROCEDURE.md](REAL_DAY_PROCEDURE.md) の段階7。`<NIC>`・`<PC2 の IP>` は当日の値に置き換える。
dry-run:
```bash
J1GEN_BUTTON_SET=wheelchair G1_HuggingFace/venv/bin/python Button_Press/Yada/contest/run_dds.py --agent Button_Press/J1-gen/contest_agent --network-interface <NIC> --camera-host <PC2 の IP> --target down --instruction "下の階" --no-loco --dry-run
```
Enter なしで最後まで:
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/auto_press.py --target down --button-set wheelchair --detector yolo --execute
```

### 後で

すき間を測る（横の動画。定規の長さを `--ref-cm` に）:
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/measure_gap.py <動画.mp4> --frame <コマ> --ref-cm 30
```
