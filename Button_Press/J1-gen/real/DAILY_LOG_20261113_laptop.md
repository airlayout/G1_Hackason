# 2026-11-13 の記録（ノート PC から無線の ssh で PC2 に入り、PC2 の上で動かす。エレベーターに行かずにできることから）

この日は、[ToDo_20261113.md](../docs/reports/ToDo_20261113.md)（11/10〜11/13 に作った「次の実機日に試すこと」）のうち、
**エレベーターに行かなくてもできること**から進めた。ノート PC は有線でつながず、**無線の ssh（`unitree@192.168.0.82`）で PC2 に入り、
押す処理もすべて PC2 の上で動かした**（DDS は無線に流れないため）。
前回は [DAILY_LOG_20261103_laptop.md](DAILY_LOG_20261103_laptop.md)。

**時刻**: 下の表はノート PC の時計（日本時間）。**PC2 の時計はちょうど 1 時間遅れている**（ノート PC 18:27 のとき PC2 17:27）。
PC2 に残る記録のフォルダ名（body_log・auto など）は PC2 の時刻なので、ノート PC の時刻 = フォルダ名 + 1 時間。

## 結果のまとめ

| 段階 | 結果 |
|---|---|
| つなぎ方 | ノート PC（WSL、NAT）→ 無線の ssh で PC2。PC2 の有線は `eth0`（192.168.123.164）。コードは `onboard.sh sync` と rsync で送った |
| 接続（A） | lowstate 1042 Hz、`mode_machine` 5、腰・腕の mode すべて 1。1 回目は腰 yaw −13.6°（再起動後は −0.1°） |
| 頭カメラの配信 | **1 回目は videohub が空の画像を返し、配信を起動できなかった**（ロボットの起動から 4 分後）。ロボットの再起動で直った |
| 頭カメラの向き（B） | **pitch 51.0°・roll +1.0°**（5 枚とも同じ）。11/3 の 51〜54° と同じ。上書き不要 |
| マイク（C） | **届く・音は 0 でない**（−32〜−47 dBFS）。マイクを起こす操作は要らなかった。5 秒で 5.1 秒分、欠けなし |
| 空中・左腕の dry-run（4） | 計画が立った（left 腕・指 0°/0°・IK 誤差 0.1 mm）。事前確認 5 つとも ✅ |
| 空中・`--arm auto` の dry-run（14） | 右 14 cm → **right**、左 16 cm → **left** を選んだ（思ったとおり） |
| **空中・左腕を送った（4）** | **初めて左腕を実機で動かした。最後まで動いて戻った**。pelvis の移動 最大 2.6 mm・回転 0.6°、足は動かない |
| 空中・`--arm auto` を送った（14。ユーザーが打った） | 右 14 cm → right、左 16 cm → left。2 回とも最後まで動いて戻った。pelvis の移動 1.5〜1.7 mm・回転 0.3〜0.4°。ユーザーの目でも「よくできていた」 |
| 押した力の推定（5） | 腕が止まっている間のばらつきは小さい（0.004〜0.06 Nm）。**空中なのに、保持中の推定が左腕 6.3〜8.8 N・右腕 6.5 N**（腕を前へ 7.5 cm 伸ばす間に増える。両腕とも姿勢によるずれ） |

## 時刻順の記録（ノート PC の時計。かっこ内は PC2 の時計）

| 時刻 | できごと |
|---|---|
| 18:06（17:06） | ロボットの起動（PC2 の uptime から） |
| 18:07 ごろ | ssh で PC2 に入れた。PC2 の `~/button_press/repo` は 11/1 の版で、`waist_turn_check.py`・`loco_pulse.py` などが無かった |
| 18:09（17:09） | PC2 の `~/button_press/repo` を `~/button_press/backup_repo_before_20261113.tgz` に控えてから `onboard.sh sync` |
| 18:09 | `check_connection.py`: すべて OK。腰 yaw −13.6° |
| 18:10（17:10） | `rgbd_server --source videohub`: 「videohub からカラーを受け取れない」。`videohub_check.py` も空の画像。`GetImageSample` は code 0 で中身が空。videohub_pc4 は 17:10:14 に起動したばかり |
| 18:10〜18:13（〜17:13） | 3 分ほど試しても空のまま。17:13 のふつうの起動（RealSense を直接）は `Device or resource busy`（videohub が `/dev/video4` を開いている）。videohub は Unitree のサービスなので止めない |
| 18:13（17:13） | マイク 5 秒（下の「マイク」） |
| 18:19（17:19） | **ロボットを再起動した**（ユーザー）。videohub_pc4 は 17:19:54 に起動 |
| 18:20（17:20） | `videohub_check.py`: 1920x1080 を 10/10 枚、約 50 枚/秒。`rgbd_server --source videohub --max-fps 5` を起動。`probe_rgbd.py`: 5.0 fps、深度 0 の画素 2.9%、中央 1.68 m |
| 18:20 | `check_connection.py`: すべて OK、腰 yaw −0.13° |
| 18:21（17:21） | `check_head_tilt.py --live`: 51.0° |
| 18:2x | ToDo の名前を変えた（`ad340cf`）。Yada の `common`・`configs`・`contest`・`sim` と `J1-gen/contest_agent` を PC2 に送った |
| 18:25（17:25） | 空中・左腕の dry-run |
| 18:26（17:26） | 空中・`--arm auto` の dry-run を 2 通り |
| 18:30（17:30） | **空中・左腕を送った**（`--execute`。ハーネス・リモコンあり）。setsid で切り離して起動（ssh が切れても戻れるように）。動いたのは 18:30:36〜51 の約 15 秒。合図が無く、ユーザーは「動いていない気がする」と見逃した（関節角の記録では動いていた） |
| 18:34（17:34） | 以後、送るコマンドはユーザーが打つことにした。`record.py --mode interval --interval-s 1` を PC2 で流し続ける（lowstate 全部・画像 1 秒に 1 枚） |
| 18:35（17:35） | `--arm auto` の dry-run を 2 通り、もう一度（どちらも ✅） |
| 18:37〜18:38（17:37〜17:38） | **ユーザーが `--arm auto` を空中で 2 回送った**（PC2 で `script -f -a ~/button_press/onboard_logs/user_20261113.log` の中で） |

## つなぎ方（PC2 の上で動かす）

- ノート PC の WSL は NAT（`eth0` 172.17.x）で、G1 の有線（192.168.123.x）にはつながっていない。`192.168.123.164` に ping は届かない
- PC2: Jetson、有線 `eth0` 192.168.123.164、無線 `wlan0` 192.168.0.82。メモリ 15 GB（使用 2.3 GB）、8 コア
- PC2 の Python は `~/button_press/venv`（conda の `lerobot` の Python 3.12.13 を `--system-site-packages` で使う）。
  cyclonedds・unitree_sdk2py・pinocchio・mujoco・cv2・onnxruntime・torch 2.11.0 が import できた
- 押す処理は PC2 で `~/button_press/venv/bin/python` を使い、`--network-interface eth0 --camera-config camera_onboard.yaml`（カメラは 127.0.0.1）
- 頭カメラの配信は `~/button_press/repo/Button_Press/J1-gen` で `setsid nohup bash real/depth_server/start_rgbd_server.sh --source videohub --max-fps 5`。
  ログは PC2 の `~/button_press/rgbd_server_20261113_*.log`
- PC2 で動いていた別の班のもの: `server.py --local --bind-iface wlan0 --no-auth`（歩行の状態の切り替え・音声・遠隔の状態表示。
  `SetFsmId` の許可 ID は 0, 1, 3, 4, 501, 702, 706）。押す処理とは別。止めていない
- PC2 に送ったもの:
  - `onboard.sh sync`: `J1-gen/{common,configs,real,weights}`、`Perception/common`（PC2 の `configs/obstacles.yaml` も上書き）
  - rsync で追加: `Yada/{common,configs,contest,sim}`、`J1-gen/contest_agent`（ボタンを押す経路の `run_dds.py` に要る。中身は変えていない）
  - どれも `--delete` なし（足すか上書きするだけ）

## 頭カメラの配信が止まっていた（videohub が空の画像）

- 1 回目（ロボットの起動から 4 分後）: `VideoClient.GetImageSample()` が code 0 で中身が空。`videohub_check.py` は `cv2.imdecode` の
  `!buf.empty()` で落ちる（空のときの表示が無いので、例外になる）
- RealSense は USB で見えていた（`lsusb` の 8086:0b3a、`--list-devices` で D435I シリアル 250122075509）
- videohub が `/dev/video4` を開いているので、ふつうの起動は `xioctl(VIDIOC_S_FMT) failed, errno=16 Device or resource busy`
- **ロボットの再起動で直った**。起動直後に videohub がカメラをつかみ損ねて、空のまま固まったと思われる（原因は確かめていない）
- 次から: 起動直後に `videohub_check.py` で 1 枚取れるかを先に見る。空なら再起動

## 頭カメラの向き（`check_head_tilt.py --live --frames 5`）

| フレーム | 面 | pitch | roll |
|---|---|---|---|
| 201〜207 | 床（点の 84〜86%、カメラから 1.22 m） | 51.0〜51.2° | +1.0° |
| 201・202 | 小さい面（6%） | 51.8〜52.0° | −1.5〜−2.2° |

- 判定: 51.0°。URDF 47.6° との差 +3.4°。事前確認（10° より大きいずれで止める）には当たらない
- 立つ位置の地図（`20261110_wheelchair_pitch53`）の 53° と近い
- 記録: PC2 の `~/button_press/repo/_local/button_press/head_tilt/20261113_172113.json`、画像 `.../probe/20261113_172043_color.png`（床と椅子）

## マイク（`mic_record.py --local-ip 192.168.123.164 --seconds 5 --label check_quiet`、PC2 で）

- 32 パケット・5.1 秒分（1 パケット 5120 バイト = 2560 サンプル = 0.16 秒）。大きさ −32〜−47 dBFS、一番強い周波数は 5.6〜6.4 kHz か 0.6〜0.8 kHz
- 高い音を 3 回見つけた: 1250〜1328 Hz、0.11〜0.30 秒、ふだんより 25〜33 dB。何の音かは分からない（周りの人の声・物音の可能性）
- **「0.1 秒より長い途切れ 31 回」は表示の誤り**: 1 パケットが 0.16 秒分なので、パケットの間が毎回 0.1 秒（`GAP_S`）を超える。
  5 秒で 5.1 秒分が届いていて、欠けはない → `mic_record.py` の途切れの判定を、パケットの長さに合わせて直す（未）
- このときロボットはまだ再起動前。マイクを起こす操作はしていない
- 記録: PC2 の `~/button_press/repo/_local/button_press/sound/20261113_171356_check_quiet.wav`（と .json）

## 空中のボタン（dry-run。何も送らない。PC2 で）

共通: `auto_press.py --target down --button-set wheelchair --air 0.40 <Y> --network-interface eth0 --camera-config camera_onboard.yaml`

| 時刻（PC2） | `--arm` | Y | 選んだ腕 | 計画 | 事前確認 |
|---|---|---|---|---|---|
| 17:25:36 | left | +0.14 | left | 指 0°/0°・経由点なし・IK 0.1 mm | 5 つとも ✅（`ai` 稼働、傾き 0.7°、カラー変化 1.41、51.0°、mode 1） |
| 17:26:08 | auto | −0.14 | **right**（右 14.0 cm） | 同上 | ✅ |
| 17:26:30 | auto | +0.16 | **left**（左 16.0 cm） | 同上 | ✅ |

- dry-run なので腕は動かず、「指先が進まなくなった（遅れ 346 mm）」→ 保持 → 戻る、の流れで終わる（予定どおり）
- 記録: 下の「証拠のファイル」

## 空中のボタン・左腕を送った（`--arm left --air 0.40 0.14 --execute`、PC2 17:30:29）

- PC2 で `setsid nohup ... auto_press.py ... --execute > ~/button_press/onboard_logs/air_left_173029.log` で起動（無線の ssh が切れても最後まで動いて戻るように）
- 事前確認 5 つとも ✅（頭カメラの向き 51.2°、面 4 枚）
- 流れ: look → reach（2.9 秒）→ settle（1 秒）→ press（2.5 秒、手前 33.2 cm → 押し込み終わり 40.7 cm）→ hold（0.6 秒）→ retract（2.5 秒）→ check → home（4.4 秒）。
  空中なので「指先が進まなくなった」は出ず、押し込み終わりまで進んだ
- 体（`body_motion.py`）: pelvis の移動 最大 2.6 mm [+2.6, +0.2, +0.4]・回転 0.60°（どちらも retract の 7.6 秒）、狙いのずれ 最大 5.7 mm、
  傾きの変化 IMU 0.82° / 脚の FK 0.60°、左右の足の位置関係 0.7 mm。11/3 の右腕（移動 最大 3.6 mm・回転 0.6° 未満）と同じくらい
- 左腕が体・頭に近づきすぎないか: 合図が無く、ユーザーは見逃した（このあとユーザーが打った 2 回で見た。下）

### 押した力の推定（`press_force.py --arm left`）

- 腕が止まっている間（look、25 周期）の推定トルクのばらつき: 0.032 0.022 0.042 0.035 0.022 0.054 0.011 Nm（肩 3・肘・手首 3）
- 押す向きの力（基準は press の前の 10 周期）:

| 段階 | 推定の力 |
|---|---|
| press（2.5 秒） | −0.2 → **11.2 N**（腕を伸ばすにつれて増える。横に 2.9 N） |
| hold（0.6 秒） | 8.6〜9.1 N |

- **何にも当たっていないのに 9 N 前後**。ばらつき（0.05 Nm 程度）より大きく、腕を前へ 7.5 cm 伸ばす間に増えるので、
  重力のモデルと実機の差、または姿勢で変わる何か（arm_sdk の PD の偏り・摩擦）によるずれと思われる。基準を press の前に取るので、
  基準の姿勢から離れるほどずれる
- MuJoCo では推定 10.1 N・実際 10.5 N で合っていた（右腕）。**実機では、このずれを引かないと「当たった・押せた」の判断に使えない**
- 右腕でも同じかを、空中の右腕を送って比べる（11/3 の記録には推定トルクの列が無い）
- 記録: `body_log/20261113_173036_down/`（`press_force.csv`・`body_motion.csv`）、`auto/20261113_173029_down.json`

## 空中のボタン・`--arm auto` を送った（ユーザーが打った。PC2 17:37・17:38）

ユーザーが ssh で PC2 に入り、`script -f -a ~/button_press/onboard_logs/user_20261113.log` の中で
`auto_press.py --target down --button-set wheelchair --arm auto --air 0.40 <Y> --network-interface eth0 --camera-config camera_onboard.yaml --execute`。

| Y | 選んだ腕 | 推定の力（押し込み中の最大 / 保持中の平均） | 止まっている間のばらつき | pelvis の移動・回転 | 狙いのずれ | 記録 |
|---|---|---|---|---|---|---|
| −0.14 | **right** | 8.7 N（横 3.4 N）/ 6.5 N | 0.004〜0.058 Nm | 1.5 mm・0.33° | 3.0 mm | `body_log/20261113_173719_down`、`auto/20261113_173712_down.json` |
| +0.16 | **left** | 10.5 N（横 3.6 N）/ 6.3 N | 0.010〜0.059 Nm | 1.7 mm・0.37° | 3.5 mm | `body_log/20261113_173800_down`、`auto/20261113_173753_down.json` |

- どちらも最後まで動いて戻った（終了コード 0）。足の位置関係の変化 0.5 mm、傾きの変化 IMU 0.5〜0.6°
- ユーザーの目: 「よくできていた」。左腕が体・頭に近づきすぎることはなかった
- **推定の力のずれは右腕でも出た**（空中で保持中 6.5 N）。左腕は 1 回目 8.8 N・2 回目 6.3 N（Y が 14 → 16 cm で違う）。
  → 押した力の推定は、空中で同じ動きをしたときの値（姿勢ごとのずれ）を引かないと使えない。直し方は机の上で考える
  （案: 押す前に、同じ経路を空中で動かしたときの推定を基準にする／重力のモデルの質量を実機に合わせる）

## 決めたこと

- 今日は有線のラボ PC を使わず、PC2 の上で押す処理を動かす（ユーザー。(b) を選んだ）
- **送る（`--execute`）コマンドはユーザーが打つ**。Claude は dry-run まで。`record.py` を流し続け、ユーザーは `script` の中で打つ（ユーザー）
- `docs/reports/REPORT_20261113.md`（次の実機日に試すこと）を `ToDo_20261113.md` に改名し、今日の REPORT と記録は別に作る（ユーザー）

## 次にやること（今日の残り）

- [x] 空中・左腕を送る
- [x] 送った回の body_log で `press_force.py`
- [x] 空中・`--arm auto` を 2 通り送る（右腕でも推定の力がずれた）
- [ ] 押した力の推定のずれを直す方法を考える（机の上で）
- [ ] 腰だけを回す（`waist_turn_check.py`。dry-run → ±5° → ±10° → ±20°）
- [ ] 1 回分だけ歩く（`loco_pulse.py`）
- [ ] Web カメラの位置を巻き尺で測る
- [ ] `mic_record.py` の途切れの判定を直す

## 証拠のファイル

PC2 の `~/button_press/repo/_local/button_press/` の下（PC2 の時刻）。あとでノート PC に持ってくる。

- `head_tilt/20261113_172113.json`、`probe/20261113_172043_*`
- `sound/20261113_171356_check_quiet.wav`・`.json`
- `auto/20261113_172536_down.json`・`20261113_172608_down.json`・`20261113_172630_down.json`、`body_log/20261113_172542_down`・`_172614_down`・`_172637_down`
- ずっと流す記録: `recordings/20261113_173448_day_watch/`（lowstate・画像 1 秒に 1 枚）。ユーザーの端末: `~/button_press/onboard_logs/user_20261113.log`
- 配信のログ: PC2 の `~/button_press/rgbd_server_20261113_1710.log`（videohub 失敗）、`..._1713_direct.log`（busy）、`..._1720.log`（動いた）
