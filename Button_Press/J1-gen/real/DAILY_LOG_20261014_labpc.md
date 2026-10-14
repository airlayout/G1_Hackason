# 2026-10-14 の記録（ラボ PC 側・有線）

この日は、ノート PC（WSL2・無線）と、ラボ PC（有線）の 2 か所で作業した。ここはラボ PC 側の記録。
ノート PC 側（無線での RGB-D の取得と YOLO、PC2 の配信サーバの起動）は [DAILY_LOG_20261014_laptop.md](DAILY_LOG_20261014_laptop.md)。

[REAL_DAY_PROCEDURE.md](REAL_DAY_PROCEDURE.md) の段階0〜3の途中まで進めた。**次回は「次回の再開手順」から始める。**

## 結果のまとめ

| 段階 | 結果 |
|---|---|
| ラボ PC の環境構築 | 済み（下の「ラボ PC の環境」） |
| 0 接続確認 | OK。lowstate 1042 Hz、`mode_machine = 5`、腰・腕のモータ mode すべて 1 |
| RGB・深度 | OK。ノート PC 側が起動した PC2 の `rgbd_server.py --source videohub --max-fps 5`（同じサーバ）を有線で受信（5.1 fps、640x360、深度 0 の画素 8.2%、遅れ約 113 ms。無線は約 170 ms） |
| YOLO でボトル検出 | OK。5/5 フレームで 1 個、確信度 0.81〜0.84、位置のばらつき約 1 cm |
| 1 収録 | **未実施** |
| 2 経路の判定 | **arm_sdk が効く → プランA（立ったまま）** |
| 2 腕を小さく動かす | OK（肩ピッチ −5°・肘 +5°、`move_arm_real.py --path arm_sdk --execute`） |
| 重力補償の比較 | arm_sdk でも効く。**実機では `--gravity-scale 0.5` を付ける**（`configs/arm.yaml` の既定は 0.0 のまま） |
| 3 ティーチング | `press_bottle` を記録した（`configs/taught_poses.yaml`） |
| 3 FK の確認 | **未完了**（腕が下がって視野の外だった） |
| 3 机の障害物の箱 | **未実施** |
| 4 較正・5 押し込み・6 繰り返し | **未実施** |

## 測った値

### arm_sdk の確認（`armsdk_probe.py --joint 22`、右肩ピッチ +0.05 rad）

| 回 | 実測の最大変位 | メモ |
|---|---|---|
| 1 | +0.105 rad | ハーネスがきつく、機体のバランスが崩れて腕が振られた。戻りでも +0.05 残った |
| 2 | +0.030 rad（指令の 59%） | 右肩だけ動いた。戻りは開始から 0.005 以内 |

→ ハーネスは「立っているときは引っ張らず、倒れたときだけ受け止める」程度に緩める。

### 重力補償（`move_arm_real.py --path arm_sdk --execute --gravity-scale <倍率>`）

| 倍率 | 最大の追従誤差（目標へ / 戻る） |
|---|---|
| 0 | 1.38° / 1.38° |
| 0.5 | 0.54° / 0.70° |
| 1.0 | 0.08° / 0.49° |

- 1.0 では終わったあとに機体が少しふらついた（weight 1→0 で制御を返す間か直後）。原因は未確定（1 回だけで、ハーネスの影響もありうる）。
- 安全側で **0.5 を使う**。腕を動かすコマンド（`move_arm_real.py`・`press_bottle.py` の `--execute`）に `--gravity-scale 0.5` を付ける。
  押し込み（段階5・6）は毎回同じ倍率で行う。
- `configs/arm.yaml` の既定（`gravity_compensation.scale`）は 0.0 のまま。一度 0.5 にしたが、MuJoCo（`sim.gravity_compensation` で別に補償）でも
  読まれて二重になり、テスト 27 件が失敗したため戻した（2026-10-15）。

### ティーチング（吊り下げて、立った高さで。ダンピング）

- `press_bottle`: 指先（FK、pelvis 座標）**[+0.392, −0.119, +0.054] m**、腕 right [deg] `[-4.7, 1.9, -14.2, 31.3, -9.9, -20.9, -2.3]`
- 吊り下げ中は腰が yaw +9.2°、pitch −5.9° ねじれていた（立っているときは −0.1°、−2.3°）。FK は lowstate の腰の角度を使うので、pelvis 座標の値は正しい。
- 記録したときのボトルの位置は測れていない。それより前（立った状態）に測った位置は pelvis 座標 [+0.35, −0.15, +0.04] m。
  記録した指先はボトルの中心より 4 cm 前（遠い）ので、記録の間にボトルが動いていた可能性がある。**次回、ボトルの位置と合わせて確かめる。**
- ⚠️ この姿勢の指先は体の右 12 cm で、あとで決まった「勧める立つ位置（ボトル）」（`configs/pipeline.yaml` の `stand`、
  押す点が右 15〜25 cm・前 30〜40 cm）の外。**次回、範囲の中にボトルを置いて記録し直す**（IK の初期値に使うだけなので、古いままでも動くが、近い方がよい）

### カメラ（PC2）

- `rgbd_server.py --source videohub` は videohub が動いたままで使えた（2026-09-29 の「次回やること」の実機での確認が済んだ）。
- 内部パラメータなど配信の値は、ノート PC 側の記録と同じ（同じサーバ）。
- 有線でも、保存画像でカラーの向き・色（ボトルが赤茶に写る）と、深度の輪郭がカラーに重なることを確かめた。
- YOLO（`locate_bottle.py --live --frames 5`、**腰の角度は lowstate の実測**）: 机の上 0.53 m で 5/5 枚、確信度 0.81〜0.84、
  pelvis 座標 [+0.35, −0.15, +0.04] m（ばらつき約 1 cm）。ノート PC 側（腰 0° と仮定）の [+0.452, −0.190, +0.027] m とは
  **ボトルの置き場所が違うので、比べられない**。腰の仮定の影響を見るには、同じ置き場所で両方を測る

## はまったこと

### `*** buffer overflow detected ***: terminated` で落ちる

`move_arm_real.py` が `[dds] ネットワークの口: ...` の直後に落ちた。**原因は `~/.bashrc` 134 行目の
`export CYCLONEDDS_HOME="$HOME/cyclonedds/install"`**（別用途の古い CycloneDDS 0.10.2）。venv の cyclonedds は
`G1_HuggingFace/cyclonedds`（releases/0.10.x）に合わせて入れてあるので、版が食い違うと DDS の初期化で落ちる。
ラボ PC で新しいターミナルを開くたびに、最初に実行する:

```bash
export CYCLONEDDS_HOME=$PWD/G1_HuggingFace/cyclonedds/install
```

（`~/.bashrc` は、`~/unitree_sdk2_python` など別のものが古い方を使っている可能性があるので書き換えていない）

### FK の確認は、腕を支えたまま行う

ダンピング中は、`teach.py` を終えると腕が下がる。`fk_check.py --overlay` は、**人が腕を支えて中指の先が頭カメラに写る状態で**実行する。

### ボトルは動かさない

ティーチングと FK 確認の間に、ボトルが机の奥の左（pelvis [+0.68, +0.36] m、右腕では届かない）へ動いていた。
押す位置を決めたら、印を付けて動かさない。

## ラボ PC の環境

- 有線 NIC `enp129s0` = 192.168.123.200（NetworkManager の `G1-ethernet`、`shared` モード）。
  `shared` モードは dnsmasq で `192.168.123.1〜191` に DHCP を配っており、G1 内部の固定アドレス（.120 / .161 / .164）と範囲が重なる。今日は問題なし。
- SSH: `ssh g1`（`~/.ssh/config` にあり。パスワード認証）。G1 PC2 は Ubuntu 20.04.6 / aarch64。
- `G1_HuggingFace/venv`（Python 3.12.15、uv で作成）: cyclonedds 0.10.2（Python）、C の CycloneDDS は `G1_HuggingFace/cyclonedds`（releases/0.10.x, 5041f35）、
  unitree_sdk2py 1.0.1、lerobot（`[unitree_g1]`）、torch 2.11.0、ultralytics 8.4.177、mujoco 3.16.0、pin 4.1.0、numpy 2.2.6、opencv-python 5.0.0.93
- `configs/arm.yaml` の `network_interface: "enp129s0"`
- 公式モデルは取得済み（`_local/button_press/models/`）。`tests/run_tests.sh` は OK
- `../G1_nav`（`origin/Dev/Navigation` の worktree、7ccb66f）は次回も使うので**残してある**。全部終わったら `git worktree remove ../G1_nav`

## 今日のデータ（`_local/`、git に入らない）

- `_local/button_press/probe/20261014_212526_*`（RGB・深度の受信確認）
- `_local/button_press/locate/20261014_2125*`、`20261014_2149*`（YOLO の検出結果）
- `_local/button_press/fk_check/20261014_2149*`（腕が視野の外。参考にならない）

## PC2 の状態（今日の終わり）

- `rgbd_server.py --source videohub --max-fps 5` が動いたまま（PID 22437、19:53 にノート PC 側が起動。ラボ PC 側では起動・停止していない）。
  止めるときは PC2 で `pkill -f "real/depth_server/rgbd_server.py"`（ノート PC 側の記録と同じ。videohub は止めない）。
- G1 の電源を切ると止まる。次回は下の「次回の再開手順」の 2. で起動し直す。

## 次回の再開手順

ラボ PC のリポジトリ直下で、上から順に。**腕を動かすコマンド（`--execute`、`armsdk_probe.py`）は人がリモコンを持って実行する。**

1. ターミナルを開いたら:

   ```bash
   export CYCLONEDDS_HOME=$PWD/G1_HuggingFace/cyclonedds/install
   ```

2. 接続確認（何も送らない）:

   ```bash
   python3 Common/network/check_g1_connectivity.py
   ```
   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_connection.py --rgbd
   ```

   深度が届かなければ PC2 で配信サーバを起動する（`ssh g1` で入って、conda の環境に入らずに）:

   ```bash
   cd ~/button_press && bash real/depth_server/start_rgbd_server.sh --source videohub --max-fps 5
   ```

3. 経路の確認（電源を入れ直したら毎回。腕モータが mode 1 か、arm_sdk のままか）:

   ```bash
   G1_HuggingFace/venv/bin/python ../G1_nav/Teleop/vendor/g1-starter-kit/tools/mode_check.py --iface enp129s0
   ```

4. **ボトルを押す位置に置いて印を付ける。** 勧める範囲（`configs/pipeline.yaml` の `stand`、REAL_DAY_PROCEDURE.md の「段階5・6」）:
   pelvis から押す点（ボトルの手前の面）まで **前 30〜40 cm、右 15〜25 cm、机の手前の縁まで 29 cm 以上**、ボトルは机の縁から 10 cm 以内。
   位置を測る:

   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/locate_bottle.py --live --frames 5
   ```

5. 段階1の収録（今日やっていない。「必ずやり切る」段階）。REAL_DAY_PROCEDURE.md の「収録（タスク5）」。

6. 段階3の続き（吊り下げて立った高さ、ダンピング）:
   1. 必要なら `press_bottle` を記録し直す（`teach.py --name press_bottle --arm right`。ボトルが今日と違う位置なら記録し直す）
   2. **腕を支えて**中指の先をボトルの数 cm 手前に保ったまま、FK の確認:

      ```bash
      G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/fk_check.py --overlay
      ```

      丸が中指の先に重なるかを `_local/button_press/fk_check/` の画像で見る
   3. 机の障害物の箱（`teach.py --obstacle table --arm right`）

7. 段階4 較正 → 段階5・6 押し込み（REAL_DAY_PROCEDURE.md のとおり。`press_bottle.py` に **`--gravity-scale 0.5`** を付ける）
   - 2026-10-15 に入った「指先が進まなくなったら止める」（`configs/press.yaml` の `contact_stop`、遅れ 5 mm）は、
     シミュレーションの「触れる前の遅れ 2 mm 以下」に合わせた値。**実機は重力補償 0.5 で追従誤差が約 0.6° 残るので、
     ボトルに触れる前に止まらないか**を最初の送信で見る（「指先が進まなくなった（遅れ … mm）」が触れる前に出たら、
     `lag_m` を広げるか、重力補償を上げる）
   - 各回の終わりに出る「体（pelvis）の動き」を書き留める（REAL_DAY_PROCEDURE.md の「段階5・6」の 4.）
