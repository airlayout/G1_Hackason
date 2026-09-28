# 実機日の手順書（ボトル押し）

コマンドはすべてラボ PC のリポジトリ直下（`G1_Hackason/`）で、1行ずつ実行する。
`<NIC>` は G1 につないでいる有線 NIC の名前（`ip -br a` で確認し、`configs/arm.yaml` の `network_interface` にも書く）。

## 安全のルール（全段階で守る）

- **人が常にリモコンを持つ。緊急時は L2+B（ダンピング）。**
- 腕を手で動かす（ティーチング・較正）ときはダンピング状態にする。**ダンピング中は全身が脱力するので、
  必ず座った状態か吊り下げた状態で行う**（`teach.py` / `calibrate.py` は開始時に確認して Enter を待つ）。
- プランB（デバッグモード + lowcmd）もバランス制御が止まるので、座った状態か吊り下げた状態で行う。
- 実機に送るスクリプトは、まず dry-run（既定）→ `--execute`（確認モードで各段階 Enter）の順。
- 実機で撮ったデータは取り直せない。収録が終わるたびにバックアップする。

## 当日の流れ（HANDOFF 8章）

| 段階 | 内容 | 使うもの（この手順書の節） | 失敗したときの対応 |
|---|---|---|---|
| 0 | 接続確認（RGB・深度・lowstate、`mode_machine` = 5） | 「段階0: 接続確認」「深度付きカメラサーバ」 | 深度がだめなら「pyrealsense2 が使えなかった場合の切り替え」 |
| 1 | ボタン撮影とボトル周りの収録 | 「収録（タスク5）」 | 必ずやり切る |
| 2 | 経路の判定（arm_sdk が効くか） | 「段階2: 経路の判定」 | 効かなければ座った状態（吊り下げ）でプランB |
| 3 | ティーチング、FK と実物の比較 | 「段階3: ティーチングと FK の確認」「重力補償の確認」 | URDF と関節の対応、指先の点を確かめる |
| 4 | 較正 | 「段階4: 較正」 | 一定のずれなら補正値を入れる |
| 5 | dry-run → 確認モードで低速 → 押し込み | 「段階5・6: 押し込み」（タスク6） | 手前の姿勢まで行ければ成功 |
| 6 | 置き場所を変えて繰り返す | 同上 | 全試行を収録する |

## 事前準備（ラボ PC）

ラボ PC（x86_64）には SETUP.md のフルセット（lerobot 込み）が入っている前提。
ボタン押し用のパッケージ（IK 用の Pinocchio = `pin` など）を追加で入れる:

```bash
G1_HuggingFace/venv/bin/pip install -r Button_Press/J1-gen/requirements.txt
```

公式モデルを取得する（初回だけ。ネットワークが必要）:

```bash
bash Button_Press/J1-gen/sim/fetch_models.sh
```

入ったかの確認（最後に `OK`）:

```bash
PYTHON=G1_HuggingFace/venv/bin/python bash Button_Press/J1-gen/tests/run_tests.sh
```

G1 につないでいる有線 NIC の名前を調べ、`configs/arm.yaml` の `network_interface` に書く:

```bash
ip -br a
```

## プランB（デバッグモード + lowcmd）を使う場合

- 座った状態か、吊り下げた状態で行う（開始時にスクリプトが確認を求める）。
- **開始直後に、まず腰の角度を lowstate で確かめる。** 腰の3関節は Kp=300 / Kd=3.0 で開始時の角度に
  保持している（MuJoCo では、公式 low_level サンプルの Kp=40 だと上体の重さで腰ピッチが約 19° 倒れた）。
  開始時から `safety.waist_max_deviation_rad`（既定 0.05 rad ≈ 2.9°）以上ずれると自動で中止する。
  スクリプトを動かしている間に、別のターミナルで腰の角度を表示して確かめる（何も送信しない）:

  ```bash
  G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_connection.py --seconds 1
  ```

  「腰 yaw / roll / pitch」が開始前とほぼ同じ（1° 以内）なら OK。倒れていくなら、すぐに止める（Ctrl+C、危なければ L2+B）。

## 重力補償の確認

腕を同じ目標へ動かし、重力補償の倍率を **0 → 0.5 → 1.0** の順に変えて、腕の下がり方（追従誤差）を比べる。
表示される「最大の追従誤差」を記録する。

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/move_arm_real.py --path lowcmd --execute --gravity-scale 0
```
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/move_arm_real.py --path lowcmd --execute --gravity-scale 0.5
```
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/move_arm_real.py --path lowcmd --execute --gravity-scale 1.0
```

- MuJoCo（プランB の条件）では、倍率 0 / 0.5 / 1.0 で腕の追従誤差が 1.74° / 0.87° / 0.01°。
- 腰の重力補償（`--waist-gravity-scale`、プランB のときだけ効く）は、腕の確認のあとで必要なら試す。
- lowstate の IMU が pelvis のものかは未確認。倍率 1.0 で逆に下がり方が大きくなる場合は、IMU の向きを疑う。

## 事前準備: PC2 用のオフラインのファイル（実機日の前に、ネットにつながったマシンで）

PC2 はインターネットにつながっていない可能性が高いので、pyrealsense2（Python 3.8 / 3.10 / 3.12 用）と
libusb-1.0 の .deb を事前にダウンロードしておく（約 17 MB）:

```bash
bash Button_Press/J1-gen/real/depth_server/fetch_offline_packages.sh
```

`_local/button_press/offline/` にできる。手元の PC で用意した場合は、このフォルダをラボ PC に持っていく
（`_local/` は git に入らないので、push しても届かない）。

## 深度付きカメラサーバ（PC2）

詳しい手順は [depth_server/README.md](depth_server/README.md)。流れ:

1. RealSense が PC2 につながっているかを確かめる（PC2 で）:

   ```bash
   lsusb | grep -i intel
   ```

2. Python の版を確かめ、pyrealsense2 が無ければオフラインのファイルで入れる。ラボ PC から PC2 へコピー:

   ```bash
   ssh unitree@192.168.123.164 mkdir -p button_press
   ```
   ```bash
   scp -r _local/button_press/offline unitree@192.168.123.164:button_press/
   ```

   PC2 で（conda を使うなら先に `source ~/miniforge3/bin/activate lerobot`）:

   ```bash
   python3 --version
   ```
   ```bash
   bash ~/button_press/offline/install_offline.sh
   ```

3. サーバのファイルを PC2 に置き（README の 3）、`run_g1_server.py` を **`--camera` なしで**起動し（README の 4）、
   `start_rgbd_server.sh` でサーバを起動する。
4. ラボ PC で届いているかを確かめて保存する:

   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/probe_rgbd.py --save
   ```

## pyrealsense2 が使えなかった場合の切り替え

次のどれかになったら、深度はあきらめて切り替える（RGB の収録と、ボトル押しそのものは続ける）:

- `lsusb` で RealSense が PC2 に見えない（PC1 につながっている）
- `install_offline.sh` が失敗する（Python の版に合う wheel が無い、など）
- `start_rgbd_server.sh` が起動しない、または `probe_rgbd.py` で深度が届かない

切り替えの手順:

1. PC2 で `rgbd_server.py` を止める（動いていれば Ctrl+C）。RealSense は 1 つのプログラムしか開けないので、
   止めないと次の手順でカメラを開けない。
2. PC2 で `run_g1_server.py` を止め、**`--camera` を付けて**起動し直す（RGB は OpenCV 経由で、既存と同じ形式・ポート 5555）:

   ```bash
   python -u src/lerobot/robots/unitree_g1/run_g1_server.py --camera
   ```

   画像が来ないときは、カメラのデバイス番号（既定 4 = `/dev/video4`）が違う可能性がある。PC2 で一覧を見て、
   `--camera-device <番号>` で指定する:

   ```bash
   ls -l /dev/v4l/by-id/
   ```

3. ラボ PC で RGB が届いているかを確かめる（既存の Perception の確認スクリプト）:

   ```bash
   G1_HuggingFace/venv/bin/python Perception/real/probe_zmq_camera.py --host 192.168.123.164 --timeout 60
   ```

4. 対象（ボトル）の位置は、タスク6の「定規で測った位置」モードで与える
   （設定のキー名と具体的なコマンドは、タスク6の実装後にここへ追記する）。

## ボトルの位置を求める（タスク4）

- **検出するときは、腕をカメラの視野から外す**（手がボトルを隠さないように）。
- ボトルはラベル付きのもの（透明な PET ボトルは深度が取れない）。
- ライブで確かめる（腰の角度は lowstate から読む）:

  ```bash
  G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/locate_bottle.py --live --frames 5
  ```

- YOLO がボトルを検出するか、枠と基準点が胴に乗っているかを、保存された画像（`_local/button_press/locate/`）で見る。
- 深度が使えないときは、`probe_zmq_camera.py` などで撮った RGB で YOLO の検出だけを確かめる。

## 収録（タスク5）

「必ずやり切る」段階（HANDOFF 8章の段階1）。lowstate も一緒に記録する（NIC は `configs/arm.yaml`）。
`--note` に、置いた場所・照明・ボトルの種類などを書いておく。

ボトルのまわり（全フレーム。Ctrl+C で止める）:

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label bottle --note "机の上、ラベル付きボトル"
```

ボタンの撮影（①の学習用。150〜300 枚。背景を変えたもの 2〜3 割、ボタンが写っていないもの 1 割）。
構図を決めてから撮るなら `enter`、動かしながら撮るなら `interval`:

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label button --mode enter
```
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label button --mode interval --interval-s 1
```

深度が使えないときは `--rgb-only` を付ける（`run_g1_server.py --camera` の 5555 から受信する）。

**収録が終わるたびに、バックアップする**（取り直せないため）。終了時に表示される `cp -r ...` を実行する。

## 段階0: 接続確認

何も送信しない。lowstate の頻度、`mode_machine`（5 であること）、腰・腕のモータの mode、腰の角度、
指先の位置（FK）を表示する。`--rgbd` で深度付きストリームも確かめる:

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_connection.py --rgbd
```

- `mode_machine` が 5 でなければ中止（機体構成が違う。IK のモデルが合わない）。
- モータの mode が 0 なら、リモコンでダンピング（FSM 1）に入れて有効化する（指令側からは有効化できない）。
  プログラムを終了するたびにゼロトルクに戻ることがあるので、**実行の前に毎回確かめる**。

## 段階2: 経路の判定（Dev/Navigation の g1-starter-kit を直接実行する）

g1-starter-kit はこのブランチに取り込まず、Dev/Navigation を別のフォルダ（`../G1_nav`）に出して実行する。
今の作業フォルダには影響しない（`git worktree`: 同じリポジトリの別のブランチを、別のフォルダに取り出す仕組み）:

```bash
git fetch origin
```
```bash
git worktree add --detach ../G1_nav origin/Dev/Navigation
```

1. モーションコントローラの状態と、どちらの経路が効くかを見る（何も動かさない）:

   ```bash
   G1_HuggingFace/venv/bin/python ../G1_nav/Teleop/vendor/g1-starter-kit/tools/mode_check.py --iface <NIC>
   ```

2. arm_sdk が効くかを、関節を 1 つだけ約 3° 動かして確かめる（右肩ピッチ = motor 22。確認を求められる）:

   ```bash
   G1_HuggingFace/venv/bin/python ../G1_nav/Teleop/vendor/g1-starter-kit/tools/armsdk_probe.py --iface <NIC> --joint 22
   ```

   - 「✅ arm_sdk が効いています」→ プランA（`--path arm_sdk`）で進める
   - 「❌ arm_sdk が効いていません」→ **座った状態か吊り下げた状態にしてから**、デバッグ状態へ切り替えてプランB:

     ```bash
     G1_HuggingFace/venv/bin/python ../G1_nav/Teleop/vendor/g1-starter-kit/tools/mode_check.py --iface <NIC> --release
     ```

     終わったら元の状態へ戻す:

     ```bash
     G1_HuggingFace/venv/bin/python ../G1_nav/Teleop/vendor/g1-starter-kit/tools/mode_check.py --iface <NIC> --restore
     ```

3. 選んだ経路で、腕を小さく（肩ピッチ −5°、肘 +5°）動かして戻す。まず dry-run、次に `--execute`:

   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/move_arm_real.py --path arm_sdk
   ```
   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/move_arm_real.py --path arm_sdk --execute
   ```

   プランB なら `--path lowcmd`。**開始直後に腰の角度を確かめる**（別のターミナルで `check_connection.py`。
   上の「プランB（デバッグモード + lowcmd）を使う場合」）。

実機日が終わったら、取り出したフォルダを片付ける:

```bash
git worktree remove ../G1_nav
```

## 段階3: ティーチングと FK の確認

⚠️ 腕を手で動かすので、ダンピング状態にする。**全身が脱力するので、必ず座った状態か吊り下げた状態で行う。**

1. 押し込み姿勢を記録する（腕を手で動かし、中指の先がボトルに触れる少し手前で Enter）。
   `configs/taught_poses.yaml` に保存され、タスク6で IK の初期値に使う:

   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/teach.py --name press_bottle --arm right
   ```

2. FK と実物を比べる。FK で求めた中指の先を頭カメラの画像に丸で描いて保存する（`_local/button_press/fk_check/`）:

   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/fk_check.py --overlay
   ```

   - 丸が画像の中の**実物の中指の先**に重なっていれば OK（FK とカメラの取り付け位置の両方が合っている）。
   - ずれていたら、指先の点（`configs/robot.yaml` の `end_effector`。今は公式メッシュの中指の先）を
     `--ee-offset X Y Z` で試しに変えて、丸が中指の先に重なる値を探す。見つかったら robot.yaml に書き写す:

     ```bash
     G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/fk_check.py --overlay --ee-offset 0.17 0.028 0.007
     ```

   - 腕をいくつかの姿勢にして、どの姿勢でも同じようにずれるか（一定のずれ）、姿勢で変わるか（関節の対応や
     モデルの違い）を記録する。

## 段階4: 較正

⚠️ 指先で触れるときは腕を手で動かすので、座った状態か吊り下げた状態で行う。
押す位置の近くで、ボトルの置き場所を変えて 3〜5 か所。各場所で、**先に腕をどけてカメラで測り、そのあとで
中指の先を触れさせる**（触れている状態で撮ると、手が写り込んで深度が狂う）:

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/calibrate.py --arm right
```

- 最後に、場所ごとの補正値の候補（差）と、平均からのずれが表で出る。ばらつきが数 mm なら一定のずれなので、
  `--write` を付けてやり直すか、`configs/localize.yaml` の `calibration.offset_pelvis_m` に平均を書く。
- ばらつきが 1 cm を超える、またはずれが 5 cm を超えるときは、補正では直らない。段階3の結果（FK）と、
  頭カメラの取り付け位置（`configs/robot.yaml` の `head_camera`）を疑う。

## 作業空間の箱の調整

`configs/press.yaml` の `workspace`（手先が入ってよい箱、pelvis 座標）は仮の値。段階3で記録した押し込み姿勢の
指先の位置（`configs/taught_poses.yaml` の `fingertip_pelvis_m`）と、段階4のボトルの位置が箱の中に入っているかを見て、
必要なら広げる（押す位置のまわり ±10〜15 cm 程度にとどめる）。

## 段階5・6: 押し込み

（タスク6の全体をつなぐスクリプトを作ったあとで、ここに手順を書く）

## 実機日が終わったら

- 収録（`_local/button_press/recordings/`）と、較正・FK 確認の画像（`_local/button_press/`）をバックアップする。
- `configs/taught_poses.yaml`、`configs/localize.yaml`（補正値）、`configs/robot.yaml`（指先の点を直した場合）、
  `configs/arm.yaml`（NIC 名など）の変更をコミットして残す。
- `git worktree remove ../G1_nav`
