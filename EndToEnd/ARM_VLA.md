# G1のアームをVLAとUIで操作する

**選定モデルはGR00T N1.7 3B。推論をRTX 5060 Ti 16GB、カメラ取得と腕の50Hz制御をG1のOrin NXに分ける。**
UIで指示を入力すると、頭部RGB画像と身体の関節角から左右の腕の目標角度を予測する。
G1の29DoF構成、Orin NX/aarch64、D435i、内部DDS `eth0` / domain `0` は
[G1_IO_SPECIFICATION.md](G1_IO_SPECIFICATION.md)の1・3・10章に合わせている。
G1側のJetPack・PyTorchや既存のIsaac環境は更新しない。

## 最初にUIを見る

リポジトリのルートで実行する。

```bash
bash EndToEnd/arm-setup.sh --ui
bash EndToEnd/arm-ui.sh --demo
```

[http://127.0.0.1:8765](http://127.0.0.1:8765)を開く。
**DEMOは通信と画面の確認専用。VLA・GPU・実機は使わず、腕の角度を保持する。**
画像、指示入力、実行、停止、左右の関節角、推論時間、履歴を表示する。
`Esc`で停止、`Ctrl+Enter` / `⌘+Enter`で実行。指示の例は英語を入力欄へ入れるだけで、自動実行しない。

## GPU PCの準備

- Linux x86_64 / Python 3.12 / `uv` / RTX 5060 Ti 16GB。
- CUDA 12.8対応ドライバ。PyTorchは`2.9.0+cu128`を専用の`.venv-vla`に入れる。
- 初回は空きディスク**30GB以上**。本体重み約6.9GBに加え、backboneとCUDAライブラリも必要。
- Hugging Faceで[nvidia/Cosmos-Reason2-2B](https://huggingface.co/nvidia/Cosmos-Reason2-2B)の利用条件に同意し、アクセス許可を取得する。
  GR00Tの公式ローダーが利用するbackboneのため、GR00T本体の重みが取得できても、この許可は別途必要。

```bash
uvx --from 'huggingface-hub==0.36.0' hf auth login
bash EndToEnd/arm-setup.sh --model
```

認証は`HF_TOKEN`でもよい。トークンはソースや設定JSONに書かない。
セットアップは既存データを削除しない。空き容量不足なら、インストール前に終了する。
モデル・backbone・GR00Tソースのrevisionは[configs/vla/groot.json](configs/vla/groot.json)、
Python依存は`requirements-vla.txt`で固定している。

**Blackwellのsm_120向けにFlashAttention2をインストールしない。**
実行用モデル設定の`use_flash_attention=false`で、公式Qwen3 backboneのPyTorch SDPA経路を使う。
HF共有キャッシュを書き換えず、`assets/cache/vla/model`に設定と重みへの参照を作る。
TensorRTは不要。

診断:

```bash
PYTHONPATH=EndToEnd/src EndToEnd/.venv-vla/bin/python -m g1_e2e.vla.tools doctor
```

GPUが複数ある場合は表示された番号を確認する。6GBのGPUを選ばない。
`arm-ui.sh --vla --device cuda:0`で16GBのGPUを指定できる。
VRAMの実測値と推論時間は後述の`benchmark`で確認する。5060 Tiでの実測速度はまだ記録していない。

## G1へ配備する

GPU PCで小さな機体用アーカイブを作る。モデルや仮想環境は含まない。

```bash
bash EndToEnd/arm-pack.sh
scp EndToEnd/assets/cache/vla/g1-arm-bridge.tar.gz unitree@192.168.0.82:/tmp/
ssh unitree@192.168.0.82
```

以下はG1のシェルで実行する。IPは仕様調査時の値なので、現場の値へ置き換える。

```bash
mkdir -p /home/unitree/work/G1_Hackason
tar -xzf /tmp/g1-arm-bridge.tar.gz -C /home/unitree/work/G1_Hackason
cd /home/unitree/work/G1_Hackason
```

機体用コードはPython 3.8以降を想定し、`numpy`、`opencv-python`（またはシステムの`python3-opencv`）、
既存の`unitree_sdk2py`とCycloneDDSを利用する。GPU PC用のrequirementsをOrinへ入れない。

```bash
python3 -c 'import numpy, cv2; print(numpy.__version__, cv2.__version__)'
ls -l /dev/v4l/by-id/
```

RealSenseの**RGB**用の安定したパスを選ぶ。`/dev/videoN`は再接続で番号が変わる。
既存のカメラ配信とUSBデバイスを同時に開かない。
SDKのパスが異なる場合は`G1_SDK_PATH`、使用するPythonは`G1_ARM_PYTHON`で指定する。

まずは送信なしで起動する:

```bash
bash EndToEnd/arm-robot.sh --camera /dev/v4l/by-id/実際のRGBデバイス名
```

この段階ではDDSのpublisherを作らない。カメラを開き、`rt/lowstate`を購読する。
`MotionSwitcher.ReleaseMode()`、デバッグモードへの切替、サービス停止は行わない。

## GPU PCからつなぐ

GPU PCの別ターミナルで、機体のローカルポートを転送する:

```bash
ssh -N -L 8766:127.0.0.1:8766 unitree@192.168.0.82
```

UIを起動する:

```bash
bash EndToEnd/arm-ui.sh --vla --robot http://127.0.0.1:8766
```

カメラと14関節の値を確認する。画面の「送信なし」状態で指示を実行すると、本物のVLA推論と指令検査を通るが、モーターへは送らない。
モデルは最初の実行時にロードされる。読み込み中も停止できる。

**実推論を一度だけ計測する**場合は、次のコマンドを使う。腕の制御開始・指令送信は行わない。

```bash
PYTHONPATH=EndToEnd/src EndToEnd/.venv-vla/bin/python -m g1_e2e.vla.tools benchmark \
  --robot http://127.0.0.1:8766 \
  --instruction 'Press the button in front of you with your right hand.'
```

`inference_ms`、`peak_vram_gb`、`shape=[8,14]`、`robot_commands_sent=0`が出る。
推論元の観測が3秒より古い場合、機体は予測を受け付けない。
3秒を超える場合は機体を動かす前に、GPU選択・他プロセス・モデルのロード状態を確認する。

## 実機で腕を動かす

他の腕制御プロセス（gesture・既存arm_sdkブリッジ・teleop）を終了し、機体を安定して立たせる。
最初は腕の周囲を空け、リモコンの非常停止を用意する。
G1側の送信なしブリッジを`Ctrl+C`で終了し、同じコマンドへ`--execute`を追加する。

```bash
bash EndToEnd/arm-robot.sh --camera /dev/v4l/by-id/実際のRGBデバイス名 --execute
```

GPU PCのUIで「実送信」を確認し、指示して実行する。`--execute`だけでは制御権を取らず、
UIで実行して有効な予測が届いた時点で腕の制御を開始する。
**歩行・腰・ハンドへの指令は送らない。** バランスは既存の機体コントローラが担当する。
UIの停止はarm_sdkのweightを下げ、制御を機体へ返す操作。非常停止とは別である。

初回は短い到達動作を試し、実際の角度が追従していることを確認する。
`arm_sdk`の有効性は機体のfirmware・モードに依存する。
反応しない場合、全身の`rt/lowcmd`へ自動で切り替えない。

## 予測と制御の契約

```text
5060 Ti PC                                  G1 / Orin NX
UI → GR00T → 左右の腕14関節 [rad] ─ HTTP ─→ 50Hz制御 → rt/arm_sdk
         ↑                                ↓
         └──── RGB JPEG・身体29関節 ────── カメラ / rt/lowstate
```

- 入力画像は標準JPEGをRGBへデコードする。現在画像と約20/30秒前の画像を渡す。
- `REAL_G1`の観測schemaを起動時に検査する。手首姿勢9次元は公式URDFから順運動学で算出し、
  位置[m]と回転行列の最初の2行を使う。既定の基準は`pelvis`、手先は左右の`wrist_yaw_link`。
  使用するcheckpointの手首フレーム・カメラ配置との一致は現場で確認する。把持点の姿勢ではない。
- ハンドは搭載未確認のため、**入力を左右7次元の固定0とする仮定**がある。
  予測されたハンド・腰・移動のactionは破棄する。ハンドの実状態を読んでいると解釈しない。
- 公式Policy APIで相対actionを絶対関節角へ復元した後、左右7関節ずつを抽出する。
  現在角度は二重に加算しない。先頭8ステップを30Hzで解釈し、G1側で50Hzに追従する。
  推論中は最後の目標を保持するため、動作は停止を挟む場合がある。連続動作の性能改善は実測後に行う。

制限値は`groot.json`の`arm`で設定する。既定では:

| 条件 | 挙動 |
|---|---|
| NaN・Inf、次元不一致、可動域外、開始姿勢から0.8rad超の予測 | 拒否して実行を停止 |
| 目標の速度 | 各関節0.4rad/sまでに制限 |
| 状態が0.25秒途絶 / カメラが1秒途絶 | 停止し、weightをゼロへ戻す |
| 指令が3秒途絶 / 予測元が3秒超経過 | 停止・拒否 |
| 目標と実角度のずれが0.5rad超 | 停止 |
| 1回の実行が30秒経過 | 自動終了 |
| 停止後の予測、同じ観測の再送 | session/観測IDで拒否 |

watchdogはG1側の独立スレッドで動く。GPU処理やネットワーク受信を制御ループに入れない。
起動姿勢から開始し、weightは約1秒で増減する。脚・腰のmotor commandは無効のままにする。
`SIGINT` / `SIGTERM`ではweightを戻して終了する。
**機体プロセスそのものの強制終了、OS停止、電源断に対してソフトウェアの復帰は保証しない。**
自己衝突・障害物回避や接触力制御は未実装。モデルの予測だけで接触安全性を判断しない。

SSH転送を使わずG1側の`--bind 0.0.0.0`で公開する場合は、両PCへ同じ24文字以上の`G1_ARM_TOKEN`を設定する。
UI自体はGPU PCのlocalhostのみで待ち受け、別Originからの操作と任意ファイル配信を拒否する。

## モデル選定の根拠と現在の検証範囲

- [NVIDIAのハードウェア要件](https://github.com/NVIDIA/Isaac-GR00T/blob/c56f54128d92f743889b7e4cecfc1ceafd88739e/getting_started/hardware_recommendation.md): 推論は16GB以上、追加学習はより大きいGPUを推奨。
- [公式Policy APIとREAL_G1](https://github.com/NVIDIA/Isaac-GR00T/blob/c56f54128d92f743889b7e4cecfc1ceafd88739e/getting_started/policy.md): REAL_G1はbase checkpointで使える事前学習対象。
  SONIC用のtagとは異なり、今回のアダプタは腕の関節角を使う。
- [モデル](https://huggingface.co/nvidia/GR00T-N1.7-3B): G1向け事前学習を出発点にできる。
  **現場のボタン押し・カメラ・ハンド構成での成功を保証するcheckpointではない。**
  固定ハンド入力や視点の違いは精度に影響し、必要に応じて現場データで追加学習する。
  互換の追加学習済みモデルは`arm-ui.sh --vla --model-path /path/to/checkpoint`で指定する。

今回の環境では空きディスク約9GBのため、重み・GPU依存の導入と5060 Tiでの実推論は行っていない。
HTTP経由の実行、SDK型とCRC、腕だけの送信内容、入力変換、速度制限、通信断・停止後の遅延結果は自動テストで確認する。
ブラウザの表示とシミュレータ・実機動作は未検証。リポジトリの`AGENTS.md`に従い、現場で確認する。

```bash
EndToEnd/.venv/bin/python -m pytest EndToEnd/tests/vla EndToEnd/tests/test_dds.py -q
```

ファイルの役割:

| 場所 | 担当 |
|---|---|
| `src/g1_e2e/vla/control.py` | モデルに依存しない腕の状態機械・watchdog・指令検査 |
| `robot.py` | Orin上のDDS・USBカメラ・制御スレッド |
| `kinematics.py` / `policy.py` | 観測とGR00Tの入出力の変換 |
| `session.py` | 観測受信・推論・停止の並行処理 |
| `app.py` / `static/` | UIとAPI |
| `tools.py` | モデル取得・GPU診断・送信なしの実推論計測 |
