# Yada の採点環境への接続

`sim/evaluate_yada.py` は、こちらのボタン押下処理をリーダーの採点環境で動かす入口。
実機はこれまでどおり `real/align_button.py` / `real/push_button.py` を使う。
実機の接続先、RealSense取得、校正ファイル、1.5mmの既存シミュレーション設定は変更していない。

## 準備

リポジトリ直下から実行する。このPCには `_local/button_press_yada/venv` を用意済み。
別のPCでは採点専用のPython環境を作り、以下の依存を入れる（Python 3.10でも動作確認した）。

```bash
python3 -m venv _local/button_press_yada/venv
_local/button_press_yada/venv/bin/python -m pip install -r Manipulation/push_button/sim/requirements_yada.txt
python3 Manipulation/push_button/sim/prepare_yada.py
```

`prepare_yada.py` は `origin/Dev/ButtonPress_Yada` の採点コードと、そのコードが使う
J1-gen・Perceptionの共通部品を、コミットIDごとの `_local/button_press_yada/evaluation_env/` に取り出す。
公式G1モデルもその中に取得する。ブランチ変更・merge・commit・pushはしない。
取得済みの参照を使う場合は `--no-fetch`。参照を明示する場合は `--ref <コミットまたは参照>`。
準備が終わったコミットだけを `current.json` に記録する。

通常配置の `Button_Press/Yada` がある場合はそちらを使う。配置を明示するには
`--yada-root /absolute/path/to/Button_Press/Yada` を付ける。
通常配置の場合はそのリポジトリの `Button_Press/J1-gen/sim/fetch_models.sh` でモデルを用意する。

## Python APIで採点する

```bash
# 上下の2試行。YOLOを省略するとカラー画像から灰色の丸を検出する。
MUJOCO_GL=egl _local/button_press_yada/venv/bin/python Manipulation/push_button/sim/evaluate_yada.py \
  --seeds smoke --workers 2

# ノイズ、カメラ取付誤差、遅れ、揺れを加え、弱点レポートも作る。
MUJOCO_GL=egl _local/button_press_yada/venv/bin/python Manipulation/push_button/sim/evaluate_yada.py \
  --set realistic --seeds practice --workers 2 --report

# 既存のボタン用YOLO重みで検出する。仮想環境にultralytics・torchも必要。
MUJOCO_GL=egl _local/button_press_yada/venv/bin/python Manipulation/push_button/sim/evaluate_yada.py \
  --weights /absolute/path/to/best.pt --seeds smoke --workers 2
```

`--seed N`、`--trials N`、`--view`、`--ablation`、`--out <パス>` は採点側へ渡す。
描画が遅いPCでは `--workers 1` を使う。画面のあるPCの `--view` は `MUJOCO_GL=egl` を外す。
結果は既定で、このリポジトリの `_local/button_press_yada/results/` に保存する。
弱点レポートは採点環境の `_local/button_press_yada/reports/` にでき、評価器が保存先を表示する。
全試行成功なら終了コード0、失敗した試行があれば1、起動・引数のエラーは元の終了コードになる。

## DDS + ZMQで模擬G1につなぐ

DDS方式にはCycloneDDSと公式 `unitree_sdk2py` が必要。このPCの採点用仮想環境には用意済み。
別のPCの導入は [採点側のGUIDE.md](https://github.com/airlayout/G1_Hackason/blob/Dev/ButtonPress_Yada/Button_Press/Yada/contest/GUIDE.md)
の準備手順を参照する。
SDKは公式ソースを `pip install -e <unitree_sdk2_pythonのディレクトリ>` で導入する。
通常のwheelインストールでは、版によってCRCの `.so` が含まれない。起動前にその不足も検査する。

```bash
MUJOCO_GL=egl _local/button_press_yada/venv/bin/python Manipulation/push_button/sim/evaluate_yada.py \
  --backend dds --seeds smoke
```

試行ごとに採点側の模擬G1を起動し、`run_yada_dds.py` がタスクファイルの上下指示を読んで接続する。
接続は `lo / domain 1`、カメラは `127.0.0.1` 固定。模擬G1の目印も確認する。
採点側のDDS送信処理を利用するため、`motor_cmd.mode=1`、制御重みの上げ下げ、状態受信の途絶への対応も適用される。
カメラの撮影時刻とIMU姿勢を観測に含め、同じ画像を3回計測したことにしない。
正式な採点はサーバ側の点灯判定。クライアントの処理完了は押下成功の判定には使わない。
判定が出たらランナーが制御重みを解除して終了する。判定ファイルをエージェントの制御入力には使わない。

DDS方式は `basic` のみ。`realistic`・並列化・弱点レポートの引数は、未対応として起動前に拒否する。
この採点用エージェントは実機への接続を拒否する。実機の設定を採点用で上書きしないための分離。

## こちらのコードとの関係

- `vision.estimate_button_target()` でRGB-Dの検出枠から表面位置・パネル法線を推定する。
- 一般用の上下ペアを画像から求め、指示された方を選ぶ。車いす用の高さの候補や曖昧な候補は採用しない。
- 異なる新しい3フレームで安定性を確認し、接近後に1回だけ再計測する。
- 最初に、推定したボタン面から離れた右側の経由点へ腕を持ち上げ、その後ボタン前へ移動する。
  経由点は指先球の表面と面の間を18cm、右側へのずれを12cmとする。
  低いボタンでも手首を折りすぎないよう、経由点の高さはpelvis基準で24cm以上とする。
  ボタン前では従来どおり指先球の表面と面の間を5cmに保ち、再計測してから押す。
- 腕のFK/IKと頭カメラの変換には採点側と同じ公式URDF・Pinocchioを使う。
  実機・既存シミュレーションのMuJoCo運動学はそのまま残す。
- `alignment.ContinuousNormalController` と `JointCommandFilter` を再利用して連続押下する。
  採点用の指先半径は8mm、押込量は2.5mm。これらは `yada.py` 内に分離している。
- 採点の弱いPDに合わせ、公式モデルの重力と公称ゲインから関節指令を補正する。
- ボタンの真の座標やseedは制御に使わず、MuJoCo / Isaacのimportもしない。

評価対象は到着後のボタン押し。骨盤が固定され、歩行・自由立位の安定性は評価できない。
既存のMuJoCo経路の自己衝突検査はこのPinocchioアダプタには移植していない。
経由点は壁への接触を避けるための改善であり、一般的な衝突回避機能ではない。
壁への接触力は採点結果に記録されるが、点灯すれば採点上は成功になる。接触力も結果と一緒に確認する。

## 検証記録

2026-10-05、採点環境 `634697e1544aa74b8f5d5869d58dbcc5dc9881d6`、
公式G1モデル `ccfc6fd`、MuJoCo 3.14.0、Pinocchio 3.9.0、Python 3.10.12で実行。
DDSはCycloneDDS 0.10.2、公式SDK `814556d15970dd2ecf1c9984e845ca02ab07e206`。
以下は接続追加時の記録（接近軌道の改善前）。77件のテストが通過（既存69件と採点接続8件）。

| 評価 | 成功 | 記録 |
| --- | --- | --- |
| API basic、カラー検出、smoke | 2 / 2 | 上14.38秒、下13.10秒。最大接触力35.58N |
| DDS basic、カラー検出、smoke | 2 / 2 | 上15.053秒、下13.304秒。最大接触力36.02N。クライアントも正常終了 |
| API basic、既存YOLO重み、smoke | 1 / 2 | 下ボタンの計測が2秒以内に安定せず中止 |
| API realistic、カラー検出、smoke | 0 / 2 | 上は再計測後の位置補正8cm超、下はFKによる押込量の上限超で中止 |

YOLO重みは既存の `/tmp/g1_elevator_yolo/models/best.pt` を使用した。
APIの時間はシミュレーション内、DDSの時間はサーバ判定の実時間。
この時点では上ボタンの移動経路で壁への接触が残っていた。
YOLO再検出、カメラ誤差や揺れへの耐性も改善が必要。
2試行だけの結果なので、未知の条件での成功率や実機動作を示すものではない。
各結果はこのリポジトリの `_local/button_press_yada/results/push_button_*smoke*.json` に保存した。

### 接近軌道の改善後（2026-10-05）

元のseed 0で接触を追跡すると、接近中の右指先が壁・柱に当たっていた。
面から離れた経由点で腕を持ち上げてからボタン前へ向かうように変更した。
実機用の処理・設定と、押下時の連続制御は変更していない。

| API basic（カラー検出） | 点灯までの時間 | 壁・盤への最大接触力 |
| --- | --- | --- |
| seed 0、上 | 16.46秒 | 0 N（改善前35.58 N） |
| seed 1、下 | 15.72秒 | 0 N |
| seed 3、下（壁が近い配置） | 15.50秒 | 0 N |
| seed 10、上（壁が遠く高いボタン） | 16.68秒 | 0 N |
| seed 11、下（壁が近く低いボタン） | 15.04秒 | 0 N |

実際に物理を動かした5試行は5/5成功、すべて壁・盤への接触力0 Nだった。
seed 0は改善前より2.08秒長くかかる。結果は
`_local/button_press_yada/results/raised_approach_basic.json`、改善後の上ボタンの動画は
`_local/button_press_yada/videos/mujoco_raised_approach_up_seed0.webm` に保存した。

DDS basicのsmokeも2/2成功、上16.600秒・下15.831秒、両方とも壁・盤への接触力0 N。
クライアントも両方正常終了した。結果は
`_local/button_press_yada/results/raised_approach_dds_smoke.json` に保存した。

97件のテストが通過（既存77件＋公式モデルを使った20配置の接近軌道の検査）。
追加した検査は画像推定を固定し、理想的な関節追従を仮定して、制御器が生成する軌道を調べる。
画像・PD追従・押下の判定を含む評価は上の5試行。20配置すべての押下成功を確認したわけではない。
経由点だけで任意の障害物を避けられる保証はなく、YOLO・realisticの改善前の結果も引き続き参照する。

```bash
MUJOCO_GL=egl _local/button_press_yada/venv/bin/python -m pytest \
  Manipulation/push_button/tests/test_yada.py Manipulation/push_button/tests/test_yada_approach.py -q
```
