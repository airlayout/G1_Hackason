# 実機日の手順書（下書き）

> ⚠️ **下書き**。タスク7で完成させる。ここには、それまでのタスクで「当日手順書に入れる」と決めた項目を
> 忘れないように集めている。

コマンドはすべてラボ PC のリポジトリ直下（`G1_Hackason/`）で、1行ずつ実行する。

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
  （具体的なコマンドはタスク7で追加する）

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

## 深度付きカメラサーバ（PC2）

手順は [depth_server/README.md](depth_server/README.md)。要点:

- `lsusb | grep -i intel` で RealSense が PC2 につながっているかを最初に確かめる。つながっていなければ、
  深度は使えないので、対象の位置を設定ファイルの値（定規で測った位置）で与えるモード（タスク6）に切り替える。
- `run_g1_server.py` は `--camera` なしで起動する（RealSense は 1 つのプログラムしか開けない）。
- ラボ PC で `real/probe_rgbd.py --save` を実行し、fps・内部パラメータ・深度を確かめて保存する。

## タスク7で追加する項目

- 接続確認（RGB・深度・lowstate、`mode_machine` = 5）
- 経路の判定: Dev/Navigation の `mode_check.py` と `armsdk_probe.py` を直接実行する手順
- ティーチング、FK の確認（手先の点 `end_effector` の実測との比較）、較正
- 作業空間の箱（`configs/press.yaml` の `workspace`、仮の値）の調整
