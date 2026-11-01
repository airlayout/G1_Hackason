# 強化学習でボトルを押す（範囲 A）

右腕 7 関節だけを強化学習の方策で動かし、ボトルの手前の面を押す。ボトルの位置は今の検出（YOLO と深度）で出す前提で、
方策には位置を数値で渡す（画像は渡さない）。計画と比べ方・見積もりは [docs/reports/rl_press_plan.md](../docs/reports/rl_press_plan.md)。

| ファイル | 中身 |
|---|---|
| `press_env.py` | 学習環境（Gymnasium）。MuJoCo、胴体固定、机とボトルの位置をエピソードごとにランダムに変える |
| `check_env.py` | 環境の確認（速さ、成功・当たりの判定が働くか） |
| `../configs/rl_press.yaml` | 設定（行動の大きさ、配置の範囲、成功の条件、報酬） |
| `requirements.txt` | 強化学習だけで使うもの（gymnasium、stable-baselines3） |

## 環境の作り方（ノート PC）

共有の `G1_HuggingFace/venv` は変えずに、別の venv を `_local/rl/venv` に作る。中から `G1_HuggingFace/venv` の
パッケージ（mujoco、torch、numpy、pin）を読めるようにして、足りない 2 つだけを入れる。リポジトリ直下で:

```bash
uv venv --seed --python /home/tvdue/.local/share/uv/python/cpython-3.12-linux-aarch64-gnu/bin/python3.12 _local/rl/venv
echo "import site; site.addsitedir('$PWD/G1_HuggingFace/venv/lib/python3.12/site-packages')" > _local/rl/venv/lib/python3.12/site-packages/g1_huggingface_venv.pth
_local/rl/venv/bin/python -m pip install -r Button_Press/J1-gen/rl/requirements.txt
```

`.pth` は「Python の起動時に読む追加の場所」を書くファイル。`import site; site.addsitedir(...)` と書くと、その場所の `.pth`
（pin の `cmeel.pth` など）も読まれる（場所だけを書くと pin が見つからない）。

## メンバーの PC で学習する（jigen@sodai-ubuntu）

学習に要るのは mujoco・gymnasium・stable-baselines3・torch・pyyaml だけ（pin は要らない）。方策は小さいので GPU は使わず、
torch は CPU 版を入れる（約 1 GB。CUDA 版より小さく、`~/yolo_env` には手を付けない）。PC は共用なので、他の人の作業を見てから
並列の数（`--envs`）を決める（8 コア 16 スレッド。他に重い処理が無ければ 14）。

1. ノート PC（WSL）で一式を作る:

```bash
bash Button_Press/J1-gen/rl/make_bundle.sh
```

2. Windows の PowerShell で送る（WSL からはその PC に届かないため。ファイル名は 1 の表示に合わせる）:

```powershell
scp \\wsl.localhost\Ubuntu\home\tvdue\G1_Hackason\_local\rl\bundle\g1_rl_<日時>.tar.gz jigen@sodai-ubuntu:~/
ssh jigen@sodai-ubuntu
```

3. メンバーの PC で、展開して venv を作る（1 回だけ。2 回目からは展開だけ）:

```bash
mkdir -p ~/g1_rl
tar -C ~/g1_rl -xzf ~/g1_rl_<日時>.tar.gz
df -h ~
python3 -m venv ~/rl_env
~/rl_env/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu
~/rl_env/bin/pip install mujoco==3.14.0 gymnasium stable-baselines3 pyyaml
~/rl_env/bin/python ~/g1_rl/Button_Press/J1-gen/rl/check_env.py --episodes 3
```

mujoco はノート PC と同じ 3.14.0 にそろえる（版が違うと、同じ方策でも少し違う動きになりうる）。`check_env.py` で 1 プロセスの速さが出る
（IK の確認は pin が無いので飛ばす）。

4. 学習する。ssh を切っても止まらないよう `nohup` で動かし、ログはファイルに書く:

```bash
cd ~/g1_rl
nohup ~/rl_env/bin/python Button_Press/J1-gen/rl/train.py --steps 5000000 --envs 14 --out runs/v1 > runs_v1.log 2>&1 &
tail -f runs_v1.log
```

`tail -f` は Ctrl+C で抜けても学習は止まらない。止めるときは `pkill -f rl/train.py`（止めるときも、その時点の方策を保存する）。

5. 結果をノート PC に持ち帰る（PowerShell。`runs/v1` を丸ごと）:

```powershell
scp -r jigen@sodai-ubuntu:~/g1_rl/runs/v1 \\wsl.localhost\Ubuntu\home\tvdue\G1_Hackason\_local\rl\runs\
```

## 学習と評価

```bash
_local/rl/venv/bin/python Button_Press/J1-gen/rl/train.py --steps 2000000 --envs 10
_local/rl/venv/bin/python Button_Press/J1-gen/rl/evaluate.py _local/rl/runs/<記録>/model_latest.zip --episodes 50
_local/rl/venv/bin/python Button_Press/J1-gen/rl/evaluate.py _local/rl/runs/<記録>/model_latest.zip --episodes 3 --view
```

- 学習の経過は `progress.csv`（と画面）の `outcome/success`・`outcome/collision`・`outcome/timeout`（直前の更新の間に終わった
  エピソードの割合）と `rollout/ep_rew_mean`（1 エピソードの報酬の合計の平均）を見る
- 設定は `--set` で上書きできる（例: `--set reward.progress=20 success.lateral_tol_m=0.01`）。使った設定は記録の `config.yaml` に残り、
  `evaluate.py` はそれを読む
- 評価の配置は seed 10000 から順に引く（学習の配置とは別で、毎回同じ。方策どうしを比べられる）
- ノート PC の速さ: 10 並列で約 1,900 周期/秒（100 万周期に約 9 分）。環境だけなら 6,000 周期/秒出るが、方策のネットワークで
  行動を出す計算が 1 回 1〜3 ms かかる（aarch64 の torch。スレッドを 2 以上にすると 3 倍遅くなったので、既定は 1）

## 確かめ方

```bash
_local/rl/venv/bin/python Button_Press/J1-gen/rl/check_env.py
```

2026-11-01 の結果（ノート PC）:
- ランダムな行動: 1 プロセスで約 1,500 周期/秒（50 Hz なので、実時間の約 30 倍）
- IK で作った動き（方策の代わり）: 10 配置中 成功 6、体に当たった 4。当たった 4 つはボトルが体の正面寄り（右 5 cm より内側）で、
  経由の姿勢から手前の姿勢へ関節空間でまっすぐ動く途中に腕が体に入った。この台本は経路の当たりを調べない（本物の計画器は調べて断る）
  ので、判定が正しく働いた結果。成功のときの押す力の最大は 13〜17 N

テスト: `tests/test_rl_env.py`（gymnasium が無い環境ではスキップ）。

## 環境の中身

- 1 周期 0.02 秒（50 Hz）。行動（7、−1〜1）× 0.01 rad を右腕の目標角に足す（実機の安全上限 0.5 rad/s と同じ）。
  PD のゲインと重力補償は `common/arm/backend_sim.py` と同じ作り
- 観測（39）: 右腕の関節角・角速度・目標角との差、指先の位置、押す点までのベクトル、指の向き、机の縁の x と天板の z、前の行動。
  すべて実機でも得られる値（指先は関節角からの FK、押す点と机は検出と `make_table_boxes.py` から）
- 開始の姿勢は 10/21 の実機の立った姿勢（両腕を下ろす）。下ろした手はモデルでは股関節の当たり判定に 8〜10 mm 入っている
  （ハンドの判定が実物より大きい箱のため）。開始より 3 mm 以上深くならないかぎり「当たった」にしない（今の計画器と同じ考え方）
- 机は天板の下に 40 cm の箱を足してある（手が天板の下をくぐる経路を塞ぐ。実機の計画の障害物と同じ考え方）
- 終わり方: success（指先が押す点から横 2 cm 以内で、押す力 2〜15 N が 0.3 秒続いた）、collision（腕・ハンドが机や体に当たった）、
  timeout（10 秒）
