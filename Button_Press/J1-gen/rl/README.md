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
