# G1 Starter Kit

Unitree G1 を **Meta Quest で遠隔操作**し、**その動きを記録して再生**し、**頭部 LiDAR の点群を確認**するための一式です。

実機で一通り動作確認したものを、他の環境でも同じ手順で立ち上げられる形にまとめてあります。ヒューマノイドの開発を始めるときの足場として使ってください。

```
Quest のコントローラ  ──▶  G1 の腕が追従        （テレオペ）
            動きを記録  ──▶  あとから再生         （教示と再生）
        頭部 LiDAR      ──▶  点群と IMU を可視化   （ロボットが見ている世界）
```

---

## ⚠️ 最初に読んでください（安全）

これは**実機の腕を動かすソフトウェア**です。腕は人を傷つけられる速度と力を持っています。

- **起動直後、両腕が自動でゼロ姿勢へ動きます。** 可動範囲に人・物が無いことを毎回確認してください
- **ロボットを安定して固定してください。** 座位または吊り下げを推奨します
- **緊急停止（電源を切れる位置）にすぐ手が届く状態で操作してください**
- テレオペの終了は必ず **`q`**。`Ctrl+C` は腕が戻らないまま止まります
- 腕の再生は既定で **dry-run**（送信しない）です。`--execute` を付けたときだけ実機が動きます
- `setup/apply_patches.py` は**起動時の腕の速度を 2 rad/s に落とす安全パッチ**を含みます（上流の既定は 30 rad/s）。必ず適用してください（`setup/install_env.sh` が自動で当てます）

初めて動かすときは、**必ず 2 人以上**で、片方が電源に手をかけた状態で試してください。

---

## 動作確認済みの構成

| | 確認済みの値 |
|---|---|
| ロボット | Unitree G1（**29DoF** で確認。23DoF も対応・自動判定） |
| 頭部 LiDAR | Livox **Mid-360S**（Mid-360 も対応・自動判定） |
| XR デバイス | Meta Quest 3（コントローラ入力） |
| ホスト PC | Ubuntu **22.04** / x86-64 / RAM 16GB |
| ROS | ROS 2 **Humble** |
| Python | **3.10**（conda 環境 `tv`） |

> **Ubuntu 24.04 では動きません。** xr_teleoperate が 20.04 / 22.04 でのみ検証されているためです。
> `pinocchio 3.1.0` / `numpy 1.26.4` もバージョン固定です。上げると逆運動学が動かなくなります。

必要なもの: G1 本体、LAN ケーブル、Quest 3 + コントローラ、Ubuntu 22.04 のノート PC、PC と Quest を同じ LAN に置ける WiFi。

---

## クイックスタート

### 1. 導入（初回だけ・30〜60 分）

```bash
git clone https://github.com/yokoshaan/g1-starter-kit.git
cd g1-starter-kit

bash setup/install_apt.sh      # ← 要 sudo。ネイティブ端末で実行（下記の注意参照）
bash setup/install_env.sh      # conda 環境・テレオペ本体・DDS（root 不要）
bash setup/install_livox.sh    # LiDAR も使う場合（root 不要）
```

> **`install_apt.sh` の注意**: VSCode の統合ターミナルでは `sudo` が通らない環境があります（`no_new_privs`）。
> デスクトップのネイティブ端末（Ctrl+Alt+T）で実行してください。
> また、長いコマンドの手貼りは端末設定によって `^[[200~` が混ざって失敗します。**1 行だけ手で打つ**形にしてあるのはそのためです。

### 2. 設定

```bash
cp config/g1.env.example config/g1.env
$EDITOR config/g1.env          # 有線インターフェース名などを書く
```

分からない値は空のままで構いません。次の手順が実機から読み取って教えてくれます。
ネットワークの組み方は **[docs/02-network.md](docs/02-network.md)**。

### 3. 疎通確認 → テレオペ

```bash
./scripts/preflight.sh         # 有線疎通・DDS・機体の DoF 判定（ロボットは動かない）
./scripts/quest_check.sh       # Quest から PC に届くか（ロボット不要）

./scripts/teleop.sh            # テレオペ開始
./scripts/teleop.sh --record   # 動きを記録しながら
```

Quest でブラウザを開き、表示された URL にアクセス → 「Virtual Reality」ボタン → ターミナルで `r` を押すと追従が始まります。詳しくは **[docs/03-teleop.md](docs/03-teleop.md)**。

### 4. 記録の再生 / LiDAR

```bash
./scripts/replay.sh --list                  # 記録の一覧
./scripts/replay.sh                         # 最新の記録を dry-run で確認
./scripts/replay.sh <episode> --execute     # ⚠️ 実機で再生

python3 tools/lidar_probe.py --write-config  # LiDAR の機種を自動検出して設定生成
./scripts/lidar_view.sh                      # 点群と IMU を表示
./scripts/record_bag.sh room1                # 点群を bag に記録
```

---

## ドキュメント

| | |
|---|---|
| [01-setup.md](docs/01-setup.md) | 導入の詳細、バージョン固定の理由、つまずいたときの確認 |
| [02-network.md](docs/02-network.md) | 有線（ロボット）と WiFi（Quest）の 2 系統をどう組むか |
| [03-teleop.md](docs/03-teleop.md) | テレオペの操作、キー、Quest 側の手順 |
| [04-record-replay.md](docs/04-record-replay.md) | 記録の形式と再生。指令トピックの選び方 |
| [05-lidar.md](docs/05-lidar.md) | 点群の表示、機種判定、bag の記録と再生 |
| [06-troubleshooting.md](docs/06-troubleshooting.md) | 症状から原因を引く表 |
| [07-next-steps.md](docs/07-next-steps.md) | ここから先の開発の道筋 |

---

## 構成

```
config/
  g1.env.example        環境ごとの設定の雛形（IP・インターフェース名・機体）
  livox/*.json.in       LiDAR 設定の雛形（機種ごとに形式が違う）
  rviz/                 点群表示のプリセット
launch/
  livox.launch.py       LiDAR ドライバ + 表示用 TF + RViz
scripts/
  lib.sh                共通処理（設定読み込み、ROS/conda の切り替え）
  preflight.sh          疎通と機体構成の確認
  quest_check.sh        Quest からの到達性の確認
  quest_ap.sh           PC 自身を Quest 用の WiFi AP にする（施設 WiFi やテザリングが駄目なとき）
  teleop.sh             テレオペ起動
  replay.sh             記録の再生
  lidar_view.sh         点群と IMU の表示
  record_bag.sh         点群の bag 記録
setup/
  install_apt.sh        apt で入れるもの（要 sudo）
  install_env.sh        conda 環境・テレオペ本体・DDS
  install_livox.sh      LiDAR ドライバ
  apply_patches.py      テレオペ本体に必要な修正（冪等・取り消し可）
tools/
  preflight.py          有線・DDS・DoF 判定
  quest_check.py        8012 で待ち受けて Quest から到達確認
  lidar_probe.py        LiDAR の機種・IP・シリアルを自動検出
  mode_check.py         腕を動かせる状態か（指令経路・モータ有効・委譲）の確認と ai の解除
  cmd_monitor.py        指令と実測を同時に眺めて「送っているのに動かない」を切り分ける
  armsdk_probe.py       rt/arm_sdk の委譲に応じる機体かを 3° だけ動かして確かめる
  replay_arm.py         記録した腕の動きの再生（dry-run 既定）
  fake_lidar.py         実機なしで点群表示を試すためのダミー配信
```

---

## このキットが解決していること

実機で立ち上げるときに実際に詰まった点を、あらかじめ潰してあります。

- **機体の DoF（23 / 29）を自動判定**します。手首の軸数で変わり、間違えると逆運動学が合いません
- **LiDAR の機種（Mid-360 / Mid-360S）を自動判定**します。**設定ファイルの形式が違い**、間違えるとドライバは「初期化成功」と表示したまま何も出さず、エラーも出しません
- **指令トピックを自動選択**します。腕を動かす経路は `rt/arm_sdk` と `rt/lowcmd` の 2 つあり、**ロボットの状態でどちらが効くかが変わります**。食い違うとどちらも「エラーを出さずに腕が動かない」形で失敗します
  - コントローラ稼働中に `rt/lowcmd` を出す → コントローラが 1000Hz で上書きし続けて無視される
  - コントローラ停止中に `rt/arm_sdk` を出す → 制御権を渡す相手が居ないので何も起きない
  - 再生は `--topic auto` で判定し、テレオペは起動前に整合を確認して食い違いを弾きます
  - 検証した機体では `rt/arm_sdk` が Regular モードでも委譲に応じませんでした。`tools/armsdk_probe.py` で 3° だけ動かして確かめられ、駄目ならデバッグ状態 + `rt/lowcmd` が経路になります
- **腕のモータが有効かを起動前に確認**します。ゼロトルク（`mode=0`）だと送信は成功するのに何も動かず、しかも**テレオペや再生を終了するたびにこの状態へ戻ります**。リモコンでダンピングに入れ直すだけですが、知らないと延々と悩みます
- **Quest 用の WiFi を PC 自身が提供**できます（`scripts/quest_ap.sh`）。施設 WiFi のクライアント分離、テザリングの混雑・6GHz 問題を回避し、Quest → PC を 1 ホップにします
- **起動時の腕の速度を落とすパッチ**を同梱しています。素の状態では起動した瞬間に両腕が高速で振られます
- **カメラが無い環境でも記録できる**ようにしてあります。素の状態では記録開始で落ちます
- **ROS 2 の DDS ドメインをロボットと分離**します（既定 42）。両方 0 だと互いにディスカバリのノイズを出し合います

---

## このキットに含まれないもの

- 歩行・全身の運動制御（付属リモコンで別途操作してください）
- ハンド（Inspire / Dex3）の制御
- 頭部カメラの映像（ロボット内蔵 PC 側の画像サービスが必要です）
- SLAM による地図作成、Nav2 による経路計画（[07-next-steps.md](docs/07-next-steps.md) に進み方を書いています）
- シミュレータ連携、模倣学習の学習側
- **無線でのロボット制御** — 読み取り専用の確認は無線でも通りますが、テレオペ・
  腕の再生・LiDAR は有線必須にしています。理由と、線を切りたい場合の進め方は
  [02-network.md](docs/02-network.md) と [07-next-steps.md](docs/07-next-steps.md) に書いています

---

## ライセンスと出典

このリポジトリのスクリプトとドキュメントは自由に使ってください。以下は各々のライセンスに従います。

- [xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate) — Unitree Robotics
- [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) — Unitree Robotics
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) / [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) — Livox
- [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds) — Eclipse

`setup/apply_patches.py` は xr_teleoperate を fork せず、手元のクローンに修正を当てる方式です。何を変えるかは `--list` で確認でき、`--revert` で元に戻せます。
