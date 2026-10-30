# Teleop — G1 を座らせて Meta Quest でエピソード記録・再生（デモ実演4）

Quest のコントローラで G1 の腕を動かし、その動きを記録して再生する。
**実体は `vendor/g1-starter-kit/`（外部リポジトリの取り込み）**で、このフォルダ直下の
スクリプトはデモの都合（保存先・タスク名・`Demo/` からの呼ばれ方）だけを引き受ける薄い層。

⚠️ **実機の腕を動かす。`SAFETY.md` を読んでから触ること。**（`SAFETY.md` は Phase 2 で作成）

## 現在の状態（2026-09-10）

Phase 0 が済んだところ。**キットを取り込んだだけで、まだ薄い層は書いていない。**
いまキットを直接使うなら `vendor/g1-starter-kit/README.md` の手順に従うこと。

| ファイル | 状態 |
|---|---|
| `vendor/g1-starter-kit/` | ✅ 取り込み済み（38ファイル / `45ead40`） |
| `config/g1.env.omen` | ✅ OMEN に配置済み。**git では追跡しない**（`config/README.md`） |
| `scripts/net.sh` | ✅ **2026-09-14 実機で往復検証済み**（面 A/B/C の切り替え） |
| `SAFETY.md` `setup.sh` `record.sh` `replay.sh` `list.sh` | ❌ 未着手 |
| `quest_ap.sh` | ⛔ **書かない。** `scripts/net.sh` が役割を引き取った（理由は下記） |

### 取り込み先から動くことは OMEN で実測済み（2026-09-10）

`Teleop/` をまるごと別の場所（`~/relocate_test/Teleop/`）に置いて
`./scripts/preflight.sh --skip-scan` を走らせ、**完走**することを確認した。

| 見たところ | 結果 |
|---|---|
| `lib.sh` の `REPO_DIR` 自己解決 | ✅ 置いた場所を指した（`BASH_SOURCE` から解決するため） |
| 新しい場所の `config/g1.env` の読み込み | ✅ `enp129s0` / `.200` / `G1_29` / domain 42 |
| conda 環境 `tv` の activate | ✅ Python 3.10.21 |
| `tools/preflight.py` の起動と有線リンク判定 | ✅ `up` / `192.168.123.200/24` |
| DDS で `rt/lowstate` を受信 | ❌ 0 件 — **G1 が電源オフのため。想定どおり** |

**キットは場所に依存しない。** 依存先（`~/xr_teleoperate` `~/miniforge3` `~/cyclonedds`）は
`$HOME` 固定でキットの外にあるので、キットを動かしても一緒に動かす必要はない。

⚠️ **ただし LiDAR 系（実演2 の経路(B)）は OMEN に入っていない。**
`~/ws_livox/install/setup.bash` と `~/.local/lib/liblivox_lidar_sdk_shared.so` が
両方とも不在（2026-09-10 実測）。つまり `scripts/lidar_view.sh` は今のままでは動かず、
`setup/install_livox.sh` を先に流す必要がある。実演2 の第一候補は経路(A)（G1 の DDS を
購読するだけ）なので、そちらが通ればこれは要らない。

## `scripts/net.sh` — ネットワークの切り替え

**Mac から実行する。** OMEN と G1 の両方を SSH で操作して、3 つの面を切り替える。
構成図と実測値は `docs/plan/g1-stack-architecture.html` の「図 1b」。

```bash
./scripts/net.sh status      # 3 面の現状（変更しない）
./scripts/net.sh teleop      # AP を上げ、G1 を AP に寄せる ← テレオペ前
./scripts/net.sh internet    # G1 を Fujitsu に戻し、AP を下ろす ← apt/pip のとき
./scripts/net.sh dds         # DDS 疎通（キットの preflight を呼ぶ）
```

`ap-up` `ap-down` `g1-ap` `g1-net` で片方だけ動かすこともできる。接続先やプロファイル名は
すべて環境変数で上書きできる（`G1_OMEN_SSH` `G1_AP_CON` など。冒頭を見ること）。

### キットの `scripts/quest_ap.sh` を使わない理由

あれは **OMEN 上で走り、`connection delete` → `device wifi hotspot` でプロファイルを作り直す。**
その結果パスフレーズが毎回再生成され、G1 側の保存済みプロファイル `g1-teleop-client` と
食い違う。`net.sh` は既存プロファイルを `nmcli connection up` するだけなので食い違わない。

### 2026-09-14 に実機で確かめたこと

- **AP は 2.4GHz にしか置けない。** 5GHz は ch36〜165 まで全て `no IR` で、
  regdb の `IR-CONCURRENT` を解釈する `CONFIG_CFG80211_REG_RELAX_NO_IR` が
  Ubuntu 標準カーネルに入っていない。リンクレートは実測 26.0 MBit/s（MCS 3）で、
  **XR 映像のビットレート天井になりうる**
- **OMEN の無線は 1 枚。** AP 稼働中は OMEN 自身がインターネットに出られない。
  下ろせば `Fujitsu_free_Wi-Fi` に自動復帰する
- **`internet` は順序が大事。** G1 を Fujitsu に戻してから AP を下ろす。
  逆にすると G1 の無線が行き先を失う
- **`nmcli` は polkit が無いと "Not authorized" で黙って失敗する。** OMEN は JS ルール、
  G1 は polkit 0.105 なので `.pkla` 形式。`net.sh` は足りない権限を検出して
  貼るべきコマンドを表示する
- **有線が抜けても無線経路で G1 に入れる。** `net.sh` は有線で届かないとき
  ProxyJump（OMEN 経由）へ自動フォールバックする。実際この日、機体を立たせた動作で
  有線が抜け、これで救われた。ただし**その状態では DDS は動かない**（OMEN が `.161` に届かない）

## なぜ vendor を丸ごとコミットしているか

**このリポジトリの他の場所とはやり方が違う。**
`Mapping/real/vendor/` は外部リポジトリを `.gitignore` して `.repos` で版だけ固定しているが、
ここは中身ごとコミットしている。

理由は、デモ機（Ubuntu 24.04 / ROS 2 Jazzy）向けに**キット本体へ手を入れる見込みがある**ため
（上流の動作確認環境は 22.04 / Humble）。submodule や `.repos` だと手元の修正を持ち回せない。

出所・commit・許諾・上流の取り直し手順は `vendor/g1-starter-kit/UPSTREAM.md` にある。

## 設計上の約束

- キットが既に潰した落とし穴（DoF 自動判定・指令トピック自動選択・腕速度の安全パッチ）を
  **書き直さない**。薄い層から呼ぶだけにする
- `setup/install_apt.sh` は**呼ばない**（`:35` が `jammy` 以外を `die` する。デモ機は 24.04）
- `setup/apply_patches.py` は**省略できない**。これはキットではなく `~/xr_teleoperate` 側を
  書き換えるもの（腕速度 30→2 rad/s の安全パッチ、カメラ無しでも記録できるパッチ）
- `config/g1.env.omen` は**追跡していない**ので clone し直すと消える。
  `setup.sh` が不在を検知して復旧手順を印字すること（`config/README.md`）
- 実演4 は **ROS 2 に依存しない**（キットで ROS を使うのは `lidar_view.sh` と
  `record_bag.sh` だけ。テレオペと再生は conda 環境 `tv` の素の Python）

## `scripts/pc2_teleop.sh` — 遠隔操作を PC2 で動かす（案 A、2026-10-26）

有線が扉まで届かないので、xr_teleoperate を**機体の中のパソコン（PC2）**で動かす。
無線は Quest ↔ PC2 と Mac ↔ PC2（ssh）だけで、機体の制御は PC2 の中の有線（eth0）のまま。
構成図・何がどこで動くか・切れたときの振る舞いは `docs/plan/2026-10-26-g1-door-teleop-demos.md` の 9 章。

```bash
./scripts/pc2_teleop.sh setup                  # 初回だけ: 配ってから PC2 に環境を作る（切り離して流す）
./scripts/pc2_teleop.sh setup-log              #   その進み具合
./scripts/pc2_teleop.sh deploy                 # 道具を直したら配り直す（PC2 の ~/teleop_tools/）
./scripts/pc2_teleop.sh check                  # 機体のモードを見る（動かさない）
./scripts/pc2_teleop.sh sensors t0_1027_s 1800  # 先にセンサー（ライダー・IMU・/dog_odom・赤外線）を録り始める
./scripts/pc2_teleop.sh start door_room4_1027  # 切り離して起こす（--motion・記録あり・素通し）。⚠️ 直後に両腕が下ろした形へ動く
#   10-30 から: TELEOP_FWD_MAX=0.5 ./scripts/pc2_teleop.sh start … で、前への歩きの上限だけを上げる（0〜1.0。既定 0.3。修正 loco-fwd-max）
#   → Quest で https://192.168.0.82:8012/?ws=wss://192.168.0.82:8012 →「Pass-through」
#     （「Virtual Reality」は全面グレーで周りが見えない。キットの 03-teleop.md の記述は古い）
#   → 操作者はロボットの後ろでロボットと同じ向き・Meta ボタン長押しで「ビューをリセット」・腕を体の横に下ろす
./scripts/pc2_teleop.sh go                     # 追従の開始（r）
./scripts/pc2_teleop.sh rec                    # 記録の開始（s）… もう一度で保存
./scripts/pc2_teleop.sh check-ep door_room4_1027 # 録った 1 本を点検（歩きの値・時刻・センサー・扉の角度）。✗ なら録り直し
#   10-30 から: 向きごとの「指令どおりに進んだ割合」（/dog_odom）と、その課題で扉が 30° を超えた本数（関門 5/10）も出す
./scripts/pc2_teleop.sh push-force door_room4_1027 # 手が扉を押した力と、前へ歩けの区間ごとの押し方（check-ep の後に。pinocchio）
./scripts/pc2_teleop.sh quit                   # 終える（q）。腕を 2 rad/s で下ろしてから、制御を 2 秒かけて純正へ返す
./scripts/pc2_teleop.sh sensors-stop t0_1027_s # センサーの記録を止める
```

2026-10-27 の T0 で分かったこと:
- 機体は R1+Y（純正の状態 501）で `rt/arm_sdk` を借りられた（`probe` で 0.05 rad の指令に 0.033 rad）
- Quest の電池が切れかけると音が鳴って没入の画面が止まる → データが途切れ、`xr-stale-stop` が 0.3 秒で一時停止した（設計どおり）。始める前に Quest を充電しておく
- 終了で腕が 30 rad/s で振り下ろされる・戻りきれないと純正がいきなり腕を取り返す、の 2 つは `safe-exit`・`go-home-release` で直した
- 強制終了（プロセスを kill）すると、純正がそのときの形で腕を保持する（力は抜けない）
- ssh 越しの `pgrep -f 名前` は、命令の中に同じ文字列があると運び役のシェル自身に当たる。`^[^ ]*python[^ ]* (-u )?teleop_hand_and_arm.py` のように行の先頭から絞る

- PC2 に tmux も screen も無いので、遠隔操作は `setsid nohup` で切り離し、操作は IPC（`--ipc`）で送る。
  **ssh が切れても遠隔操作は止まらない**（止めるのは Quest の両スティック 0.5 秒か `quit`）
- 起動役（`pc2/teleop_start.sh`）は、修正（`record-loco-cmd`・`stick-pause`・`xr-stale-stop`・
  `record-timestamp`・`state-watchdog`）が当たっていなければ起こさない
- Mac が `Physical_AI_5G` に居ないときは `G1_PC2_JUMP=ubuntu@100.99.102.70`（OMEN の Tailscale）を付けると中継できる
- 環境づくり（`pc2/setup_pc2_tv.sh`）は既存の物に書き込まない: 既存の `~/miniforge3` に環境 `tv` を足すだけ・
  `~/.bashrc` は触らない・`~/unitree_sdk2_python` は `.pth` でつなぐだけ。DDS の C の部品は機械全体の
  `/usr/local`（0.10.2。Python 3.8 側もこれで動いている）を使い、`~/cyclonedds`（0.10.5）は使わない。
  部品の版は OMEN の動いている環境から作った `pc2/tv_constraints.txt` に固定。手の部品（dex-retargeting）は入れない
  （torch と pip の pin を引き込み、conda の pinocchio とぶつかる。手なしでは読み込まれない）
- 記録（data.json）の左右の腕には、角度に加えて速さ（qvel）と推定トルク（torque＝tau_est）が入る（修正 `record-arm-effort`、10-27）。`check-ep` が左右別に動きの合計・速さとトルクの最大（関節つき）・上限への張り付きを出す
- `pc2/ik_bench.py`: 腕の逆運動学の時間を機体なしで測る（OMEN では中央 1.1 ms・95% 1.3 ms、2026-10-27）
