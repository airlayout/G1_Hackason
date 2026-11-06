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
| `SAFETY.md` `setup.sh` `record.sh` `replay.sh` `list.sh` | ❌ 未着手。記録・再生・一覧は 2026-11-03 から `scripts/pc2_teleop.sh` が引き受けた（下の「OMEN から PC2 の遠隔操作を動かし…」の節） |
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

## OMEN から PC2 の遠隔操作を動かし、記録して再生する（2026-11-03）

遠隔操作は実績のある案 A のまま **PC2（機体の中）で動かす**。無線は Quest ↔ PC2 と OMEN ↔ PC2（ssh）だけで、
機体の制御は PC2 の中の有線（eth0）。操作する機械を Mac から OMEN に替えられるようにした（Mac からも今までどおり使える）。
記録は PC2 の `~/teleop_data/<課題名>/episode_XXXX/`、再生も PC2 の上で走る。

```bash
# Mac で 1 回だけ（道具を直したらその都度）: この入口と配るもの一式を OMEN の ~/g1_teleop/ へ送る
./scripts/pc2_teleop.sh install-omen

# ここから OMEN で（OMEN の ~/.ssh/config の g1-wifi＝192.168.0.82 を自動で使う。OMEN も Physical_AI_5G）
cd ~/g1_teleop/Teleop
./scripts/pc2_teleop.sh patch                        # 道具を直した後: PC2 へ配り、修正を当て直し、自己テスト 11 本（PC2 の上で）
./scripts/pc2_teleop.sh check                        # 機体の状態。最後の 3 行（✓/✗）が立った形・座位の遠隔操作・座位の再生を始めてよいか
./scripts/pc2_teleop.sh start arms_1103 --no-legs    # 足なしで起こす（足ありは --no-legs を外す・座位は --seated）。⚠️ 直後に両腕が下ろした形へ動く
#   → Quest で https://192.168.0.82:8012/?ws=wss://192.168.0.82:8012 →「Pass-through」
./scripts/pc2_teleop.sh go / rec / quit              # r・s・q
./scripts/pc2_teleop.sh list arms_1103               # 録った記録（長さ・足あり／なし／座位・歩いた秒）
./scripts/pc2_teleop.sh replay arms_1103             # 最新の 1 本を確かめるだけ（何も送らない。軌道の図を PC2 の ~/teleop_logs/ に）
./scripts/pc2_teleop.sh replay arms_1103 2 --execute # 2 本目を実機で（y/N・Enter か Ctrl+C で中断）
```

**形は 3 つある。** 足あり・足なしは `--motion`（純正の通常運転 R1+X・FSM 500/501 で立たせたまま、腕だけを借りる）。
座位は `--motion` なし（座らせてダンピングのまま、純正を止めて腕だけ。下の節）。

| | 足あり（既定） | 足なし（`--no-legs`） | 座位（`--seated`。11-04） |
|---|---|---|---|
| 機体 | 立たせて通常運転（FSM 500/501） | 同じ | 座らせてダンピング（L2+B・FSM 1） |
| 腕の指令 | `rt/arm_sdk`（純正から借りる） | 同じ | `rt/lowcmd`（純正を止め、脚と腰は起動時の角度で固める） |
| スティック | 左で前後・横、右で旋回（前の上限は `TELEOP_FWD_MAX`） | 倒しても歩かない（修正 `no-loco`） | 倒しても歩かない（修正 `seated-controls`） |
| 記録の歩き（actions.body.qpos） | 送った値 | 0 | 0 |
| 記録の課題の説明（text.desc） | `legs=walk` | `legs=none` | `legs=seated` |
| 再生 | 腕＋歩き（`--legs`）。`--no-legs` で腕だけ | 腕だけ | `--seated` で腕だけ |
| 終わった後 | 純正が腕を取り返す | 同じ | 保持係が最後の姿勢を保つ |

一時停止（両スティック 0.5 秒）・A ボタンの終了・Quest のデータが 0.3 秒途切れたときの一時停止は、どの形でも効く
（座位で効くのは修正 `seated-controls` から。上流は `--motion` のときだけだった）。
`start`・`replay --execute` は、機体の状態がその形に合わなければ理由を出して断る（`form_check.py`。11-04 朝は FSM 0 のまま
立った形で起こし、`go` しても腕が動かなかった。キットの `check` の「→ 腕を動かせる状態です」はモードを見ない）。

- 再生は記録の 1 コマごとの時刻（time_ns）どおりに流す。遠隔操作の周期は 30 Hz より遅く（10-29 は中央 34.5 ms）、
  名目の 30 Hz で流すと速く・歩きは 5% ほど短くなるため。確かめで時刻どおりの腕の速さも見る（30 rad/s 超は断る。10-29 の 1 本は最大 22.9 rad/s で警告）
- 歩きの再生は開ループ。記録を始めた時と同じ位置・向きに立たせ、進む先を空ける。確かめ（`--execute` なし）が
  「指令どおりなら前へ何 m・左へ何 m・向き何度」を出すので、それを見てから流す（10-29 の 1 本は前へ 4.25 m・左へ 0.84 m・−12°。
  扉の前では前への指令に 12% しか進まなかった）
- 再生（`--execute`）は ssh の前で動く。⚠️ 無線が落ちて ssh が切れても PC2 の側はすぐには気づかず、再生は最後まで進む
  （Enter の中断も届かない）。操作する側で ssh・端末を閉じたときだけ中断する（歩きを 0 にして腕を保ったまま制御を返す）。
  リモコンを持った人を横に置く。遠隔操作が動いている間は再生を断る（腕を取り合うため）
- 再生は腰を遠隔操作と同じ強さ（kp 300・kd 3）で保つ（`rt/arm_sdk` の重みは腰も純正から取り上げる。11-03 実機）。`--legs` は腰を保つ形でだけ動く
- 接続先は `G1_PC2_SSH` で替えられる。既定は ~/.ssh/config で 192.168.0.82 を指す名前（Mac は `g1wifi`・OMEN は `g1-wifi`）
- `install-omen` の送り先は OMEN の `~/g1_teleop/`（他班の写し `~/G1_Hackason`・`~/G1_Hackason_fresh` には触らない）
- 機体なしで確かめたこと（11-03）: OMEN の上から PC2 へ配って修正を当て（PC2 と OMEN の修正済みの 2 ファイルが一致）、
  PC2 の上で自己テスト 8 本合格（足なし 11 項目・再生の流れを偽の DDS で丸ごと）、一覧、10-29 の 1 本の再生の確かめ、
  足なしの起動の試し（`TELEOP_DRY=1`。起こさずに `--no-loco` 付きの命令を出す）。**機体を動かす操作は 1 度もしていない**

### 座位（`--seated`）と保持係（2026-11-04）

通常運転で立たせるのは時間がかかるので、**座らせたままダンピングで腕だけを遠隔操作する**形。キット本来の
「デバッグ状態＋`rt/lowcmd`」（`--motion` なし）で、遠隔操作が起動のときに純正を止め、腕以外（脚・腰）を起動時の角度で固める。

```bash
# 機体を座らせ、リモコンの L2+B でダンピング（FSM 1）に。ゼロトルク（FSM 0）ではモータが無効で何も動かない
./scripts/pc2_teleop.sh check                         # 「✓ 座位の遠隔操作」を見る
./scripts/pc2_teleop.sh start arms_1104 --seated      # 保持係を起こしてから遠隔操作を起こす
./scripts/pc2_teleop.sh go / rec / quit               # 今までと同じ
./scripts/pc2_teleop.sh hold                          # 保持係の状態（quit の後は「姿勢を保っている」になる）
./scripts/pc2_teleop.sh replay arms_1104 2 --seated --execute   # 座位の再生（腕だけ・rt/lowcmd。座位の遠隔操作を 1 回 quit した後に）
./scripts/pc2_teleop.sh start arms_1104 --seated      # 続けてもう 1 回（ダンピングの入れ直しは要らない）
./scripts/pc2_teleop.sh release                       # 座位を終える: 力を 2 秒で抜いて止まる（その後はゼロトルク。座ったまま）
./scripts/pc2_teleop.sh restore                       # 立たせる日の前に純正を戻す（確認あり。電源の入れ直しでもよい）
```

**保持係（`scripts/pc2/seat_hold.py`）**: 座位では、遠隔操作・再生が終わって指令が途切れるとモータがゼロトルクに落ち、
腕が垂れ、次の回の前にダンピングに入れ直す必要があった（キットの説明書）。保持係は PC2 に常駐して `rt/lowcmd` を見張り、
ほかの送り手が 30 ms 黙ったら、その最後の指令（quit の後は腕を下ろして座った姿勢）を 250 Hz で送り続ける。
ほかの送り手が送り始めたらすぐに黙る。遠隔操作・再生の作りは変えていない（保持係の指令には `reserve[3]` に印を付けて見分ける）。

- 引き継がない: 純正が動いている（裏の糸が 0.5 秒ごとに歩きの RPC の GetFsmId を聞き、2 回続けて答えが無いときだけ
  「止まっている」とする。起こした直後は「動いている」扱い）・送る速さが 1 秒に 20〜400 件の外か、見た件数が 10 件未満
  （遠隔操作は 250・再生は 50・純正の制御は 1000 前後）・kp が全部 0・モータが無効の関節がある・指令と実測が 0.3 rad より離れている
- すぐに送るのをやめる（力を抜かない）: `rt/lowstate` が 0.1 秒途切れた・モータが無効になった（後でリモコンで有効にされた
  瞬間に古い姿勢へ跳ねないよう、持っていた指令も捨てる）・純正が答えた（restore の後など。取り合わない）
- 保持係は 1 つだけ（排他の鍵。2 つ目は起きずに終わる）
- 2 秒で力を抜いて止める: `release`・モータの温度が 70 °C 以上・60 分保った
- 保持係が落ちたり PC2 が止まったりしたときは、今までと同じくゼロトルク（座っているので倒れない）
- 座位が動いている間、立った形の `start`・`replay` は断る（`release` してから。純正も止まっているので `restore` か電源の入れ直し）
- 座位の `replay --execute` は純正を止めない。座位の遠隔操作を 1 回起こして quit した後（純正が止まり、保持係が保っている）に流す。
  再生が純正を止めると、止めてから最初の指令までの数秒、誰も姿勢を保たないため（その日の最初が再生でも、先に遠隔操作を 1 回）
- 座位を始めてよいかは、純正がダンピング（FSM 1）か、「止まっている」証拠（切り替えの RPC が空の名前を答えた・保持係が保っている）
  があるときだけ。2 つの RPC がどちらも答えないだけでは通さない（PC2 が混んで答えが遅れただけかもしれず、立っている機体の
  純正を止めて崩すため。11-04 のコードの見直しの指摘）
- **`start` のたびに腕を下ろした初期位置（全関節 0＝上腕は下・前腕は前へ水平）へ戻す**（11-04 夕。修正 `arm-home-hold`・`arm-home-wait`）。
  座位で start をくり返すと、腕の止まる場所がずれていった。上流は起動と終わり（2 rad/s）に「今の実測＋1 周期ぶん」を送るので、
  出せる力が 0.64 N·m までしかなく、前腕を水平に保つ 1.9 N·m（肘。URDF の重さ）に負けて 0 へ届かず、垂れた所で止まった。
  保持係はそこを保ち、次の start がまたそこから始まっていた。今は 2 rad/s 以下では前に送った目標から寄せて 0 まで届かせ
  （ぶつかったときは目標と実測の差を 0.1 rad＝8 N·m までにして押し続けない）、「Press [r]」の前に着いたかを待って
  `[start] 腕を下ろした初期位置に戻した（… 秒・いちばん大きいずれ … rad）` と出す。5 秒で着かなければ警告を出して先へ進む。
  quit の後の「腕を戻す」も 0 まで届くので、保持係が保つのも 0 の姿勢になる

⚠️ 機体を動かしての試験はまだ（11-04 16:38 に PC2 へ当て、PC2 の上で自己テスト 10 本合格・`check` は誤りなし。
腕を初期位置へ戻す直し（`arm-home-*`）は PC2 にまだ当てていない）。最初の日に確かめること:

1. ダンピングから純正を止めた後もモータが有効のままか（キットの手順どおりなら残る。`hold` と保持係のログの「モータの有効」）
2. 遠隔操作が終わってから保持係が引き継ぐまで（約 30 ms）にモータが無効に落ちないか。落ちる場合はログに
   「モータの有効 29→0（…ほかの送り手が黙ってから 0.0xx 秒）」が残る（その秒数がゼロトルクへの落ち方の実測になる）
3. `release` の後にゼロトルクで腕が垂れる量（座位なので腕は下ろした姿勢から少し垂れるだけのはず）
4. `restore` で純正を戻したときの状態（FSM がいくつで始まるか。脚が動きうるので支えておく）
5. `start` のたびに `[start] 腕を下ろした初期位置に戻した` が出るか。座った姿勢で前腕を水平に出すと手が太ももに近いので、
   当たって警告（戻りきらない）が出るなら、座位の初期位置を変える

機体なしで確かめたこと（11-04）: 修正 `seated-controls` は素の上流 817fb00 に当たり、2 回目は何も変えず、全部外すと素に戻る。
OMEN の本物の SDK で自己テスト 9 本合格（`seated_controls` は 9 項目）。保持係と形の判定は試験 35 本（`tests/test_seat_hold.py`。
わざと壊した 14 通りを全部見つける）と、OMEN の本物の DDS（lo・domain 7）での自己テスト `seat_hold_selftest.py` に 3 回続けて合格
（純正が答えないと 1.5 秒で確かめる・2 つ目は起きない・遠隔操作が止まって 33 ms で引き継ぎ・250 件/秒・印と CRC が正しい・
送り始めるとすぐ黙る・状態が途切れて 98 ms でやめる・止める指示で kp 80→0 を 2.0 秒）。
⚠️ 最初の版は裏の糸で歩きの RPC の口を作り、cyclonedds の型の登録が糸を跨いで落ちることがあった（DDS の口は全部主の糸で作る）

**PC2 の重さ（11-04 に機体の上で実測）**: PC2（Orin NX）の Python の DDS は OMEN の約 5 倍重い（1 件を戻すのに LowState_ 0.81 ms・
LowCmd_ 0.56 ms）。機体の `rt/lowstate` は 647 件/秒、ダンピング中の純正の `rt/lowcmd` は 991 件/秒で、SDK の聞き役で 2 つを聞くだけで
1 コアを使い切った（SDK の聞き役は最新の 1 件しか残さず、純正の指令を約 4 割取りこぼして 609 件/秒に見えた）。最初の保持係は
自己テストが PC2 でだけ段 4・5 で落ちた（偽の状態が途切れて保つのをやめる・止める指示の後に 5 秒終わらない）。そこで保持係は:

- `rt/lowstate` を聞き役なしで、最新の 1 件だけ残す読み手から 5 周期に 1 回（50 Hz）取りに行く（SDK の `Read(0)` は
  cyclonedds 0.10.2 で永久に待つので使わず、cyclonedds の `DataReader.take` を直に使う）
- `rt/lowcmd` の読み手は `IgnoreLocal.Process` で自分の指令を受けない（印での見分けは二重の守りとして残す）
- 保っている間は同じ指令なので、詰め直しと CRC は指令が変わったときだけ
- 終わるときは後片付けの後に `os._exit` で抜ける（Python の後始末を待たない）

読むだけで同じ組み立てを機体で回すと、純正が動いている間で CPU 71%（ほとんどが純正の 991 件/秒。保持係はこの間送らない）。
純正を止めた後（遠隔操作の最中・保っている間）の見積もりは 2 割ほど。自己テストの偽の機体も 200 Hz に下げ、PC2 で 3 回続けて合格。
`check` の `[Reader] take sample error`（`form_check.py` が聞き役を閉じた直後に届いた 1 件）も、聞き役なしで 1 件取りに行く形にして消した。
`pc2_teleop.sh` の配る段は `scp -O`（既定の SFTP は無線で 27 本に 97 秒、`-O` は 11 秒）。
