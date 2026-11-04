# このディレクトリは外部リポジトリの取り込みです

`g1-starter-kit` は**このリポジトリで書いたものではありません**。
上流をそのままコピーして置いています。

| | |
|---|---|
| 取得元 | https://github.com/yokoshaan/g1-starter-kit |
| commit | `45ead40`（`docs: 実機で踏んだ落とし穴を反映（指令経路・モータ無効・arm_sdk・無線）`） |
| 取得日 | 2026-09-10 |
| 取得元のホスト | `ubuntu@192.168.123.200:~/g1-starter-kit`（clone 済みのものを複製） |
| 作者 | Kouta Yokoyama（`mtside01@gmail.com` / `k.yokoyama@play-robotics.com`） |
| 取り込んだ範囲 | `git ls-files` の 38 ファイルすべて。`.git/` と `config/g1.env` は**含めていない** |

## 許諾

上流に `LICENSE` ファイルは**ありません**。許諾は README の「ライセンスと出典」節の
以下の一文だけです（原文引用）。

> このリポジトリのスクリプトとドキュメントは自由に使ってください。以下は各々のライセンスに従います。
>
> - [xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate) — Unitree Robotics
> - [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) — Unitree Robotics
> - [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) / [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) — Livox
> - [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds) — Eclipse

キットが依存する上記 4 つの外部プロジェクトは**ここには含まれていません**。
それぞれのライセンスに従って各自の環境に導入されます（`setup/install_env.sh` 等）。

## なぜ submodule ではなく丸ごと置いているか

デモ機（OMEN / Ubuntu 24.04 / ROS 2 Jazzy）向けに**キット本体へ手を入れる可能性がある**ため
（上流の動作確認環境は 22.04 / Humble）。submodule だと手元の修正を持ち回せません。

この判断はこのリポジトリの既存方針（外部リポジトリは `.gitignore` して `.repos` で版だけ固定。
`Mapping/real/vendor/` がその形）と**違います**。理由は `Teleop/README.md` にも書いてあります。

## 手を入れたら

**上流と差し替えたくなるので、変更箇所は必ずコメントで分かるようにしてください。**
取り込んだ以上 `git diff upstream` が使えません。

```
# [G1_Hackason 改変] jazzy 対応。上流 45ead40 は humble 前提。
```

## 上流を取り直す手順

```bash
# 1. 比較元を用意する（OMEN には ~/g1-starter-kit が残してある。消さないこと）
git clone https://github.com/yokoshaan/g1-starter-kit /tmp/kit-upstream
cd /tmp/kit-upstream && git log --oneline 45ead40..HEAD   # 取り込み後に上流で何が起きたか

# 2. 手元の改変を洗い出す（45ead40 との差分＝こちらが足したもの）
cd /tmp/kit-upstream && git checkout 45ead40
diff -ru --exclude=.git --exclude=UPSTREAM.md \
  /tmp/kit-upstream \
  <このリポジトリ>/Teleop/vendor/g1-starter-kit

# 3. 上流の新しい commit を取り込むなら、2 の差分を当て直す
```

## 取り込みに際して除外したもの

- `.git/` — 残すと親リポジトリが embedded repository として扱い、壊れた gitlink になる
- `config/g1.env` — キットの `.gitignore` に入っている環境固有設定。
  OMEN 用の値は `Teleop/config/g1.env.omen` にあり、`Teleop/setup.sh` がここへコピーする

## 取り込み後に足した変更（上流 45ead40 との差）

`diff -ru` で出る差は次の 2 つだけ。どちらも上流には送っていない。

### 1. OMEN の `~/g1-starter-kit` にあった手元の変更（2026-10-26 に取り込み）

OMEN（`ubuntu@192.168.123.200`）のキットは 45ead40 のまま、コミットされていない変更が
8 ファイル・343 行あった。リポジトリの写しに無かったので、新しい機械に入れると当たらなかった。

| ファイル | 中身 |
|---|---|
| `setup/apply_patches.py` | 修正 `state-watchdog`（lowstate が 0.2 秒途切れたら送信を止めて強制終了。2026-09-30 に有線の抜き差しで全 29 関節が飽和トルクで跳ねた事故の対策） |
| `tools/state_watchdog_selftest.py` | 上の修正の自己テスト（新規・実機不要） |
| `tools/mode_check.py` | RPC が応答しないとき、配信を観測してモードを推定する |
| `scripts/quest_ap.sh`・`scripts/lib.sh`・`config/g1.env.example` | AP の PC の IP・SSID・パスワードを設定で固定できるように（パスワードは空欄のまま） |
| `README.md`・`docs/02-network.md`・`docs/06-troubleshooting.md` | 上の 3 つの説明 |

### 2. `[G1_Hackason 改変]` の印をつけた変更（2026-10-26）

| ファイル | 中身 |
|---|---|
| `setup/apply_patches.py` | 修正 `record-loco-cmd`（手なしでも `--motion` の歩きの指令を記録に残す）と `stick-pause`（両スティック 0.5 秒長押しで一時停止／再開。上流の `loco_wrapper.Damp()` は存在しない関数で、押すと teleop が落ちて腕を勝手に下ろしていた）。当てる順・外す順の決まりと、見出しの表示の直し |
| `tools/stick_pause_selftest.py` | 上の 2 つと xr-stale-stop の自己テスト（新規・実機不要） |
| `setup/apply_patches.py`（10-26 午後） | 修正 `record-timestamp`（data.json の 1 コマごとに time_ns）・`xr-stale-stamp`（televuer がデータの届いた時刻を残す）・`xr-stale-stop`（Quest のデータが 0.3 秒途切れたら一時停止。上流は無線が切れると最後のスティックの値で歩き続けた）。修正どうしの前提（DEPENDS）と当てる順・外す順の決まり |
| `tools/record_timestamp_selftest.py` | record-timestamp の自己テスト（新規・実機不要） |
| `setup/apply_patches.py`（10-27） | 修正 `xr-stale-log`（途切れの長さを残す）・`safe-exit`（終了で腕を 2 rad/s で下ろす）・`go-home-release`（戻しきれなくても制御を 2 秒で返す）・`arm-tau-getter`＋`record-arm-effort`（記録の左右の腕に速さとトルクを入れる。上流は空欄） |
| `tools/go_home_selftest.py`・`tools/arm_effort_selftest.py` | 上の自己テスト（新規・実機不要） |
| `setup/apply_patches.py`（10-30） | 修正 `loco-fwd-max`（左スティックを前へ倒したときの歩きの上限を `--loco-fwd-max` で 0〜1.0 m/s に。既定は上流と同じ 0.3、後ろ・横・旋回は 0.3 のまま）。扉を押して前へ出られるかの関門で上げて試すため |
| `tools/loco_fwd_max_selftest.py` | 上の自己テスト（新規・実機不要。16 項目） |
| `setup/apply_patches.py`（11-03） | 修正 `no-loco`（足なし: `--no-loco` で `--motion` のまま歩きの指令を送らない。記録の歩きは 0）。行を足すだけの置換（適用後が適用前を含む）は目印を先に見るように直した（`loco-fwd-max` は流すたびに `--loco-fwd-max` を二重に足していた） |
| `tools/no_loco_selftest.py` | 上の自己テスト（新規・実機不要。11 項目）。`loco_fwd_max_selftest.py`・`stick_pause_selftest.py` は `no_loco=False` を渡すように直した |
| `tools/loco_replay.py`・`tools/replay_arm.py`（11-03） | 再生の `--legs`（記録の歩きの指令も純正へ流す。送り方は `Door/real/door_stock_io.py` と同じ: SetVelocity・20 回/秒・止めるときは 0 を 3 回・FSM 500/501 だけ）。記録の time_ns どおりの再生。腰を遠隔操作と同じ kp 300・kd 3 で保つ（`--legs` は `--waist hold` でだけ）。時刻どおりの腕の速さの確かめ（30 rad/s 超は断る）。正常に終わったときに制御権を返した直後に重み 1 で掴み直していた二重の後始末を 1 回に（済んだ印は送り終えてから立てる） |
| `tools/replay_legs_selftest.py` | 上の自己テスト（新規・実機不要。偽の DDS と偽の純正で、main() と同じつなぎ方の再生を丸ごと回す。正常・歩いている最中の中断・後始末の途中の例外・SetVelocity の拒否・腕の速さ・入口の断り。約 20 秒） |
| `setup/apply_patches.py`（11-04） | 修正 `seated-controls`（座位: `--motion` なし＝純正を止めて `rt/lowcmd` でも、A の終了・両スティックの一時停止・Quest の途切れの一時停止を効かせる。上流はこの塊が `--motion` のときだけ。歩きの指令は `--motion` のときだけ送り、記録には全身の角度と歩き 0 を残す） |
| `tools/seated_controls_selftest.py` | 上の自己テスト（新規・実機不要。9 項目。歩きの口に一度も触らないことも見る）。`stick_pause_selftest.py` は塊の抜き出しの始まりを上流のコメント `# high level control` に、`loco_fwd_max_selftest.py` は `motion=True` を渡すように直した |

確かめ方: 上流の xr_teleoperate（817fb00）の素の写しに全部当てた結果が、OMEN で動いている
`~/xr_teleoperate` の 3 ファイルと 1 文字違わず一致すること、全部外すと素の版に戻ること、
自己テスト 2 本が合格することを 2026-10-26 に確かめた。
2026-11-03 にも同じ確かめをした（素の写しに 46 か所・2 回目は何も変わらない・OMEN の実物との差は `no-loco` の分だけ・
全部外すと素に戻る・`no-loco` だけ外すと 11-03 の前の OMEN と一致）。そのうえで OMEN の `~/xr_teleoperate` に
`no-loco` を当て（控えは `~/xr_teleoperate_backups/20261103T175848/`）、自己テスト 8 本が合格した。
同じ日に PC2 の `~/xr_teleoperate` にも `loco-fwd-max` と `no-loco` を当て（控えは PC2 の
`~/xr_teleoperate_backups/20261103T172217/`。PC2 の時計は中国時間）、PC2 の上で自己テスト 8 本が合格した。
当てた後の `teleop_hand_and_arm.py`・`robot_control/robot_arm.py` は OMEN と PC2 で md5 が一致する。
2026-11-04 の `seated-controls` も同じ確かめをした（素の写しに 49 か所・2 回目は何も変わらない・今の OMEN との差は
3 か所だけ・全部外すと素に戻る・`no-loco` を外すと乗っている `seated-controls` も外れる）。OMEN の一時の写し（`/tmp`）に当てて、
本物の SDK で自己テスト 9 本が合格した。OMEN と PC2 の `~/xr_teleoperate` にはまだ当てていない（PC2 は `pc2_teleop.sh patch` で当てる）。
