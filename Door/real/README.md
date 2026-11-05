# Door/real — 段 5: 状態入力の生徒を実機の G1 で動かす

段 2.5 の状態入力の生徒（実機で取れる値だけを見る方策）と HOMIE の脚を、PC2 の上で 50 Hz で回す。
段 4（脚だけの合格の線）は 2026-10-16 のユーザー判断で省いた。最初の試しは **5 秒だけ・ウィンチで吊ったまま**。

| ファイル | 役目 |
|---|---|
| `door_core.py` | 方策の計算（観測を作る → 鏡写し → 正規化 → LSTM → 増分を積算 → HOMIE → 関節の目標）。numpy だけ |
| `door_frame.py` | 機体の姿勢から扉の値（枠の位置・寸法）を作る |
| `door_run_real.py` | 実機の制御の輪（状態の切り替え・止め方・記録）。`rt/lowcmd` を書くのはこれ 1 本 |
| `export_door_policy.py` | ckpt と HOMIE の重みを npz に書き出す（Mac、torch）。torch と numpy の出力の一致を確かめてから書く |
| `check_golden.py` | Isaac Sim の正解の記録（VIRAL6 パッチ 0050）と `door_core.py` を突き合わせる |
| `deploy_to_pc2.sh` | PC2 の `~/door_real/` へ配り md5 で確かめ、PC2 の Python を調べる |
| `target_guard.py` | 10-26: 腕の積み上げを今の角度 ±δ に収める（直し B）・送る目標を切る・立つ HOMIE の指令を 0（直し 8）。学習（VIRAL6 パッチ 0069）と 1 字も違わない同じファイル |
| `door_foot_load.py`・`leg_model_g1_29dof_rev_1_0.npz` | 10-26: 脚の関節の角度・トルクと IMU から左右の足の鉛直の力を逆算する（numpy だけ）。モデルは公式の g1_29dof_rev_1_0（`export_leg_model.py` が Mac の MuJoCo で書き出す） |
| `door_run_stock.py`・`door_stock_io.py` | 11-02: **純正の脚の口**。純正の歩行のまま腕だけ `rt/arm_sdk` で借り、歩きは生徒の指令を `SetVelocity` で渡す（下の「純正の脚で動かす」） |
| `door_vision_*.py`・`export_vision_student.py`・`pc2/run_vision.sh` | 11-05: **頭のカラーの画像だけの生徒**。`door_run_stock.py --door-source vision` で扉の値を画像から当てる。カメラは別のプロセス（純正の VideoClient → 画像の網 → UDP）。書き出し・電源を入れてからの手順は `docs/plan/2026-11-05-g1-door-vision-rl.md` の 9.3・9.4 |
| `analyze_stock.py` | 11-02: `door_run_stock.py` の記録の解析（段・扉の角度・指令に対して進んだ割合・RPC・腕の追従・観測が学習の範囲か・押す試し） |
| `tests/` | 機体なしの試験（10-26 に 103 件。扉の値・リモコン・段の進み方・B で立ったまま・L2+B で脱力・Ctrl+C の 1 回目と 2 回目・ssh の切断・ボタンの押しっぱなし・lowstate の途切れ・NaN。重みと正解の記録は環境変数 `DOOR_POLICY`・`DOOR_GOLDEN` で渡す） |

## 10-26 に足したこと（計画書 `docs/plan/2026-10-26-g1-door-stand-foot-load-check.md`）

- **HOLD で両足に体重が乗っているかを確かめてから A を受け付ける**。画面に「足 L ○ N・R ○ N（体重の ○%）」が出る。
  1 秒の平均で両足の合計が体重の 85% 以上・軽い方の足が 35% 以上でなければ、A を押すと理由が出て受け付けない
  （脚が 0.2 rad/s 以上で動いている間・1 秒たまっていない間・値が壊れている間も受け付けない）。
  HOLD に入ってから 1 秒以上待つ。引数: `--load-check {on,off}`（off は比べる用）・`--min-load-total 0.85`・`--min-load-each 0.35`・
  `--body-mass-kg`（体重の分母。既定はモデルの 33.34 kg。機体の日に綱を緩めて立てたときの逆算の合計から直す）。記録に `foot_fz`
- ⚠️ **全体重を足に乗せると HOLD の体は前へ 8〜10° 傾く**（足首 pitch の kp 40 のたわみ。10-16・10-23 の記録）。
  A の条件「傾き 8° 以下」（`--max-stand-tilt-deg`）と同時に満たせないことがある。傾きの線は Isaac Sim の E2 の結果を見て決める
- **腕の積み上げを今の角度 ±0.3 rad に収める**（直し B、`door_core.step` の既定）。止められた腕の前で目標がたまらない。
  ただし肩 pitch・roll（kp 100・上限 25 N·m）は差 0.3 rad でも 30 N·m で上限に張り付くので、押し付けの力は減らない（Isaac Sim の V3）
- **立つ HOMIE に歩く指令 0**（直し 8、`door_core.step` の既定）。10-16 の正解の記録は両方の無い評価で取ったので、`check_golden.py` が記録の設定を見て切る
- 方策の npz に `arm_accum_guard_delta` が無い（B なしで学習した）ときは、起動の表示で知らせる

## 機体に載せる前にしたこと（2026-10-16）

1. **重みの書き出し**: 反復 45,150（OMEN `ckpt/s25rl_best_it45150_ovr6/`）。torch と numpy の差 1e-15
2. **正解の記録との突き合わせ**（8 台・各 620〜860 ステップ）: 観測の組み立て 1e-8・HOMIE 1e-7・生徒の網 最大 5e-3
   （GPU の cuDNN の LSTM が TF32 で計算するため。丸めを真似ると縮む）・通しで腕の目標 最大 0.004 rad・歩く／立つの切替は全ステップ一致
3. Python 3.8・numpy 1.21 でも同じ結果（PC2 の既定の Python が 3.8）

突き合わせで分かった、実装を間違えやすい所:
- 観測の項目は**名前の辞書順**につながる（設定ファイルの並びではない）
- **腕のトルクの観測はシミュレータで常に 0 だった**（＋雑音）。実機でも 0 を入れる
- 把手が左の扉の鏡写しは**観測の項目ごと・正規化の前**。HOMIE の観測は映さない
- HOMIE が見る指令は 1 ステップ前のもの（生徒の出力は 1 ステップ遅れて脚に届く）
- シミュレータはリセット直後の最初の観測を HOMIE の履歴に入れない（実機は HOMIE を STAND から回し続けるので関係ない）

## ⚠️ 実機でシミュレータと違えていること

| 項目 | シミュレータ | 実機 | 理由 |
|---|---|---|---|
| 関節の目標 | 可動域の外にも出る（左手首の roll は 76% の時間、端の止めに 25 N·m で押し付けていた） | 可動域の端から `--clip-margin`（既定 0.05 rad）内側で切る | 端の止めに全力で押し付けると機械を傷める。同じ切り方をシミュレータに入れても成功率は下がらなかった（下の表） |
| 扉の角度 | LiDAR の計測を真似た値 | **閉じている前提の 0**（最初の版） | LiDAR の推定は次の版。5 秒では扉は開き始める所まで（シミュレータで 5° を越えるのは 4.1〜5.9 秒） |
| 枠の位置 | 測位の誤差を真似た値 | 巻き尺の最初の姿勢＋脚の odom（`rt/dog_odom`） | 純正の歩行を止めると odom が止まるかもしれない（未確認）。止まったら最初の姿勢のまま |
| 観測の雑音 | 入っている（関節の角度 ±0.01 など） | 入れない | 実機の値には元から雑音がある |

### 目標を可動域で切ったときの成功率（Isaac Sim、VIRAL6 パッチ 0051、45,150・256 回）

| 切り方 | 学習の扉 | 別の扉の組（seed=1） | b2 の扉 |
|---|---|---|---|
| 切らない | 77.0% | 82.0% | 76.2% |
| 端ちょうど | 81.6% | — | — |
| 端から 0.05 rad 内側（実機の既定） | 82.0% | 84.0% | 79.7% |

押し付けは方策の癖で、成功には要らなかった（OMEN `exp_clip_1016.log`・`exp_clip2_1016.log`）。

## 最初の試し（5 秒・ウィンチで吊る）

**人**: リモコン係（B でいつでも脱力）・ウィンチ係・PC 係（Enter でも脱力）。扉の両脇のガラスの前に養生、床にマット。

1. Mac で配る: `bash Door/real/deploy_to_pc2.sh <door_policy_it45150.npz>`（PC2 の Python の調べも出る。`OK` の出た方で動かす）
2. 機体を吊り、扉の前の床の印に置く。**pelvis（両足の間の中心）の位置を巻き尺で測る**:
   x = 閉じた扉の板の面からの距離（手前なので負。シミュレータは −1.5〜−0.6 m）、y = 開口の中心から左右（左が正、±0.5 m の内）、向き（扉に正対で 0）
3. リモコンで純正の歩行を止める（L2+B で脱力 → L2+R2 でデバッグ）。`door_run_real.py` は純正の歩行が動いていると起動しない
4. PC2 で起こす（例。扉の寸法は 幅,高さ,レバーの高さ,バックセット [m]）:
   ```bash
   tmux new -s door   # ssh が切れても止まらないように（切れたら door_run_real.py 自身も脱力して終わる）
   cd ~/door_real && python3 door_run_real.py --policy door_policy_it45150.npz --iface eth0 \
       --start-x=-1.00 --start-y=0.00 --start-yaw-deg=0 --door 0.94,2.05,1.00,0.064
   ```
5. リモコン: **start** → 3 秒で学習の初めの姿勢へ（HOLD）→ ウィンチを下ろして足を床に着ける（綱は少し緩める）→
   **A** で HOMIE だけで立つ（STAND。おかしければ B で立ったまま止める）→ 2 秒以上たってから **A** で扉の方策を 5 秒 → POST（立ったまま）→
   ウィンチで支えてから **L2+B** で脱力 → PC で Ctrl+C で終わる
6. 記録は `~/door_real/logs/door_real_<日時>.npz`。Mac へ持ってきて見る

### 止め方は 2 段（2026-10-16 ユーザー判断。脱力で床に崩れたことがあるので、ふだんの停止では脱力しない）

| 止め方 | 何が起きるか | 起きるきっかけ |
|---|---|---|
| **立ったまま止まる** | STAND・DOOR からは POST（扉の方策を止め、HOMIE が指令 0 で立つ。腕はその場）。RAMP・HOLD からは FREEZE（最後の目標角を PD で固く保つ） | リモコンの **B か select**／PC の **Enter**・**Ctrl+C の 1 回目**／ssh の切断（SIGHUP。プログラムは終わらない）／自動: 1 周期が 60 ms 空く・0.5 秒の平均が 25 ms 超・DOOR で odom が 0.5 s 途切れる・方策の計算で例外 |
| **脱力**（kp 0・kd 8） | 関節が柔らかくなり、支えが無いと崩れる | リモコンの **L2+B**／**Ctrl+C の 2 回目**（脱力を 1 秒送って終わる）／自動（制御できない・倒れている）: `rt/lowstate` が 0.1 s 途切れる・関節や IMU の値が NaN／四元数が壊れている・傾き 30° 超（シミュレータの最大 12.4°）・関節の速さ 20 rad/s 超（同 11.7）・目標が NaN |

- WAIT と DAMP では Ctrl+C か Enter で終わる。リモコンの B では終わらない
- start と A は、全部のボタンを離してから押したときだけ効く（起動時に押しっぱなしでも進まない）
- STAND の初めの 0.5 秒は既定の姿勢から HOMIE の目標へ移す
- 純正の制御へ戻すのは入れていない。純正の立ち上がりは「Damp → しゃがみから立つ」で脱力を通る（`SimpleWalk/real/walk_forward_real_sdk.py`）。立ったまま引き継げるかを吊った状態で試してから、別のボタンに入れる
- ⚠️ HOMIE 自体が実機で不安定なら、立ったまま止まるでも立てない（L2+B で脱力し、ウィンチで受ける）。電源断・SIGKILL では何も送れない。**PC2 では tmux の中で起こし、最初のうちはウィンチを外さない**
- 脱力に入った時点で記録を一度 `door_real_<日時>_damp.npz` に書く（その後に落ちても残る）

脱力は kp 0・kd 8（unitree_rl_gym の deploy_real と同じ）。

## 10-16 の 1 回目の結果と、立つだけの試し

1 回目（扉の 1 m 手前・`pc2/run_first.sh`）は扉の方策まで進まなかった（記録 `logs_real/door_real_20261016_173324_damp.npz`）:

- HOLD の 20 秒で、吊られた体の傾きが 9.6 → 15.7° に増えた。その姿勢から A で HOMIE に任せた
- STAND の 2 周期目で 1 周期が 91 ms 空き、自動で POST へ。POST の 1.6 秒、HOMIE（指令 0）は立て直そうとして
  後ろへ 0.54 m・横へ 0.23 m・向き ±30°・傾き最大 22°。右の股の向き（関節 8）が 21.9 rad/s で自動の脱力
- 周期の乱れの元は計算ではなく、DDS の受け取り（lowstate・dog_odom が約 1000 回/秒ずつ Python の callback を呼ぶ）と
  BLAS の糸。`--no-send` で WAIT を 20 秒ずつ比べると、最長の周期は odom あり・BLAS 既定 48.4 ms → odom なし・BLAS 1 本 23.1 ms

直したこと:

| 何を | どうしたか |
|---|---|
| 傾いたまま立たせない | HOLD で傾きが `--max-stand-tilt-deg`（既定 8°）を越えている間は A を受け付けない |
| 最初の計算の遅さ | 起動時に方策を空回しする（`warm_up`、hold と door を 3 回ずつ） |
| 立つだけの試し | `--stand-only`: STAND で A を押しても扉の方策へ進まない |
| 周期の乱れ | PC2 の起動スクリプトで `OPENBLAS_NUM_THREADS=1`。立つだけの試しは `--odom none` |

PC2 の起動スクリプトは `pc2/`（`~/door_real/` に置く）。PC2 には tmux が無いので、`door_start.sh` が接続から切り離して起こす:

```bash
cd ~/door_real && ./door_start.sh run_stand.sh   # 立つだけ（傾き 20° で脱力）
cd ~/door_real && ./door_start.sh                # 扉の試し（run_first.sh）
./door_stop.sh                                   # 立ったまま止める（PC の Enter と同じ。何度送っても脱力しない）
```

⚠️ 機体が再起動するとリモコンがつながるまで時間がかかる（10-16 は 2 回とも数分届かなかった）。届いているかは
`rt/lowstate` の `wireless_remote[2:4]` で確かめる。通常運転（FSM 501）から純正を外すのは SDK でもできる
（`LocoClient.Damp()` → `MotionSwitcherClient.ReleaseMode()`。吊ってから）。

## まだ無いもの

- LiDAR で扉の角度を測る部分（最初の試しで頭の LiDAR の記録を取り、それで作る）
- 事前地図と測位から枠の位置を取る部分（最初は巻き尺＋odom）

## 純正の脚で動かす（11-02、`door_run_stock.py`）

計画 `docs/plan/2026-11-02-g1-door-real-stock-legs.md`。脚は HOMIE を使わず、純正の歩行（通常運転 500/501）のまま動かす。
`rt/lowcmd` は送らない。腕 14 関節だけを `rt/arm_sdk` で借り（重み 0 ↔ 1 を 2 秒）、歩きは別の糸から `SetVelocity`（有効 0.5 秒）を 20 回/秒。
生徒の観測の脚と腰の行動には、HOMIE を計算だけ回した出力（影の HOMIE、`--leg-actions shadow`）を入れる。

⚠️ 生徒は脚と腰の観測に強く頼っている。Isaac Sim で脚と腰の観測を隠すと 0/256（11-02、反復 65,350）。
純正の歩き方の関節の値が学習の範囲に入っているかを、記録の後に `analyze_stock.py --policy` の「観測」で必ず見る。

| 段 | すること | PC の指示（`./stock_cmd.sh`） |
|---|---|---|
| WAIT | 純正が全身を持つ | `a` で腕を借りる |
| ARMS_IN | 重みを 2 秒で 0 → 1、腕を学習の初めの姿勢へ 1 rad/s | — |
| READY | 腕を保つ。影の HOMIE を回し、ライダーへ REF。傾き 8° 以下・両足に体重 85%（片足 35%）・ライダーの準備で `d` を受け付ける | `d` で扉の方策 |
| DOOR | 生徒が腕と歩きを動かす。時間（`SECS`）・扉の先 1.5 m・扉の角度（`STOP_DEG`）で POST | `stop` で止める |
| POST | 歩き 0・腕はその場で保つ（純正が立つ） | `p`（押す試し）・`a`（構え直す）・`r`（腕を戻す） |
| PROBE | 手順 11: 腕は止めた姿勢のまま、前へ 0.3 m/s（`p0.5` で 0.5）を 2 秒 | — |
| ARMS_OUT | 腕を借りる前の角度へ戻し、重みを 2 秒で 0 へ | — |

止め方: ふだんは立ったまま（`stop`・Enter・Ctrl+C の 1 回目・ssh の切断・リモコンの B／select・傾き 10°・関節 20 rad/s・
lowstate の途切れ・周期の遅れ・SetVelocity の失敗 5 回・ライダーの見張り）。脱力はリモコンの L2+B（純正のもの）・Ctrl+C の 2 回目・傾き 30° だけ。
プログラムが落ちても、純正はその時の形で腕を保ち（10-27 に確認）、速度は 0.5 秒で切れる。

**SetVelocity は別のプロセスで送る（11-04〜、`LOCO=relay` が既定）**: `run_stock.sh` は `door_run_stock.py --loco relay` で起こし、
`loco_relay.py` が子のプロセスで LocoClient を持って SetVelocity を 20 回/秒で送る。主の側とは UDP（127.0.0.1:47341・47342）で、
主は変わった時の指令（歩く・止まる・脱力）に加え、毎周期「今の望み」を丸ごと送る（1 発落ちても次の周期で届く）。
子は状態（送った数・続けて失敗した数・番号・応答の時間・FSM・子の pid）を 50 回/秒で返す。
11-04 の実機で、同じプロセスからは返事が 1/4〜1/2 落ちた（3104）。同じ失敗を子のプロセスへ移して直した例がある（4paradigm/phanthymotus-driver 24274c5）。
- 子は、主の電文が 0.3 秒来なければ速度 0 を送り、親が居なくなれば止めてから終わる。Ctrl+C と ssh の切断では終わらない（主が止める指令・脱力を送るのに要る）
- 主は、腕を構える〜方策の間に子の状態が 0.5 秒来なければ立ったまま止まる（子が居ないまま `d` で歩かせない）。記録の列 `vel_status_age` が状態の古さ
- 子が居ないときに脱力が要れば（傾き 30° 等）、主が自分で LocoClient を作って Damp を 1 回送る。pid の違う状態（居残った前の子）は受け付けない
- 起動の時の FSM は子が読む（主のプロセスは LocoClient を持たない）。前の作りに戻すのは `LOCO=inproc`

```bash
# Mac: 配る（重みは logs_real/policy/ に置いてある。リポジトリには入れない）
bash Door/real/deploy_to_pc2.sh Door/real/logs_real/policy/door_policy_f3sc_r4_it65350.npz
# PC2: 推定器 → 純正の口（機体は吊って綱を緩め、リモコンで通常運転に入れてから）
cd ~/door_real && THICK=<板の厚み m（巻き尺）> ./lidar_start.sh   # 扉の高さ・厚みはその日に巻き尺で測る
DOOR=0.94,2.05,0.99,0.064 SECS=5 CENTER="0.047 -0.016" ./door_start.sh run_stock.sh
./stock_cmd.sh a     # 腕を借りる → READY（画面に「足 L ○ N・R ○ N（体重の ○%）」）
./stock_cmd.sh d     # 扉の方策
./stock_cmd.sh stop  # 立ったまま止める（いつでも）
./stock_cmd.sh r     # 腕を戻す → WAIT
./stock_cmd.sh q     # 終わる
# Mac: 記録を読む
Navigation/.venv/bin/python Door/real/analyze_stock.py <door_stock_*.npz> --policy Door/real/logs_real/policy/door_policy_f3sc_r4_it65350.npz
```

**生の点群も録る（11-03〜）**: `door_start.sh run_stock.sh` は、方策のプロセスが動いている間ずっと `pc2/rec_follow.sh` で
ライダー＋頭の IMU（`~/mapping_runs/<日時>_run_stock_<日時>_li`）と `/dog_odom`（`…_odom`）を別々のプロセスで録る。
方策が終わると書き切って止まる（上限 `REC_MAX` 既定 900 秒・約 5 GB）。PC2 の空きが `REC_MIN_FREE_GB`（既定 8 GB）未満なら起こさない。
`REC=0` で録らずに起こす。録っている様子は `logs/rec_latest.log`。推定器は REF を受けた時刻を `lidar_node_*.npz` の `ref_wall` に残す。

```bash
# Mac: 写す（1 回 0.5〜2 GB。Mac の空きを先に確かめる）
rsync -a -e "ssh -i $G1_KEY" unitree@192.168.0.82:mapping_runs/<日時>_run_stock_<日時>_li Door/real/logs_real/<日>/
# Mac: 実機と同じ時刻に REF を出して再生し、推定を実機の推定と時刻で突き合わせる（--rec-clock）
Navigation/.venv/bin/python Door/real/door_lidar_node.py --thickness <板の厚み> --replay <…_li> \
  --ref-from <実機の lidar_node_*.npz> --rec-clock --speed 50 --out <再生の npz>
```
11-03 の検証: 旋回の記録 ① を再生すると、実機の推定との差は中央 0.5 cm・最大 3.7 cm（向き 0.25° 以内）。
録りながら READY を保っても、制御の周期は 99% 23.7 ms（録らないとき 22.0 ms）、推定器は 10 回/秒のまま。
