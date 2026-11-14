# 実機のエレベーター呼びボタンを押す

ボトル押しの段階接近、保持・再開・復帰、非同期認識、試行記録を
`real/run_push.py --task button --profile field` から使える。
右腕と頭RGB-Dカメラ、既存の実測カメラ・手先校正を使用する。
`field` プロファイルを使った実機接触試験は実施していない。

## 関節角を補正しながら実機で押す（robust）

`real/run_robust_button.sh` は、初期姿勢から関節角の補正を有効にする。
接近・押下・退避・帰還の全区間で実測角を読み、重力補償に上限付きの積分トルクを加える。
腰角とIMUの姿勢もFKに反映して手先の目標を補正する。
接近・押下中は追従が遅れる間、参照経路の進行を保留して補正を続ける。
ボタンの5cm手前で補正を始める方式ではない。関節指令は50Hzの速度・加速度制限を通す。

```bash
# 認識・補正付きの物理検証・DDS出力を確認。重み0で腕は動かさない。
bash Manipulation/push_button/real/run_robust_button.sh check --button-direction down

# 接触の5mm手前まで接近し、補正しながら帰還・解除する。
bash Manipulation/push_button/real/run_robust_button.sh precontact --button-direction down

# 物理検証に合格したら、本押し・退避・帰還・解除・点灯確認を行う。
bash Manipulation/push_button/real/run_robust_button_press.sh --button-direction down
```

バッシュの既定は直進押下。推定上のボタン5cm手前で、ボタン中心と手先の
進行線が3mm以内となる位置を、新しい画像3枚で確認してから押下へ進む。
これは画像から検出したボタン位置と、関節・カメラ校正から推定した手先の比較であり、
画像で指先そのものを検出する方式ではない。位置目標の補正は固定するが、
関節トルク補正、速度・加速度制限、衝突・状態監視と点灯後の帰還は維持する。
基準位置の模擬試験では完走した一方、ボタン位置を上下・左右へ2mmずらした場合に
パネル縁へ接触する条件があった。実機では2026-11-14の2試行で点灯・復帰・解除を確認した。

```bash
# この別名のバッシュからも同じ直進押下を選べる。
# 認識・計画・重み0の出力確認。腕は動かない。
bash Manipulation/push_button/real/run_robust_button_press_straight_trial.sh --check --button-direction down

# 本押し。物理検証が通過すれば確認入力を待たず腕が動く。
bash Manipulation/push_button/real/run_robust_button_press_straight_trial.sh --button-direction down
```

バッシュからの既定値は `--readiness-policy collection --button-feedback joint --button-policy collection --button-press-mode straight_trial --planning-budget-s 300`。
点灯後の帰還も物理検証するため、実行開始までの計画時間は以前より長くなる場合がある。
起動前の状態監視はボトル用バッシュと共通にし、状態期限400ms・脚速度1.0rad/s・IMU角速度0.3rad/sを使う。
校正時からの腰角0.025rad超・傾き2度超の差は警告として記録し、それだけでは開始を拒否しない。
骨盤の絶対的な傾き0.15rad（約8.6度）、必要ファイル、FSM 501、他の腕制御との競合は確認する。
`check`・`precontact`・`push` の全モードへ同じ起動前条件を渡す。
厳しい起動前確認が必要な場合は `--readiness-policy strict` を明示する。
収集用の `collection` は基準位置の1条件で、実機と同じ補正処理を物理検証する。
押下深さ・追従・帰還の偏差は診断情報として扱うが、実際の衝突、過大な接触力、関節余裕は合否に使う。
関節フィードバックでは、次の指令先で障害物との余裕が3mm未満なら、減速指令で再検査する。
収集用では選択した手先とパネルの穴の縁の組み合わせは空間余裕の対象外とし、0.5mmを超える侵入を拒否する。
経路候補の静的検査と、物理検証で実際にたどった姿勢の余裕の合否基準は3mmとする。
標準の4条件（基準・奥3mm・左3mm・右3mm）は `--button-policy standard` で選べる。
従来の補正なし実行は `--button-press-mode feedback --button-feedback off --button-policy standard --planning-budget-s 30`。
Pythonの直接起動では従来の `standard/off` と `feedback` が既定である。

補正付きでは、目標の押し込み量を実測ストローク（既定15mm、`--stroke-mm`で変更）にする。
以前の4mmのたわみ補償と1mmの追加押し込みは加えない。押下後は接触中に蓄積した補助トルクを減らす。
積分トルク上限は肩・肘 `[6,6,4,6]`、手首 `[2,1.5,1.5]` Nm。
全体のトルク上限は既存の `[25,25,25,25,25,5,5]` Nmを維持する。
補正が20秒間進まない場合、または補正動作が既定120秒を超えた場合は、通過済みの経路を戻る。
この復帰にも30秒の上限があり、通信異常や衝突では制御を解除する。
親プロセスの待ち時間には画像取得・探索・補正・復帰・解除の時間を含める。

収集用では画像の経過時間による停止を外し、初回は異なる3枚を使う（安定幅10mm、最大30秒）。
計画時の押下方向を保持し、接触前の横方向補正を5mm以内に制限する。
対象位置が横方向5mm・押下方向10mmを超えて変わった場合も、それを理由に停止しない。
計画中・実行直前の対象位置の差は警告として記録し、画像の更新時にも25mmの位置変化だけでは観測を捨てない。
補正量の5mm上限と1回の画像更新あたり0.5mmの変化量制限は維持し、変化量と上限適用をログに残す。
新しい位置は異なる3枚で安定を確認して受け取り、不正な数値・対象の一意性・半径の整合性は確認する。
接触の5mm手前以降は画像による目標変更を止めるが、実測角と胴体姿勢による補正は継続する。
joint補正では、最初に対象ボタンを特定した画像の明るさを基準に固定する。腕の接近開始後、
対象ボタンの新しい画像1枚で基準から30以上の増光を検出したら、
位置や深度にかかわらず押し込みをやめ、押下前に計算・検査した短い経路で開始姿勢へ帰還する。
点灯位置から近い経由点へ戻り、衝突余裕を検査した関節空間の短い経路を使う。
帰還中は追従誤差によって参照経路を保留しない。実測姿勢から各指令への衝突検査と、
関節の速度・加速度制限は引き続き適用する。計画時には点灯時点を変えた物理検証も行う。
異常時の復帰や接触前診断はこの高速化の対象外。通常版・高速版のどちらのバッシュにも適用する。
画像の判定は選択したボタンの領域を用い、3D位置推定が遮蔽で失敗しても続ける。
基準の枠・明るさ・時刻と検出時の明るさ・時刻、帰還・制御解除の結果を記録する。
検出枠内の反射やカメラの露出変化も増光として判定される可能性がある。
関節状態の期限400ms、脚速度1.0rad/s、IMU角速度0.3rad/sなどの監視は残る。
動作中に増光を検出しなかった場合、帰還後の点灯確認には別々の3枚を使う。
その場合、画像不足や判定混在は成功にしない。動作中に増光を検出した場合はその時点の判定を成功として保持する。
終了コード0は点灯確認・動作完了・制御解除の成立を表し、自動の押し直しは行わない。

最初の設定は `real/robust_button_setup.example.json` と `real/robust_button_trial.example.json` を
`_local/robust_button/active_setup.json` と `trial.json` にコピーする。
G1で制御する場合は `real/robust_button_g1_setup.example.json` を参考に
`execution_host: "g1"` と既存の `remote_runtime_root` を指定する。
fungiwindowsからはSSHで新しいコード専用ディレクトリへ転送し、G1の既存Python・NIC・校正・重みを使う。
実行中のコードを上書きせず、ボトル版と共通のロックで同時実行を防ぐ。

```bash
# G1へコードだけを転送して終了。腕の動作、RGB-D、DDSは開始しない。
bash Manipulation/push_button/real/run_robust_button.sh check --deploy-only
```

校正JSONは**制御を実行するPCから読める場所**に必要である。G1実行ならG1側、
fungiwindowsで直接実行するならfungiwindows側に、ラボPCで使った実測校正をコピーする。
コード転送は既存の校正や重みを上書きしない。G1側の `active_setup.json` が指す
`trial.json` の `base_transform` を確認する。カメラや手先の取り付けを変えた場合は再校正する。
収集用でも `position_uncertainty_m` と出典を保存し、0へ書き換えない。
半径が等しいボタン・手先も収集用では扱うが、標準モードの寸法・校正余裕の条件は維持する。

ログは `_local/robust_button/runs/日時_モード_ID/`。G1実行時はG1の同じ相対パスへ保存する。
設定・元の校正・ソースhash・参照経路・実測角・IMU・FK誤差・補助トルク・dq・ddq・tau_est・
点灯判定・帰還・解除を記録する。SDKが返さない追加計測値はnullとなる。
シミュレーションと実機が同じ補正設定を使ったことも検証する。

実機の2試行はこの配置での確認であり、補正ゲインや他の配置の網羅的な検証ではない。
シミュレーションは骨盤固定であり、実機の立位バランスを保証しない。
保存済みログからのオフライン検証は `sim/verify_button_tracking.py --help` を参照。

## 設定と対象選択

`real/button_field_trial.example.json` をコピーし、`stroke` を使用するボタンの実測値にする。
例の1.5mmは実物の測定値を表すものではない。許容するストロークは1.5〜15mm。
`clearance` は押下後の点灯確認用の退避距離（既定5cm）で、最後の再計測位置3cmとは別。

設定は **CLI > JSON > プロファイル** の順に適用する。既定の`strict`は従来の1回再計測。
`field`は10・7・5・3cmの段階接近とRGB-D記録が既定で、台の設定は不要。
`--approach-mode single`で従来の接近を選べる。
`staged`と`--alignment-mode iterative`は併用できない。

複数のボタンが映る場合、`--target-pixel U V`で押すボタンの画像上の画素を指定する。
候補が一つの場合は省略できる。初回以降は3D位置で同じボタンを追跡し、
保持からの再開でも別のボタンへ自動で切り替えない。
対象の選び直しは新しい試行として開始する。

画像の確認と画素座標の表示は既存の`real/localize_button.py --preview`を使える。
カメラ配信と校正の準備は[CAMERA.md](CAMERA.md)を参照。

## FK誤差を調整する

| 設定 | fieldの既定 | strictの既定 | 意味 |
| --- | --- | --- | --- |
| `--fk-tolerance-m` | 0.007（7mm） | 0.003（3mm） | 次段階へ進むための手先位置の収束幅 |
| `--fk-abort-m` | 0.010（10mm） | 0.008（8mm） | 補正後に許容しない誤差。strictの従来入口では元の1cm指令追従検査を維持 |

`field`ではFK誤差7mm以内を連続3回確認して進む。
許容幅の指定範囲は1〜10mm、中止幅は許容幅より大きく20mm以下。
既存の試行JSONに`fk_tolerance_m`を指定している場合は、その値が既定値より優先される。
さらに、`button_radius_m - tip_radius_m - position_uncertainty_m`が許容幅以上であることを
実機接続前に検査する。小さいボタンでは大きな許容幅を指定できない。

例えば余裕のあるボタンで8mmまで許容する場合は、次を追加する。

```bash
--fk-tolerance-m 0.008 --fk-abort-m 0.012
```

これは接近時のFK位置合わせの設定であり、IK計算精度・ボタンの押込量・過押し制限を緩めるものではない。
位置計測の安定幅は別項目`--position-stability-m`（既定4mm、新しい3枚）。
押下中も横ずれがボタンと手先・校正の余裕を超えた場合は中止する。

## 計画表示・接触前診断・本押下

リポジトリ直下から実行する。モデル・校正・重みは使用する環境のものを指定する。

```bash
cp Manipulation/push_button/real/button_field_trial.example.json _local/button_field.json
# コピーしたJSONのstrokeを実測値へ変更する。

# 現在の保存状態と、新しい頭RGB-D画像から計画表示。腕の指令は送らない。
python3 Manipulation/push_button/real/run_push.py --task button \
  --trial-config _local/button_field.json \
  --model /path/to/menagerie/unitree_g1/g1.xml \
  --base-transform /path/to/calibration.json --weights /path/to/button_best.pt \
  --state-json /path/to/current_state.json --target-pixel 320 180 \
  --output _local/button_preview_01.json

# 最初の実機確認は接触前診断。4段階を確認し、押さずに復帰・解除する。
python3 Manipulation/push_button/real/run_push.py --task button \
  --trial-config _local/button_field.json \
  --model /path/to/menagerie/unitree_g1/g1.xml \
  --base-transform /path/to/calibration.json --weights /path/to/button_best.pt \
  --arm-socket /tmp/g1_button_arm.sock --target-pixel 320 180 \
  --stop-before-contact --execute --calibrated --output _local/button_approach_01.json

# 診断後の本押下は、上のコマンドから--stop-before-contactを外し、
# --outputを新しい名前（例: _local/button_press_01.json）にする。
```

画素値320・180は例であり、実際のプレビュー上の対象に置き換える。
SDKへ直接接続する場合は`--arm-socket`の代わりに`--network-interface enp3s0`を指定する。
実機への指令には`--execute --calibrated`と実行時のYES入力が必要。
Navigationの移動指令は事前に停止する。この入口はNavigationのミッションを取り消さない。

## 段階接近と保持

各段階では手先表面をボタン表面の10→7→5→3cm手前へ運び、0.6秒静止してから、
新しい異なるRGB-D画像3枚で対象位置・パネル法線を更新する。
残りの経路を検査し、FKで補正・収束を確認して次へ進む。
目標位置の変化が8mmを超えた場合やFKが未収束の場合は、その段階で再確認する。
追加確認は4段階全体で最大2回（`--approach-extra-retries 0..2`）。
FK中止幅の超過、衝突、通信・指令・関節状態の異常を追加確認で救済しない。

計測不能や一時的な脚・骨盤の動きでは、その場で保持する。
100ms未満の速度超過から戻り、300ms安定した場合は接近中だけ再計測から自動再開する。
継続する動きや試行開始姿勢から3度を超える腰・傾きの変化は操作者の確認を待つ。
監視の基準を取り直しても、カメラや手先の校正は書き換えない。

- `r`: 対象の同一性を保ち、古い完成結果を破棄して新しい3枚と現在姿勢から4段階を組み直す。
- `b`: 現在姿勢から検査して往路を逆再生し、開始姿勢へ復帰する。
- `q`: 試行を終了して解除を試みる。復帰動作は行わない。

押下開始後の異常では`r`を受け付けず、`b`または`q`だけを受け付ける。
入力待ちの間も50Hzの状態監視と保持を継続する。

## 押下結果と記録

最後に確定した位置・法線へ連続して接近・押下し、退避・静止後の新しい3枚で点灯を確認する。
点灯判定は従来と同じ増光方式で、`--lamp-min-rise`は既定30。
明確な未点灯時だけ同じストロークで1回押し直し、対象消失・画像不足・判定混在では押し直さない。
点灯確認の再検出で押下目標を更新しない。

往路には各段階の補正と途中再開までの経路を記録し、現在姿勢からの接続と逆再生を衝突検査する。
実測で開始姿勢から2度以内を連続3回確認して解除する。
通常試行の終了コード0には点灯・復帰・解除指令送信の完了がすべて必要。
接触前診断は診断完了・復帰・解除で0、操作者の取り消しは1、未確認・異常は2、中断は130。
`release_completed`は解除指令の送信成功であり、機体側の解除確認を示さない。
`physical_press_success`は実機では常に`null`。

結果JSONには段階ごとの計測位置・法線・FK誤差・追加確認理由・時間、点灯確認画像、
保持・再開イベント、29軸の状態、送信指令・トルク、適用設定、校正・ソース・モデルの版を保存する。
RGB-D、内部パラメータ、撮影時刻の関節角・IMUは同名の`*_rgbd/`へNPZで保存する。
容量制限のある別スレッドで保存し、欠落や保存失敗は`rgbd_recording`に記録する。
記録有効時は既存のJSON・RGB-D・点灯画像を上書きしない。

`--gravity-scale 0..1`と`--dynamics-feedforward`はボトル側と同じ任意の補償。
既定は両方無効で、動的補償には0より大きい重力補償倍率が必要。

実機では接触前診断を先に行い、その後上・下の対象をそれぞれ選び、
実測ストロークでの押下・点灯・復帰・解除を記録する。模擬試験の通過を実機の押下成功に数えない。

## ソフトウェア検証（2026-11-01）

全体回帰は441件通過、歩行方策資産が未配置の既存3件はスキップ。
実機入口・段階接近・FK許容幅・中断・点灯・復帰・解除の最終重点試験は49件通過。
上下2高さと前後・法線角の違いを組み合わせた6配置で、Menagerieを使う接近・押下・退避・復帰の経路検査も通過した。
これらは実機接触や自由立位の成功を示すものではない。

```bash
G1_MUJOCO_MODEL=/path/to/menagerie/unitree_g1/g1.xml \
OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 MUJOCO_GL=egl \
LIBGL_ALWAYS_SOFTWARE=1 CUDA_VISIBLE_DEVICES='' \
python3 -m pytest -q -rs Manipulation/push_button/tests
```

頭カメラFKの試験にはJ1-genの公式URDFも必要。
ZMQ・腕ブリッジの試験にはローカルソケットを作成できる環境を使う。
