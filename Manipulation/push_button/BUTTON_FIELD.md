# 実機のエレベーター呼びボタンを押す

ボトル押しの段階接近、保持・再開・復帰、非同期認識、試行記録を
`real/run_push.py --task button --profile field` から使える。
右腕と頭RGB-Dカメラ、既存の実測カメラ・手先校正を使用する。
実機の呼びボタンへの接触試験は、この変更では実施していない。

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
