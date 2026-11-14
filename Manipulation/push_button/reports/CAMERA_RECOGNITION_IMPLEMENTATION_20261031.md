# 頭部カメラ認識の改善実装と後日の評価手順

今回の範囲は実装・単体テスト・構文検査まで。画像の描画、物理シミュレーション、配置探索、
接触前診断、10試行、実機動作は実行していない。配置は未確定で、改善後の成功率は未検証。
既存の0/10の記録・RGB-D・映像はそのまま保存する。

今回の確認は79件成功・3件スキップ。対象は台認識、IK再探索、配置引数・投影計算、
結果集計、既存のボトル制御・非同期認識・状態履歴・カメラ形式・実機入口の単体テスト。
スキップは `G1_MUJOCO_MODEL` を必要とする既存の実機入口結合テスト3件。
ローカルZMQテストはソケット作成を許可した環境で通過した。
変更したPythonファイル10本の構文検査と `git diff --check` も通過した。

## 変更内容

- 台推定は `x_min/x_max/y_min/y_max` の四辺からX・Y方向を一辺ずつ選ぶ。
  固定した辺の法線、30点以上、長さ80mm以上、直線残差4mm以下の条件は維持する。
  深度欠損や腕による内部輪郭を除外し、辺の外側に有効な台面より低い深度があることも確認する。
  輪郭を半画素補正して平面へ投影し、2px標本化で最小側・最大側に逆向きに生じるずれを抑える。
  有効な対辺の補正量が4mmを超えて食い違う場合は棄却する。
  同軸の候補は直線残差、長さ、有効点数、辺名の順で決定する。
- ボトルの事前計画と移動は、現在姿勢からのIK・経路検査を最初に試す。
  失敗時だけ肩pitch/roll各±0.4rad、yaw±0.6radと0の組み合わせを固定順で試す（合計最大27候補）。
  初期値は関節限界の25mrad内側へ制限し、数値計算中だけに使う。
  各解は実際の開始姿勢から経路検査する。移動直前には状態を再取得して再検査する。
  状態監視のRuntimeErrorは再探索せず中止する。速度・加速度・衝突・鮮度の制限は変更しない。
- realistic評価の `--bottle-x/--bottle-y` は、立ち位置の乱れを適用する前の中心座標。
  省略時は従来どおり `(0.33, -0.25)` m。台高・範囲・5cm押下の採点条件は維持する。
  replayでは配置も比較し、異なる配置の処理時間ログは再利用しない。

`controller.json` の `table_diagnostics` に撮影時刻、各辺の点数・長さ・残差・補正量、
候補画素、採用辺・棄却理由を保存する。`ik_searches` には初期値の変更量、失敗理由、採否を保存する。
認識失敗でYOLOへ到達しない場合も台の診断を残す。

## 実行環境の準備

以下のコマンドはリポジトリルートから後日実行する例。出力先は毎回新しい名前を使う。
venv・モデル・重み・J1-genのURDFは既存手順と同じ。今回これらの依存は更新していない。

このホストではTritonを他のネイティブライブラリより先に読み込む必要があったため、
評価用の起動関数に読み込み順の回避を入れる。GPUが公開されていない環境ではMesaのCPU描画を使う。

```bash
SIM_PY=/tmp/opencode/g1realistic/bin/python
export MUJOCO_GL=egl
export LIBGL_ALWAYS_SOFTWARE=1
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json
export CUDA_VISIBLE_DEVICES=''
export OPENBLAS_NUM_THREADS=2
export OMP_NUM_THREADS=2

bottle_eval() {
  "$SIM_PY" -c 'import triton, runpy, sys; script=sys.argv.pop(1); sys.argv[0]=script; runpy.run_path(script, run_name="__main__")' \
    Manipulation/push_button/sim/run_realistic_bottle.py "$@"
}
```

## 配置の静的検査

`--check-layout` は描画・物理ステップ・DDS接続を行わない。
指定した配置にseedの取付誤差・立ち位置の乱れを適用し、次を `layout_checks.json` へ記録する。

- 押下前と5cm押下後のボトル輪郭の画角余裕（8px以上）。
- 直交する台の辺が各80mm以上、画角内に入るか。
- 既存の厳密なIK・衝突検査による接近・押下・後退・homeまでの経路。

`ready=true` は静的検査の合格であり、RGB-D認識や押下成功を示さない。
腕の遮蔽・身体の揺れ・動的な追従はこの検査の対象外で、`dynamic_success_rate` はnull。
採点用のボトル真値はこの診断だけに使い、実行制御へ渡さない。

以下の `(0.36, -0.13)` は未検証の候補例であり、推奨配置として確定していない。

```bash
bottle_eval --check-layout --camera-profile head --seeds 0 1 2 3 4 5 6 7 8 9 \
  --bottle-x 0.36 --bottle-y -0.13 \
  --out _local/bottle_push/camera_improvement/layout_candidate_01
```

配置探索を行う場合はX=0.30〜0.40m、Y=-0.18〜-0.12mを5mm刻みで調べ、
全開発seedの静的条件に通る候補を、最小の画角余裕が大きい順に選ぶ。
同点は従来配置からの距離、X、Yの昇順とする。候補がなければその制約を報告し、
画角・関節限界・合格基準を暗黙に変更しない。

## 接触前診断と10試行

静的検査後に配置を固定し、下記変数をその値に設定する。各コマンドは単独で実行する。

```bash
# 仮値。静的検査後に確定した配置へ置き換える。
BOTTLE_X=0.36
BOTTLE_Y=-0.13

bottle_eval --seeds 0 1 2 3 4 5 6 7 8 9 --camera-profile head --timing-mode measured \
  --bottle-x "$BOTTLE_X" --bottle-y "$BOTTLE_Y" \
  --gravity-scale 1 --detector-imgsz 960 --stop-before-contact --video \
  --out _local/bottle_push/camera_improvement/approach_01

bottle_eval --seeds 0 1 2 3 4 5 6 7 8 9 --camera-profile head --timing-mode measured \
  --bottle-x "$BOTTLE_X" --bottle-y "$BOTTLE_Y" \
  --gravity-scale 1 --detector-imgsz 960 --video \
  --out _local/bottle_push/camera_improvement/push_01
```

元の採点基準を維持し、失敗を認識・経路・追従・画像確認・復帰に分ける。
10/10を3反復確認してからコードと配置を凍結し、未使用seed 200〜219で18/20以上を確認する。
失敗した試行・反復も保存する。取付誤差、対象消失、古い画像、状態通信断も既存オプションで再検証する。
配置変更があるため、旧配置の0/10との差を認識アルゴリズムだけの効果として扱わない。

## 保存結果の映像・集計

以下は保存結果だけを読み、シミュレーション・YOLO推論を起動しない後処理。
`--videos` を省くとJSONとMarkdownの集計だけを生成する。

```bash
"$SIM_PY" Manipulation/push_button/sim/review_realistic_bottle.py \
  _local/bottle_push/camera_improvement/push_01 \
  --out _local/bottle_push/camera_improvement/push_01_review --videos
```

- `review.json/review.md`: 採点結果、停止理由、移動量、画像確認・home・解除、
  台の採用辺の頻度、評価時間に加算された処理時間の中央値・95%点・最大値。
- `ten_trials.webm`: 外部視点を最大5列で並べる。8fpsを維持し、終了した試行は最終フレームを保持する。
- `seed_*_recognition.webm`: 保存RGB-Dの頭部カメラ画像へ台の候補点・採用辺とYOLO枠を重ねる。
  緑=採用、黄=有効な非採用候補、赤=棄却。撮影時刻と配信遅延も表示する。
  未配信フレームの生成はせず、保存画像を次の撮影時刻まで保持する。
  頭部映像は最初〜最後の保存撮影時刻を対象とし、外部映像と同じ開始時刻・長さではない。

処理時間と配置、使用ソース・モデルのハッシュは元のmanifestを引き継ぐ。
新しい診断情報がない旧ログも集計できるが、過去の辺判定結果は復元・捏造しない。
