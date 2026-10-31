# ボトルが画角内にある状態から始めるrealistic試験（2026-10-31）

変更対象はrealistic評価入口と保存結果の集計。実機の認識・押下制御、モデル、重み、
ノイズ分布、鮮度・衝突・追従・成功判定の閾値は変更していない。

## 開始条件と記録

- `--camera-profile head` の省略時配置をX=0.36m、Y=-0.12mに変更した。
  明示したX/Yは各軸で優先する。legacyはX=0.33m、Y=-0.25m。
- `--fault none` の通常試験でも、全seedの取付誤差・立ち位置を適用した画角、
  台の直交する二辺、接近・押下・復帰経路を検査し、`layout_checks.json`に保存する。
  不成立のseedは物理・腕I/O・検出器を起動せず、開始条件不成立として全試行数に含める。
  移動量・home誤差はnull、腕指令数は0。seedの再抽選は行わない。
- 初回の配信RGB-Dから`initial_rgb.jpg`を保存し、YOLOの初回結果を
  `initial_detection.jpg`と`detections.json`へ保存する。認識処理へ到達しなくても初期画像は残す。
- 成功数と実際の手先接触到達数を分け、開始条件不成立・認識失敗・接触未到達・
  押下失敗・復帰失敗を集計する。接触は手先とボトルの接触力が0.1Nを超えた場合。
  成功条件は前方50±10mm、横10mm以内、傾き15度以内、画像確認、home、解除、不要接触の制限。
- replayの配置省略時は保存値を使用する。座標のない旧ログはheadでも従来配置と解釈する。
  配置の明示変更は条件不一致で拒否する。ソース・モデル・重みの一致条件も引き続き必要。

## 実行条件

新しい保存先は`_local/bottle_push/visible_start_20261031_run_01/`。
外部視点・RGB-D認識映像と集計は同名に`_review`を付けたディレクトリ。
以前の0/5の画像・結果は上書きしていない。

```bash
MUJOCO_GL=egl LIBGL_ALWAYS_SOFTWARE=1 \
__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json \
CUDA_VISIBLE_DEVICES='' OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
/tmp/opencode/g1realistic/bin/python \
  -c 'import triton, runpy, sys; script=sys.argv.pop(1); sys.argv[0]=script; runpy.run_path(script, run_name="__main__")' \
  Manipulation/push_button/sim/run_realistic_bottle.py \
  --seeds 0 1 2 3 4 5 6 7 8 9 --camera-profile head --timing-mode measured \
  --gravity-scale 1 --detector-imgsz 960 --video \
  --out _local/bottle_push/visible_start_20261031_run_01
```

Tritonの先行importは、このホストのネイティブライブラリの読み込み順への既存の対処。
CPU描画、CPUの実測処理時間を使い、YOLOは既存の`yolo26n.pt`、入力960、Torch 2スレッド。
実行ソースのコピー・ハッシュ、モデル・重み・引数・ホスト情報はmanifestに保存した。

## 10試行の結果

**初期の見切れは解消したが、手先接触0/10・押下成功0/10で、押す機構の検証は未達。**

全10seedの配置検査が合格し、最小画角余裕は24.87pxだった。
全seedの初回画像でボトル全体が見え、実際のYOLOが検出した（初回信頼度0.651〜0.794）。
新しい3フレームからの位置推定も全seedで成立した。

| seed | 到達した段階 | 停止理由 | 手先接触 | 復帰・解除 |
| --- | --- | --- | --- | --- |
| 0〜6 | 初回計測・全経路の事前計算 | 事前計算後、最初のボトル計測が0.5秒より古くなった | なし | 腕指令0回、解除済み |
| 7・9 | 接近・再計測・押下フェーズ | 押下中のボトル追跡が更新されず、鮮度超過で停止 | なし | home復帰・解除完了 |
| 8 | 接近・再計測 | 胴体の曲面深度の棄却が続き、安定した新しい3枚を取得できなかった | なし | 接近経路を逆順に復帰・解除完了 |

腕を有効化したのは3/10、押下フェーズへ進んだのは2/10。フェーズへの到達は接触を意味しない。
全seedの最大手先接触力は0Nで、台・自己・手先以外のボトル・その他への不要接触も0N。
通常試験の分類は認識失敗10件。他の分類は0件。試験全体の終了コード1はこの不合格結果による。
実測時間を物理へ加える評価なので、同じseedの成否はホスト負荷にも影響される。

残る課題は、事前計画を終えた後の計測の更新と、接近後の深度推定・連続追跡。
今回、鮮度の閾値緩和、oracleへの切り替え、失敗seedの再抽選、制御の変更は行っていない。

成果物:

- `visible_start_20261031_run_01/`: manifest、配置検査、全10試行のJSON・RGB-D・外部映像、`run.log`。
- 同ディレクトリの`initial_recognition_all_seeds.jpg`: 初回YOLO画像10枚の一覧。
- `visible_start_20261031_run_01_review/`: `review.md`・`review.json`、外部映像10本の一覧、各seedの認識映像。
- `visible_start_20261031_rejected_layout/`: headで旧配置を明示した実行確認。開始条件不成立1/1、
  腕指令0回、物理シーン・状態・映像を生成せず終了コード1。移動量・home誤差はnull。

いずれも`_local/bottle_push/`配下。評価完了時に実行ソースのハッシュが実装と一致することを確認した。

## 自動検査

配置既定値・軸別の明示指定・旧ログの配置解釈・replayの配置不一致拒否、
不成立seedが物理・腕制御を起動しないこと、接触と成功の分離、分類と映像なしの集計を検査した。
ボトル・台認識・実機入口の関連テストは95件通過。モデル環境変数を指定しないため、
既存のモデル依存テスト16件はスキップした。実際のモデルを使う評価は上記の10試行で実施した。
通常のバッチ入口でも配置検査を呼び、拒否した全seedを母数に残すことを検査した。
`git diff --check`も通過。
