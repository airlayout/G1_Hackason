# 中指・手首カメラで押す

G1 29DoFの固定ハンドの右中指で、呼びボタンまたは固定したラベル付きペットボトルを押す。
頭のRGB-Dで対象を選び、手首画像で位置を補正する。手首映像が無い環境は頭カメラで動作する。

接近と復帰は衝突回避経路、対象付近は直線の手先運動を使う。公式URDFの衝突形状に加え、
省略されているプラスチック製ハンドのメッシュとカメラ取付具も検査する。
押下後は対象から離れてから、現在の実測姿勢から復帰経路を検証する。

## 準備

リポジトリ直下でPython 3.10以上を使う。各コマンドは1行ずつ実行する。
既存の試験venvがある場合は、そのPythonを`P`に指定してよい。

```bash
python3 -m venv _local/wrist_press/venv
P=_local/wrist_press/venv/bin/python
$P -m pip install -r Manipulation/wrist_press/requirements.txt
$P -m Manipulation.wrist_press.runtime
```

`runtime`は`Dev/ButtonPress_Yada`と`Dev/ButtonPress`の固定コミットから必要なソースを
`_local/wrist_press/sources/`へ取り出す。checkout、merge、commit、pushはしない。
使用版は`_local/wrist_press/current.json`に残る。別の版を評価する場合は
`--yada-ref <SHA> --j1-ref <SHA>`を指定する。
G1モデルが無い場合は、エラーに表示された既存の`fetch_models.sh`を実行し、再度準備する。
既存モデルを使う場合は`--model-dir /absolute/path/to/g1_description`で指定できる。

YOLOの重みはGitに含めない。ボタン用は▲／▼、ボトル用はCOCOの`bottle`を使い分ける。
学習済みボタン重みを持っている場合:

```bash
$P -m Manipulation.wrist_press.weights --button-source /absolute/path/to/best.pt
$P -m Manipulation.wrist_press.weights --download-bottle
```

ボタン重みが無い場合は既存のデータ生成・学習スクリプトを利用できる:

```bash
MUJOCO_GL=egl $P -m Manipulation.wrist_press.weights --train-buttons --seeds 100 --epochs 30
```

生成する重みはシミュレーション用。実物や手首の近接画像での精度は別途確認する。
読み込み時にクラス名を検査し、重みが無い／違う場合に別の検出器へ自動変更しない。
使用モデルのSHA256・クラス・生成条件は`_local/wrist_press/weights.json`へ保存する。
Ultralyticsとその配布重みのライセンスは利用する版のAGPL-3.0／商用条件を確認する。

## MuJoCoで押下から復帰まで試す

```bash
MUJOCO_GL=egl $P -m Manipulation.wrist_press.evaluate --seeds 0 1
MUJOCO_GL=egl $P -m Manipulation.wrist_press.evaluate --mode bottle --seeds 0 --head-only --output _local/wrist_press/results/bottle.json
MUJOCO_GL=egl $P -m Manipulation.wrist_press.evaluate --seeds 0 1 --eval-set realistic
```

画面のあるPCでは`--view`を付けられる。EGLを利用できない場合は、従来のYadaと同様に
GLFW／仮想ディスプレイを設定する。

ボタンモードではYadaの乗り場を利用し、手首カメラと手全体の物理衝突形状を追加する。
既存のYadaファイル・標準採点は変更しない。新しいランナーは点灯後も継続し、腕の復帰と
全区間の非対象接触を調べる。`--head-only`で手首カメラとの比較ができる。
`--detector classic`は**ボタン用の明示的な画像処理診断**。YOLOの評価結果と混ぜない。

ボトルは固定治具上の不透明ラベル付き円柱モデル。転倒や押して移動させるタスクではない。
接触情報を採点側だけで測る。制御側はYOLO・深度・関節状態だけを使い、接触は追従誤差からの
推定に留める。人工的なボトルをYOLOが検出できない場合は認識失敗として記録する。
接触後は短い押し込みで止め、圧力を上げ続けない。

汎用モデルでは人工形状を検出できない場合、試験専用のボトルモデルを作れる。
実物用の汎用モデルは上書きせず、使用する重みを明示する。

```bash
MUJOCO_GL=egl $P -m Manipulation.wrist_press.train_bottle --images 40 --epochs 15
MUJOCO_GL=egl $P -m Manipulation.wrist_press.evaluate --mode bottle --weights _local/wrist_press/bottle_yolo/weights/best.pt --seeds 0
```

全体・頭RGB・手首RGBを同期した動画を保存する場合は、ボトル評価コマンドに
`--video-dir _local/wrist_press/video`を追加する。WebM動画と撮影時刻のJSONを保存する。
動画はシミュレーション時間の等速・10 fpsで、接触力はシミュレーターで測った値を表示する。
録画処理は観測の表示だけに使い、制御へ正解座標を渡さない。

このデータ生成に限り、学習用の正解枠をセグメンテーションから作る。
制御・評価中の認識器には画像だけを渡す。学習の種10000以降と評価の種は分ける。

結果JSONには点灯までの時間、復帰までの時間、段階別時間、使用カメラ、補正回数、
認識・計画時間、非対象接触力、物理的な押下成否、復帰成否を別々に保存する。
`success`は対象の物理押下・復帰・非対象接触1 N未満・制御エラーなしを要求する。
接触力1 Nは数値的な微小接触を区別する評価閾値で、実機の力上限ではない。
全工程10秒は評価目標で、達成した試行だけ`within_time_target`を立てる。
`full_cycle_s`はシミュレーション時間、`wall_time_s`は描画・推論・計画も含むPC上の実時間。
実機で同じ秒数になると確認した結果ではない。
Isaacで接触力を取得できない場合は`null`とし、無接触を確認した扱いにしない。

## 既存Yadaの入口を使う

`yada_agent/agent.py`は標準の`Agent.reset/act/close`、`Observation/Action`に対応する。
準備時に表示されたソースルートを`R`に入れて実行する:

```bash
R=_local/wrist_press/sources/634697e1544a-7356db679c63
MUJOCO_GL=egl $P "$R/Button_Press/Yada/contest/evaluate.py" --agent Manipulation/wrist_press/yada_agent --seeds smoke
bash "$R/Button_Press/Yada/contest/evaluate_isaac.sh" --agent Manipulation/wrist_press/yada_agent --seeds smoke
```

これらは**既存の点灯時終了という採点**を使う。復帰までの評価は新しい`evaluate`を使う。
エージェントにはシミュレーターのimportや正解のボタン座標を含めない。
Isaacの標準入口では頭カメラを利用できる。手首画像を`CameraFrame`として観測の任意フィールド
`wrist`に渡せば同じ制御本体が利用する。

新しいランナーとDDS入口では関節目標とトルクのフィードフォワードを分離する。
標準Yadaの`Action`は関節目標だけなので、この入口に限って位置による重力補償を使う。
その結果、標準入口では幾何検査で拒否する条件が増える場合がある。
新しいMuJoCo側ではトルク項をPDに等価な形で加え、軌道・手の形状と重力補償を分けて検査する。
押下時の補助トルクは`JᵀF`で上限付きに加える。これは力センサーのフィードバックではなく、
実際の接触力にはPD項も加わる。ボトルは既定で押下力の追加項を0にする。

Isaac Labを導入したPythonでは`-m Manipulation.wrist_press.evaluate --sim isaac`で
アプリ起動から行う。手首カメラ・手の衝突形状・固定ボトルの追加入口も用意したが、
このPCにはIsaac Labが無く動作未検証。接触力・ボトル接触の採点が取得できない場合は
完全な成功と判定しない。

## DDSと実機カメラ

Yadaの模擬G1サーバを先に起動し、DDSを導入したPythonで使う。

```bash
$P -m Manipulation.wrist_press.dds --network-interface lo --camera-host 127.0.0.1
$P -m Manipulation.wrist_press.dds --network-interface lo --camera-host 127.0.0.1 --execute
```

既定は指令を送らないdry-run。実機へ送る場合は実測設定を`--config`で渡し、
`real_calibrated: true`と`--execute`を指定する。実機は今回の自動試験では動かさない。
手首は既存のRGB-D単一メッセージ形式の別配信を`--wrist-host HOST --wrist-port 5557`で受ける。
RGBだけを使用する場合は`--wrist-rgb-only`で深度を使用しない。
カメラデバイスの直接取得サーバは型番確定後に既存の配信元へ接続する。
実機の下半身が動いている／開始時から骨盤姿勢が変わった場合は押下を中止する。

画像はRGB、深度はカラーに位置合わせされた光学軸方向の距離[m]、
時刻は露光時刻が必要。PC間の時計を同期する。関節履歴を露光時刻へ補間して座標変換し、
古い画像・同じ露光・未来の時刻は補正に使わない。
手首がRGBのみの場合は、頭で推定した対象面と画像上の視線の交点から横ずれを求める。
対象までの距離を単眼の中心点だけから推定しない。

## 設定と検査

[config.yaml](config.yaml)に指先オフセット、手首の光学座標変換と取付具寸法、
障害物の箱、壁との余裕、速度・加速度・jerk、押込量、時間上限をまとめる。
初期値は公式モデルと**仮想カメラ用**であり実物の校正値ではない。
遮蔽を調べる別の仮想配置は[configs/wrist_oblique.yaml](configs/wrist_oblique.yaml)にある。
輪郭で中心を補正する処理は`refine_button_centers: true`で試せるが、既定では無効。
追加の計画時余裕は`planning_padding_m`で指定できる。実行時の幾何検査は常に行う。
押し込み指令のコンプライアンス補償と、実測指先の押込量の上限は別に扱う。
安全な経路を求められない／観測を失う／上限を超えた場合は、最後に検査した指令を保持して
失敗を返す。未検査の復帰動作を自動実行しない。

```bash
$P -m pip install pytest
PYTHON=$P bash Manipulation/wrist_press/tests/run_tests.sh
```

通常のCIではモデル・Pinocchio・Coalが無いロボット幾何のテストは明示的にスキップする。
実モデルを準備した環境では、手全体の衝突・往復経路・接触許可の範囲も検査する。

確認した結果と未解決の条件は[VERIFICATION.md](VERIFICATION.md)に記載する。
