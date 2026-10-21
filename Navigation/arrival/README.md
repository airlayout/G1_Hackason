# 到着位置の計測

既存navigationで到着したときの、パネルに対する立ち位置・向き・停止までの時間を測る。
押下機構の認識器、モデル、制御器はimportしない。ロボットへ移動・関節指令を送る機能はない。

最初に読むもの:

- [接続条件](CONNECTION_CONTRACT.md): 将来の巡回側／押下側の責任とシナリオ確認。
- [検証結果](VERIFICATION.md): 自動検査と合成データで確認した範囲、実機の残作業。
- [実機データの取得状況](reports/field/report.md): 30試行すべて未取得。
- [合成データの解析検証](reports/synthetic/report.md): 既知のずれと欠測を含む30試行の解析。

合成データは解析処理の検査用で、歩行シミュレーションではない。実機の到着精度・押下成功率は示さない。
ROS/DDS/実機カメラでの新しい記録処理は、実機接続では未検証。

## オフラインで試す

リポジトリ直下、Python 3.10以上とnumpyを使う。ROS・SDK・押下用依存は不要。
セッション作成先は新しいディレクトリを指定する。既存セッションは上書きしない。

```bash
python3 -m Navigation.arrival synthetic --session _local/arrival_measurement/demo
python3 -m Navigation.arrival analyze --session _local/arrival_measurement/demo
bash Navigation/arrival/tests/run_tests.sh
```

`analysis/report.md`、全30行の`analysis/trials.csv`、品質・校正値・入力SHA256を含む
`analysis/results.json`を出力する。CSVの空欄とJSONのnullは未測定であり、0ではない。
`expected.json`は合成データの解析的な正解で、解析処理は読み込まない。

## 実機での計測手順

既存navigationの起動・走行許可・停止操作は、既存の[実機手順](../nav2_stable/safety/runbook_nav2_session.md)を使う。
ここで追加するプロセスは購読と保存だけを行う。計測のために押下機構は起動しない。

### 1. セッションと校正

```bash
python3 -m Navigation.arrival init --session _local/arrival_measurement/field
```

`session.json`には基準試行`reference`と、3経路×10回の固定一覧を作る。
試行順の初期値はroute_1_01→route_2_01→route_3_01→route_1_02…とし、時間によるドリフトと
経路の違いを混同しにくくする。3経路は同じ基準立位へ向かう異なる接近経路として現場で登録する。
基準立位は通行・観測に支障のない位置に設け、最適な押下位置という意味にはしない。

`session.json`の`target`に対象ID・押したいボタン・共通のnavigation目標を記入する。
`nav_goal`の形式は`{"frame_id":"map","child_frame_id":"base_link","xyz":[x,y,0],"xyzw":[qx,qy,qz,qw]}`。
目標が未記入でも記録はできるが、目標に対するnavigation誤差は未測定になる。

`calibration.json`の初期値は意図的に未校正。実測値が得られた項目だけ記入する。

| 項目 | 意味 |
|---|---|
| `id`, `measured` | 校正の識別子と実測済みであること。仮想配置を実測扱いにしない |
| `camera`, `optical_frame` | 保存するカメラ名と光学座標の名前。画像メタデータと一致させる |
| `joints` | 骨盤からカメラまでの可動経路。各要素は`T_parent_joint`、関節軸`axis`、実測角の`q_index`、必要なら`offset_rad` |
| `T_last_optical` | 最後の関節から光学座標への実測4×4変換。光学座標は右x・下y・前z |
| `T_pelvis_imu` | 骨盤に固定されたIMUの実測取付変換。IMU姿勢はworld←IMU、wxyz順 |
| `imu_joints`, `T_last_imu` | IMUが腰より上にある場合の可動経路。指定した場合は固定の`T_pelvis_imu`に優先する |
| `T_base_pelvis` | 床基準のnavigation座標から骨盤への実測立位変換。省略時はnavとの位置比較をしない |
| `translation_uncertainty_m`, `rotation_uncertainty_rad` | 校正の不確かさ。未測定ならnullのまま残す |

可動経路は順番に`T_parent_joint × Rot(axis, q[q_index]+offset_rad)`を掛け、最後に取付変換を掛ける。
関節順はG1 29DoFのmotor index順（脚0–11、腰12–14、左腕15–21、右腕22–28）。
腰角は観測値であり、動作方式や可動範囲の指定ではない。

`base_link`はnavigationでは床基準、押下の幾何では骨盤が基準となる。
固定の`T_base_pelvis`を使う比較は静止立位の近似で、脚姿勢・身体の揺れによる誤差も残る。
実機のIMU／LiDARの搭載リンクとnavigationのframe定義を確認してから値を入れる。

PC間で時計を同期し、同期誤差を測って`clock.synchronized`と`clock.max_offset_s`を記入する。
既定の補間予算では時計誤差上限50 ms、関節履歴の最大間隔100 ms。これは解析条件であり制御の安全閾値ではない。
未確認なら初期値のまま記録できるが、時刻をまたぐ計測結果は未測定になる。

### 2. 購読プロセス

3つの端末で、各依存がある既存環境のPythonを使う。別PCの場合は同じセッション定義を配り、
計測後に`streams/ros.jsonl`、`streams/body.jsonl`、`streams/frames.jsonl`と`frames/`を1つのディレクトリへ集める。
同一streamの二重起動はファイルロックで拒否する。既存の記録への追記は可能。

ROS 2環境（rclpy、tf2_ros、標準メッセージ）:

```bash
python3 -m Navigation.arrival record-ros --session _local/arrival_measurement/field --duration 600
```

G1 SDK環境（unitree_sdk2py、numpy）:

```bash
python3 -m Navigation.arrival record-body --session _local/arrival_measurement/field --interface eth0 --duration 600
```

`record-body`は受信時刻とデバイスtickを別に保存する。デバイスtickを撮影時刻と同じ時計だと仮定しない。
実測した状態受信時刻の誤差上限がある場合のみ`--time-uncertainty 秒`を指定する。
省略時も生データは保存するが、露光時刻の姿勢計算と停止時間の計測には使わない。

RGB-D購読環境（numpy、opencv-python、pyzmq）:

```bash
python3 -m Navigation.arrival record-camera --session _local/arrival_measurement/field --host 192.168.123.164 --optical-frame head_optical --duration 600
```

既存の5556番`G1RGBD1`形式を受ける。JPEGはBGRとして復号してRGBで保存し、深度をmへ変換する。
深度はRGBに位置合わせ済み、歪み補正済みであることが必要。RGBだけの5555番形式から距離は推測しない。
送信元のtimestampが実際の露光時刻と確認できた場合のみ`--exposure-verified`を付ける。
未確認の送信時刻・受信時刻を露光時刻として代用しない。

ROS記録は目標pose、GoalStatusのUUID、map→base_link、実測SLAMのodom、速度指令、状態を保存する。
`/g1/sdk_odom`のような指令の積分値を実測姿勢として使用しない。
自律巡回の目標は`/goal_pose`に出ないため、共通目標は必ずsessionの`target.nav_goal`にも記入する。

### 3. 基準と30試行

基準立位で、静止を含む5秒以上を記録する。

```bash
python3 -m Navigation.arrival mark --session _local/arrival_measurement/field start --trial reference
python3 -m Navigation.arrival mark --session _local/arrival_measurement/field end --trial reference
```

各接近では、**目標を送る前**にstartを記録し、既存navigationで移動する。
到達通知後も最低3秒間を残し、停止状態が記録できてからendを記録する。
start前の位置取りは試行に含めない。試行中に人が介入した場合は必ず記録する。

```bash
python3 -m Navigation.arrival mark --session _local/arrival_measurement/field start --trial route_1_01
python3 -m Navigation.arrival mark --session _local/arrival_measurement/field end --trial route_1_01
```

失敗でもendを付ける。例:

```bash
python3 -m Navigation.arrival mark --session _local/arrival_measurement/field manual_intervention --trial route_1_01 --reason "リモコンで停止"
python3 -m Navigation.arrival mark --session _local/arrival_measurement/field end --trial route_1_01 --reason "timeout"
```

1試行に複数Goalがある場合、`ros.jsonl`で対象のGoal UUIDを確認して`mark ... bind_goal --trial ID --goal-id UUID`を使う。
新規Goalが一意でなければ到達結果を推定しない。過去のGoalのSUCCEEDEDが配信され直しても到達とは扱わない。
同じ試行IDは再使用できない。再試行する評価は別セッションとし、元の失敗を残す。
startのまま終了した試行は`incomplete`、未実施は`not_recorded`として集計する。

### 4. オフライン指定と分析

`streams/frames.jsonl`から基準と各試行のフレームIDを選ぶ。
到達直後・停止後・数秒後の複数枚を残し、停止後のフレームを最低1枚指定する。
画面のある環境では、パネルの四隅を順にクリックし、最後に対象ボタン中心をクリックする。
Enterで保存、Rでやり直し、Escで変更を保存せず閉じる。

```bash
python3 -m Navigation.arrival annotate --session _local/arrival_measurement/field --frame-id FRAME_ID
python3 -m Navigation.arrival analyze --session _local/arrival_measurement/field
```

GUIを使えない場合は`annotate`に`--panel '[[u,v],...]' --button u v`を渡せる。
パネルの奥の壁や手を含めず、同じ物理面を囲む。ボタン周辺は平面当てはめから除外する。

## 出力の解釈

- パネル面とボタン中心視線の交点を原点とし、パネルに対する骨盤の前後・左右・高さ・向きを求める。突出量や押下可否は求めない。
- 画像と関節は露光時刻で対応させる。関節履歴の外挿、未校正変換、未確認の時計、欠損深度は拒否する。
- 停止確認は脚速度0.05 rad/s以下と、実測odomの0.2秒差分速度0.02 m/s・0.05 rad/s以下が0.5秒続いた時点。
  既定値は計測用の仮説であり、再開許可ではない。閾値は`analysis`に保存し、実機静止ノイズを確認して評価を分ける。
- 到達時刻は対象Goalの終端statusを最初に観測した時刻。実際の物理的到達時刻やaction result受信時刻とは区別する。
- 到達後3秒のodom最大変位・yaw変化・IMU姿勢変動を別々に出す。必要な期間が欠ける値は未測定。
- 基準には、静止確認後の最初の有効な指定画像を使う。試行も静止確認後の最初の有効画像を採用する。
  静止後画像が無い場合は観測値だけ残し、偏り・ばらつきの統計から外す。
- 位置は母平均・母標準偏差、向きは円平均・円標準偏差、絶対値p95・最大値を出す。全30行に欠測・手動介入を残す。
- navigationと局所観測の比較は、基準時のパネル姿勢で地図を対応させた後の**不一致量**。
  カメラ、取付校正、床→骨盤の近似誤差を含むので、SLAMの真の誤差とは断定しない。

保存形式は`session.json`と`calibration.json`、JSONLの各stream、RGB/depth_m/Kを持つNPZ、
手動指定の`annotations.json`で構成する。時刻はUnix秒、距離はm、角度はrad、
4×4変換は`T_parent_child`（子の点を親へ変換）、JSON/CSV欠測値はnull/空欄とする。
