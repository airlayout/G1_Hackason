# Mapping / real

G1の部屋Mappingを、内蔵LIOと生LiDAR＋IMUの2方式で実行する現場用環境。
操作はすべて`mapctl`から行い、成果物形式はbackendによらず共通になる。

## Dockerなしで実機を手動計測する

### このPCで無線計測する

PCとG1を5GHzの`Physical_AI_5G`へ接続する。現在の`.env`はPCの
`wlp128s20f3`とG1のWi-Fi IP `192.168.0.82`を使用する設定になっている。
センサー中継のSSHは`G1_SSH_HOST=100.78.135.14`でTailscale経由にしている。
直接Wi-Fi IPへのSSHが停止する場合を避ける設定で、物理通信は同じ5GHz Wi-Fiを使う。
有線ケーブルは不要。次の1コマンドで診断・計測・ライブ表示を開始する。

```bash
cd /home/ubuntu/dev/G1_Sodai/Mapping/real && ./mapctl start --backend raw --name room_a && ./mapctl view --live
```

5〜10秒静止した後に純正コントローラで低速移動する。終了時はG1を止め、
端末でCtrl+Cを押して表示を閉じ、`./mapctl stop`、`./mapctl validate`を実行する。
完了までWi-Fi接続を維持する。

無線方式は`MAPPING_TRANSPORT=ssh`、`MAPPING_RUNTIME=native`を使用する。
LiDAR/IMUはG1内部`eth0`のDDS domain 0からCDRのままSSHで中継し、
PC内はループバック・domain 46でFAST-LIO2へ入力する。RGBもG1のWi-Fi IPへ接続する。
中継は診断・計測に合わせて自動起動・終了し、G1へ歩行指令は送らない。
SSH鍵で接続できることが前提。初回の`./mapctl build`で中継スクリプトをG1へ配置する。
G1のDHCPアドレスが変わった場合は`.env`の`G1_HOST`と`CAMERA_HOST`を更新する。

### 初回構築と共通操作

Ubuntu 24.04 x86_64にROS 2 Jazzy、PCL・Eigen・OctoMapの開発ライブラリが
導入済みなら、`native`環境を使用できる。FAST-LIO2と共通ノードを
リポジトリ内の`_local/mapping/`へ構築し、ホストのROSやconda環境を変更しない。
不足するCycloneDDS・pcl_ros・pyzmqは、固定版のdebを同じ場所へ展開する。
直接DDS購読ではDocker・sudo・G1へのSSHは計測自体には必要ない。
無線SSH中継ではG1へのSSH鍵認証を使用する。

```bash
cd Mapping/real
# .envが既にあれば保持し、次の値を設定する。
test -f .env || cp .env.example .env
# MAPPING_RUNTIME=native
# G1_IFACE=<ip -br addressで確認した実機用Ethernet NIC>
./mapctl build
./mapctl doctor --backend raw
```

G1上のカメラ配信が起動済みなら、そのまま利用する。未起動の場合だけ下記の
カメラ専用serverを起動する。カメラの焦点距離は推測で埋めず、未取得なら
`CAMERA_FX/FY/CX/CY`を0のままにする。この場合、画像は記録されるが
`validate`は内部パラメータ未確定のWARNを表示する。

```bash
# 端末1: 計測開始。autoではなくrawを明示する。
./mapctl start --backend raw --name room_a
./mapctl status

# 端末2: ライブ地図・軌跡・RGB
./mapctl view --live

# G1を5〜10秒静止させてから、純正リモコンで低速に部屋を巡回する。
# 計測処理から歩行指令やSLAM開始・終了APIは送信しない。

# 端末1: 停止・保存。G1とカメラ配信はこの完了まで接続したままにする。
./mapctl stop
./mapctl validate
./mapctl view
```

成果物は`runs/<session>/`にまとまる。`map/map_raw.pcd`、`map/map_raw.bt`、
`map/map_raw.octomap.json`、`raw/rosbag2/`、軌跡とログを保存する。
停止時は地図を保存した後に各プロセスを正常終了し、rosbagを最後に閉じる。
端末を閉じても記録は続くため、必ず`./mapctl stop`を使う。

再生成はDocker版と同じC++実装・計測時の設定を使う。

```bash
./mapctl rebuild <session_id> --output /tmp/room_rebuilt.pcd
bash tests/run_tests.sh
```

2026-10-21にこのnative経路を実機で確認した。LiDAR約9.96Hz、IMU約200Hz、
640×360 RGB約5Hzを受信し、短時間記録から33,609点・86スキャンの地図を保存した。
保存PCDとrosbagから再生成したPCDはSHA-256が一致した。RVizでの保存地図配信と
GUI起動も確認済み。歩行しながらの地図品質とカメラ内部パラメータの確定は別途確認する。

同日の移動計測では3,443スキャンから地図を再生成し、床基準化して3D表示まで確認した。
絶対位置やSLAMドリフトの定量精度は未評価。今回の操作・保存先は
[`../MAPPING_QUICKSTART.md`](../MAPPING_QUICKSTART.md)にまとめた。

native環境の対象backendは`raw`と`sim`。内蔵SLAMを使う`onboard`と
Docker field kitの`bundle`は、以下のDocker環境向けである。

## backend

| backend | 処理場所 | 主な依存 | PCD保存 |
|---|---|---|---|
| `onboard` | G1内蔵SLAMサービス | Unitree SDK2 | 登録済み点群から共通再構成 |
| `raw` | 現場Ubuntu PC | ROS 2 Humble / Jazzy、FAST-LIO2、OctoMap | 空き観測で訂正した共通地図を保存 |
| `sim` | Isaac Sim＋現場PC | ROS 2 Humble、FAST-LIO2、OctoMap | rawと同じ処理で保存 |

いずれも、生LiDAR・IMU・RGB・CameraInfo・正規化済みodometry・登録点群を同じ
rosbag2へ並行記録する。これが再処理可能な正本になる。

## FAST-LIO2＋OctoMapの統合経路

`raw` / `sim` は、FAST-LIO2の歪み補正済み登録点群と、そのスキャン終端時刻の
LiDAR姿勢を使ってOctoMapを更新する。後の光線が通過した場所は占有確率を下げ、
空きになった場所の点を `/g1_mapping/map` から削除する。
表示・停止時のPCD保存・事後再生成は、同じC++実装と設定を使う。

```bash
./mapctl build                         # 選択した実行環境を構築・更新する
./mapctl start --backend raw --name room_a
./mapctl view --live                   # 別端末
./mapctl stop                         # 訂正済みmap_raw.pcdを保存
./mapctl rebuild <session_id> --output /tmp/room_rebuilt.pcd
```

`auto` は従来どおりUnitree内蔵方式を優先する。この統合経路には `--backend raw`
（Isaac Simでは `sim`）を明示する。内蔵方式の蓄積処理は別経路のままである。

保存先は `map/map_raw.pcd`、占有木は `map/map_raw.bt`、設定・処理スキャン数は
`map/map_raw.octomap.json`。PCDには `density` に加えて `occupancy` を持たせる。
`density` は観測回数の表示用指標で、占有確率ではない。

OctoMapの設定は `.env` の `OCTOMAP_*`、出力点群の間引きは `MAP_VOXEL_SIZE` で指定する。
計測時の設定はmanifestにも保存し、事後再生成で再利用する。
既定のレイ上限は3mで、その外側など「空きの証拠がない」点は未知として残す。
未観測と空きは区別し、空スキャンだけで障害物を消すことはしない。

FAST-LIO2のLiDAR姿勢配信は固定commitへの小さなpatchで追加している。
LiDAR–IMUの推定外部パラメータを含むため、IMUの位置をLiDAR原点として代用しない。
点群と姿勢は時刻の完全一致で組み合わせる。GLIMやループ閉じ込みは含まない。

## Docker方式の必要環境

- Ubuntu PC
- Docker Engine + Docker Compose
- G1と直結するEthernet NIC
- PCD自動回収を使う場合は、G1へログインできるSSH鍵

ROS 2やPCLをホストへ直接インストールする必要はない。

## Docker方式の初回準備

```bash
cp .env.example .env
# .envのG1_IFACEと必要なトピック名を編集
chmod +x mapctl scripts/*.sh docker/*.sh
./mapctl build
```

## 実機なしでの動作確認

```bash
./mapctl doctor --mock
./mapctl start --backend mock --name local_test
./mapctl status
./mapctl stop
./mapctl validate
```

mockでも実際と同じセッション状態遷移を通り、PCD、trajectory、rosbag metadata、
品質レポートを生成する。

## 現場操作

G1 PC2では、歩行modeに触れないカメラ専用serverを先に起動する。

```bash
ssh -t unitree@192.168.123.164 'bash ~/mapping_tools/start_camera_only.sh'
```

この端末は計測中そのまま開いておく。別端末の現場PCで次を実行する。

```bash
./mapctl doctor --backend auto
./mapctl start --backend auto --name room_a
./mapctl status
./mapctl stop
./mapctl validate
```

`mapctl view --live`では、密度色付き蓄積点群、最新登録点群、自己位置、軌跡、RGBを
同時表示する。青は観測回数が少なく、黄〜赤は繰り返し観測された領域を表す。
密度1.0までに必要なスキャン数は`.env`の`DENSITY_TARGET_SCANS`で変更できる。

## rosbagから地図を作り直す

通信断などでG1へ「Mapping終了」を送れないと、G1側にPCDが書き出されずセッションが
`map_raw.pcd`なしで終わる。この場合はrosbagに残った地図点群から作り直せる。

```bash
./mapctl rebuild                          # 最新セッション
./mapctl rebuild <session_id>             # セッション指定
./mapctl rebuild --voxel 0.02             # 解像度を上げる（既定 0.05m）
./mapctl rebuild --method accumulate --voxel 0    # 訂正なし・間引きなし
./mapctl rebuild --method accumulate --topic /unitree/slam_mapping/points
./mapctl rebuild --force                  # 既存のmap_raw.pcdを上書きする
```

`raw` / `sim` の既定はOctoMap再生成で、共通C++実装が `db3` の登録点群と
`/g1_mapping/sensor_pose` を読む。`metadata.yaml`が欠けていても再生成できる。
`--voxel` を省略すると計測時の値を使う。OctoMap再生成では正の値が必要になる。

`onboard` の既定、および明示した `--method accumulate` は従来の単純蓄積で、
ホストPythonのみで動く。この経路では同一ボクセルの最初の1点を残し、空き観測による
訂正は行わない。`--voxel 0` による間引きなしの指定もこの経路で使用する。

センサ姿勢の記録がない古いraw/simセッションは、既定のOctoMap再生成でエラーにする。
古い生LiDAR＋IMUを使って作り直す場合は次を使う。
この旧記録向けの再処理スクリプトはDocker方式で実行する。

```bash
./quickstart/replay_fastlio.sh <session_id>
```

再生するのは生LiDARとIMUのみで、古い登録点群や地図を再入力しない。
記録に `/clock` がある場合はその時計を使い、ない場合はbagの再生時計を生成する。
シミュレーションではゼロ開始の正のセンサ時刻を受け付け、時刻0のスキャンは除外する。
訂正済みPCDと、`derived/<再生ID>/rosbag2` に新しい点群・センサ姿勢を保存する。
終了時に表示する `mapctl rebuild --bag-dir ...` コマンドで、その結果を再生成できる。

既存の`map_raw.pcd`は`--force`なしでは上書きしない。

作り直したら`./mapctl validate`で検証し、`./mapctl view <session_id>`で確認する。

### 床基準の高さ処理（地図Zを鉛直と仮定しない）

FAST-LIOの初期地図座標は必ずしも水平・上向きではない。`Z < 2`だけでは
「床から2m未満」にはならない。まずIMU加速度を地図座標で確認し、軌跡を挟む
平行な床・天井面と照合する。初期IMU座標と地図初期座標が一致する場合以外は、
IMU加速度を記録姿勢で地図座標へ回転してから使う。

`quickstart/postprocess_floor_pcd.py` は、確認済みの `--up ux uy uz` と
`--trajectory trajectory/trajectory.tum` を使い、床・天井を推定して別ディレクトリへ
出力する（numpy/scipy/matplotlib/PyYAMLが必要）。元PCDは上書きしない。
入力は、上記 `mapctl rebuild --output ...` で元地図とは別に再生成したPCDを推奨する。

- 半径15cm以内に他の2点がない孤立点のみ除去する。
- 床をZ=0、床法線を+Zにする右手座標系へ剛体変換する。
- 処理済み全点群を複製し、保存値で床高さ `height < 2.0 m` を満たす点だけ残す。
- 高さ0〜2mを青→水色→緑→黄→赤で着色し、元の密度・観測回数等も保持する。
- `processing.json` に平面支持数・残差・変換行列・ハッシュを残す。

近傍点による孤立点除去はOctoMapとは別の任意の補助処理で、`--no-denoise` で省略できる。
同一ソースの確認済み座標変換は `--up` の代わりに `--reference-processing processing.json`
を指定して再利用できる。比較時はこの方法で床基準だけを揃え、元の点は除去せずに表示できる。

床・天井が検出できない場合や軌跡と整合しない場合は停止する。高さ処理は
ループ閉じ込みや姿勢再最適化の代わりではなく、XYの絶対方位も確定しない。
再生成時の `.bt` は元の地図座標のままで、床座標のPCDとは混在させない。
床基準の結果は、G1に接続せずDockerなしで次のように開ける。

```bash
bash quickstart/view_floor_map.sh runs/<session_id>/derived/<処理出力ディレクトリ>
```

## RViz表示

Mapping中のライブ表示は、別のGUI端末から実行する。

```bash
./mapctl view --live
```

保存済みPCDはG1に接続していない開発PCでも表示できる。

```bash
./mapctl view                         # 最新セッション
./mapctl view <session_id>            # セッション指定
./mapctl view <session_id> --publish-only
```

`--publish-only`はPCDを`/g1_mapping/map`へ配信するだけで、RVizを起動しない。
GUI版はX11/XWaylandの`DISPLAY`を可視化専用コンテナへ渡す。Mappingコンテナへ
RVizやディスプレイソケットを渡すことはない。

GUI接続で失敗する場合は`echo $DISPLAY`を確認する。Xauthorityが標準位置にない環境では、
実在する認証ファイルを`XAUTHORITY`へ設定してから実行する（例:
`export XAUTHORITY=/run/user/<UID>/gdm/Xauthority`）。

詳細は[`../FIELD_RUNBOOK.md`](../FIELD_RUNBOOK.md)を参照。

## 統合経路の検証

```bash
bash tests/run_tests.sh
```

`liboctomap-dev`がある環境では、偽点の削除・静止構造と未知領域の保持・保存後の
確率更新・表示周期に依存しない再生成をC++で検証する。CIにもこの依存を追加している。
ROS 2とビルド済みworkspaceがある環境では `test_occupancy_ros.py` が実ノードを起動し、
偽点が消えること、保存失敗の伝播、PCD保存とmetadataなしdb3からの再生成の一致を確認する。
ROSや3DGSの依存がないホストでは、それぞれのテストを理由付きでskipする。

## オフライン配備

開発PCでimagesを構築した後、次を実行する。

```bash
./mapctl bundle
```

`dist/`に3つのDocker images（onboard、raw、visualization）、ソース、チェックサムを
まとめたfield kitが作られる。
現場PCで展開し、`install-field-kit.sh`を実行する。

## 外部依存の固定

利用するcommitは[`vendor/mapping.repos`](vendor/mapping.repos)と
[`docker/Dockerfile`](docker/Dockerfile)のbuild argsで固定している。更新時は両方を同時に
変更し、mock、Docker build、rosbag replayを再検証する。
