# 実機Mapping：開始 → 保存 → 浮遊点除去

FAST-LIO2＋OctoMapを使う、native＋SSH方式の無線計測手順。
G1は純正リモコンで操作する。PCとG1を通信可能な同じネットワークへ接続する。
無線では5GHz Wi-Fiを推奨する。端末間通信が遮断されるゲストWi-Fi等は避ける。

## 0. 初回準備

PCにはUbuntu 24.04 x86_64、ROS 2 Jazzy、PCL・Eigen・OctoMapの開発ライブラリが必要。
G1側のPython・Unitree SDK2、カメラ配信、SSH鍵認証も準備する。
導入の詳細は[`real/README.md`](real/README.md)を参照。
Dockerや有線DDSで計測する場合も同文書の構築・設定手順を使う。

リポジトリのルートから実行する。以降のPC側コマンドは`Mapping/real/`で実行する。

```bash
cd Mapping/real
test -f .env || cp .env.example .env
```

計測環境に合わせて`.env`を編集する。既存の`.env`も接続先を確認する。

| 設定 | 指定する内容 |
|---|---|
| `MAPPING_RUNTIME` / `MAPPING_TRANSPORT` | この手順では`native` / `ssh` |
| `G1_IFACE` | PCのWi-Fi NIC名。`ip -br address`で確認する |
| `G1_HOST` / `CAMERA_HOST` | G1とカメラ配信先の、PCから到達できるアドレス |
| `G1_SSH_USER` / `G1_SSH_HOST` | G1のSSHユーザーと接続先。ホスト省略時は`G1_HOST`を使う |
| `ROS_DOMAIN_ID` | PC内の処理に使うdomain。他のROS処理との重複を避ける（例：46） |
| `MAPPING_RUNS_DIR` | PC側の保存先。既定は`Mapping/real/runs/` |

G1側のNICやSDK配置が既定値と異なる場合は、`G1_SENSOR_IFACE`や
`G1_SENSOR_SDK_PATH`等も変更する。`.env`はGit管理対象外。
初回と関連コードの更新後は次を実行する。G1の中継・記録スクリプトも配置される。

```bash
./mapctl build
```

## 1. 計測開始

G1とカメラ配信へ接続し、診断が成功することを確認する。
G1を静止させてから開始し、開始後も10秒程度は静止を保つ。

```bash
./mapctl doctor --backend raw
./mapctl start --backend raw --name room_a
./mapctl view --live
```

表示された`[STARTED]`のセッションIDと`[OUTPUT]`の保存先を控える。
`--name`は計測場所に合わせて変更する。`raw`を明示するとOctoMap経路を使う。
純正リモコンでゆっくり巡回する。FAST-LIO2で点群を位置合わせし、
同時刻のLiDAR姿勢からOctoMapの占有・空きを更新する。
後続の光線が通過した場所の偽点は削除する。生LiDAR・IMU・RGB等はG1内とPCで記録する。
Wi-Fi切断中もG1記録は続き、復帰後は地図用データを時刻順に補送する。
追いつくまでは地図表示が遅れる。RVizの`Camera`には最新のRGB映像を表示する。

## 2. 完了・保存

G1を止め、表示端末でCtrl+Cを押してから実行する。
表示を閉じるだけでは計測は停止しない。

```bash
./mapctl stop
./mapctl validate
```

`[OUTPUT]`のセッションディレクトリにPCD、OctoMap、rosbag、軌跡を保存する。
`stop`はG1記録を停止し、原本を照合付きで回収して、FAST-LIO2 → OctoMapを最初から実行する。
最終地図は`map/map_raw.pcd`、回収した生データは`raw/g1/rosbag2/`に保存する。
再処理には計測時間の約1.25倍以上を要する。G1にも原本が残る。
通信断や回収失敗で`[PENDING]`になった場合は、接続復帰後に`./mapctl recover`を実行する。
回収の途中で切れても、同じコマンドで続きから再開できる。
カメラ内部パラメータ未設定のWARNは、点群保存の失敗とは別である。

## 3. 事後処理（必要な場合のみ）

保存済みの`map/map_raw.pcd`にはOctoMap処理が適用されている。
床基準への補正・高さカット・着色が必要な場合は、以下の手順を追加する。
使用するPython 3環境にnumpy・scipy・matplotlib・PyYAMLを用意する。

まず、対象rosbagのIMUと記録姿勢から**地図座標での上方向**を求め、床・天井面と照合する。
過去の計測値や単純な+Zは流用しない。以下では確認した上方向の3成分を入力する。
`RUN`には`[OUTPUT]`に表示された対象セッションの絶対パスを指定する。
出力先が既に存在する場合は別名に変更する。入力PCDは上書きしない。

```bash
read -r -p '対象セッションの保存先（絶対パス）: ' RUN
SESSION="$(basename "$RUN")"
read -r -p '確認済みの上方向 ux uy uz（空白区切り）: ' UP_X UP_Y UP_Z

# 全記録スキャンとLiDAR姿勢を、計測時のOctoMap実装・設定で再統合する。
./mapctl rebuild "$SESSION" --method octomap \
  --output "$RUN/derived/postprocess/map_rebuilt.pcd"

# 床基準へ変換し、全高版と床から2m未満の高さ着色版を保存する。
OPENBLAS_NUM_THREADS=1 python3 quickstart/postprocess_floor_pcd.py \
  "$RUN/derived/postprocess/map_rebuilt.pcd" \
  "$RUN/derived/floor_postprocessed" \
  --trajectory "$RUN/trajectory/trajectory.tum" \
  --up "$UP_X" "$UP_Y" "$UP_Z" \
  --up-source '対象セッションのIMU・記録姿勢・床天井面で確認' \
  --no-denoise --max-height 2.0
```

- 追加の孤立点フィルタを使う場合は`--no-denoise`を外す。
  既定では半径15cm以内に他の点が2点未満しかない点を除去する。半径は`--radius`で変更する。
  観測が少ない実物の点も落とし得るため、必要な場合だけ使う。
- 床をZ=0、床法線を+Zに変換し、天井も含む全高の処理済みPCDを保存する。
  そのコピーから床高さ2m以上を削除し、0m＝青、1m＝緑、2m付近＝赤で着色する。
- 平面・軌跡が整合しなければ停止する。支持点数・残差・変換行列は`processing.json`に残る。
- 同じ入力PCDの確認済み座標変換を再利用する場合は、`--up`の代わりに
  `--reference-processing`へ既存の`processing.json`を指定できる。
- この処理はSLAMドリフトの補正・ループ閉じ込み・絶対XY校正は行わない。

## 4. 3D表示

通常の保存地図は次で確認する。

```bash
./mapctl view
```

第3節の床基準処理を実行した場合は、native環境のRVizで高さ着色版を表示できる。
別端末では`RUN`を対象セッションの保存先に設定し直す。

```bash
bash quickstart/view_floor_map.sh "$RUN/derived/floor_postprocessed"
```

再生成した`.bt`は元の地図座標のままなので、床基準PCDと同じ座標だと扱わない。

## 5. 保存先と共有

PC側は`MAPPING_RUNS_DIR/<セッションID>/`に保存する。既定は`Mapping/real/runs/`以下。
native＋SSH＋rawで計測した場合の、セッション内の主な保存先は以下のとおり。

| 内容 | セッション内の相対パス |
|---|---|
| PC側のライブ記録 | `raw/rosbag2/` |
| G1から回収した生センサー原本 | `raw/g1/rosbag2/` |
| 最終地図・占有木・OctoMap設定 | `map/map_raw.pcd`、`map/map_raw.bt`、`map/map_raw.octomap.json` |
| 軌跡 | `trajectory/trajectory.tum` |
| 全記録から再生成した点群（第3節） | `derived/postprocess/map_rebuilt.pcd` |
| 床基準化済み全高版（第3節） | `derived/floor_postprocessed/map_floor_aligned.pcd` |
| 床から2m未満・高さ着色版（第3節） | `derived/floor_postprocessed/height_lt_2m/map_floor_height_lt_2m.pcd` |
| 床基準処理の条件・変換行列（第3節） | `derived/floor_postprocessed/processing.json` |

再処理・共有用には、回収済みの`raw/g1/rosbag2/`をディレクトリごと渡す。
`metadata.yaml`とすべての`*.db3`を同じ構成で保つ。
点群・処理条件も共有する場合はセッションディレクトリを丸ごと渡す。
`runs/`はGit管理対象外なので、Gitへのpushだけでは計測データは共有されない。

## 過去の計測例（2026-10-21）

セッション`20261021_204548_room_a`では3,443スキャンを再統合した。
追加の孤立点フィルタで18,017点を除去し、全高版1,128,103点、2m未満版643,074点を保存した。
追加フィルタなしの比較版は2m未満653,565点、天井高は約2.80mだった。
これらは当該計測の実績であり、新しい環境に既存ファイルがあることや同じ結果を前提としない。
